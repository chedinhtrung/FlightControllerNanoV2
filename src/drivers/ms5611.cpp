#include "drivers/ms5611.h"

#include <Wire.h>
#include <math.h>

#include "config.h"

namespace {
constexpr uint8_t MS5611_ADDR = 0x77;  // Default I2C address (CSB high). Use 0x76 if CSB low.

// Commands from the MS5611 datasheet.
constexpr uint8_t CMD_RESET = 0x1E;      // Reset internal state machine and PROM interface.
constexpr uint8_t CMD_ADC_READ = 0x00;   // Read 24-bit ADC result after conversion.
constexpr uint8_t CMD_PROM_READ = 0xA0;  // PROM base read command; offsets are 0xA0 + 2*n.

// Conversion commands: D1 = pressure, D2 = temperature.
// OSR = 4096 gives best resolution/noise at the cost of conversion time.
constexpr uint8_t CMD_CONV_D1_OSR4096 = 0x48;
constexpr uint8_t CMD_CONV_D2_OSR4096 = 0x58;

constexpr uint32_t CONV_TIME_US = 10000;  // 4096 OSR max conversion time is about 9.04 ms.
constexpr uint32_t BARO_ALT_HZ = 30;
constexpr uint32_t BARO_ALT_PERIOD_US = 1000000UL / BARO_ALT_HZ;
constexpr float ALT_FILTER_ALPHA = 0.05f;

bool write_command(uint8_t cmd) {
    Wire.beginTransmission(MS5611_ADDR);
    Wire.write(cmd);
    return Wire.endTransmission() == 0;
}
}  // namespace

// Send sensor reset command and wait for PROM to become readable.
bool MS5611::reset() {
    if (!write_command(CMD_RESET)) {
        return false;
    }
    return true;
}

// Read all PROM calibration words (factory coefficients + reserved/CRC words).
bool MS5611::read_prom() {
    for (uint8_t i = 0; i < 8; ++i) {
        Wire.beginTransmission(MS5611_ADDR);
        Wire.write((uint8_t)(CMD_PROM_READ + (i * 2)));
        if (Wire.endTransmission(false) != 0) {
            return false;
        }

        if (Wire.requestFrom(MS5611_ADDR, (uint8_t)2) != 2) {
            return false;
        }

        prom[i] = ((uint16_t)Wire.read() << 8) | (uint16_t)Wire.read();
    }
    return true;
}

// Start a pressure or temperature conversion using a specific conversion command.
bool MS5611::start_conversion(uint8_t command) {
    return write_command(command);
}

// Read 24-bit conversion result from ADC read command.
bool MS5611::read_adc(uint32_t &value) {
    Wire.beginTransmission(MS5611_ADDR);
    Wire.write(CMD_ADC_READ);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if (Wire.requestFrom(MS5611_ADDR, (uint8_t)3) != 3) {
        return false;
    }

    value = ((uint32_t)Wire.read() << 16) |
            ((uint32_t)Wire.read() << 8) |
            (uint32_t)Wire.read();
    return true;
}

// Initialize I2C access, reset the sensor, and cache PROM calibration coefficients.
void MS5611::setup() {
    Wire.begin();
    is_ready = reset();
    if (!is_ready) {
        return;
    }

    // One-time boot wait for reset completion (setup-time only, not loop-time).
    const uint32_t t0 = micros();
    while ((uint32_t)(micros() - t0) < 3000U) {
    }

    is_ready = read_prom();
    state = ReadState::kIdle;
    has_fresh_sample = false;
    pressure_count_since_temp = 0;
    have_temperature_sample = false;
    last_update_us = micros();
    calibrate();
}

void MS5611::calibrate() {
    if (!is_ready) {
        return;
    }

    constexpr int kSamples = 100;
    float sum_pressure = 0.0f;
    int valid = 0;
    BaroData sample{};

    while (valid < kSamples) {
        kick(sample);
        if (read(sample)) {
            sum_pressure += sample.pres_pa;
            ++valid;
        }
    }

    if (valid > 0) {
        ground_pressure_pa = sum_pressure / (float)valid;
    }

    // Reset altitude LPF so first post-calibration sample re-initializes at the new baseline.
    filter_initialized = false;
    filtered_altitude_m = 0.0f;
}

void MS5611::compute_compensation(BaroData &data) {
    // Datasheet compensation (integer math path, first order).
    const int64_t c1 = prom[1];
    const int64_t c2 = prom[2];
    const int64_t c3 = prom[3];
    const int64_t c4 = prom[4];
    const int64_t c5 = prom[5];
    const int64_t c6 = prom[6];

    const int64_t dT = (int64_t)raw_temperature - (c5 << 8);
    const int64_t temp_x100 = 2000 + ((dT * c6) >> 23);  // 0.01 degC
    const int64_t off = (c2 << 16) + ((c4 * dT) >> 7);
    const int64_t sens = (c1 << 15) + ((c3 * dT) >> 8);
    const int64_t pres_x100 = ((((int64_t)raw_pressure * sens) >> 21) - off) >> 15;  // 0.01 mbar

    data.temp_c = (float)temp_x100 * 0.01f;
    data.pres_pa = (float)pres_x100;  // 0.01 mbar numerically equals Pa.
}

// Returns true when a newly completed sample pair (D1 + D2) is available.
bool MS5611::read(BaroData &data) {
    if (!is_ready) {
        return false;
    }
    if (!has_fresh_sample) {
        return false;
    }
    has_fresh_sample = false;
    compute_compensation(data);
    const float raw_altitude_m = 44330.0f * (1.0f - powf(data.pres_pa / ground_pressure_pa, 0.19029495f));

    if (!filter_initialized) {
        filtered_altitude_m = raw_altitude_m;
        filter_initialized = true;
    } else {
        filtered_altitude_m += ALT_FILTER_ALPHA * (raw_altitude_m - filtered_altitude_m);
    }

    data.altitude_m = filtered_altitude_m;
    return true;
}

// Periodic update helper aligned with the project loop period.
void MS5611::kick(BaroData &data) {
    if (!is_ready) {
        return;
    }

    const uint32_t now = micros();
    if (state == ReadState::kIdle && (uint32_t)(now - last_update_us) < BARO_ALT_PERIOD_US) {
        return;
    }

    if (state == ReadState::kIdle) {
        // Kick pressure conversion and return immediately.
        if (start_conversion(CMD_CONV_D1_OSR4096)) {
            conversion_started_us = now;
            state = ReadState::kWaitPressure;
        }
        return;
    }

    if (state == ReadState::kWaitPressure) {
        if ((uint32_t)(now - conversion_started_us) < CONV_TIME_US) {
            return;
        }
        if (!read_adc(raw_pressure)) {
            state = ReadState::kIdle;
            return;
        }
        ++pressure_count_since_temp;
        const bool need_temperature = !have_temperature_sample || (pressure_count_since_temp >= 10);

        if (need_temperature) {
            if (start_conversion(CMD_CONV_D2_OSR4096)) {
                conversion_started_us = now;
                state = ReadState::kWaitTemperature;
            } else {
                state = ReadState::kIdle;
            }
        } else {
            compute_compensation(data);
            has_fresh_sample = true;
            last_update_us = now;
            state = ReadState::kIdle;
        }
        return;
    }

    if (state == ReadState::kWaitTemperature) {
        if ((uint32_t)(now - conversion_started_us) < CONV_TIME_US) {
            return;
        }
        if (!read_adc(raw_temperature)) {
            state = ReadState::kIdle;
            return;
        }

        have_temperature_sample = true;
        pressure_count_since_temp = 0;
        compute_compensation(data);
        has_fresh_sample = true;
        last_update_us = now;
        state = ReadState::kIdle;
    }
}
