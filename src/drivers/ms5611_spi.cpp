#include "drivers/ms5611_spi.h"

#include <SPI.h>

#include "config.h"

namespace {
constexpr uint8_t CMD_RESET = 0x1E;
constexpr uint8_t CMD_ADC_READ = 0x00;
constexpr uint8_t CMD_PROM_READ = 0xA0;
constexpr uint8_t CMD_CONV_D1_OSR4096 = 0x48;
constexpr uint8_t CMD_CONV_D2_OSR4096 = 0x58;
constexpr uint32_t CONV_TIME_US = 10000;
constexpr uint32_t BARO_DELAY_US = 90000;

const SPISettings kBaroSpiSettings(4000000, MSBFIRST, SPI_MODE0);

bool spiWriteCommand(uint8_t cmd) {
    SPI.beginTransaction(kBaroSpiSettings);
    digitalWrite(MS5611_SPI_CS_PIN, LOW);
    SPI.transfer(cmd);
    digitalWrite(MS5611_SPI_CS_PIN, HIGH);
    SPI.endTransaction();
    return true;
}
}

bool MS5611SPI::reset() {
    return spiWriteCommand(CMD_RESET);
}

bool MS5611SPI::read_prom() {
    for (uint8_t i = 0; i < 8; ++i) {
        SPI.beginTransaction(kBaroSpiSettings);
        digitalWrite(MS5611_SPI_CS_PIN, LOW);
        SPI.transfer((uint8_t)(CMD_PROM_READ + (i * 2)));
        const uint8_t msb = SPI.transfer(0x00);
        const uint8_t lsb = SPI.transfer(0x00);
        digitalWrite(MS5611_SPI_CS_PIN, HIGH);
        SPI.endTransaction();

        prom[i] = ((uint16_t)msb << 8) | (uint16_t)lsb;
    }
    return true;
}

bool MS5611SPI::start_conversion(uint8_t command) {
    return spiWriteCommand(command);
}

bool MS5611SPI::read_adc(uint32_t &value) {
    SPI.beginTransaction(kBaroSpiSettings);
    digitalWrite(MS5611_SPI_CS_PIN, LOW);
    SPI.transfer(CMD_ADC_READ);
    const uint8_t b2 = SPI.transfer(0x00);
    const uint8_t b1 = SPI.transfer(0x00);
    const uint8_t b0 = SPI.transfer(0x00);
    digitalWrite(MS5611_SPI_CS_PIN, HIGH);
    SPI.endTransaction();

    value = ((uint32_t)b2 << 16) | ((uint32_t)b1 << 8) | (uint32_t)b0;
    return true;
}

bool MS5611SPI::setup() {
    SPI.begin();
    pinMode(MS5611_SPI_CS_PIN, OUTPUT);
    digitalWrite(MS5611_SPI_CS_PIN, HIGH);

    is_ready = reset();
    if (!is_ready) {
        return false;
    }

    const uint32_t t0 = micros();
    while ((uint32_t)(micros() - t0) < 3000U) {
    }

    is_ready = read_prom();
    state = ReadState::Idle;
    has_fresh_sample = false;
    pressure_count_since_temp = 0;
    have_temperature_sample = false;
    return is_ready;
}

void MS5611SPI::compute_compensation(BaroData &data) {
    const int64_t c1 = prom[1];
    const int64_t c2 = prom[2];
    const int64_t c3 = prom[3];
    const int64_t c4 = prom[4];
    const int64_t c5 = prom[5];
    const int64_t c6 = prom[6];

    const int64_t dT = (int64_t)raw_temperature - (c5 << 8);
    const int64_t temp_x100 = 2000 + ((dT * c6) >> 23);
    const int64_t off = (c2 << 16) + ((c4 * dT) >> 7);
    const int64_t sens = (c1 << 15) + ((c3 * dT) >> 8);
    const int64_t pres_x100 = ((((int64_t)raw_pressure * sens) >> 21) - off) >> 15;

    data.temp_c = (float)temp_x100 * 0.01f;
    data.pres_pa = (float)pres_x100;
}

bool MS5611SPI::read(BaroData &data) {
    if (!is_ready || !has_fresh_sample) {
        return false;
    }
    data.timestamp = micros() - BARO_DELAY_US;
    has_fresh_sample = false;
    compute_compensation(data);
    return true;
}

void MS5611SPI::kick() {
    if (!is_ready) {
        return;
    }

    const uint32_t now = micros();
    if (state == ReadState::Idle) {
        if (start_conversion(CMD_CONV_D1_OSR4096)) {
            conversion_started_us = now;
            state = ReadState::WaitPressure;
        }
        return;
    }

    if (state == ReadState::WaitPressure) {
        if ((uint32_t)(now - conversion_started_us) < CONV_TIME_US) {
            return;
        }
        if (!read_adc(raw_pressure)) {
            state = ReadState::Idle;
            return;
        }
        ++pressure_count_since_temp;
        const bool need_temperature = !have_temperature_sample || (pressure_count_since_temp >= 10);

        if (need_temperature) {
            if (start_conversion(CMD_CONV_D2_OSR4096)) {
                conversion_started_us = now;
                state = ReadState::WaitTemp;
            } else {
                state = ReadState::Idle;
            }
        } else {
            has_fresh_sample = true;
            state = ReadState::Idle;
        }
        return;
    }

    if (state == ReadState::WaitTemp) {
        if ((uint32_t)(now - conversion_started_us) < CONV_TIME_US) {
            return;
        }
        if (!read_adc(raw_temperature)) {
            state = ReadState::Idle;
            return;
        }

        have_temperature_sample = true;
        pressure_count_since_temp = 0;
        has_fresh_sample = true;
        state = ReadState::Idle;
    }
}
