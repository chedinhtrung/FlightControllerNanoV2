#include "devices/barometer.h"

#include <math.h>

Barometer::Barometer(BarometerDriver &driver, uint32_t period_us)
    : driver_(driver), period_us_(period_us), last_kick_us_(micros()) {}

bool Barometer::setup() {
    if (!driver_.setup()) {
        // Placeholder: optional barometer setup error handling.
        return false;
    }
    return calibrate();
}

void Barometer::update() {
    const uint32_t now = micros();
    if ((uint32_t)(now - last_kick_us_) < period_us_) {
        return;
    }
    last_kick_us_ = now;
    driver_.kick();
}

bool Barometer::read(BaroData &out) {
    BaroData sample{};
    if (!driver_.read(sample)) {
        return false;
    }

    const float raw_altitude_m = 44330.0f * (1.0f - powf(sample.pres_pa / ground_pressure_pa_, 0.19029495f));
    if (!filter_initialized_) {
        filtered_altitude_m_ = raw_altitude_m;
        filter_initialized_ = true;
    } else {
        filtered_altitude_m_ += BARO_ALT_FILTER_ALPHA * (raw_altitude_m - filtered_altitude_m_);
    }

    sample.altitude_m = filtered_altitude_m_;
    out = sample;
    return true;
}

bool Barometer::calibrate() {
    constexpr int kSamples = 100;
    float sum_pressure = 0.0f;
    int valid = 0;
    BaroData sample{};

    while (valid < kSamples) {
        driver_.kick();
        if (!driver_.read(sample)) {
            continue;
        }
        sum_pressure += sample.pres_pa;
        ++valid;
    }

    if (valid <= 0) {
        // Placeholder: optional barometer calibration error handling.
        return false;
    }

    ground_pressure_pa_ = sum_pressure / (float)valid;
    filtered_altitude_m_ = 0.0f;
    filter_initialized_ = false;
    return true;
}
