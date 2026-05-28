#include "devices/barometer.h"
#include "debug.h"

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

void Barometer::kick() {
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
    sample.timestamp = micros();

    const float raw_altitude_m = 44330.0f * (1.0f - powf(sample.pres_pa / sea_level_pressure_pa_, 0.19029495f));
    const float filtered_altitude_m = altitude_lpf_.update(raw_altitude_m);

    sample.altitude_m = filtered_altitude_m - altitude_zero_m_;
    out = sample;
    return true;
}

void Barometer::reset() {
    if (!altitude_lpf_.initialized()) {
        altitude_zero_m_ = 0.0f;
        return;
    }
    altitude_zero_m_ = altitude_lpf_.value();
}

bool Barometer::calibrate() {
    constexpr int kSamples = 100;
    float sum_pressure = 0.0f;
    int valid = 0;
    BaroData sample{};

    while (valid < kSamples) {
        driver_.kick();

        delay(20);
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

    const float avg_pressure_pa = sum_pressure / (float)valid;
    const float start_altitude_m =
        44330.0f * (1.0f - powf(avg_pressure_pa / sea_level_pressure_pa_, 0.19029495f));

    altitude_lpf_.reset(start_altitude_m);
    altitude_zero_m_ = start_altitude_m;
    return true;
}
