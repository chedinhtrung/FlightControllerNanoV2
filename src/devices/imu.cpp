#include "devices/imu.h"

Imu::Imu(ImuDriver &driver, uint32_t period_us)
    : driver_(driver), period_us_(period_us), last_update_us_(micros()) {}

bool Imu::setup() {
    if (!driver_.setup()) {
        // Placeholder: optional setup error handling.
        return false;
    }
    return true;
}

bool Imu::read(ImuData &out) {
    const uint32_t now = micros();
    if ((uint32_t)(now - last_update_us_) < period_us_) {
        return false;
    }

    if (!driver_.read(out)) {
        // Placeholder: optional read error handling.
        return false;
    }

    last_update_us_ = now;
    return true;
}
