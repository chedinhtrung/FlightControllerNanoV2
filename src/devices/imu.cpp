#include "devices/imu.h"
#include "debug.h"

Vec3 Imu::remapAndSign(const Vec3 &sensor_frame) const {
    const float arr[3] = {sensor_frame.x, sensor_frame.y, sensor_frame.z};
    Vec3 body = {
        arr[IMU_MAP_X_SRC],
        arr[IMU_MAP_Y_SRC],
        arr[IMU_MAP_Z_SRC]
    };
    const Vec3 orientation_sign = {
        (float)IMU_MAP_X_SIGN,
        (float)IMU_MAP_Y_SIGN,
        (float)IMU_MAP_Z_SIGN
    };
    body *= orientation_sign;
    return body;
}

Imu::Imu(ImuDriver &driver, uint32_t period_us)
    : driver_(driver), period_us_(period_us), last_update_us_(micros()) {}

bool Imu::calibrate() {
    constexpr int kSamples = 1000;
    gyro_bias_radps_ = {};

    Vec3 sum_gyro_dps{};
    int valid = 0;

    for (int i = 0; i < kSamples; ++i) {
        ImuData sample{};
        if (!driver_.read(sample)) {
            continue;
        }

        sum_gyro_dps += remapAndSign(sample.gyro);
        ++valid;
        delay(4);
    }

    if (valid == 0) {
        return false;
    }

    gyro_bias_radps_ = (sum_gyro_dps * (1.0f / (float)valid)) * RAD_PER_DEG;
    return true;
}

bool Imu::setup() {
    if (!driver_.setup()) {
        // Placeholder: optional setup error handling.
        return false;
    }
    return calibrate();
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

    //debug::log(out);

    out.gyro = remapAndSign(out.gyro);
    out.accel = remapAndSign(out.accel);

    // Output gyro in rad/s for the rest of the stack.
    out.gyro *= RAD_PER_DEG;

    debug::log(out.gyro);

    // Body-frame calibration.
    out.gyro -= gyro_bias_radps_;
    out.accel -= accel_bias_g_;
    out.accel *= accel_scale_;

    last_update_us_ = now;
    return true;
}
