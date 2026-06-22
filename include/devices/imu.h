#ifndef DEVICE_IMU_H
#define DEVICE_IMU_H

#include <Arduino.h>

#include "config.h"
#include "interfaces.h"

class Imu {
public:
    explicit Imu(ImuDriver &driver, uint32_t period_us = PERIOD_US);

    bool setup();
    bool read(ImuData &out);

private:
    bool calibrate();
    Vec3 remapAndSign(const Vec3 &sensor_frame) const;

    static constexpr int IMU_MAP_X_SRC = 0;
    static constexpr int IMU_MAP_X_SIGN = -1;
    static constexpr int IMU_MAP_Y_SRC = 1;
    static constexpr int IMU_MAP_Y_SIGN = -1;
    static constexpr int IMU_MAP_Z_SRC = 2;
    static constexpr int IMU_MAP_Z_SIGN = 1;

    Vec3 accel_bias_g_ = {0.0f, 0.0f, 0.0f};
    Vec3 accel_scale_ = {1.0f, 1.0f, 1.0f};
    Vec3 gyro_bias_radps_ = {0.0f, 0.0f, 0.0f};

    ImuDriver &driver_;
    uint32_t period_us_ = PERIOD_US;
    uint32_t last_update_us_ = 0;
};

#endif
