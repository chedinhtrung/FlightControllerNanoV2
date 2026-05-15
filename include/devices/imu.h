#ifndef DEVICE_IMU_H
#define DEVICE_IMU_H

#include <Arduino.h>

#include "config.h"
#include "interfaces.h"
#include "drivers/mpu9250.h"

class Imu {
public:
    explicit Imu(ImuDriver &driver, uint32_t period_us = PERIOD_US);

    bool setup();
    bool read(ImuData &out);

private:
    ImuDriver &driver_;
    uint32_t period_us_ = PERIOD_US;
    uint32_t last_update_us_ = 0;
};

#endif
