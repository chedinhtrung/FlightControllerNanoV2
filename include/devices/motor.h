#ifndef DEVICE_MOTOR_H
#define DEVICE_MOTOR_H

#include "drivers/motors.h"
#include "interfaces.h"

class MotorDevice {
public:
    explicit MotorDevice(MotorDriver &driver);

    void write(float throttle, float yaw_adjust, float pitch_adjust, float roll_adjust);

private:
    MotorDriver &driver_;
};

#endif
