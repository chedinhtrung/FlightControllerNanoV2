#ifndef DEVICE_SERVO_H
#define DEVICE_SERVO_H

#include "drivers/servo.h"
#include "interfaces.h"

class ServoDevice {
public:
    explicit ServoDevice(ServoDriver &driver);
    bool setup();
    void write(float angle_deg);

private:
    ServoDriver &driver_;
};

#endif
