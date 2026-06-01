#ifndef SERVO_H
#define SERVO_H

#include <Arduino.h>
#include <Servo.h>
#include "interfaces.h"

#define SERVO_PIN 19

class ServoOutput : public ServoDriver {
public:
    ServoOutput();
    bool setup() override;
    void write_angle(int angle_deg) override;

private:
    Servo servo_;
    bool attached_ = false;
};

#endif
