#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include "interfaces.h"

#define FR A8
#define FL 2
#define BR 11
#define BL 5

struct RawMotor {
    uint16_t fl = 0;
    uint16_t fr = 0;
    uint16_t bl = 0;
    uint16_t br = 0;
};

struct MotorCommand {
    float fl = 0.0f;
    float fr = 0.0f;
    float bl = 0.0f;
    float br = 0.0f;
};

class Motor : public MotorDriver {
public:
    RawMotor motor_report;

    Motor();

    void set_motor_raw(int fl, int fr, int bl, int br);
    void set_motor(const MotorCommand &cmd) override;
};

#endif
