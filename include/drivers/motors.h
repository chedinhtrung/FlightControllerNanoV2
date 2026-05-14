#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

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

class Motor {
public:
    RawMotor motor_report;

    Motor();

    void set_motor_raw(int fl, int fr, int bl, int br);
    void set_motor(float fl, float fr, float bl, float br);
};

#endif
