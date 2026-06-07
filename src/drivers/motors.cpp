#include "drivers/motors.h"

namespace {
inline float clamp01(float v) {
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

inline int motor_float_to_raw(float m) {
    // 12-bit write: 1024..2047 maps to approx 1000us..2000us style ESC range.
    return (int)(m * 1023.0f + 1024.0f);
}
}

Motor::Motor() {
    pinMode(FL, OUTPUT);
    pinMode(FR, OUTPUT);
    pinMode(BL, OUTPUT);
    pinMode(BR, OUTPUT);
    analogWriteResolution(12);
    analogWriteFrequency(2000);
}

void Motor::set_motor_raw(int fl, int fr, int bl, int br) {
    analogWrite(FL, fl);
    analogWrite(FR, fr);
    analogWrite(BL, bl);
    analogWrite(BR, br);
}

void Motor::set_motor(const MotorCommand &cmd) {
    /*
        FrontLeft  ---- ^^^^^^ ---- FrontRight

        BackLeft   ---- ^^^^^^ ---- BackRight
    */

    const float fl = clamp01(cmd.fl);
    const float fr = clamp01(cmd.fr);
    const float bl = clamp01(cmd.bl);
    const float br = clamp01(cmd.br);

    const int mfl = motor_float_to_raw(fl);
    const int mfr = motor_float_to_raw(fr);
    const int mbl = motor_float_to_raw(bl);
    const int mbr = motor_float_to_raw(br);

    motor_report.fl = (uint16_t)mfl;
    motor_report.fr = (uint16_t)mfr;
    motor_report.bl = (uint16_t)mbl;
    motor_report.br = (uint16_t)mbr;

    set_motor_raw(mfl, mfr, mbl, mbr);
}
