#include "drivers/servo.h"

ServoOutput::ServoOutput() {}

bool ServoOutput::setup() {
    servo_.attach(SERVO_PIN);
    attached_ = servo_.attached();
    return attached_;
}

void ServoOutput::write_angle(int angle_deg) {
    if (!attached_) {
        return;
    }
    const int clamped = constrain(angle_deg, 0, 180);
    servo_.write(clamped);
}
