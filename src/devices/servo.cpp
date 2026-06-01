#include "devices/servo.h"

ServoDevice::ServoDevice(ServoDriver &driver) : driver_(driver) {}

bool ServoDevice::setup() {
    return driver_.setup();
}

void ServoDevice::write(float angle_deg) {
    driver_.write_angle((int)angle_deg);
}
