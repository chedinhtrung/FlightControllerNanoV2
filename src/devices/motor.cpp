#include "devices/motor.h"

MotorDevice::MotorDevice(MotorDriver &driver) : driver_(driver) {}

void MotorDevice::command_motor(float throttle, float pitch_adjust, float roll_adjust, float yaw_adjust) {
    MotorCommand cmd;
    cmd.fl = throttle + pitch_adjust + roll_adjust - yaw_adjust;
    cmd.fr = throttle + pitch_adjust - roll_adjust + yaw_adjust;
    cmd.bl = throttle - pitch_adjust + roll_adjust + yaw_adjust;
    cmd.br = throttle - pitch_adjust - roll_adjust - yaw_adjust;
    driver_.set_motor(cmd);
}
