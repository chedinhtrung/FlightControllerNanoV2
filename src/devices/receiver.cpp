#include "devices/receiver.h"

Receiver::Receiver(ReceiverDriver &ppm_driver) : ppm_driver_(ppm_driver) {}

bool Receiver::read(PPMCommand &cmd) {
    cmd_type_ = CMDType::PPM;
    return ppm_driver_.read(cmd);
}

bool Receiver::read(RPICommand &cmd) {
    cmd_type_ = CMDType::RPI;
    // Placeholder: RPI driver routing will be added when the driver is wired in.
    return ppm_driver_.read(cmd);
}

PPMCommand Receiver::to_anglemode(PPMCommand cmd) const {
    // Convert controller input to desired angle/rate
    cmd.C2 = (-cmd.C2 + 1500.0f) / 1000.0f * 90.0f;
    cmd.C4 = (cmd.C4 - 1500.0f) / 1000.0f * 90.0f;
    cmd.C1 = (cmd.C1 - 1500.0f) / 1000.0f * 80.0f - 0.06f;

    cmd.C3 = (cmd.C3 - 1000.0f) / 1000.0f;

    if (cmd.C3 > 1.0f) cmd.C3 = 1.0f;
    else if (cmd.C3 < 0.0f) cmd.C3 = 0.0f;

    if (cmd.C2 > 35.0f) cmd.C2 = 35.0f;
    else if (cmd.C2 < -35.0f) cmd.C2 = -35.0f;

    if (cmd.C4 > 35.0f) cmd.C4 = 35.0f;
    else if (cmd.C4 < -35.0f) cmd.C4 = -35.0f;

    if (cmd.C1 > 30.0f) cmd.C1 = 30.0f;
    else if (cmd.C1 < -30.0f) cmd.C1 = -30.0f;

    return cmd;
}

CMDType Receiver::type() const {
    return cmd_type_;
}
