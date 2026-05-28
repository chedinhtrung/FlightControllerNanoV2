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

PPMCommand Receiver::to_anglemode(const PPMCommand& cmd) const {
    // Convert controller input to desired angle in degrees
    PPMCommand out;
    out.C2 = (-cmd.C2 + 1500.0f) / 1000.0f * 70.0f;
    out.C4 = (cmd.C4 - 1500.0f) / 1000.0f * 70.0f;
    out.C1 = (cmd.C1 - 1500.0f) / 1000.0f * 70.0f;

    out.C3 = (cmd.C3 - 1000.0f) / 1000.0f;

    // Throttle in 0 -- 1
    out.C3 = constrain(out.C3, 0.0f, 1.0f);

    // Pitch (degrees)
    out.C2 = constrain(out.C2, -35.0f, 35.0f);

    // Roll (degrees)
    out.C4 = constrain(out.C4, -35.0f, 35.0f);

    // Yaw rate (degrees / s)
    out.C1 = constrain(out.C1, -35.0f, 35.0f);

    return out;
}

PPMCommand Receiver::to_vxy_mode(const PPMCommand& cmd) const {
    // Convert controller input to desired angle/rate
    PPMCommand out;
    out.C2 = (cmd.C2 - 1500.0f) / 1000.0f * 1.8;
    out.C4 = (cmd.C4 - 1500.0f) / 1000.0f * 1.8;
    out.C1 = (cmd.C1 - 1500.0f) / 1000.0f * 70;  // map 1000 - 2000 to range -35 -> 35

    out.C3 = (cmd.C3 - 1000.0f) / 1000.0f;

    // Throttle in 0 -- 1
    out.C3 = constrain(out.C3, 0.0f, 1.0f);

    // vx
    out.C2 = constrain(out.C2, -0.9f, 0.9f);

    // vy
    out.C4 = constrain(out.C4, -0.9f, 0.9f);

    // Yaw rate (degrees / s)
    out.C1 = constrain(out.C1, -35.0f, 35.0f);

    return out;
}

PPMCommand Receiver::to_vxyz_mode(const PPMCommand& cmd) const {
    // Convert controller input to desired angle/rate
    PPMCommand out;
    out.C2 = (cmd.C2 - 1500.0f) / 1000.0f * 1.8;
    out.C4 = (cmd.C4 - 1500.0f) / 1000.0f * 1.8;
    out.C1 = (cmd.C1 - 1500.0f) / 1000.0f * 70;  // map 1000 - 2000 to range -35 -> 35

    out.C3 = -(cmd.C3 - 1500.0f) / 1000.0f * 1e-3; // map 1000 - 2000 to range -0.5m/s -> 0.5m/s. Negative sign due to up = negative z

    // vz in -0.5m/s - 0.5m/s
    out.C3 = constrain(out.C3, -0.5f, 0.5f);

    // vx
    out.C2 = constrain(out.C2, -0.9f, 0.9f);

    // vy
    out.C4 = constrain(out.C4, -0.9f, 0.9f);

    // Yaw rate (degrees / s)
    out.C1 = constrain(out.C1, -35.0f, 35.0f);

    return out;
}

CMDType Receiver::type() const {
    return cmd_type_;
}
