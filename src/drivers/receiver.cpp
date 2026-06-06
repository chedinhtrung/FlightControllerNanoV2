#include "drivers/receiver.h"

PPMReceiver::PPMReceiver() {
    // TODO: initialize i Bus with Serial (Dake H743 does not support PPM)
}

bool PPMReceiver::read(PPMCommand &cmd) {
    cmd.timestamp = micros();
    // TODO: read channels from i Bus


    // Prevent zeroes at startup
    if (cmd.C3 < 1000.0f) {
        cmd.C3 = 1000.0f;
    } else if (cmd.C3 > 2000.0f) {
        cmd.C3 = 2000.0f;
    }

    if (cmd.C4 < 900.0f) {
        cmd.C4 = 1500.0f;
    } else if (cmd.C4 > 2000.0f) {
        cmd.C4 = 2000.0f;
    }

    if (cmd.C2 < 900.0f) {
        cmd.C2 = 1500.0f;
    } else if (cmd.C2 > 2000.0f) {
        cmd.C2 = 2000.0f;
    }

    if (cmd.C1 < 900.0f) {
        cmd.C1 = 1500.0f;
    } else if (cmd.C1 > 2000.0f) {
        cmd.C1 = 2000.0f;
    }

    if (cmd.C5 < 900.0f) {
        cmd.C5 = 1000.0f;
    } else if (cmd.C5 > 2000.0f) {
        cmd.C5 = 2000.0f;
    }

    return true;
}

bool PPMReceiver::read(RPICommand &cmd) {
    (void)cmd;
    // Placeholder: PPM driver does not produce RPI command packets.
    return false;
}

bool RPIReceiver::read(PPMCommand &cmd) {
    (void)cmd;
    // Placeholder: RPI driver does not produce PPM command packets.
    return false;
}

bool RPIReceiver::read(RPICommand &cmd) {
    (void)cmd;
    // Placeholder: RPI transport decode to be implemented.
    return false;
}
