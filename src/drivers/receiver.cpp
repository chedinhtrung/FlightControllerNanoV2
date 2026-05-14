#include "drivers/receiver.h"

Receiver::Receiver() {
    in.begin(PPM_PIN);
}

ReceiverData Receiver::recv() {
    ReceiverData data;
    data.ThrottleIn = in.read(3);    // Channel 3 = throttle
    data.RollIn = in.read(4);        // Channel 4 = roll
    data.YawIn = in.read(1);         // Channel 1 = yaw
    data.PitchIn = in.read(2);       // Channel 2 = pitch
    data.AuxChannel5In = in.read(5); // Aux Channel 5
    data.AuxChannel6In = in.read(6); // Aux Channel 6

    // Prevent zeroes at startup
    if (data.ThrottleIn < 1000.0f) {
        data.ThrottleIn = 1000.0f;
    } else if (data.ThrottleIn > 2000.0f) {
        data.ThrottleIn = 2000.0f;
    }

    if (data.RollIn < 900.0f) {
        data.RollIn = 1500.0f;
    } else if (data.RollIn > 2000.0f) {
        data.RollIn = 2000.0f;
    }

    if (data.PitchIn < 900.0f) {
        data.PitchIn = 1500.0f;
    } else if (data.PitchIn > 2000.0f) {
        data.PitchIn = 2000.0f;
    }

    if (data.YawIn < 900.0f) {
        data.YawIn = 1500.0f;
    } else if (data.YawIn > 2000.0f) {
        data.YawIn = 2000.0f;
    }

    return data;
}

ReceiverData Receiver::to_anglemode(ReceiverData data) {
    // Convert controller input to desired angle/rate
    data.PitchIn = (-data.PitchIn + 1500.0f) / 1000.0f * 90.0f;
    data.RollIn = (data.RollIn - 1500.0f) / 1000.0f * 90.0f;
    data.YawIn = (data.YawIn - 1500.0f) / 1000.0f * 80.0f - 0.06f;

    data.ThrottleIn = (data.ThrottleIn - 1000.0f) / 1000.0f;

    if (data.ThrottleIn > 1.0f) {
        data.ThrottleIn = 1.0f;
    } else if (data.ThrottleIn < 0.0f) {
        data.ThrottleIn = 0.0f;
    }

    if (data.PitchIn > 35.0f) {
        data.PitchIn = 35.0f;
    } else if (data.PitchIn < -35.0f) {
        data.PitchIn = -35.0f;
    }

    if (data.RollIn > 35.0f) {
        data.RollIn = 35.0f;
    } else if (data.RollIn < -35.0f) {
        data.RollIn = -35.0f;
    }

    if (data.YawIn > 30.0f) {
        data.YawIn = 30.0f;
    } else if (data.YawIn < -30.0f) {
        data.YawIn = -30.0f;
    }

    return data;
}
