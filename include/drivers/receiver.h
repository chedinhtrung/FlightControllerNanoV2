#ifndef RECEIVER_H
#define RECEIVER_H

#include <Arduino.h>
#include <PulsePosition.h>

#define PPM_PIN 6

struct ReceiverData {
    float ThrottleIn = 1000.0f;
    float RollIn = 1500.0f;
    float YawIn = 1500.0f;
    float PitchIn = 1500.0f;
    float AuxChannel5In = 1000.0f;
    float AuxChannel6In = 1000.0f;
};

class Receiver {
public:
    Receiver();

    ReceiverData recv();
    ReceiverData to_anglemode(ReceiverData data);

private:
    PulsePositionInput in;
};

#endif
