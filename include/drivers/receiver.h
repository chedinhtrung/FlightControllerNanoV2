#ifndef RECEIVER_H
#define RECEIVER_H

#include <Arduino.h>
#include <PulsePosition.h>
#include "interfaces.h"

#define PPM_PIN 6

struct PPMCommand {
    float C1 = 1500.0f;
    float C2 = 1500.0f;
    float C3 = 1000.0f;
    float C4 = 1500.0f;
    float C5 = 1000.0f;
    float C6 = 1000.0f;
};

struct RPICommand {
};

class PPMReceiver : public ReceiverDriver {
public:
    PPMReceiver();

    bool read(PPMCommand &cmd) override;
    bool read(RPICommand &cmd) override;

private:
    PulsePositionInput in;
};

class RPIReceiver : public ReceiverDriver {
public:
    bool read(PPMCommand &cmd) override;
    bool read(RPICommand &cmd) override;
};

#endif
