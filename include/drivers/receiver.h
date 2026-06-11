#ifndef RECEIVER_H
#define RECEIVER_H

#include <Arduino.h>
#include "interfaces.h"

#define PPM_PIN 6

struct PPMCommand
{
    float C1 = 1500.0f;
    float C2 = 1500.0f;
    float C3 = 1000.0f;
    float C4 = 1500.0f;
    float C5 = 1000.0f;
    float C6 = 1000.0f;
    float C7 = 1000.0f;
    float C8 = 1000.0f;
    float C9 = 1000.0f;
    float C10 = 1000.0f;
    uint32_t timestamp;
};

struct IBusFrame
{
    uint8_t len;
    uint8_t command;
    uint16_t channels[14];
    uint16_t checksum;
};

struct RPICommand
{
};

class PPMReceiver : public ReceiverDriver
{
public:
    PPMReceiver();

    bool read(PPMCommand &cmd) override;
    bool read(RPICommand &cmd) override;

    

private:
    HardwareSerial _serial = HardwareSerial(UART5);
    bool parseIBus(uint8_t b, IBusFrame& out);

private:
    uint8_t buffer[32];
    uint8_t index = 0;
    bool checksum();
};

class RPIReceiver : public ReceiverDriver
{
public:
    bool read(PPMCommand &cmd) override;
    bool read(RPICommand &cmd) override;
};

#endif
