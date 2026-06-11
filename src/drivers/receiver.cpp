#include "drivers/receiver.h"
#include "debug.h"

PPMReceiver::PPMReceiver()
{
    _serial.setRx(PB5);
    _serial.setTx(PB6_ALT1);
    _serial.begin(115200, SERIAL_8N1);
}

bool PPMReceiver::parseIBus(uint8_t b, IBusFrame &out)
{
    if (index == 0 && b != 0x20)
        return false;

    buffer[index++] = b;

    if (index == 2 && buffer[1] != 0x40)
    {
        index = 0;
        return false;
    }

    if (index < 32)
        return false;

    index = 0;

    if (!checksum())
        return false;

    out.len = buffer[0];
    out.command = buffer[1];

    for (int i = 0; i < 14; i++)
    {
        out.channels[i] =
            (uint16_t)buffer[2 + i * 2] |
            ((uint16_t)buffer[3 + i * 2] << 8);
    }

    out.checksum =
        (uint16_t)buffer[30] |
        ((uint16_t)buffer[31] << 8);

    return true;
}

bool PPMReceiver::checksum()
{
    uint16_t sum = 0xFFFF;

    for (int i = 0; i < 30; i++)
        sum -= buffer[i];

    uint16_t received =
        (uint16_t)buffer[30] |
        ((uint16_t)buffer[31] << 8);

    return sum == received;
}

bool PPMReceiver::read(PPMCommand &cmd)
{
    IBusFrame frame;
    cmd.timestamp = micros();
    // read channels from i Bus
    bool read_ok = false;
    while (_serial.available())
    {
        read_ok = parseIBus(_serial.read(), frame);
    }
    if (!read_ok)
    {
        return false;
    }

    cmd.C1 = static_cast<float>(frame.channels[0]);
    cmd.C2 = static_cast<float>(frame.channels[1]);
    cmd.C3 = static_cast<float>(frame.channels[2]);
    cmd.C4 = static_cast<float>(frame.channels[3]);
    cmd.C5 = static_cast<float>(frame.channels[4]);
    cmd.C6 = static_cast<float>(frame.channels[5]);
    cmd.C7 = static_cast<float>(frame.channels[6]);
    cmd.C8 = static_cast<float>(frame.channels[7]);
    cmd.C9 = static_cast<float>(frame.channels[8]);
    cmd.C10 = static_cast<float>(frame.channels[9]);

    // Prevent zeroes at startup
    if (cmd.C3 < 1000.0f)
    {
        cmd.C3 = 1000.0f;
    }
    else if (cmd.C3 > 2000.0f)
    {
        cmd.C3 = 2000.0f;
    }

    if (cmd.C4 < 900.0f)
    {
        cmd.C4 = 1500.0f;
    }
    else if (cmd.C4 > 2000.0f)
    {
        cmd.C4 = 2000.0f;
    }

    if (cmd.C2 < 900.0f)
    {
        cmd.C2 = 1500.0f;
    }
    else if (cmd.C2 > 2000.0f)
    {
        cmd.C2 = 2000.0f;
    }

    if (cmd.C1 < 900.0f)
    {
        cmd.C1 = 1500.0f;
    }
    else if (cmd.C1 > 2000.0f)
    {
        cmd.C1 = 2000.0f;
    }

    if (cmd.C5 < 900.0f)
    {
        cmd.C5 = 1000.0f;
    }
    else if (cmd.C5 > 2000.0f)
    {
        cmd.C5 = 2000.0f;
    }

    return read_ok;
}

bool PPMReceiver::read(RPICommand &cmd)
{
    (void)cmd;
    // Placeholder: PPM driver does not produce RPI command packets.
    return false;
}

bool RPIReceiver::read(PPMCommand &cmd)
{
    (void)cmd;
    // Placeholder: RPI driver does not produce PPM command packets.
    return false;
}

bool RPIReceiver::read(RPICommand &cmd)
{
    (void)cmd;
    // Placeholder: RPI transport decode to be implemented.
    return false;
}
