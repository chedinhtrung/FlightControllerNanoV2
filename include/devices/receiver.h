#ifndef DEVICE_RECEIVER_H
#define DEVICE_RECEIVER_H

#include "drivers/receiver.h"
#include "interfaces.h"

enum class CMDType : uint8_t {
    PPM = 0,
    RPI = 1
};

class Receiver {
public:
    explicit Receiver(ReceiverDriver &driver);

    bool read(PPMCommand &cmd);
    bool read(RPICommand &cmd);
    PPMCommand to_anglemode(const PPMCommand& cmd) const;
    PPMCommand to_vxy_mode(const PPMCommand& cmd) const;
    PPMCommand to_vxyz_mode(const PPMCommand& cmd) const;
    CMDType type() const;

private:
    ReceiverDriver &ppm_driver_;
    CMDType cmd_type_ = CMDType::PPM;
};

#endif
