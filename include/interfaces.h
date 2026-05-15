#ifndef INTERFACES_H
#define INTERFACES_H

struct ImuData;
struct MotorCommand;

class ImuDriver {
public:
    virtual ~ImuDriver() = default;
    virtual bool setup() = 0;
    virtual bool read(ImuData &data) = 0;
};

class MotorDriver {
public:
    virtual ~MotorDriver() = default;
    virtual void set_motor(const MotorCommand &cmd) = 0;
};

#endif
