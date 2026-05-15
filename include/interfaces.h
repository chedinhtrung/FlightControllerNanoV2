#ifndef INTERFACES_H
#define INTERFACES_H

struct ImuData;
struct MotorCommand;
struct PPMCommand;
struct RPICommand;
struct BaroData;

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

class ReceiverDriver {
public:
    virtual ~ReceiverDriver() = default;
    virtual bool read(PPMCommand &cmd) = 0;
    virtual bool read(RPICommand &cmd) = 0;
};

class BarometerDriver {
public:
    virtual ~BarometerDriver() = default;
    virtual bool setup() = 0;
    virtual void kick() = 0;
    virtual bool read(BaroData &data) = 0;
};

#endif
