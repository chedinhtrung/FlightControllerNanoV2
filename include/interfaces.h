#ifndef INTERFACES_H
#define INTERFACES_H

struct ImuData;

class ImuDriver {
public:
    virtual ~ImuDriver() = default;
    virtual bool setup() = 0;
    virtual bool read(ImuData &data) = 0;
};

#endif
