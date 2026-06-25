#ifndef INTERFACES_H
#define INTERFACES_H

#include <Arduino.h>
#include "datastructs.h"

struct RawImuData
{
    VectInt16 accel;
    VectInt16 gyro;
};

struct ImuData
{
    Vec3 gyro;
    Vec3 accel;
    uint32_t timestamp;
};

struct BaroData {
    float temp_c = 0.0f;
    float pres_pa = 0.0f;
    float altitude_m = 0.0f;
    uint32_t timestamp;
};

struct MotorCommand;
struct PPMCommand;
struct RPICommand;
struct MTF02Data;

class ImuDriver {
public:
    virtual ~ImuDriver() = default;
    virtual bool setup() = 0;
    virtual bool read(ImuData &data) = 0;
    virtual bool write() { return false; }
};

class MotorDriver {
public:
    virtual ~MotorDriver() = default;
    virtual void set_motor(const MotorCommand &cmd) = 0;
    virtual bool write() { return false; }
};

class ServoDriver {
public:
    virtual ~ServoDriver() = default;
    virtual bool setup() = 0;
    virtual void write_angle(int angle_deg) = 0;
    virtual bool write() { return false; }
};

class ReceiverDriver {
public:
    virtual ~ReceiverDriver() = default;
    virtual bool read(PPMCommand &cmd) = 0;
    virtual bool read(RPICommand &cmd) = 0;
    virtual bool write() { return false; }
};

class BarometerDriver {
public:
    virtual ~BarometerDriver() = default;
    virtual bool setup() = 0;
    virtual void kick() = 0;
    virtual bool read(BaroData &data) = 0;
    virtual bool write() { return false; }
};

class OpticalFlowDriver {
public:
    virtual ~OpticalFlowDriver() = default;
    virtual bool setup() = 0;
    virtual bool parse() = 0;
    virtual bool has_bytes() = 0;
    virtual bool read(MTF02Data &data) = 0;
    virtual bool write() { return false; }
};

#endif
