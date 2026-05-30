#ifndef IMU
#define IMU

#include "config.h"
#include "datastructs.h"
#include "interfaces.h"
#include <Arduino.h>

#define IMUADDR 0x68 // MPU9250 I2C address

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

class MPU9250 : public ImuDriver
{
    RawImuData raw;
public:
    bool setup() override;
    bool read(ImuData &data) override;
};

#endif
