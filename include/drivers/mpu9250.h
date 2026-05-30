#ifndef IMU
#define IMU

#include "config.h"
#include "datastructs.h"
#include "interfaces.h"
#include <Arduino.h>

#define IMUADDR 0x68 // MPU9250 I2C address

class MPU9250 : public ImuDriver
{
    RawImuData raw;
public:
    bool setup() override;
    bool read(ImuData &data) override;
};

#endif
