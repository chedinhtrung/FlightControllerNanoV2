#ifndef IMU 
#define IMU

#include "config.h"
#include "datastructs.h"
#include <Arduino.h>

#define IMUADDR 0x68 // MPU9250 I2C address

// Orientation mapping (sensor -> body). Source index: 0=x, 1=y, 2=z.
// Signs are 1 or -1.

#define IMU_MAP_X_SRC 1
#define IMU_MAP_X_SIGN -1
#define IMU_MAP_Y_SRC 0
#define IMU_MAP_Y_SIGN 1
#define IMU_MAP_Z_SRC 2
#define IMU_MAP_Z_SIGN 1

struct RawImuData {
    VectInt16 accel;
    VectInt16 gyro;
};

struct ImuData {
    Vec3 gyro; 
    Vec3 accel;
};

class MPU9250 {
    public: 
        // Raw values for data gathering
        RawImuData raw;
        
        Vec3 accel_bias_g {0, 0, 0};
        Vec3 gyro_bias;

        uint32_t last_update_us = micros();
        
        void setup();
        void calibrate();
        bool read(ImuData &data);

        void update(ImuData &data);
        
};

#endif 
