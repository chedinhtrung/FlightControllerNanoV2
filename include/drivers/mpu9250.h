#ifndef IMU 
#define IMU

#include "config.h"
#include "datastructs.h"
#include <Arduino.h>

#define IMUADDR 0x68 // MPU9250 I2C address

struct RawImuData {
    VectInt16 accel;
    VectInt16 gyro;
};

struct ImuData {
    Vect gyro; 
    Vect accel;
};

class MPU9250 {
    public: 
        // Raw values for data gathering
        RawImuData raw;
        
        Vect accel_bias_g;
        Vect gyro_bias;
        
        void setup();
        void calibrate();
        bool read(ImuData &data);
        
};

#endif 
