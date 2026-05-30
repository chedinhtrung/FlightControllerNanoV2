#ifndef MADGWICK
#define MADGWICK
#include <Arduino.h>
#include <datastructs.h>
#include "interfaces.h"
#include "mathutils.h"

#include "BasicLinearAlgebra.h"

class Madgwick {
    private: 
        float beta; 
        uint32_t last_update_us = micros();

    public: 
        Madgwick() = default;
        Madgwick(float beta_, const Vec3 &g_body);
        Quaternion q = {0,0,0,1};

        void update(const ImuData &imu);

};


#endif
