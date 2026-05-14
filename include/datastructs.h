#ifndef DATASTRUCT
#define DATASTRUCT

#include <Arduino.h>

struct EulerAngle {
    float yaw = 0.0;
    float pitch = 0.0;
    float roll = 0.0;
};

struct Vect {
    float z = 0.0;
    float x = 0.0;
    float y = 0.0;
};

struct Quaternion {
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    float w = 0.0;
};

struct Report {
  
};


struct VectInt16 {
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
};

#endif