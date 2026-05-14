#ifndef DATASTRUCT
#define DATASTRUCT

#include <Arduino.h>

struct EulerAngle {
    float yaw = 0.0;
    float pitch = 0.0;
    float roll = 0.0;
};

struct Vect {
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;

    inline Vect operator+(const Vect &rhs) const {
        return {x + rhs.x, y + rhs.y, z + rhs.z};
    }

    inline Vect operator-(const Vect &rhs) const {
        return {x - rhs.x, y - rhs.y, z - rhs.z};
    }

    inline Vect operator*(float scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }

    inline Vect operator*(const Vect &rhs) const {
        return {x * rhs.x, y * rhs.y, z * rhs.z};
    }

    inline Vect& operator+=(const Vect &rhs) {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    inline Vect& operator-=(const Vect &rhs) {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    inline Vect& operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    inline Vect& operator*=(const Vect &rhs) {
        x *= rhs.x;
        y *= rhs.y;
        z *= rhs.z;
        return *this;
    }
};

struct Quaternion {
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    float w = 0.0;

    inline Quaternion operator+(const Quaternion &rhs) const {
        return {x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w};
    }

    inline Quaternion operator-(const Quaternion &rhs) const {
        return {x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w};
    }

    inline Quaternion operator*(float scalar) const {
        return {x * scalar, y * scalar, z * scalar, w * scalar};
    }

    // Hamilton product: this * rhs
    inline Quaternion operator*(const Quaternion &rhs) const {
        return {
            (w * rhs.x) + (x * rhs.w) + (y * rhs.z) - (z * rhs.y),
            (w * rhs.y) - (x * rhs.z) + (y * rhs.w) + (z * rhs.x),
            (w * rhs.z) + (x * rhs.y) - (y * rhs.x) + (z * rhs.w),
            (w * rhs.w) - (x * rhs.x) - (y * rhs.y) - (z * rhs.z)
        };
    }

    inline Quaternion& operator+=(const Quaternion &rhs) {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        w += rhs.w;
        return *this;
    }

    inline Quaternion& operator-=(const Quaternion &rhs) {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        w -= rhs.w;
        return *this;
    }

    inline Quaternion& operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        w *= scalar;
        return *this;
    }

    inline Quaternion& operator*=(const Quaternion &rhs) {
        *this = *this * rhs;
        return *this;
    }
};

struct Report {
  
};


struct VectInt16 {
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
};

#endif
