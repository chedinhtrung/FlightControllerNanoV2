#ifndef DATASTRUCT
#define DATASTRUCT

#include <Arduino.h>

struct Quaternion;

struct EulerAngle
{
    float yaw = 0.0;
    float pitch = 0.0;
    float roll = 0.0;
};

struct Vec3
{
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;

    inline Vec3() = default;
    inline Vec3(const Quaternion &q);
    constexpr inline Vec3(float x_, float y_, float z_) : x(x_),y(y_), z(z_) {}; 

    inline Vec3 operator+(const Vec3 &other) const
    {
        return {x + other.x, y + other.y, z + other.z};
    }

    inline Vec3 operator-(const Vec3 &other) const
    {
        return {x - other.x, y - other.y, z - other.z};
    }

    inline Vec3 operator*(float scalar) const
    {
        return {x * scalar, y * scalar, z * scalar};
    }

    inline Vec3 operator*(const Vec3 &other) const
    {
        return {x * other.x, y * other.y, z * other.z};
    }

    inline Vec3 &operator+=(const Vec3 &other)
    {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    inline Vec3 &operator-=(const Vec3 &other)
    {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    inline Vec3 &operator*=(float scalar)
    {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    inline Vec3 &operator*=(const Vec3 &other)
    {
        x *= other.x;
        y *= other.y;
        z *= other.z;
        return *this;
    }
};

struct Vec3WithTrust
{
    Vec3 value{};
    Vec3 trust{};
};

struct FloatWithTrust
{
    float value = 0.0f;
    float trust = 0.0f;
};

struct Quaternion
{
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    float w = 0.0;

    inline Quaternion() = default;

    inline Quaternion(const Vec3 &v) : x(v.x), y(v.y), z(v.z), w(0.0f) {}

    constexpr Quaternion(float x_, float y_, float z_, float w_)
        : x(x_), y(y_), z(z_), w(w_) {}

    inline Quaternion operator+(const Quaternion &other) const
    {
        return Quaternion{x + other.x, y + other.y, z + other.z, w + other.w};
    }

    inline Quaternion operator-(const Quaternion &other) const
    {
        return Quaternion{x - other.x, y - other.y, z - other.z, w - other.w};
    }

    inline Quaternion operator*(float scalar) const
    {
        return Quaternion{x * scalar, y * scalar, z * scalar, w * scalar};
    }

    // Quaternion mult
    inline Quaternion operator*(const Quaternion &other) const
    {
        return Quaternion{
            (w * other.x) + (x * other.w) + (y * other.z) - (z * other.y),
            (w * other.y) - (x * other.z) + (y * other.w) + (z * other.x),
            (w * other.z) + (x * other.y) - (y * other.x) + (z * other.w),
            (w * other.w) - (x * other.x) - (y * other.y) - (z * other.z)};
    }

    inline Quaternion &operator+=(const Quaternion &other)
    {
        x += other.x;
        y += other.y;
        z += other.z;
        w += other.w;
        return *this;
    }

    inline Quaternion &operator-=(const Quaternion &other)
    {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        w -= other.w;
        return *this;
    }

    inline Quaternion &operator*=(float scalar)
    {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        w *= scalar;
        return *this;
    }

    inline Quaternion &operator*=(const Quaternion &other)
    {
        *this = *this * other;
        return *this;
    }

    // Complementary quaternion (conjugate)
    inline Quaternion T() const
    {
        return Quaternion{-x, -y, -z, w};
    }
};

inline Vec3::Vec3(const Quaternion &q) : x(q.x), y(q.y), z(q.z) {}

// Things to report to the Flight Computer
struct Report
{
};

struct VectInt16
{
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
};

inline Vec3 normalize(Vec3 v)
{
    const float mag2 = (v.x * v.x) + (v.y * v.y) + (v.z * v.z);
    if (mag2 <= 0.0f)
    {
        return {};
    }

    const float inv_mag = 1.0f / sqrtf(mag2);
    return v * inv_mag;
}

inline Quaternion normalize(Quaternion q)
{
    const float mag2 = (q.x * q.x) + (q.y * q.y) + (q.z * q.z) + (q.w * q.w);
    if (mag2 <= 0.0f)
    {
        return {};
    }

    const float inv_mag = 1.0f / sqrtf(mag2);
    return q * inv_mag;
}

inline float dot(const Vec3 &a, const Vec3 &b)
{
    return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

inline Vec3 cross(const Vec3 &a, const Vec3 &b)
{
    return {
        (a.y * b.z) - (a.z * b.y),
        (a.z * b.x) - (a.x * b.z),
        (a.x * b.y) - (a.y * b.x)};
}

#endif
