#ifndef MATHUTILS
#define MATHUTILS
#include "BasicLinearAlgebra.h"
#include "datastructs.h"
#include <math.h>
#include "config.h"

inline EulerAngle to_deg(const EulerAngle &e)
{
    return {
        e.yaw * DEG_PER_RAD,
        e.pitch * DEG_PER_RAD,
        e.roll * DEG_PER_RAD};
}

inline EulerAngle to_rad(const EulerAngle &e)
{
    return {
        e.yaw * RAD_PER_DEG,
        e.pitch * RAD_PER_DEG,
        e.roll * RAD_PER_DEG};
}

// Compute rotation quaternion from two direction vectors
inline Quaternion quatFromTwoUnitVectors(Vec3 from, Vec3 to)
{
    float c = dot(from, to);
    Vec3 v = cross(from, to);

    if (c < -0.9999f)
    {
        // 180 degrees: choose any axis perpendicular to from
        Vec3 axis = normalize(abs(from.x) < 0.9f
                                  ? cross(from, Vec3{1, 0, 0})
                                  : cross(from, Vec3{0, 1, 0}));

        return Quaternion{0.0f, axis.x, axis.y, axis.z}; // w,x,y,z
    }

    Quaternion q{
        1.0f + c,
        v.x,
        v.y,
        v.z};

    return normalize(q);
}

// Convert quaternion to yaw - pitch - roll euler angles in radian
inline EulerAngle quaternionToEuler(const Quaternion &q)
{
    EulerAngle e;

    // Roll: rotation around X axis
    const float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    const float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    e.roll = atan2f(sinr_cosp, cosr_cosp);

    // Pitch: rotation around Y axis
    const float sinp = 2.0f * (q.w * q.y - q.z * q.x);

    if (fabsf(sinp) >= 1.0f)
    {
        e.pitch = copysignf((float)M_PI / 2.0f, sinp);
    }
    else
    {
        e.pitch = asinf(sinp);
    }

    // Yaw: rotation around Z axis
    const float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    const float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    e.yaw = atan2f(siny_cosp, cosy_cosp);

    return e;
}

inline Quaternion eulerToQuaternion(const EulerAngle &e)
{
    // Half angles
    const float cy = cosf(e.yaw * 0.5f);
    const float sy = sinf(e.yaw * 0.5f);

    const float cp = cosf(e.pitch * 0.5f);
    const float sp = sinf(e.pitch * 0.5f);

    const float cr = cosf(e.roll * 0.5f);
    const float sr = sinf(e.roll * 0.5f);

    Quaternion q;

    // ZYX intrinsic rotation:
    // yaw(Z) -> pitch(Y) -> roll(X)

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

// Convert euler angle rate of change to body frame rate of change
inline Vec3 eulerRatesToBodyRates(const EulerAngle &attitude,
                                  const EulerAngle &euler_rate)
{
    const float phi = attitude.roll;
    const float theta = attitude.pitch;

    const float roll_dot = euler_rate.roll;
    const float pitch_dot = euler_rate.pitch;
    const float yaw_dot = euler_rate.yaw;

    const float sphi = sinf(phi);
    const float cphi = cosf(phi);
    const float stheta = sinf(theta);
    const float ctheta = cosf(theta);

    Vec3 omega_body;

    omega_body.x = roll_dot - yaw_dot * stheta;

    omega_body.y = pitch_dot * cphi + yaw_dot * sphi * ctheta;

    omega_body.z = -pitch_dot * sphi + yaw_dot * cphi * ctheta;

    return omega_body;
}

inline BLA::Matrix<3, 3> skewSymmetric(const Vec3 &v)
{
    return {
        0.0f, -v.z, v.y,
        v.z, 0.0f, -v.x,
        -v.y, v.x, 0.0f};
}

inline BLA::Matrix<3, 3> exp(const Vec3 &u)
{
    // According to p.18 eq (77) Joan Sola
    // Convert a rotation vector into a rotation matrix with exponential map
    const float theta2 = dot(u, u);
    const float theta = sqrtf(theta2);
    const BLA::Matrix<3, 3> I = {
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f};

    if (theta < 1e-6f)
    {
        // First-order approximation near zero to avoid numerical instability.
        return I + skewSymmetric(u);
    }

    const BLA::Matrix<3, 3> K = skewSymmetric(u);
    const float a = sinf(theta) / theta;
    const float b = (1.0f - cosf(theta)) / theta2;
    return I + (a * K) + (b * (K * K));
}

inline Quaternion qexp(const Vec3 &u)
{
    // According to Joan Sola p.22 eq. (101)
    // Convert a rotation vector to a corresponding rotation quaternion
    const float theta2 = dot(u, u);
    const float theta = sqrtf(theta2);

    if (theta < 1e-6f)
    {
        // For tiny angles: sin(theta/2)/theta ~= 1/2, cos(theta/2) ~= 1.
        return normalize(Quaternion{
            0.5f * u.x,
            0.5f * u.y,
            0.5f * u.z,
            1.0f});
    }

    const float half_theta = 0.5f * theta;
    const float s_over_theta = sinf(half_theta) / theta;

    return Quaternion{
        s_over_theta * u.x,
        s_over_theta * u.y,
        s_over_theta * u.z,
        cosf(half_theta)};
}

inline BLA::Matrix<3, 3> quatToRotMat(const Quaternion &q_in)
{
    const Quaternion q = normalize(q_in);
    const float x = q.x;
    const float y = q.y;
    const float z = q.z;
    const float w = q.w;

    const float xx = x * x;
    const float yy = y * y;
    const float zz = z * z;
    const float xy = x * y;
    const float xz = x * z;
    const float yz = y * z;
    const float wx = w * x;
    const float wy = w * y;
    const float wz = w * z;

    return {
        1.0f - 2.0f * (yy + zz), 2.0f * (xy - wz), 2.0f * (xz + wy),
        2.0f * (xy + wz), 1.0f - 2.0f * (xx + zz), 2.0f * (yz - wx),
        2.0f * (xz - wy), 2.0f * (yz + wx), 1.0f - 2.0f * (xx + yy)};
}

inline float blend(float a, float b, float t)
{
    return a + t * (b - a);
}

inline float sqr(float x)
{
    return x * x;
}

#endif
