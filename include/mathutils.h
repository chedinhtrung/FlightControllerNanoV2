#ifndef MATHUTILS
#define MATHUTILS
#include "datastructs.h"

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
inline Quaternion quatFromTwoUnitVectors(Vec3 from, Vec3 to) {
    float c = dot(from, to);
    Vec3 v = cross(from, to);

    if (c < -0.9999f) {
        // 180 degrees: choose any axis perpendicular to from
        Vec3 axis = normalize(abs(from.x) < 0.9f
            ? cross(from, Vec3{1,0,0})
            : cross(from, Vec3{0,1,0}));

        return Quaternion{0.0f, axis.x, axis.y, axis.z}; // w,x,y,z
    }

    Quaternion q{
        1.0f + c,
        v.x,
        v.y,
        v.z
    };

    return normalize(q);
}

// Convert quaternion to yaw - pitch - roll euler angles in radian
inline EulerAngle quaternionToEuler(const Quaternion& q)
{
    EulerAngle e;

    // Roll: rotation around X axis
    const float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    const float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    e.roll = atan2f(sinr_cosp, cosr_cosp);

    // Pitch: rotation around Y axis
    const float sinp = 2.0f * (q.w * q.y - q.z * q.x);

    if (fabsf(sinp) >= 1.0f) {
        e.pitch = copysignf((float)M_PI / 2.0f, sinp);
    } else {
        e.pitch = asinf(sinp);
    }

    // Yaw: rotation around Z axis
    const float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    const float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    e.yaw = atan2f(siny_cosp, cosy_cosp);

    return e;
}

// Convert euler angle rate of change to body frame rate of change 
inline Vec3 eulerRatesToBodyRates(const EulerAngle& attitude,
                              const EulerAngle& euler_rate)
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

    omega_body.y = pitch_dot * cphi
                 + yaw_dot * sphi * ctheta;

    omega_body.z = -pitch_dot * sphi
                 + yaw_dot * cphi * ctheta;

    return omega_body;
}

inline Vec3 get_compensated_vxy(){
    
}

#endif
