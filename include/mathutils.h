#ifndef MATHUTILS
#define MATHUTILS
#include "datastructs.h"

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

inline EulerAngle quaternionToEuler(const Quaternion& q)
{
    EulerAngle e;

    // Roll: rotation around X axis
    const float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    const float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    e.roll = atan2f(sinr_cosp, cosr_cosp) * DEG_PER_RAD;

    // Pitch: rotation around Y axis
    const float sinp = 2.0f * (q.w * q.y - q.z * q.x);

    if (fabsf(sinp) >= 1.0f) {
        e.pitch = copysignf((float)M_PI / 2.0f, sinp);
    } else {
        e.pitch = asinf(sinp) * DEG_PER_RAD;
    }

    // Yaw: rotation around Z axis
    const float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    const float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    e.yaw = atan2f(siny_cosp, cosy_cosp) * DEG_PER_RAD;

    return e;
}

#endif