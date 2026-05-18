#ifndef PID_H
#define PID_H

#include "datastructs.h"
#include "mathutils.h"

struct MotorAdjust
{
    float yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
};

class PID
{
public:
    PID() = default;
    PID(float p_term, float i_term, float d_term);

    float calculate(float new_error);
    void reset();

private:
    float P = 0.0;
    float I = 0.0;
    float D = 0.0;

    float last_error = 0.0;
    float last_iterm = 0.0;
};

class AttiStabilizer
{
    // Double loop stabilizer, inner = rate, outer = angle.
public:
    PID y_rate_pid = PID(0.0008, 1e-4, 3e-7);
    PID x_rate_pid = PID(0.0009, 1e-4, 3.5e-7);
    PID z_rate_pid = PID(0.003, 1.2e-4, 0);

    MotorAdjust compute_rpy_adjust(Quaternion q, EulerAngle target, Vec3 gyro);
    void reset();

    inline float angle_error_to_angle_rate(float angle)
    {
        float a = fabsf(angle);

    float mult;

    if (a < 4.0f) {
        mult = 1.5f;
    } else if (a < 8.0f) {
        float t = (a - 4.0f) / 4.0f;
        mult = 1.5f + t * (4.0f - 1.5f);
    } else if (a < 15.0f) {
        float t = (a - 8.0f) / 7.0f;
        mult = 3.0f + t * (2.0f - 4.0f);
    } else {
        mult = 2.0f;
    }

    float rate = angle * mult;

    constexpr float MAX_RATE_DPS = 120.0f;
    return constrain(rate, -MAX_RATE_DPS, MAX_RATE_DPS);
    }
};

class VelStabilizer {

    PID x_rate_pid = PID(5.0f, 0.0f, 0.0f);
    PID y_rate_pid = PID(5.0f, 0.0f, 0.0f);

    inline EulerAngle vel_error_to_angle_target(Vec3 v_error, float yawrate)
    {
        return EulerAngle {
            yawrate,
            -1.0f * x_rate_pid.calculate(v_error.x),
            y_rate_pid.calculate(v_error.y)
        };
    }
};

#endif
