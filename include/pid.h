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

        if (a < 4.0f)
        {
            mult = 1.5f;
        }
        else if (a < 8.0f)
        {
            float t = (a - 4.0f) / 4.0f;
            mult = 1.5f + t * (4.0f - 1.5f);
        }
        else if (a < 15.0f)
        {
            float t = (a - 8.0f) / 7.0f;
            mult = 4.0f + t * (2.0f - 4.0f);
        }
        else
        {
            mult = 2.0f;
        }

        float rate = angle * mult;

        constexpr float MAX_RATE_DPS = 120.0f;
        return constrain(rate, -MAX_RATE_DPS, MAX_RATE_DPS);
    }
};

class VelStabilizer
{

    PID vx_pid_l1 = PID(25.0f, 2e-4f, 0.2f);
    PID vy_pid_l1 = PID(25.0f, 2e-4f, 0.2f);

    PID vx_pid_l2 = PID(50.0f, 1e-4f, 0.1f);
    PID vy_pid_l2 = PID(50.0f, 1e-4f, 0.1f);

public:
    inline float deadband(float x, float db)
    {
        return (fabsf(x) < db) ? 0.0f : x;
    }

    inline float blend(float a, float b, float t)
    {
        return a + t * (b - a);
    }

    inline EulerAngle vel_error_to_angle_target(Vec3 v_error, float yawrate)
    {
        constexpr float VEL_DEADBAND = 0.07f; // m/s
        constexpr float SWITCH_VEL = 0.15f;   // m/s
        constexpr float MAX_ANGLE = 25.0f;     // degrees, assuming your angle targets are deg

        v_error.x = deadband(v_error.x, VEL_DEADBAND);
        v_error.y = deadband(v_error.y, VEL_DEADBAND);

        // Blend factor: 0 = low-error PID, 1 = high-error PID
        float tx = constrain(fabsf(v_error.x) / SWITCH_VEL, 0.0f, 1.0f);
        float ty = constrain(fabsf(v_error.y) / SWITCH_VEL, 0.0f, 1.0f);

        float pitch_l1 = vx_pid_l1.calculate(v_error.x);
        float pitch_l2 = vx_pid_l2.calculate(v_error.x);

        float roll_l1 = vy_pid_l1.calculate(v_error.y);
        float roll_l2 = vy_pid_l2.calculate(v_error.y);

        float pitch_target = blend(pitch_l1, pitch_l2, tx);
        float roll_target = blend(roll_l1, roll_l2, ty);

        pitch_target = constrain(pitch_target, -MAX_ANGLE, MAX_ANGLE);
        roll_target = constrain(roll_target, -MAX_ANGLE, MAX_ANGLE);

        return EulerAngle{
            yawrate,
            -pitch_target,
            roll_target};
    }

    void reset();
};

#endif
