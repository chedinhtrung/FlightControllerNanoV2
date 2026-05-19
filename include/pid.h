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
    PID(float p_term, float i_term, float d_term, float i_lim = 0.1f, float d_lim = 0.12f);

    float calculate(float new_error);
    void reset();

private:
    float P = 0.0;
    float I = 0.0;
    float D = 0.0;
    float I_LIM = 0.1f;
    float D_LIM = 0.12f;

    float last_error = 0.0;
    float last_iterm = 0.0;
};

class AttiStabilizer
{
    // Double loop stabilizer, inner = rate, outer = angle.
public:
    PID y_rate_pid = PID(0.0008f, 1e-4f, 3e-7f, 0.1f, 0.12f);
    PID x_rate_pid = PID(0.0009f, 1e-4f, 3e-7f, 0.1f, 0.12f);
    PID z_rate_pid = PID(0.003f, 1.2e-4f, 0.0f, 0.1f, 0.12f);

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

    PID vx_pid_l1 = PID(30.0f,1.2e-3f, 0.0f, 2.0f, 0.12f);
    PID vy_pid_l1 = PID(30.0f, 1.2e-3f, 0.0f, 2.0f, 0.12f);

    PID vx_pid_l2 = PID(40.0f, 0.0f, 0.0f, 2.0f, 0.12f);
    PID vy_pid_l2 = PID(40.0f, 0.0f, 0.0f, 2.0f, 0.12f);

public:
    inline float deadband_x(float x)
    {
        constexpr float DB_ENTER = 0.01f;
        constexpr float DB_EXIT = 0.04f;

        static bool in_deadband = true;

        const float ax = fabsf(x);

        if (in_deadband)
        {
            if (ax > DB_EXIT)
                in_deadband = false;
        }
        else
        {
            if (ax < DB_ENTER)
                in_deadband = true;
        }

        return in_deadband ? 0.0f : x;
    }

    inline float deadband_y(float y)
    {
        constexpr float DB_ENTER = 0.01f;
        constexpr float DB_EXIT = 0.04f;

        static bool in_deadband = true;

        const float ax = fabsf(y);

        if (in_deadband)
        {
            if (ax > DB_EXIT)
                in_deadband = false;
        }
        else
        {
            if (ax < DB_ENTER)
                in_deadband = true;
        }

        return in_deadband ? 0.0f : y;
    }

    inline float velHoldAuthorityFromHeight(float h_m)
    {
        // computes how much gain to adjust at different height
        constexpr float FULL_H = 1.2f;     // full authority below this
        constexpr float WEAK_H = 4.0f;     // weak authority above this
        constexpr float MIN_SCALE = 0.25f; // keep some damping

        if (h_m <= FULL_H)
            return 1.0f;
        if (h_m >= WEAK_H)
            return MIN_SCALE;

        float t = (h_m - FULL_H) / (WEAK_H - FULL_H);

        // Smoothstep, nicer than linear.
        t = t * t * (3.0f - 2.0f * t);

        return 1.0f + t * (MIN_SCALE - 1.0f);
    }

    inline float slewLimit(float target, float current, float max_rate_dps, float dt)
    {
        float max_step = max_rate_dps * dt;
        float delta = target - current;
        delta = constrain(delta, -max_step, max_step);
        return current + delta;
    }

    inline EulerAngle vel_error_to_angle_target(Vec3 v_error, float yawrate)
    {
        constexpr float VEL_DEADBAND = 0.06f;
        constexpr float SWITCH_VEL = 0.20f;
        constexpr float MAX_ANGLE = 15.0f;
        constexpr float MAX_SLEW_DPS = 60.0f;

        static float last_pitch = 0.0f;
        static float last_roll = 0.0f;

        v_error.x = deadband_x(v_error.x);
        v_error.y = deadband_y(v_error.y);

        // Blend factor: 0 = low-error PID, 1 = high-error PID
        float tx = constrain(fabsf(v_error.x) / SWITCH_VEL, 0.0f, 1.0f);
        tx = tx * tx * (3 - 2*tx);
        float ty = constrain(fabsf(v_error.y) / SWITCH_VEL, 0.0f, 1.0f);
        ty = ty * ty * (3 - 2*ty);

        float pitch_l1 = vx_pid_l1.calculate(v_error.x);
        float pitch_l2 = vx_pid_l2.calculate(v_error.x);

        float roll_l1 = vy_pid_l1.calculate(v_error.y);
        float roll_l2 = vy_pid_l2.calculate(v_error.y);

        float pitch_target = blend(pitch_l1, pitch_l2, tx);
        float roll_target = blend(roll_l1, roll_l2, ty);

        pitch_target = constrain(pitch_target, -MAX_ANGLE, MAX_ANGLE);
        roll_target = constrain(roll_target, -MAX_ANGLE, MAX_ANGLE);

        pitch_target = slewLimit(pitch_target, last_pitch, MAX_SLEW_DPS, DT);
        roll_target = slewLimit(roll_target, last_roll, MAX_SLEW_DPS, DT);

        last_pitch = pitch_target;
        last_roll = roll_target;

        return EulerAngle{
            yawrate,
            -pitch_target,
            roll_target};
    }

    void reset();
};

#endif
