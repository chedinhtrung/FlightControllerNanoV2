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
    PID y_rate_pid = PID(0.0004f, 0.8e-4f, 2.5e-7f, 0.15f, 0.12f);
    PID x_rate_pid = PID(0.0005f, 0.8e-4f, 2.5e-7f, 0.15f, 0.12f);
    PID z_rate_pid = PID(0.003f, 2e-3f, 0.0f, 0.15f, 0.12f);

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

    PID vx_pid_l1 = PID(23.0f, 0.0f, 0.1e-4f, 1.0f, 0.0f);
    PID vy_pid_l1 = PID(23.0f, 0.0f, 0.1e-4f, 1.0f, 0.0f);

    PID vx_pid_l2 = PID(35.0f, 0.0f, 0.6e-4f, 0.0f, 1.5f);
    PID vy_pid_l2 = PID(35.0f, 0.0f, 0.6e-4f, 0.0f, 1.5f);

public:
    inline float deadband_x(float x)
    {
        constexpr float DB_ENTER = 0.01f;
        constexpr float DB_EXIT = 0.02f;

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
        constexpr float DB_EXIT = 0.02f;

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
        constexpr float FULL_H = 2.0f;     // full authority below this
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

    inline EulerAngle vxy_error_to_angle_target(Vec3 target_v, Vec3 v_error, float yawrate)
    {
        constexpr float VEL_DEADBAND = 0.06f;
        constexpr float SWITCH_VEL = 0.15f;
        constexpr float MAX_ANGLE = 25.0f;
        constexpr float MAX_SLEW_DPS = 70.0f;

        static float last_pitch = 0.0f;
        static float last_roll = 0.0f;

        v_error.x = deadband_x(v_error.x);
        v_error.y = deadband_y(v_error.y);

        // Blend factor: 0 = low-error PID, 1 = high-error PID
        float tx = constrain(fabsf(v_error.x) / SWITCH_VEL, 0.0f, 1.0f);
        tx = tx * tx * (3 - 2 * tx);
        float ty = constrain(fabsf(v_error.y) / SWITCH_VEL, 0.0f, 1.0f);
        ty = ty * ty * (3 - 2 * ty);

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

        // feed forward term
        constexpr float FFWD_DEG_PER_MPS = 4.0f;
        float pitch_fwd = -target_v.x * FFWD_DEG_PER_MPS; // each m/s target needs about 2.5 degs to MAINTAIN due to drag
        float roll_fwd = target_v.y * FFWD_DEG_PER_MPS;

        last_pitch = pitch_target;
        last_roll = roll_target;

        return EulerAngle{
            yawrate,
            -pitch_target + pitch_fwd,
            roll_target + roll_fwd};
    }
    void reset();
};

class VzStabilizer
{
private:
    PID vz_pid = PID(0.30, 2e-2f, 0.0f, 0.05f, 0.05f);

public:
    inline float thrust_adjust_from_vz_error(float vz_error)
    {
        float adj = vz_pid.calculate(vz_error);
        adj = constrain(adj, -0.3, 0.3);
        return adj;
    }
    inline void reset() { vz_pid.reset(); }
};

class PositionHoldController
{
private:
    float KP_POS = 1.0f; // m/s per m error
    float MAX_V = 0.8f;  // m/s

public:
    Vec3 target;
    bool active = false;
    inline Vec3 vel_from_pos_error(const Vec3& pos_error)
    {

        float dist = sqrt(dot(pos_error, pos_error));

        float mult;

        if (dist < 0.1f)
        {
            mult = 0.2f;
        }
        else if (dist < 0.4f)
        {
            float t = (dist - 0.1f) / 0.3f;
            mult = 0.2f + t * 0.4f;
        }
        else
        {
            mult = 0.6f;
        }

        Vec3 v_cmd = pos_error * mult;
        v_cmd.x = constrain(v_cmd.x, -MAX_V, MAX_V);
        v_cmd.y = constrain(v_cmd.y, -MAX_V, MAX_V);            
        
        return v_cmd;
    }
};
#endif
