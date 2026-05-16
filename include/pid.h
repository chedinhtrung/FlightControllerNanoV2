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
    PID y_rate_pid = PID(0.0005, 1e-4, 2.5e-7);
    ;
    PID x_rate_pid = PID(0.0005, 1e-4, 2.5e-7);
    PID z_rate_pid = PID(0.0018, 1.2e-4, 0);

    MotorAdjust compute_rpy_adjust(Quaternion q, EulerAngle target, Vec3 gyro);
    void reset();

    inline float angle_error_to_angle_rate(float angle)
    {
        // Angle should be in degrees. Computes the desired angle using a custom non linear curve
        float abs_angle = abs(angle);
        float mult = 1;
        if (abs_angle < 4)
        {
            mult = 1.7f;
        }
        else if (abs_angle >= 4 && abs_angle < 8)
        {
            mult = 3.0f;
        }
        else
        {
            mult = 2.0f;
        }
        return angle * mult;
    }
};

#endif
