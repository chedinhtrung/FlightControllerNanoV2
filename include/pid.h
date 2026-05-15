#ifndef PID_H
#define PID_H

#include "datastructs.h"

struct MotorAdjust {   
    float yaw = 0.0f;
    float pitch = 0.0f; 
    float roll = 0.0f;
};

class PID {
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

class AttiStabilizer {

    public:
        PID y_rate_pid = PID(0.0005, 1e-4, 2.5e-7); ;
        PID x_rate_pid = PID(0.0005, 1e-4, 2.5e-7);
        PID z_rate_pid = PID(0.0018, 1.2e-4, 0);

        MotorAdjust compute_rpy_adjust(Quaternion q, EulerAngle target, Vec3 gyro);
        void reset();
          
};

#endif
