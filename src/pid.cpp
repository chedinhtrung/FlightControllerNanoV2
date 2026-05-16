#include "pid.h"
#include "config.h"
#include "mathutils.h"

PID::PID(float p_term, float i_term, float d_term) {
    P = p_term;
    I = i_term;
    D = d_term;
}

float PID::calculate(float new_error) {
    const float pterm = P * new_error;

    // Trapezoidal integration
    last_iterm += I * 0.5 * (new_error + last_error) * DT;

    if (last_iterm > 0.1) {
        last_iterm = 0.1;
    } else if (last_iterm < -0.1) {
        last_iterm = -0.1;
    }

    const float dterm_raw = D * (new_error - last_error) / DT;
    float dterm = dterm_raw;

    if (dterm > 0.12) {
        dterm = 0.12;
    } else if (dterm < -0.12) {
        dterm = -0.12;
    }

    last_error = new_error;

    float pid_out = pterm + dterm + last_iterm;

    if (pid_out > 0.4) {
        pid_out = 0.4;
    } else if (pid_out < -0.4) {
        pid_out = -0.4;
    }

    return pid_out;
}

void PID::reset() {
    last_error = 0.0;
    last_iterm = 0.0;
}

// Computes normalized adjust to motor from body orientation q, euler target, and gyro reading 
// Euler target: roll and pitch are absolute angles, yaw is angle rate
MotorAdjust AttiStabilizer::compute_rpy_adjust(Quaternion q, EulerAngle target, Vec3 gyro){

    EulerAngle e = quaternionToEuler(q);  

    float pitch_error = target.pitch - (e.pitch * DEG_PER_RAD + 0.3); // some sensor mount calibration
    float roll_error = target.roll - (e.roll * DEG_PER_RAD + 4.5);

    float pitchrate_target, rollrate_target;

    // use nonlinear curve to calculate desired rates
    pitchrate_target = angle_error_to_angle_rate(pitch_error);
    

    rollrate_target = angle_error_to_angle_rate(roll_error);
    

    float yawrate_target = 2.8*target.yaw;

    EulerAngle euler_rate_target;
    euler_rate_target.pitch = pitchrate_target;
    euler_rate_target.roll = rollrate_target;
    euler_rate_target.yaw = yawrate_target;
    
    // convert to desired body rate

    Vec3 body_rate_target = eulerRatesToBodyRates(e, euler_rate_target);

    // feed the error to pid
    float pitchadjust = y_rate_pid.calculate(body_rate_target.y - gyro.y * DEG_PER_RAD);
    float rolladjust = x_rate_pid.calculate(body_rate_target.x - gyro.x * DEG_PER_RAD);
    float yawadjust = z_rate_pid.calculate(body_rate_target.z - gyro.z * DEG_PER_RAD);
    
    return MotorAdjust {
        .yaw = yawadjust,
        .pitch  = pitchadjust,
        .roll = rolladjust
    };
    
}


void AttiStabilizer::reset(){
    x_rate_pid.reset();
    y_rate_pid.reset();
    z_rate_pid.reset();
}

 