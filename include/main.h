#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "debug.h"
#include "devices/imu.h"
#include "devices/barometer.h"
#include "devices/motor.h"
#include "devices/optical_flow.h"
#include "devices/receiver.h"
#include "drivers/motors.h"
#include "drivers/mtf02.h"
#include "drivers/ms5611.h"
#include "drivers/mpu9250.h"
#include "drivers/receiver.h"
#include "madgwick.h"
#include "pid.h"
#include "velocity_kf.h"


enum class FlightState
{
    DISARMED,
    ARMED_IDLE,
    TAKEOFF,
    AIRBORNE
};

extern FlightState flightstate;

extern MPU9250 imu;
extern Imu imu_device;
extern ImuData imu_data;

extern Madgwick madgw;

extern PPMReceiver ppm_receiver;
extern Receiver receiver;
extern PPMCommand control_raw;

extern Motor motor;
extern MotorDevice motor_device;

extern MS5611 baro;
extern Barometer barometer;
extern BaroData baro_data;

extern MTF02 mtf02;
extern OpticalFlow optical_flow;
extern MTF02Data mtf02_data;

extern VelKF2 vel_kf;

extern unsigned long last_active;

extern AttiStabilizer atti_stabilizer;
extern VelStabilizer vel_stabilizer;

extern Vec3LPF vel_ctl_lpf;

inline void reset_flight_controllers()
{
    atti_stabilizer.reset();
    vel_stabilizer.reset();
    vel_kf.reset();
}

inline void update_optical_flow(int time_buffer_us)
{
    // update state and KF with optical flow's velocity and range
    // Low priority parse window: consume bytes while data is pending and loop time remains.
    while (optical_flow.has_bytes() && (micros() - last_active) < (PERIOD_US - time_buffer_us))
    {
        optical_flow.kick();
    }

    bool flow_new_data = optical_flow.read(mtf02_data);

    if (flow_new_data)
    {
        Vec3WithTrust v_v1 = optical_flow.get_compensated_v1frame_vxy(mtf02_data, imu_data.gyro, madgw.q);
        vel_kf.updateFlow(Vec3WithTrust{vel_ctl_lpf.update(v_v1.value), v_v1.trust});
        FloatWithTrust vz_range = optical_flow.get_compensated_vz(mtf02_data.data.dist_mm * 1e-3f, imu_data.accel, madgw.q);
        vel_kf.updateRange(vz_range);
    }
}

inline void update_baro(){
  barometer.kick();
  if (barometer.read(baro_data))
  {
    //debug::plot(baro_data.altitude_m);
    FloatWithTrust baro_vel = barometer.get_vz_baro(baro_data.altitude_m);
    //debug::plot(baro_vel.value);
    //el_kf.updateBaro(baro_vel);
  }
}

inline EulerAngle compute_angle_target_from_cmd(const PPMCommand &rpy_cmd, const PPMCommand& vxy_cmd)
{

    Vec3 command_target{
        vxy_cmd.C2,
        vxy_cmd.C4,
        vxy_cmd.C1};

    EulerAngle angle_target_vel;

    Vec3 v_est = vel_kf.velocity();
    angle_target_vel = vel_stabilizer.vxy_error_to_angle_target(command_target - v_est, command_target.z);

    EulerAngle angle_target_stick{
        rpy_cmd.C1 * 0.5f,
        rpy_cmd.C2 * 0.5f,
        rpy_cmd.C4};

    float authority = vel_stabilizer.velHoldAuthorityFromHeight(mtf02_data.data.dist_mm * 1e-3);

    EulerAngle angle_target;
    angle_target.yaw = angle_target_vel.yaw;
    angle_target.pitch = angle_target_vel.pitch * authority + angle_target_stick.pitch * (1 - authority);
    angle_target.roll = angle_target_vel.roll * authority + angle_target_stick.roll * (1 - authority);
    return angle_target;
}