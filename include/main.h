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

#include "eskf.h"

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

extern ESKF eskf;

extern unsigned long last_active;

extern AttiStabilizer atti_stabilizer;
extern VelStabilizer vel_stabilizer;

inline void reset_flight_controllers()
{
    atti_stabilizer.reset();
    vel_stabilizer.reset();
}

inline void update_optical_flow(int time_buffer_us)
{
    // update EKSF 
    // Low priority parse window: consume bytes while data is pending and loop time remains.
    while (optical_flow.has_bytes() && (micros() - last_active) < (PERIOD_US - time_buffer_us))
    {
        optical_flow.kick();
    }

    bool flow_new_data = optical_flow.read(mtf02_data);

    if (flow_new_data)
    {
        // update eskf with flow data. Refer to doc on update model
        eskf.correct_flow_and_range(mtf02_data);
        debug::plot(eskf.nominal.v);
    }
}

inline void update_baro()
{
    barometer.kick();
    if (barometer.read(baro_data))
    {
        // debug::plot(baro_data.altitude_m);
        FloatWithTrust baro_vel = barometer.get_vz_baro(baro_data.altitude_m);
        // debug::plot(baro_vel.value);
        // el_kf.updateBaro(baro_vel);
    }
}

inline EulerAngle compute_angle_target_from_cmd(const PPMCommand &rpy_cmd, const PPMCommand &vxy_cmd)
{

    Vec3 command_target{
        vxy_cmd.C2,
        vxy_cmd.C4,
        vxy_cmd.C1};

    EulerAngle angle_target_vel;

    Vec3 v_world = eskf.nominal.v;

    EulerAngle e = quaternionToEuler(eskf.nominal.q);
    float cy = cosf(e.yaw);
    float sy = sinf(e.yaw);
    
    // convert to v1 frame for control
    Vec3 v_v1{
        cy * v_world.x + sy * v_world.y,
        -sy * v_world.x + cy * v_world.y,
        v_world.z};

    angle_target_vel = vel_stabilizer.vxy_error_to_angle_target(command_target - v_v1, command_target.z);

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