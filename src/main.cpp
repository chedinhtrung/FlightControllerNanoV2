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

MPU9250 imu;
Imu imu_device(imu);
ImuData imu_data;

Madgwick madgw;

PPMReceiver ppm_receiver;
Receiver receiver(ppm_receiver);
PPMCommand control_raw;

Motor motor;
MotorDevice motor_device(motor);

MS5611 baro;
Barometer barometer(baro);
BaroData baro_data;

MTF02 mtf02(Serial3);
OpticalFlow optical_flow(mtf02);
MTF02Data mtf02_data;

VelKF2 vel_kf = VelKF2();
Vec3LPF vel_ctl_lpf = Vec3LPF(0.2);

AttiStabilizer atti_stabilizer = AttiStabilizer();
VelStabilizer  vel_stabilizer = VelStabilizer();

unsigned long last_active = micros();

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin();
  Wire.setClock(400000);
  
  delay(5000);
  if (!imu_device.setup()) {
    // Placeholder: optional IMU setup error handling.
  }
  if (!barometer.setup()) {
    // Placeholder: optional barometer setup error handling.
  }
  if (!optical_flow.setup()) {
    // Placeholder: optional optical flow setup error handling.
  }
  delay(500);

  // Initial read and initialize the attitude estimate.
  while (!imu_device.read(imu_data)) {
    // Placeholder: optional initial IMU read error handling.
    delay(1);
  }
  madgw = Madgwick(MW_BETA, imu_data.accel);

  // Safety lock: arm loop only after throttle is low.
  do {
    receiver.read(control_raw);
    delay(10);
  } while (control_raw.C3 > 1050.0f);
}

void loop() {
  last_active = micros();

  if (!imu_device.read(imu_data)) {
    // Placeholder: optional IMU read error handling.
  }
  madgw.update(imu_data);
  vel_kf.predict(imu_data.accel, madgw.q);

  PPMCommand cmd_raw{};
  if (!receiver.read(cmd_raw)) {
    // Placeholder: optional receiver read error handling.
  }
  PPMCommand rd = receiver.normalize(cmd_raw);       //IMPORTANT: forgetting this line will cause drone to fly away

  Vec3 vel_target {
    rd.C2 * -0.025f, 
    rd.C4 * 0.025f,
    rd.C1
  };

  // calculate errors

  Vec3 v_est = vel_kf.velocity();
  v_est = vel_ctl_lpf.update(v_est);

  EulerAngle angle_target = vel_stabilizer.vel_error_to_angle_target(vel_target - v_est, vel_target.z);

  //debug::plot(angle_target);

  MotorAdjust m_adjust = atti_stabilizer.compute_rpy_adjust(madgw.q, angle_target, imu_data.gyro);

  // Output to motor, lock until throttle is not 0

  if (rd.C3 > 0.01 && rd.C3 <= 1.0){
    motor_device.write(rd.C3, m_adjust.yaw, m_adjust.pitch, m_adjust.roll);
  } else {
    motor.set_motor(MotorCommand{});
    atti_stabilizer.reset();
    vel_stabilizer.reset();
    //alt.filter.reset();
  }

  barometer.kick();
  barometer.read(baro_data);
  
  // Low priority parse window: consume bytes while data is pending and loop time remains.
  while (optical_flow.has_bytes() && (micros() - last_active) < (PERIOD_US - 200)) {
    optical_flow.kick();
    optical_flow.read(mtf02_data);
  }

  //debug::log(mtf02_data, "optical_flow");
  Vec3 v_v1 = optical_flow.get_compensated_v1frame_vxy(mtf02_data, imu_data.gyro, madgw.q);
  vel_kf.updateFlow(v_v1, imu_data.gyro, mtf02_data.flow_quality);

  while(micros() - last_active < DT*1e6){}
  
}
