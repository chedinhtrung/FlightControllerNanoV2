#include <Arduino.h>
#include <Wire.h>

#include "debug.h"
#include "devices/imu.h"
#include "drivers/motors.h"
#include "drivers/mtf02.h"
#include "drivers/ms5611.h"
#include "drivers/mpu9250.h"
#include "drivers/receiver.h"
#include "madgwick.h"
#include "pid.h"

MPU9250 imu;
Imu imu_device(imu);
ImuData imu_data;

Madgwick madgw;

Receiver receiver;

ReceiverData control_raw;
ReceiverData control_cmd;

Motor motor;

MS5611 baro;
BaroData baro_data;

MTF02 optical_flow(Serial3);
MTF02Data mtf02_data;

PID y_rate_pid(0.0005, 1e-4, 2.5e-7); ;
PID x_rate_pid(0.0005, 1e-4, 2.5e-7);
PID z_rate_pid(0.0018, 1.2e-4, 0);

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
  baro.setup();
  optical_flow.setup();
  delay(500);

  // Initial read and initialize the attitude estimate.
  if (!imu.read(imu_data)) {
    // Placeholder: optional initial IMU read error handling.
  }
  madgw = Madgwick(MW_BETA, imu_data.accel);

  // Safety lock: arm loop only after throttle is low.
  do {
    control_raw = receiver.recv();
    delay(10);
  } while (control_raw.ThrottleIn > 1050.0f);
}

void loop() {
  last_active = micros();

  imu_device.update();
  if (!imu_device.read(imu_data)) {
    // Placeholder: optional IMU read error handling.
  }
  madgw.update(imu_data);

  ReceiverData rd = receiver.recv();
  rd = receiver.to_anglemode(rd);       //IMPORTANT: forgetting this line will cause drone to fly away

  
  // calculate errors

  EulerAngle e = quaternionToEuler(madgw.q);  
  double pitch_error = rd.PitchIn - (e.pitch * DEG_PER_RAD + 0.3); // some sensor mount calibration
  double roll_error = rd.RollIn - (e.roll * DEG_PER_RAD + 4.5);

  double pitchrate_target, rollrate_target;

  // use square root curve to calculate desired rates
  if (pitch_error >= 0){
    pitchrate_target = pitch_error/(0.025*sqrt(pitch_error) + 0.45);
  } else {
    pitchrate_target = pitch_error/(0.025*sqrt(-pitch_error) + 0.45);
  }

  if (roll_error >= 0){
    rollrate_target = roll_error/(0.018*sqrt(roll_error) + 0.45);;
  } else {
    rollrate_target = roll_error/(0.018*sqrt(-roll_error) + 0.45);
  }
  

  double yawrate_target = 2.8*rd.YawIn;

  EulerAngle euler_rate_target;
  euler_rate_target.pitch = pitchrate_target;
  euler_rate_target.roll = rollrate_target;
  euler_rate_target.yaw = yawrate_target;
  
  // convert to desired body rate

  Vec3 body_rate_target = eulerRatesToBodyRates(e, euler_rate_target);

  // feed the error to pid
  double pitchadjust = y_rate_pid.calculate(body_rate_target.y - imu_data.gyro.y * DEG_PER_RAD);
  double rolladjust = x_rate_pid.calculate(body_rate_target.x - imu_data.gyro.x * DEG_PER_RAD);
  double yawadjust = z_rate_pid.calculate(body_rate_target.z - imu_data.gyro.z * DEG_PER_RAD);

  //Map to motor output

  float fl = rd.ThrottleIn + pitchadjust + rolladjust - yawadjust;
  float fr = rd.ThrottleIn + pitchadjust - rolladjust + yawadjust;
  float bl = rd.ThrottleIn - pitchadjust + rolladjust + yawadjust;
  float br = rd.ThrottleIn - pitchadjust - rolladjust - yawadjust;

  // Output to motor, lock until throttle is not 0
  // Danger: forgetting to convert rd.ThrottleIn to percentage will lead to flyaway

  if (rd.ThrottleIn > 0.01 && rd.ThrottleIn <= 1.0){
    motor.set_motor(fl, fr, bl, br);
  } else {
    motor.set_motor(0.0f, 0.0f, 0.0f, 0.0f);
    x_rate_pid.reset();
    y_rate_pid.reset();
    z_rate_pid.reset();
    //alt.filter.reset();
  }

  baro.kick(baro_data);
  if (baro.read(baro_data)) {
    Serial.printf("alt_m: %.3f\n", baro_data.altitude_m);
  }

  optical_flow.kick();
  if (optical_flow.read(mtf02_data)) {
    debug::log(mtf02_data, "optical_flow");
  }

  while(micros() - last_active < DT*1e6){}
  
}
