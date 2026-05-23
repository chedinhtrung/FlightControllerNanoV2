#include "main.h"

FlightState flightstate = FlightState::DISARMED;

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

Vec3LPF vel_ctl_lpf = Vec3LPF(0.6);

unsigned long last_active = micros();

AttiStabilizer atti_stabilizer = AttiStabilizer();
VelStabilizer vel_stabilizer = VelStabilizer();

void setup()
{
  Serial.begin(115200);
  delay(200);

  Wire.begin();
  Wire.setClock(400000);

  delay(5000);
  if (!imu_device.setup())
  {
    // Placeholder: optional IMU setup error handling.
  }
  if (!barometer.setup())
  {
    // Placeholder: optional barometer setup error handling.
  }
  if (!optical_flow.setup())
  {
    // Placeholder: optional optical flow setup error handling.
  }
  delay(500);

  // Initial read and initialize the attitude estimate.
  while (!imu_device.read(imu_data))
  {
    // Placeholder: optional initial IMU read error handling.
    delay(1);
  }
  madgw = Madgwick(MW_BETA, imu_data.accel);

  // Safety lock: arm loop only after throttle is low.
  do
  {
    receiver.read(control_raw);
    delay(10);
  } while (control_raw.C3 > 1050.0f);
}

void loop()
{
  last_active = micros();

  if (!imu_device.read(imu_data))
  {
    // Placeholder: optional IMU read error handling.
  }
  madgw.update(imu_data);
  vel_kf.predict(imu_data.accel, madgw.q);

  update_optical_flow(1000);
  //update_baro();

  PPMCommand cmd_raw{};
  if (!receiver.read(cmd_raw))
  {
    // Placeholder: optional receiver read error handling.
  }

  PPMCommand rpy_cmd = receiver.to_anglemode(cmd_raw); // IMPORTANT: forgetting this line will cause drone to fly away
  PPMCommand vxy_cmd = receiver.to_vxy_mode(cmd_raw);

  bool airborne =
      rpy_cmd.C3 > 0.2f &&
      mtf02_data.data.dist_mm > 40;

  EulerAngle angle_target;

  if (airborne)
  {
    angle_target = compute_angle_target_from_cmd(rpy_cmd, vxy_cmd);
  }
  else
  {
    vel_kf.reset();
    angle_target = EulerAngle{
        rpy_cmd.C1,
        rpy_cmd.C2 * 0.5f,
        rpy_cmd.C4 * 0.5f};
  }

  MotorAdjust m_adjust = atti_stabilizer.compute_rpy_adjust(madgw.q, angle_target, imu_data.gyro);

  // Output to motor, lock until throttle is not 0

  float throttle = rpy_cmd.C3;

  if (throttle > 0.01 && throttle <= 1.0)
  {
    motor_device.write(throttle, m_adjust.yaw, m_adjust.pitch, m_adjust.roll);
  }
  else
  {
    motor.set_motor(MotorCommand{});
    reset_flight_controllers();
  }

  //debug::plot(vel_kf.velocity());
  while (micros() - last_active < PERIOD_US)
  {
  }
}
