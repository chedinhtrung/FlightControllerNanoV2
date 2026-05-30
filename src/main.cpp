#include "main.h"

ICM42688P imu;
Imu imu_device(imu);
ImuData imu_data;

PPMReceiver ppm_receiver;
Receiver receiver(ppm_receiver);
PPMCommand control_raw;

Motor motor;
MotorDevice motor_device(motor);

MS5611SPI baro;
Barometer barometer(baro);
BaroData baro_data;

MTF02 mtf02(Serial3);
OpticalFlow optical_flow(mtf02);
MTF02Data mtf02_data;

unsigned long last_active = micros();

AttiStabilizer atti_stabilizer = AttiStabilizer();
VelStabilizer vxy_stabilizer = VelStabilizer();

VzStabilizer vz_stabilizer = VzStabilizer();

ESKF eskf = ESKF();

StateMachine statemachine = StateMachine();

void setup()
{
  Serial.begin(115200);
  delay(200);

  delay(5000);
  if (!imu_device.setup())
  {
    Serial.println("IMU Failure");
    while (true)
    {
    }
  }
  if (!barometer.setup())
  {
    Serial.println("Baro Failure");
    while (true)
    {
    }
  }
  if (!optical_flow.setup())
  {
    Serial.println("Optical flow failure");
    while (true)
    {
    }
  }
  delay(500);

  // Initial read and initialize the attitude estimate.
  while (!imu_device.read(imu_data))
  {
    Serial.println("IMU Failure");
    delay(5);
  }

  // Safety lock: arm loop only after throttle is low.

  eskf.setup(imu_data.accel);

  while (!barometer.read(baro_data))
  {
    barometer.kick();
    Serial.println("Waiting for baro");
    delay(10);
  }
  eskf.reset_baro_offset(baro_data.altitude_m);

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

  eskf.propagate(imu_data);
  eskf.correct_gravity(imu_data.accel);
  
  //debug::log(quaternionToEuler(eskf.nominal.q) * DEG_PER_RAD);

  /*
  update_optical_flow(1000);
  update_baro();

  Vec3 v_world = eskf.nominal.v;

  EulerAngle e = quaternionToEuler(eskf.nominal.q);
  float cy = cosf(e.yaw);
  float sy = sinf(e.yaw);

  Vec3 v_v1{
      cy * v_world.x + sy * v_world.y,
      -sy * v_world.x + cy * v_world.y,
      v_world.z};

  //debug::plot(v_v1);

  PPMCommand cmd_raw{};
  bool receiver_ok = receiver.read(cmd_raw);
  if (!receiver_ok)
  {
    // Placeholder: optional receiver read error handling.
  }

  PPMCommand rpy_cmd = receiver.to_anglemode(cmd_raw); // IMPORTANT: forgetting this line will cause drone to fly away
  PPMCommand vxyz_cmd = receiver.to_vxyz_mode(cmd_raw);

  // debug::log(vxyz_cmd);

  // state machine uses rpy cmd for thrust 0-1
  statemachine.update(rpy_cmd, eskf.nominal, eskf.h_terrain);

  FlightState flightstate = statemachine.get_flightstate();
  FlightMode flightmode = statemachine.get_flightmode();

  const bool motors_allowed =
      flightstate != DISARMED;

  const bool ground_state =
      flightstate == DISARMED ||
      flightstate == ARMED;

  const bool manual_attitude =
      flightmode == ANGLE ||
      flightstate == DISARMED ||
      flightstate == ARMED ||
      flightstate == TAKEOFF;

  const bool manual_throttle =
      flightstate == DISARMED ||
      flightstate == ARMED ||
      flightstate == TAKEOFF ||
      flightmode != VXYZ;

  const bool use_vz_control =
      motors_allowed &&
      !manual_throttle;

  if (flightstate == DISARMED)
  {
    // eskf.reset_baro_offset(baro_data.altitude_m);
    // eskf.reset_zero_vxy(0.01f);
    reset_flight_controllers();
  }

  EulerAngle angle_target{};

  if (manual_attitude)
  {
    angle_target = EulerAngle{
        rpy_cmd.C1,
        rpy_cmd.C2 * 0.5f,
        rpy_cmd.C4 * 0.5f};
  }
  else
  {
    angle_target = compute_angle_target_from_cmd(rpy_cmd, vxyz_cmd);
  }

  MotorAdjust m_adjust = atti_stabilizer.compute_rpy_adjust(
      eskf.nominal.q,
      angle_target,
      imu_data.gyro);

  // Output to motor, lock until throttle is not 0

  float throttle = 0.0f;

  if (manual_throttle)
  {
    throttle = rpy_cmd.C3;
  }
  else
  {
    // ESKF v.z is positive down.
    constexpr float HOVER_THRUST = 0.47f;

    float vz_cmd_down = vxyz_cmd.C3;

    if (flightstate == LANDING)
    {
      // Controlled descent near ground.
      // Positive down velocity means descending.
      vz_cmd_down = 0.45f;
    }

    float vz_error = vz_cmd_down - eskf.nominal.v.z;

    float thrust_adjust = vz_stabilizer.thrust_adjust_from_vz_error(vz_error);

    // Mind the sign: + caused fly away.
    throttle = HOVER_THRUST - thrust_adjust;
    throttle = constrain(throttle, 0.0f, 1.0f);
  }

  if (motors_allowed && throttle > 0.1f && throttle <= 1.0f)
  {
    // debug::log(throttle);
    // motor_device.write(throttle, m_adjust.yaw, m_adjust.pitch, m_adjust.roll);
  }
  else
  {
    // debug::log(throttle);
    // motor.set_motor(MotorCommand{0.0f, 0.0f, 0.0f, 0.0f});

    motor_device.write(0.0f, 0.0f, 0.0f, 0.0f);
    reset_flight_controllers();
  }

  // debug::plot(e * DEG_PER_RAD);
  // debug::plot(imu_data.accel);
  */
  while (micros() - last_active < PERIOD_US)
  {
  }
}
