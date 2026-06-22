#include "main.h"
#include "message_helpers.h"

ICM42688P imu;
Imu imu_device(imu);
ImuData imu_data;

PPMReceiver ppm_receiver;
Receiver receiver(ppm_receiver);
PPMCommand control_raw;

Motor motor;
MotorDevice motor_device(motor);

unsigned long last_active = micros();

AttiStabilizer atti_stabilizer = AttiStabilizer();
VelStabilizer vxy_stabilizer = VelStabilizer();

VzStabilizer vz_stabilizer = VzStabilizer();

PositionHoldController pos_hold_controller = PositionHoldController();
AltHoldController alt_hold_controller = AltHoldController();

ESKF eskf = ESKF();

StateMachine statemachine = StateMachine();

void setup()
{
  Serial.begin(115200);

  motor_device.write(0.0f, 0.0f, 0.0f, 0.0f);

  delay(5000);
  if (!imu_device.setup())
  {
    Serial.println("IMU Failure");
    while (true)
    {
    }
  }

  // Initial read and initialize the attitude estimate.
  while (!imu_device.read(imu_data))
  {
    Serial.println("IMU Failure");
    delay(5);
  }

  eskf.setup(imu_data.accel);

  // Safety lock: arm loop only after throttle is low.
  do
  {
    bool read_ok = receiver.read(control_raw);
    delay(2);

    debug::log(control_raw, read_ok ? "OK" : "Failed");
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

  //debug::plot(Vec3{imu_data.gyro.y, static_cast<float>(mtf02_data.data.flow_x) * static_cast<float>(mtf02_data.data.dist_mm) * 1e-5f, 0});

  //debug::log(quaternionToEuler(eskf.nominal.q) * DEG_PER_RAD);

  // debug::log(imu_data.accel, "accel(g)");

  PPMCommand cmd_raw{};
  bool cmd_read_success = receiver.read(cmd_raw);
  if (!cmd_read_success)
  {

  }
  else {
    PPMCommand rpy_cmd = receiver.to_anglemode(cmd_raw); // IMPORTANT: forgetting this line will cause drone to fly away
    PPMCommand vxyz_cmd = receiver.to_vxyz_mode(cmd_raw);

    debug::log(rpy_cmd.C3, "Throttle");
    motor.set_motor_raw((uint16_t)1024, (uint16_t)1024,(uint16_t)1400,(uint16_t)1024);
  }


 

  while (micros() - last_active < PERIOD_US)
  {
  }
}
