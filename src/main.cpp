#include "main.h"

ICM42688P imu;
Imu imu_device(imu);
ImuData imu_data;

unsigned long last_active = micros();

AttiStabilizer atti_stabilizer = AttiStabilizer();
VelStabilizer vxy_stabilizer = VelStabilizer();

VzStabilizer vz_stabilizer = VzStabilizer();

ESKF eskf = ESKF();

StateMachine statemachine = StateMachine();

void setup()
{
  Serial.begin(115200);

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

  // Safety lock: arm loop only after throttle is low.

  eskf.setup(imu_data.accel);

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
  
  debug::log(quaternionToEuler(eskf.nominal.q) * DEG_PER_RAD);

  
  //debug::log(imu_data.accel, "accel(g)");
  
  while (micros() - last_active < PERIOD_US)
  {
  }
}
