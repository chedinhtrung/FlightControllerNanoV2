#include <Arduino.h>
#include <Wire.h>

#include "debug.h"
#include "drivers/mpu9250.h"
#include "madgwick.h"

MPU9250 imu;
Madgwick madgw;
ImuData imu_data;

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin();
  Wire.setClock(400000);
  
  delay(2000);
  imu.setup();
  delay(500);

  // initial read and initialize the madgwick
  imu.read(imu_data);
  madgw = Madgwick(MW_BETA, imu_data.accel);
}

void loop() {
  imu.update(imu_data);
  madgw.update(imu_data);

  EulerAngle e = quaternionToEuler(madgw.q);
  debug::log(e);
}
