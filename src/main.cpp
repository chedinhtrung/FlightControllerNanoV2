#include <Arduino.h>
#include <Wire.h>

#include "debug.h"
#include "drivers/mpu9250.h"

MPU9250 imu;
ImuData imu_data;

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin();
  Wire.setClock(400000);

  imu.setup();

}

void loop() {
  if (imu.read(imu_data)) {
    log(imu_data, "IMU");
  } else {
    Serial.println("IMU read failed");
  }

  delay(4);
}
