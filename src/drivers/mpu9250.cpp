#include "drivers/mpu9250.h"
#include "Wire.h"
#include <Arduino.h>

namespace {
constexpr uint8_t REG_WHO_AM_I      = 0x75; // Device identity register (expected 0x71 for MPU9250)
constexpr uint8_t REG_PWR_MGMT_1    = 0x6B; // Power management 1: reset, sleep, clock source
constexpr uint8_t REG_PWR_MGMT_2    = 0x6C; // Power management 2: sensor standby control per axis
constexpr uint8_t REG_SMPLRT_DIV    = 0x19; // Sample rate divider for output data rate
constexpr uint8_t REG_CONFIG        = 0x1A; // Gyro DLPF and external sync configuration
constexpr uint8_t REG_GYRO_CONFIG   = 0x1B; // Gyroscope full-scale range and self-test control
constexpr uint8_t REG_ACCEL_CONFIG  = 0x1C; // Accelerometer full-scale range and self-test control
constexpr uint8_t REG_ACCEL_CONFIG2 = 0x1D; // Accelerometer DLPF and sample averaging/decimation
constexpr uint8_t REG_ACCEL_XOUT_H  = 0x3B; // Start of accel/temp/gyro burst data block

constexpr float GYRO_LSB_PER_DPS  = 131.0f;   // +-250 dps
constexpr float ACCEL_LSB_PER_G   = 8192.0f;  // +-4 g

bool writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(IMUADDR);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

uint8_t readRegister(uint8_t reg) {
    Wire.beginTransmission(IMUADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return 0x00;
    }

    if (Wire.requestFrom(IMUADDR, 1) != 1) {
        return 0x00;
    }
    return Wire.read();
}

bool readBurstRaw(VectInt16 &accel, VectInt16 &gyro) {
    Wire.beginTransmission(IMUADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if (Wire.requestFrom(IMUADDR, 14) != 14) {
        return false;
    }

    accel.x = (Wire.read() << 8) | Wire.read();
    accel.y = (Wire.read() << 8) | Wire.read();
    accel.z = (Wire.read() << 8) | Wire.read();

    Wire.read();
    Wire.read();

    gyro.x = (Wire.read() << 8) | Wire.read();
    gyro.y = (Wire.read() << 8) | Wire.read();
    gyro.z = (Wire.read() << 8) | Wire.read();
    return true;
}
}

void MPU9250::setup(){
    Wire.begin();

    // Optional sanity read. Expected WHO_AM_I for MPU9250 is 0x71.
    const uint8_t whoami = readRegister(REG_WHO_AM_I);
    if (whoami != 0x71) {
        return;
    }

    // DEVICE_RESET = 1 (PWR_MGMT_1[7]).
    writeRegister(REG_PWR_MGMT_1, 0x80);
    delay(100);

    // CLKSEL = 1 (auto-select best available PLL), TEMP_DIS = 1, SLEEP = 0.
    writeRegister(REG_PWR_MGMT_1, 0x09);
    delay(10);

    // Enable all accel/gyro axes.
    writeRegister(REG_PWR_MGMT_2, 0x00);

    // Gyro DLPF_CFG = 1 (~184 Hz BW, 1 kHz internal sample rate).
    writeRegister(REG_CONFIG, 0x01);

    // Sample rate = internal_rate/(1+SMPLRT_DIV) = 1kHz/(1+2) = 333 Hz
    writeRegister(REG_SMPLRT_DIV, 0x02);

    // GYRO_FS_SEL = 0 => +-250 dps (131 LSB/dps).
    writeRegister(REG_GYRO_CONFIG, 0x00);

    // ACCEL_FS_SEL = 1 => +-4 g (8192 LSB/g).
    writeRegister(REG_ACCEL_CONFIG, 0x08);

    // Accel DLPF: A_DLPFCFG = 1 (~184 Hz), FCHOICE_B = 0.
    writeRegister(REG_ACCEL_CONFIG2, 0x01);

    calibrate();

}

void MPU9250::calibrate(){
    accel_bias_g = {};
    gyro_bias = {};

    constexpr int kSamples = 400;
    long sum_ax = 0;
    long sum_ay = 0;
    long sum_az = 0;
    long sum_gx = 0;
    long sum_gy = 0;
    long sum_gz = 0;
    int valid = 0;

    for (int i = 0; i < kSamples; ++i) {
        RawImuData sample{};
        if (!readBurstRaw(sample.accel, sample.gyro)) {
            continue;
        }
        sum_ax += sample.accel.x;
        sum_ay += sample.accel.y;
        sum_az += sample.accel.z;
        sum_gx += sample.gyro.x;
        sum_gy += sample.gyro.y;
        sum_gz += sample.gyro.z;
        ++valid;
        delay(2);
    }

    if (valid == 0) {
        return;
    }

    // Assume level during calibration: X/Y should be 0 g, Z should be +1 g.
    accel_bias_g.x = (sum_ax / (float)valid) / ACCEL_LSB_PER_G;
    accel_bias_g.y = (sum_ay / (float)valid) / ACCEL_LSB_PER_G;
    accel_bias_g.z = ((sum_az / (float)valid) / ACCEL_LSB_PER_G) - 1.0f;

    // Assume stationary during calibration: gyro should be 0 dps.
    gyro_bias.x = (sum_gx / (float)valid) / GYRO_LSB_PER_DPS;
    gyro_bias.y = (sum_gy / (float)valid) / GYRO_LSB_PER_DPS;
    gyro_bias.z = (sum_gz / (float)valid) / GYRO_LSB_PER_DPS;
}

bool MPU9250::read(ImuData &data) {
    if (!readBurstRaw(raw.accel, raw.gyro)) {
        return false;
    }

    data.gyro.x = (raw.gyro.x / GYRO_LSB_PER_DPS) - gyro_bias.x;
    data.gyro.y = (raw.gyro.y / GYRO_LSB_PER_DPS) - gyro_bias.y;
    data.gyro.z = (raw.gyro.z / GYRO_LSB_PER_DPS) - gyro_bias.z;

    data.accel.x = (raw.accel.x / ACCEL_LSB_PER_G) - accel_bias_g.x;
    data.accel.y = (raw.accel.y / ACCEL_LSB_PER_G) - accel_bias_g.y;
    data.accel.z = (raw.accel.z / ACCEL_LSB_PER_G) - accel_bias_g.z;
    return true;
}
