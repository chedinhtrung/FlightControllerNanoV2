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
constexpr float GYRO_DPS_PER_LSB  = 1.0f / GYRO_LSB_PER_DPS;
constexpr float ACCEL_G_PER_LSB   = 1.0f / ACCEL_LSB_PER_G;

const Vect ORIENTATION_SIGN = {IMU_MAP_X_SIGN, IMU_MAP_Y_SIGN, IMU_MAP_Z_SIGN};

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

    constexpr int kSamples = 1000;
    long sum_gx = 0;
    long sum_gy = 0;
    long sum_gz = 0;
    int valid = 0;

    for (int i = 0; i < kSamples; ++i) {
        RawImuData sample{};
        if (!readBurstRaw(sample.accel, sample.gyro)) {
            continue;
        }
        sum_gx += sample.gyro.x;
        sum_gy += sample.gyro.y;
        sum_gz += sample.gyro.z;
        ++valid;
        delay(4);
    }

    if (valid == 0) {
        return;
    }

    // Assume stationary during calibration: gyro should be 0 dps.
    gyro_bias.x = (sum_gx / (float)valid) / GYRO_LSB_PER_DPS;
    gyro_bias.y = (sum_gy / (float)valid) / GYRO_LSB_PER_DPS;
    gyro_bias.z = (sum_gz / (float)valid) / GYRO_LSB_PER_DPS;
}

bool MPU9250::read(ImuData &data) {
    if (!readBurstRaw(raw.accel, raw.gyro)) {
        return false;
    }

    Vect gyro_base = {
        raw.gyro.x * GYRO_DPS_PER_LSB,
        raw.gyro.y * GYRO_DPS_PER_LSB,
        raw.gyro.z * GYRO_DPS_PER_LSB
    };

    Vect accel_base = {
        raw.accel.x * ACCEL_G_PER_LSB,
        raw.accel.y * ACCEL_G_PER_LSB,
        raw.accel.z * ACCEL_G_PER_LSB
    };

    gyro_base = (gyro_base - gyro_bias);
    accel_base = (accel_base - accel_bias_g);

    const float gyro_arr[3] = {gyro_base.x, gyro_base.y, gyro_base.z};
    const float accel_arr[3] = {accel_base.x, accel_base.y, accel_base.z};

    data.gyro = {
        gyro_arr[IMU_MAP_X_SRC],
        gyro_arr[IMU_MAP_Y_SRC],
        gyro_arr[IMU_MAP_Z_SRC]
    };
    data.accel = {
        accel_arr[IMU_MAP_X_SRC],
        accel_arr[IMU_MAP_Y_SRC],
        accel_arr[IMU_MAP_Z_SRC]
    };

    data.accel *= ORIENTATION_SIGN;
    data.gyro *= ORIENTATION_SIGN;
    return true;
}
