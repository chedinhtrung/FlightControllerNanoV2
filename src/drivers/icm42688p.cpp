#include "drivers/icm42688p.h"
#include <SPI.h>

namespace
{
    const SPISettings kImuSpiSettings(4000000, MSBFIRST, SPI_MODE0);

    inline int16_t toInt16(uint8_t msb, uint8_t lsb)
    {
        return (int16_t)((msb << 8) | lsb);
    }
}

uint8_t ICM42688P::makeGyroConfig0(GyroFsSel fs_sel, uint8_t odr_code)
{
    return (((uint8_t)fs_sel & 0x07) << 5) | (odr_code & 0x0F);
}

uint8_t ICM42688P::makeAccelConfig0(AccelFsSel fs_sel, uint8_t odr_code)
{
    return (((uint8_t)fs_sel & 0x07) << 5) | (odr_code & 0x0F);
}

float ICM42688P::gyroLsbPerDps(GyroFsSel fs_sel)
{
    switch (fs_sel)
    {
    case GyroFsSel::Dps2000:
        return 16.4f;
    case GyroFsSel::Dps1000:
        return 32.8f;
    case GyroFsSel::Dps500:
        return 65.5f;
    case GyroFsSel::Dps250:
        return 131.0f;
    case GyroFsSel::Dps125:
        return 262.0f;
    case GyroFsSel::Dps62_5:
        return 524.3f;
    case GyroFsSel::Dps31_25:
        return 1048.6f;
    case GyroFsSel::Dps15_625:
        return 2097.2f;
    default:
        return 131.0f;
    }
}

float ICM42688P::accelLsbPerG(AccelFsSel fs_sel)
{
    switch (fs_sel)
    {
    case AccelFsSel::G16:
        return 2048.0f;
    case AccelFsSel::G8:
        return 4096.0f;
    case AccelFsSel::G4:
        return 8192.0f;
    case AccelFsSel::G2:
        return 16384.0f;
    default:
        return 8192.0f;
    }
}

uint8_t ICM42688P::readRegister(uint8_t reg)
{
    SPI.beginTransaction(kImuSpiSettings);
    digitalWrite(ICM42688P_CS_PIN, LOW);
    SPI.transfer(reg | READ_MASK);
    const uint8_t value = SPI.transfer(0x00);
    digitalWrite(ICM42688P_CS_PIN, HIGH);
    SPI.endTransaction();
    return value;
}

bool ICM42688P::writeRegister(uint8_t reg, uint8_t value)
{
    SPI.beginTransaction(kImuSpiSettings);
    digitalWrite(ICM42688P_CS_PIN, LOW);
    SPI.transfer(reg & ~READ_MASK);
    SPI.transfer(value);
    digitalWrite(ICM42688P_CS_PIN, HIGH);
    SPI.endTransaction();
    return true;
}

bool ICM42688P::readBurst(uint8_t start_reg, uint8_t *buffer, size_t len)
{
    SPI.beginTransaction(kImuSpiSettings);
    digitalWrite(ICM42688P_CS_PIN, LOW);
    SPI.transfer(start_reg | READ_MASK);
    for (size_t i = 0; i < len; ++i)
    {
        buffer[i] = SPI.transfer(0x00);
    }
    digitalWrite(ICM42688P_CS_PIN, HIGH);
    SPI.endTransaction();
    return true;
}

bool ICM42688P::setup()
{
    SPI.begin();
    pinMode(ICM42688P_CS_PIN, OUTPUT);
    digitalWrite(ICM42688P_CS_PIN, HIGH);
    delay(5);

    // Ensure we are in USER BANK 0.
    writeRegister(REG_BANK_SEL, 0x00);

    // Software reset (datasheet requires >=1ms wait before further access).
    writeRegister(REG_DEVICE_CONFIG, DEVICE_SOFT_RESET);
    delay(2);

    // Reset also returns to bank 0, but write explicitly for safety.
    writeRegister(REG_BANK_SEL, 0x00);

    if (readRegister(REG_WHO_AM_I) != WHO_AM_I_EXPECTED)
    {
        return false;
    }

    // Keep default drive config from datasheet reset value.
    writeRegister(REG_DRIVE_CONFIG, DRIVE_CONFIG_DEFAULT);

    // Set gyro + accel to low-noise mode.
    writeRegister(REG_PWR_MGMT0, PWR_MGMT0_LN_6AXIS);
    delay(1);

    const uint8_t gyro_cfg0 = makeGyroConfig0(GYRO_FS_SEL, ODR_500HZ);
    const uint8_t accel_cfg0 = makeAccelConfig0(ACCEL_FS_SEL, ODR_500HZ);

    writeRegister(REG_GYRO_CONFIG0, gyro_cfg0);
    writeRegister(REG_ACCEL_CONFIG0, accel_cfg0);
    delay(1);

    // Cache scale factors that exactly match configured full-scale ranges.
    gyro_dps_per_lsb_ = 1.0f / gyroLsbPerDps(GYRO_FS_SEL);
    accel_g_per_lsb_ = 1.0f / accelLsbPerG(ACCEL_FS_SEL);

    // Basic sanity check to catch register write/config mismatch early.
    if (readRegister(REG_GYRO_CONFIG0) != gyro_cfg0)
    {
        return false;
    }
    if (readRegister(REG_ACCEL_CONFIG0) != accel_cfg0)
    {
        return false;
    }

    return true;
}

bool ICM42688P::read(ImuData &data)
{
    uint8_t burst[12] = {0};
    if (!readBurst(REG_ACCEL_DATA_X1, burst, sizeof(burst)))
    {
        return false;
    }
    data.timestamp = micros();
    raw_.accel.x = toInt16(burst[0], burst[1]);
    raw_.accel.y = toInt16(burst[2], burst[3]);
    raw_.accel.z = toInt16(burst[4], burst[5]);
    raw_.gyro.x = toInt16(burst[6], burst[7]);
    raw_.gyro.y = toInt16(burst[8], burst[9]);
    raw_.gyro.z = toInt16(burst[10], burst[11]);

    data.gyro = {
        raw_.gyro.x * gyro_dps_per_lsb_,
        raw_.gyro.y * gyro_dps_per_lsb_,
        raw_.gyro.z * gyro_dps_per_lsb_};
    data.accel = {
        raw_.accel.x * accel_g_per_lsb_,
        raw_.accel.y * accel_g_per_lsb_,
        raw_.accel.z * accel_g_per_lsb_};

    return true;
}
