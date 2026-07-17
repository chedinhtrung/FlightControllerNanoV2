#include "drivers/lsm6dsv.h"

namespace
{
    const SPISettings kImuSpiSettings(4000000, MSBFIRST, SPI_MODE3);

    inline int16_t toInt16(uint8_t lsb, uint8_t msb)
    {
        return static_cast<int16_t>(
            (static_cast<uint16_t>(msb) << 8) |
            static_cast<uint16_t>(lsb));
    }
}

uint8_t LSM6DSV::readRegister(uint8_t reg)
{
    spi_.beginTransaction(kImuSpiSettings);
    digitalWrite(LSM6DSV_CS_PIN, LOW);

    spi_.transfer(reg | READ_MASK);
    const uint8_t value = spi_.transfer(0x00);

    digitalWrite(LSM6DSV_CS_PIN, HIGH);
    spi_.endTransaction();

    return value;
}

bool LSM6DSV::writeRegister(uint8_t reg, uint8_t value)
{
    spi_.beginTransaction(kImuSpiSettings);
    digitalWrite(LSM6DSV_CS_PIN, LOW);

    spi_.transfer(reg & static_cast<uint8_t>(~READ_MASK));
    spi_.transfer(value);

    digitalWrite(LSM6DSV_CS_PIN, HIGH);
    spi_.endTransaction();

    return true;
}

bool LSM6DSV::readBurst(uint8_t start_reg, uint8_t *buffer, size_t len)
{
    if (buffer == nullptr || len == 0)
    {
        return false;
    }

    spi_.beginTransaction(kImuSpiSettings);
    digitalWrite(LSM6DSV_CS_PIN, LOW);

    spi_.transfer(start_reg | READ_MASK);
    for (size_t i = 0; i < len; ++i)
    {
        buffer[i] = spi_.transfer(0x00);
    }

    digitalWrite(LSM6DSV_CS_PIN, HIGH);
    spi_.endTransaction();

    return true;
}

bool LSM6DSV::setup()
{
    pinMode(LSM6DSV_CS_PIN, OUTPUT);
    digitalWrite(LSM6DSV_CS_PIN, HIGH);

    spi_.begin();
    delay(5);

    if (readRegister(REG_WHO_AM_I) != WHO_AM_I_EXPECTED)
    {
        return false;
    }

    // Reset the device.
    writeRegister(REG_CTRL3_C, CTRL3_SW_RESET);

    // SW_RESET is cleared by the sensor when reset has completed.
    const uint32_t reset_start_ms = millis();
    while ((readRegister(REG_CTRL3_C) & CTRL3_SW_RESET) != 0)
    {
        if ((millis() - reset_start_ms) > 100)
        {
            return false;
        }
        delay(1);
    }

    // Enable contiguous multi-byte reads and prevent torn samples.
    writeRegister(REG_CTRL3_C, CTRL3_IF_INC | CTRL3_BDU);

    // Full-scale selection is separate from ODR on the LSM6DSV.
    writeRegister(REG_CTRL6_G, CTRL6_G_250DPS);
    writeRegister(REG_CTRL8_XL, CTRL8_XL_4G);

    // Enable gyro and accelerometer at 480 Hz in high-performance mode.
    writeRegister(REG_CTRL2_G, CTRL2_G_480HZ);
    writeRegister(REG_CTRL1_XL, CTRL1_XL_480HZ);

    // Datasheet specifies up to 30 ms gyroscope turn-on time.
    delay(35);

    if (readRegister(REG_CTRL3_C) != (CTRL3_IF_INC | CTRL3_BDU))
    {
        return false;
    }

    if (readRegister(REG_CTRL2_G) != CTRL2_G_480HZ)
    {
        return false;
    }

    if (readRegister(REG_CTRL1_XL) != CTRL1_XL_480HZ)
    {
        return false;
    }

    if ((readRegister(REG_CTRL6_G) & 0x0F) != CTRL6_G_250DPS)
    {
        return false;
    }

    if ((readRegister(REG_CTRL8_XL) & 0x03) != CTRL8_XL_4G)
    {
        return false;
    }

    gyro_dps_per_lsb_ = 8.75e-3f;  // 8.75 mdps/LSB at +/-250 dps.
    accel_g_per_lsb_ = 0.122e-3f;  // 0.122 mg/LSB at +/-4 g.

    return true;
}

bool LSM6DSV::read(ImuData &data)
{
    uint8_t burst[12] = {0};

    // 0x22..0x27: gyro XYZ
    // 0x28..0x2D: accelerometer XYZ
    if (!readBurst(REG_OUTX_L_G, burst, sizeof(burst)))
    {
        return false;
    }

    data.timestamp = micros();

    raw_.gyro.x = toInt16(burst[0], burst[1]);
    raw_.gyro.y = toInt16(burst[2], burst[3]);
    raw_.gyro.z = toInt16(burst[4], burst[5]);

    raw_.accel.x = toInt16(burst[6], burst[7]);
    raw_.accel.y = toInt16(burst[8], burst[9]);
    raw_.accel.z = toInt16(burst[10], burst[11]);

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