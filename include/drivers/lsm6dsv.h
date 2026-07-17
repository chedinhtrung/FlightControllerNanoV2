#ifndef LSM6DSV16X_H
#define LSM6DSV16X_H

#include <Arduino.h>
#include <SPI.h>

#include "interfaces.h"

#define LSM6DSV_CS_PIN   PB1
#define LSM6DSV_SCK_PIN  PE12
#define LSM6DSV_MISO_PIN PE13
#define LSM6DSV_MOSI_PIN PE14

class LSM6DSV : public ImuDriver
{
public:
    bool setup() override;
    bool read(ImuData &data) override;

private:
    SPIClass spi_ =
        SPIClass(LSM6DSV_MOSI_PIN,
                 LSM6DSV_MISO_PIN,
                 LSM6DSV_SCK_PIN);

    static constexpr uint8_t READ_MASK = 0x80;

    static constexpr uint8_t REG_WHO_AM_I = 0x0F;
    static constexpr uint8_t REG_CTRL1_XL = 0x10;
    static constexpr uint8_t REG_CTRL2_G = 0x11;
    static constexpr uint8_t REG_CTRL3_C = 0x12;
    static constexpr uint8_t REG_CTRL6_G = 0x15;
    static constexpr uint8_t REG_CTRL8_XL = 0x17;
    static constexpr uint8_t REG_OUTX_L_G = 0x22;

    static constexpr uint8_t WHO_AM_I_EXPECTED = 0x70;

    // CTRL3 bits
    static constexpr uint8_t CTRL3_SW_RESET = 0x01;
    static constexpr uint8_t CTRL3_IF_INC = 0x04;
    static constexpr uint8_t CTRL3_BDU = 0x40;

    // High-performance mode is OP_MODE = 000.
    // ODR code 1000 selects 480 Hz.
    static constexpr uint8_t CTRL1_XL_480HZ = 0x08;
    static constexpr uint8_t CTRL2_G_480HZ = 0x08;

    // CTRL6 FS_G[3:0] = 0001 selects +/-250 dps.
    static constexpr uint8_t CTRL6_G_250DPS = 0x01;

    // CTRL8 FS_XL[1:0] = 01 selects +/-4 g.
    static constexpr uint8_t CTRL8_XL_4G = 0x01;

    RawImuData raw_{};
    float gyro_dps_per_lsb_ = 0.0f;
    float accel_g_per_lsb_ = 0.0f;

    uint8_t readRegister(uint8_t reg);
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readBurst(uint8_t start_reg, uint8_t *buffer, size_t len);
};

#endif