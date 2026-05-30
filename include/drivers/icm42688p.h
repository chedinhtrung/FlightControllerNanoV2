#ifndef ICM42688P_H
#define ICM42688P_H

#include <Arduino.h>
#include "interfaces.h"

#define ICM42688P_CS_PIN 9

class ICM42688P : public ImuDriver
{
public:
    bool setup() override;
    bool read(ImuData &data) override;

private:
    enum class GyroFsSel : uint8_t {
        Dps2000 = 0,
        Dps1000 = 1,
        Dps500 = 2,
        Dps250 = 3,
        Dps125 = 4,
        Dps62_5 = 5,
        Dps31_25 = 6,
        Dps15_625 = 7,
    };

    enum class AccelFsSel : uint8_t {
        G16 = 0,
        G8 = 1,
        G4 = 2,
        G2 = 3,
    };

    // Datasheet section: USER BANK 0 register map.
    static constexpr uint8_t READ_MASK = 0x80;
    static constexpr uint8_t REG_DEVICE_CONFIG = 0x11;
    static constexpr uint8_t REG_DRIVE_CONFIG = 0x13;
    static constexpr uint8_t REG_PWR_MGMT0 = 0x4E;
    static constexpr uint8_t REG_GYRO_CONFIG0 = 0x4F;
    static constexpr uint8_t REG_ACCEL_CONFIG0 = 0x50;
    static constexpr uint8_t REG_WHO_AM_I = 0x75;
    static constexpr uint8_t REG_BANK_SEL = 0x76;
    static constexpr uint8_t REG_ACCEL_DATA_X1 = 0x1F;

    static constexpr uint8_t WHO_AM_I_EXPECTED = 0x47;
    static constexpr uint8_t DEVICE_SOFT_RESET = 0x01;
    static constexpr uint8_t DRIVE_CONFIG_DEFAULT = 0x05;
    static constexpr uint8_t PWR_MGMT0_LN_6AXIS = 0x0F;

    // ODR encoding from datasheet for register bits [3:0].
    static constexpr uint8_t ODR_500HZ = 0x0F;

    // Chosen UI output configuration.
    static constexpr GyroFsSel GYRO_FS_SEL = GyroFsSel::Dps250;
    static constexpr AccelFsSel ACCEL_FS_SEL = AccelFsSel::G4;

    RawImuData raw_{};
    float gyro_dps_per_lsb_ = 0.0f;
    float accel_g_per_lsb_ = 0.0f;

    static uint8_t makeGyroConfig0(GyroFsSel fs_sel, uint8_t odr_code);
    static uint8_t makeAccelConfig0(AccelFsSel fs_sel, uint8_t odr_code);
    static float gyroLsbPerDps(GyroFsSel fs_sel);
    static float accelLsbPerG(AccelFsSel fs_sel);

    uint8_t readRegister(uint8_t reg);
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readBurst(uint8_t start_reg, uint8_t *buffer, size_t len);
};

#endif
