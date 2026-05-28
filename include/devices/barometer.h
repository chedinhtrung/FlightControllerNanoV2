#ifndef DEVICE_BAROMETER_H
#define DEVICE_BAROMETER_H

#include <Arduino.h>

#include "drivers/ms5611.h"
#include "interfaces.h"
#include "lpf.h"

constexpr uint32_t BARO_ALT_HZ = 60;
constexpr uint32_t BARO_ALT_PERIOD_US = 1000000UL / BARO_ALT_HZ;
constexpr float BARO_ALT_FILTER_ALPHA = 0.3f;

class Barometer {
public:
    explicit Barometer(BarometerDriver &driver, uint32_t period_us = BARO_ALT_PERIOD_US);

    bool setup();
    void kick();
    bool read(BaroData &out);
    void reset();

private:
    bool calibrate();

    BarometerDriver &driver_;
    uint32_t period_us_ = 0;
    uint32_t last_kick_us_ = 0;
    float sea_level_pressure_pa_ = 101325.0f;
    float altitude_zero_m_ = 0.0f;
    LPF altitude_lpf_{BARO_ALT_FILTER_ALPHA};
    
};

#endif
