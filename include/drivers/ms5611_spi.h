#ifndef MS5611_SPI_H
#define MS5611_SPI_H

#include <Arduino.h>
#include "interfaces.h"

constexpr uint8_t MS5611_SPI_CS_PIN = 10;

class MS5611SPI : public BarometerDriver {
public:
    uint16_t prom[8] = {0};
    bool is_ready = false;

    bool setup() override;
    bool read(BaroData &data) override;
    void kick() override;

private:
    enum class ReadState : uint8_t {
        Idle,
        WaitPressure,
        WaitTemp
    };

    bool reset();
    bool read_prom();
    bool start_conversion(uint8_t command);
    bool read_adc(uint32_t &value);
    void compute_compensation(BaroData &data);

    ReadState state = ReadState::Idle;
    uint32_t conversion_started_us = 0;
    uint32_t raw_pressure = 0;
    uint32_t raw_temperature = 0;
    bool has_fresh_sample = false;
    uint8_t pressure_count_since_temp = 0;
    bool have_temperature_sample = false;
};

#endif
