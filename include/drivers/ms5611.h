#ifndef MS5611_H
#define MS5611_H

#include <Arduino.h>

struct BaroData {
    float temp_c = 0.0f;
    float pres_pa = 0.0f;
    float altitude_m = 0.0f;
};

class MS5611 {
public:
    // Factory PROM calibration words C1..C6 are stored in prom[1]..prom[6].
    uint16_t prom[8] = {0};
    bool is_ready = false;
    uint32_t last_update_us = micros();

    void setup();
    void calibrate();
    bool read(BaroData &data);
    void kick(BaroData &data);

private:
    enum class ReadState : uint8_t {
        kIdle,
        kWaitPressure,
        kWaitTemperature
    };

    bool reset();
    bool read_prom();
    bool start_conversion(uint8_t command);
    bool read_adc(uint32_t &value);
    void compute_compensation(BaroData &data);

    ReadState state = ReadState::kIdle;
    uint32_t conversion_started_us = 0;
    uint32_t raw_pressure = 0;
    uint32_t raw_temperature = 0;
    bool has_fresh_sample = false;
    uint8_t pressure_count_since_temp = 0;
    bool have_temperature_sample = false;
    float ground_pressure_pa = 101325.0f;
    float filtered_altitude_m = 0.0f;
    bool filter_initialized = false;
};

#endif
