#ifndef DEVICE_RPI_H
#define DEVICE_RPI_H

#include <Arduino.h>

#define RPI_SERIAL Serial2
#define PUBLISH_RATE_HZ 50    // Messages should be written at max this rate.

enum class RPiMessageType : uint8_t {
    STATE = 0x01,
    LABELED_SCALAR = 0x02,
    LABELED_VEC3 = 0x03,
    STRING = 0x04,
    COMMAND = 0x05,
};

class RPi {
public:
    explicit RPi(HardwareSerial &serial = RPI_SERIAL);

    void setup(uint32_t baud = 460800);
    bool write(RPiMessageType type, const uint8_t *payload, uint8_t len);

private:
    static constexpr uint8_t kMagic = 0xAA;

    uint8_t checksum(RPiMessageType type, uint8_t len, const uint8_t *payload) const;
    bool can_publish();

    HardwareSerial &serial_;
    uint32_t last_publish_us_ = 0;
};

#endif
