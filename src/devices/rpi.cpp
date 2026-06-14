#include "devices/rpi.h"

RPi::RPi(HardwareSerial &serial) : serial_(serial) {}

void RPi::setup(uint32_t baud) {
    serial_.begin(baud);
    last_publish_us_ = 0;
}

bool RPi::write(RPiMessageType type, const uint8_t *payload, uint8_t len) {
    if (!can_publish()) {
        return false;
    }

    const uint8_t *safe_payload = payload;
    uint8_t safe_len = len;
    if (safe_payload == nullptr) {
        safe_len = 0;
    }

    const uint8_t type_byte = static_cast<uint8_t>(type);
    const uint8_t cks = checksum(type, safe_len, safe_payload);

    size_t written = 0;
    written += serial_.write(kMagic);
    written += serial_.write(type_byte);
    written += serial_.write(safe_len);
    if (safe_len > 0) {
        written += serial_.write(safe_payload, safe_len);
    }
    written += serial_.write(cks);

    const bool success = written == static_cast<size_t>(safe_len) + 4U;
    if (success) {
        last_publish_us_ = micros();
    }
    return success;
}

uint8_t RPi::checksum(RPiMessageType type, uint8_t len, const uint8_t *payload) const {
    uint8_t cks = kMagic ^ static_cast<uint8_t>(type) ^ len;
    for (uint8_t i = 0; i < len; ++i) {
        cks ^= payload[i];
    }
    return cks;
}

bool RPi::can_publish() {
    if (PUBLISH_RATE_HZ <= 0) {
        return true;
    }

    constexpr uint32_t kMicrosPerSecond = 1000000UL;
    const uint32_t publish_period_us = kMicrosPerSecond / PUBLISH_RATE_HZ;
    const uint32_t now_us = micros();

    if (last_publish_us_ == 0) {
        return true;
    }

    return (uint32_t)(now_us - last_publish_us_) >= publish_period_us;
}
