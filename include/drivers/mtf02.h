#ifndef MTF02_H
#define MTF02_H

#include <Arduino.h>
#include "interfaces.h"

struct MTF02Payload {
    uint32_t time_ms = 0;
    uint32_t dist_mm = 0;
    uint8_t strength = 0;
    uint8_t precision = 0;
    uint8_t dist_status = 0;
    int16_t flow_x = 0;
    int16_t flow_y = 0;
    uint8_t flow_quality = 0;
    uint8_t flow_status = 0;
};

struct MTF02Data{
    MTF02Payload data; 
    uint32_t timestamp;
};

class MTF02 : public OpticalFlowDriver {
public:
    explicit MTF02(HardwareSerial &serial);

    bool setup() override;
    bool parse() override;
    bool has_bytes() const override;
    bool read(MTF02Payload &out) override;

private:
    static constexpr uint8_t kHead = 0xEF;
    static constexpr uint8_t kRangeMsgId = 0x51;
    static constexpr uint8_t kMaxPayloadLen = 64;

    HardwareSerial &serial_;
    uint32_t baud_ = 115200;
    MTF02Payload flow_data_{};
    bool has_new_sample_ = false;

    uint8_t status_ = 0;
    uint8_t dev_id_ = 0;
    uint8_t sys_id_ = 0;
    uint8_t msg_id_ = 0;
    uint8_t seq_ = 0;
    uint8_t len_ = 0;
    uint8_t checksum_ = 0;
    uint8_t payload_[kMaxPayloadLen] = {0};
    uint8_t payload_cnt_ = 0;

    bool parse_char(uint8_t byte_in);
    bool decode_current_frame();
    bool checksum_ok() const;
    void reset_parser();
};

#endif
