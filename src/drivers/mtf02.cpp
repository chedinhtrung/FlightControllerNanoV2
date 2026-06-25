#include "drivers/mtf02.h"

MTF02::MTF02() {}

bool MTF02::setup() {
    serial_.begin(baud_);
    reset_parser();
    return true;
}

bool MTF02::parse() {
    if (serial_.available() <= 0) {
        return false;
    }
    const uint8_t byte_in = (uint8_t)serial_.read();
    parse_char(byte_in);
    return true;
}

bool MTF02::has_bytes() {
    return serial_.available() > 0;
}

bool MTF02::read(MTF02Data &out) {
    if (!has_new_sample_) {
        return false;
    }
    out = flow_data_;
    has_new_sample_ = false;
    return true;
}

bool MTF02::parse_char(uint8_t byte_in) {
    switch (status_) {
        case 0:
            if (byte_in == kHead) {
                frame_start_us_ = micros();
                status_ = 1;
            }
            break;
        case 1:
            dev_id_ = byte_in;
            status_ = 2;
            break;
        case 2:
            sys_id_ = byte_in;
            status_ = 3;
            break;
        case 3:
            msg_id_ = byte_in;
            status_ = 4;
            break;
        case 4:
            seq_ = byte_in;
            status_ = 5;
            break;
        case 5:
            len_ = byte_in;
            if (len_ == 0) {
                status_ = 7;
            } else if (len_ > kMaxPayloadLen) {
                reset_parser();
            } else {
                status_ = 6;
            }
            break;
        case 6:
            payload_[payload_cnt_++] = byte_in;
            if (payload_cnt_ == len_) {
                payload_cnt_ = 0;
                status_ = 7;
            }
            break;
        case 7:
            checksum_ = byte_in;
            if (checksum_ok()) {
                const bool decoded = decode_current_frame();
                reset_parser();
                return decoded;
            }
            reset_parser();
            break;
        default:
            reset_parser();
            break;
    }

    return false;
}

bool MTF02::decode_current_frame() {
    if (msg_id_ != kRangeMsgId || len_ < sizeof(MTF02Payload)) {
        return false;
    }

    // Payload layout is exactly the MicoAir frame payload.
    memcpy(&flow_data_.data, payload_, sizeof(MTF02Payload));
    flow_data_.timestamp = frame_start_us_;
    has_new_sample_ = true;
    return true;
}

bool MTF02::checksum_ok() const {
    uint8_t checksum = 0;
    checksum += kHead;
    checksum += dev_id_;
    checksum += sys_id_;
    checksum += msg_id_;
    checksum += seq_;
    checksum += len_;
    for (uint8_t i = 0; i < len_; ++i) {
        checksum += payload_[i];
    }
    return checksum == checksum_;
}

void MTF02::reset_parser() {
    status_ = 0;
    payload_cnt_ = 0;
    len_ = 0;
    frame_start_us_ = 0;
}
