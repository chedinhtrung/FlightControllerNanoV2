#ifndef MESSAGE_HELPERS_H
#define MESSAGE_HELPERS_H

#include <cstdint>
#include <cstring>

#ifdef BUILT_ON_PI
struct Vec3 { float x, y, z; };
struct Quaternion { float x, y, z, w; };
struct NominalState {
    Vec3 p;
    Vec3 v;
    Quaternion q;
    Vec3 ab;
    Vec3 wb;
};

enum class RPiMessageType : uint8_t {
    STATE = 0x01,
    LABELED_SCALAR = 0x02,
    LABELED_VEC3 = 0x03,
    STRING = 0x04,
    COMMAND = 0x05,
};
#else

#include "eskf.h"
#include "devices/rpi.h"

#endif

#include <math.h>

constexpr uint8_t ESKF_STATE_PAYLOAD_VERSION = 1;
constexpr uint8_t RPI_MESSAGE_MAGIC = 0xAA;
constexpr uint8_t RPI_MAX_PAYLOAD_LEN = 255;
constexpr uint8_t RPI_MAX_LABEL_LEN = 32;

struct __attribute__((packed)) ESKFStatePayload {
    uint32_t timestamp_us = 0;

    float px_m = 0.0f;
    float py_m = 0.0f;
    float pz_m = 0.0f;

    int16_t vx_mmps = 0;
    int16_t vy_mmps = 0;
    int16_t vz_mmps = 0;

    int16_t qx_q15 = 0;
    int16_t qy_q15 = 0;
    int16_t qz_q15 = 0;
    int16_t qw_q15 = 0;

    int16_t abx_mmps2 = 0;
    int16_t aby_mmps2 = 0;
    int16_t abz_mmps2 = 0;

    int16_t wbx_uradps = 0;
    int16_t wby_uradps = 0;
    int16_t wbz_uradps = 0;

    float h_terrain_m = 0.0f;
};

static_assert(sizeof(ESKFStatePayload) == 46, "ESKFStatePayload must stay tightly packed");

struct __attribute__((packed)) LabelledVec3Payload {
    int16_t x_mm = 0;
    int16_t y_mm = 0;
    int16_t z_mm = 0;
    char label[RPI_MAX_LABEL_LEN] = {0};
};

static_assert(sizeof(LabelledVec3Payload) == 38, "LabelledVec3Payload must stay tightly packed");

inline int16_t quantize_signed(float value, float scale, int16_t min_value = INT16_MIN, int16_t max_value = INT16_MAX)
{
    long scaled = lroundf(value * scale);
    if (scaled < (long)min_value)
    {
        return min_value;
    }
    if (scaled > (long)max_value)
    {
        return max_value;
    }
    return (int16_t)scaled;
}

inline int16_t quantize_unit_q15(float value)
{
    if (value <= -1.0f)
    {
        return INT16_MIN;
    }
    if (value >= 1.0f)
    {
        return INT16_MAX;
    }
    return quantize_signed(value, 32767.0f);
}

inline float dequantize_signed(int16_t value, float scale)
{
    return ((float)value) / scale;
}

inline float dequantize_unit_q15(int16_t value)
{
    if (value == INT16_MIN)
    {
        return -1.0f;
    }
    return dequantize_signed(value, 32767.0f);
}

inline ESKFStatePayload pack(const NominalState &state, uint32_t timestamp_us, float h_terrain_m)
{
    ESKFStatePayload payload{};
    payload.timestamp_us = timestamp_us;

    payload.px_m = state.p.x;
    payload.py_m = state.p.y;
    payload.pz_m = state.p.z;

    // Velocity in mm/s gives +/-32.7 m/s range with 1 mm/s resolution
    payload.vx_mmps = quantize_signed(state.v.x, 1000.0f);
    payload.vy_mmps = quantize_signed(state.v.y, 1000.0f);
    payload.vz_mmps = quantize_signed(state.v.z, 1000.0f);

    // Quaternion 
    payload.qx_q15 = quantize_unit_q15(state.q.x);
    payload.qy_q15 = quantize_unit_q15(state.q.y);
    payload.qz_q15 = quantize_unit_q15(state.q.z);
    payload.qw_q15 = quantize_unit_q15(state.q.w);

    // Accel bias in milli-m/s^2 
    payload.abx_mmps2 = quantize_signed(state.ab.x, 1000.0f);
    payload.aby_mmps2 = quantize_signed(state.ab.y, 1000.0f);
    payload.abz_mmps2 = quantize_signed(state.ab.z, 1000.0f);

    // Gyro bias in urad/s 
    payload.wbx_uradps = quantize_signed(state.wb.x, 1000000.0f);
    payload.wby_uradps = quantize_signed(state.wb.y, 1000000.0f);
    payload.wbz_uradps = quantize_signed(state.wb.z, 1000000.0f);

    payload.h_terrain_m = h_terrain_m;
    return payload;
}

inline NominalState unpack(const ESKFStatePayload &payload)
{
    NominalState state{};

    state.p.x = payload.px_m;
    state.p.y = payload.py_m;
    state.p.z = payload.pz_m;

    state.v.x = dequantize_signed(payload.vx_mmps, 1000.0f);
    state.v.y = dequantize_signed(payload.vy_mmps, 1000.0f);
    state.v.z = dequantize_signed(payload.vz_mmps, 1000.0f);

    state.q.x = dequantize_unit_q15(payload.qx_q15);
    state.q.y = dequantize_unit_q15(payload.qy_q15);
    state.q.z = dequantize_unit_q15(payload.qz_q15);
    state.q.w = dequantize_unit_q15(payload.qw_q15);

    state.ab.x = dequantize_signed(payload.abx_mmps2, 1000.0f);
    state.ab.y = dequantize_signed(payload.aby_mmps2, 1000.0f);
    state.ab.z = dequantize_signed(payload.abz_mmps2, 1000.0f);

    state.wb.x = dequantize_signed(payload.wbx_uradps, 1000000.0f);
    state.wb.y = dequantize_signed(payload.wby_uradps, 1000000.0f);
    state.wb.z = dequantize_signed(payload.wbz_uradps, 1000000.0f);

    return state;
}

inline float unpack_h_terrain_m(const ESKFStatePayload &payload)
{
    return payload.h_terrain_m;
}

inline const uint8_t *payload_bytes(const ESKFStatePayload &payload)
{
    return reinterpret_cast<const uint8_t *>(&payload);
}

inline LabelledVec3Payload pack(const Vec3 &value, const char *label)
{
    LabelledVec3Payload payload{};
    payload.x_mm = quantize_signed(value.x, 1000.0f);
    payload.y_mm = quantize_signed(value.y, 1000.0f);
    payload.z_mm = quantize_signed(value.z, 1000.0f);

    if (label != nullptr)
    {
        strncpy(payload.label, label, RPI_MAX_LABEL_LEN - 1);
        payload.label[RPI_MAX_LABEL_LEN - 1] = '\0';
    }

    return payload;
}

inline Vec3 unpack(const LabelledVec3Payload &payload)
{
    return {
        dequantize_signed(payload.x_mm, 1000.0f),
        dequantize_signed(payload.y_mm, 1000.0f),
        dequantize_signed(payload.z_mm, 1000.0f)};
}

inline const char *unpack_label(const LabelledVec3Payload &payload)
{
    return payload.label;
}

inline const uint8_t *payload_bytes(const LabelledVec3Payload &payload)
{
    return reinterpret_cast<const uint8_t *>(&payload);
}

struct RPiFrameParser {
    enum class State : uint8_t {
        WAIT_MAGIC = 0,
        READ_TYPE = 1,
        READ_LEN = 2,
        READ_PAYLOAD = 3,
        READ_CHECKSUM = 4
    };

    State state = State::WAIT_MAGIC;
    RPiMessageType type = RPiMessageType::STATE;
    uint8_t len = 0;
    uint8_t payload[RPI_MAX_PAYLOAD_LEN] = {0};
    uint8_t payload_idx = 0;
    uint8_t checksum_accum = 0;

    inline void reset()
    {
        state = State::WAIT_MAGIC;
        len = 0;
        payload_idx = 0;
        checksum_accum = 0;
    }

    inline bool parse_byte(uint8_t byte_in)
    {
        switch (state)
        {
        case State::WAIT_MAGIC:
            if (byte_in == RPI_MESSAGE_MAGIC)
            {
                checksum_accum = byte_in;
                payload_idx = 0;
                len = 0;
                state = State::READ_TYPE;
            }
            return false;

        case State::READ_TYPE:
            type = static_cast<RPiMessageType>(byte_in);
            checksum_accum ^= byte_in;
            state = State::READ_LEN;
            return false;

        case State::READ_LEN:
            len = byte_in;
            checksum_accum ^= byte_in;

            if (len > RPI_MAX_PAYLOAD_LEN)
            {
                reset();
                return false;
            }

            if (len == 0)
            {
                state = State::READ_CHECKSUM;
                return false;
            }

            state = State::READ_PAYLOAD;
            return false;

        case State::READ_PAYLOAD:
            payload[payload_idx++] = byte_in;
            checksum_accum ^= byte_in;

            if (payload_idx == len)
            {
                state = State::READ_CHECKSUM;
            }
            return false;

        case State::READ_CHECKSUM:
        {
            const bool checksum_ok = checksum_accum == byte_in;
            state = State::WAIT_MAGIC;
            payload_idx = 0;
            checksum_accum = 0;
            return checksum_ok;
        }
        }

        reset();
        return false;
    }

    template <typename PayloadT>
    inline bool unpack_payload(PayloadT &payload_out) const
    {
        if (len != sizeof(PayloadT))
        {
            return false;
        }

        memcpy(&payload_out, payload, sizeof(PayloadT));
        return true;
    }
};

#endif
