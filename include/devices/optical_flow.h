#ifndef DEVICE_OPTICAL_FLOW_H
#define DEVICE_OPTICAL_FLOW_H

#include "drivers/mtf02.h"
#include "interfaces.h"
#include "datastructs.h"
#include "lpf.h"

// In reality flow arrives late, often several ms behind the actual velocity 
// This constant needs to be tuned.
constexpr uint32_t FLOW_DELAY_US = 25000; //25ms 

class OpticalFlow {
public:
    explicit OpticalFlow(OpticalFlowDriver &driver);

    bool setup();
    bool kick();
    bool has_bytes() const;
    bool read(MTF02Data &out);

private:
    OpticalFlowDriver &driver_;
    Vec3 last_flow_radps_{0.0f, 0.0f, 0.0f};
    uint32_t last_flow_ms_ = 0;

    bool flow_reject_initialized_ = false;
    bool range_vz_initialized_ = false;
    float last_height_m_ = 0.015f;
    float last_range_vz = 0.0f;
    uint32_t last_range_update_us_ = 0;
};

inline Vec3WithTrust get_raw_flow_with_trust(const MTF02Data &flowdata, Vec3 gyro){
    constexpr float FLOW_RAD_PER_CENTIRAD = 0.01f;
    constexpr float FLOW_QUALITY_MIN = 20.0f;
    constexpr float FLOW_QUALITY_MAX = 255.0f;
    constexpr float GYRO_MULTIPLIER_MAX = 3.6f;

    const float range_m = 0.001f * static_cast<float>(flowdata.data.dist_mm);

    bool flow_ok =
        flowdata.data.dist_status == 1 &&
        flowdata.data.flow_status == 1 &&
        flowdata.data.flow_quality >= 20;

    if (!flow_ok)
    {
        return Vec3WithTrust{Vec3{}, Vec3{}};
    }

    if (range_m < 0.01f || range_m > 5.0f ||
        flowdata.data.flow_status != 1 ||
        flowdata.data.dist_status != 1)
    {
        return Vec3WithTrust{Vec3{}, Vec3{}};
    }

    // Optical flow in sensor angular rate units (rad/s).
    Vec3 flow_radps{
        static_cast<float>(flowdata.data.flow_x) * FLOW_RAD_PER_CENTIRAD,
        static_cast<float>(flowdata.data.flow_y) * FLOW_RAD_PER_CENTIRAD,
        0.0f};

    // flow_radps = flow_lpf.update(flow_radps); // legacy, no longer needs

    float q_norm = flowdata.data.flow_quality / 255.0f;
    q_norm = constrain(q_norm, 0.0f, 1.0f);

    // Low quality = low confidence = high scale
    float quality_scale = 1.0f / (0.70f + 0.30f * q_norm);

    constexpr float FULL_TRUST_H = 2.0f;
    constexpr float WEAK_TRUST_H = 4.0f;
    constexpr float MAX_HEIGHT_SCALE = 1.5f;

    float height_scale = 1.0f;

    if (range_m > FULL_TRUST_H)
    {
        float t = (range_m - FULL_TRUST_H) / (WEAK_TRUST_H - FULL_TRUST_H);
        t = constrain(t, 0.0f, 1.0f);

        // sigma grows smoothly from 1x to 4x.
        height_scale = 1.0f + t * (MAX_HEIGHT_SCALE - 1.0f);
    }

    // Per-axis gyro confidence.
    // Keep this moderate because ESKF already models rotation-induced flow.
    // x-flow is mainly affected by body y-rate; y-flow by body x-rate.
    constexpr float GYRO_REF = 3.2f;        // rad/s
    constexpr float GYRO_NOISE_GAIN = 1.5f;
    constexpr float MAX_GYRO_SCALE = 2.0f;

    float gyro_scale_x = 1.0f + GYRO_NOISE_GAIN * fabsf(gyro.y) / GYRO_REF;
    float gyro_scale_y = 1.0f + GYRO_NOISE_GAIN * (0.6*fabsf(gyro.x) + 0.4*fabs(gyro.z)) / GYRO_REF;

    gyro_scale_x = constrain(gyro_scale_x, 1.0f, MAX_GYRO_SCALE);
    gyro_scale_y = constrain(gyro_scale_y, 1.0f, MAX_GYRO_SCALE);

    Vec3 trust = {
        1.0f / (height_scale * gyro_scale_x * quality_scale),
        1.0f / (height_scale * gyro_scale_y * quality_scale),
        0.0f};

    return Vec3WithTrust{
        flow_radps,
        trust};
}

#endif
