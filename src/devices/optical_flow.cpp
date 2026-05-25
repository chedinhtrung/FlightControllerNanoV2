#include "devices/optical_flow.h"
#include "config.h"
#include "mathutils.h"
#include "debug.h"

OpticalFlow::OpticalFlow(OpticalFlowDriver &driver) : driver_(driver) {}

bool OpticalFlow::setup()
{
    if (!driver_.setup())
    {
        // Placeholder: optional optical flow setup error handling.
        return false;
    }

    gyro_lpf = Vec3LPF(0.3);
    flow_lpf = Vec3LPF(0.8);
    return true;
}

bool OpticalFlow::kick()
{
    return driver_.parse();
}

bool OpticalFlow::has_bytes() const
{
    return driver_.has_bytes();
}

bool OpticalFlow::read(MTF02Data &out)
{
    if (driver_.read(out))
    {
        return true;
    }
    return false;
}

Vec3WithTrust OpticalFlow::get_compensated_v1frame_vxy(const MTF02Data &flowdata, const Vec3 gyro, const Quaternion &q)
{
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

    if (range_m < 0.004f || range_m > 5.0f ||
        flowdata.data.flow_status != 1 ||
        flowdata.data.dist_status != 1)
    {
        return Vec3WithTrust{Vec3{}, Vec3{}};
    }

    // Optical flow in sensor angular rate units (rad/s).
    Vec3 compensated_flow{
        static_cast<float>(flowdata.data.flow_x) * FLOW_RAD_PER_CENTIRAD,
        static_cast<float>(flowdata.data.flow_y) * FLOW_RAD_PER_CENTIRAD,
        0.0f};

    compensated_flow = flow_lpf.update(compensated_flow);
    Vec3 filtered_gyro = gyro_lpf.update(gyro);

    // gate compensation and multiplier according to quality
    float gyro_multiplier = 0.0f;
    if (flowdata.data.flow_quality >= FLOW_QUALITY_MIN)
    {
        /*
        const float q_norm = (static_cast<float>(flowdata.flow_quality) - FLOW_QUALITY_MIN) /
                             (FLOW_QUALITY_MAX - FLOW_QUALITY_MIN);
        const float q_clamped = constrain(q_norm, 0.0f, 1.0f);
        gyro_multiplier = q_clamped * GYRO_MULTIPLIER_MAX;
        */
        gyro_multiplier = 1.0f;
    }

    const Vec3 scaled_gyro = filtered_gyro * gyro_multiplier;

    // debug::plot(Vec3{compensated_flow.x, filtered_gyro.y, 0.0f});

    // Compensate for rotation-induced flow using quality-scaled gyro.
    compensated_flow = Vec3{
        compensated_flow.x - scaled_gyro.y,
        compensated_flow.y + scaled_gyro.x,
        0.0f};

    // Corrected COM velocity, expressed in body/sensor axes.
    Vec3 v_body = compensated_flow * range_m;
    v_body -= cross(filtered_gyro, R_G_TO_FLOW);

    // Rotate body -> local world using Madgwick q.
    Quaternion vq(v_body);
    Quaternion vwq = q * vq * q.T();
    Vec3 v_world{vwq.x, vwq.y, vwq.z};

    // We only want horizontal velocity.
    v_world.z = 0.0f;

    // Convert world horizontal velocity -> V1 frame.
    // V1 is yaw-aligned, no roll/pitch.
    EulerAngle e = quaternionToEuler(q);
    const float cy = cosf(e.yaw);
    const float sy = sinf(e.yaw);

    Vec3 v_v1{
        cy * v_world.x + sy * v_world.y,
        -sy * v_world.x + cy * v_world.y,
        0.0f};

    // compute trust multiplier

    float q_norm = flowdata.data.flow_quality / 255.0f;
    q_norm = constrain(q_norm, 0.0f, 1.0f);

    // Low quality = low confidence = high sigma
    float quality_scale = 1.0f / (0.55f + 0.45 * q_norm);

    // gyro magnitude, rad/s

    // More rotation -> less confidence.
    // Tune GYRO_REF. At gyro_mag == GYRO_REF,
    // sigma roughly doubles from this term.
    constexpr float GYRO_REF = 1.0f; // rad/s
    constexpr float GYRO_NOISE_GAIN = 1.6f;

    float gyro_xz = sqrtf(gyro.x * gyro.x + gyro.z * gyro.z * 0.09f);
    float gyro_y_abs = fabsf(gyro.y);

    float gyro_scale_x = 1.0f + GYRO_NOISE_GAIN * gyro_xz / GYRO_REF;
    float gyro_scale_y = 1.0f + GYRO_NOISE_GAIN * gyro_y_abs / GYRO_REF;

    // Height-based trust reduction.
    //
    // Below 2m: full trust.
    // From 2m to 4m: gradually trust less.
    // Above 4m: very weak trust.
    constexpr float FULL_TRUST_H = 1.2f;
    constexpr float WEAK_TRUST_H = 3.0f;
    constexpr float MAX_HEIGHT_SCALE = 4.0f;

    float height_scale = 1.0f;

    if (range_m > FULL_TRUST_H)
    {
        float t = (range_m - FULL_TRUST_H) / (WEAK_TRUST_H - FULL_TRUST_H);
        t = constrain(t, 0.0f, 1.0f);

        // sigma grows smoothly from 1x to 4x.
        height_scale = 1.0f + t * (MAX_HEIGHT_SCALE - 1.0f);
    }

    Vec3 trust = {
        1.0f / (height_scale * gyro_scale_y * quality_scale), // trust of x depends on gyro y
        1.0f / (height_scale * gyro_scale_x * quality_scale),
        0.0f};

    return Vec3WithTrust{v_v1, trust};
}

FloatWithTrust OpticalFlow::get_compensated_vz(
    float range_m,
    const Vec3 &accel,
    const Quaternion &q)
{
    constexpr float MIN_RANGE_M = 0.03f;
    constexpr float MAX_RANGE_M = 3.0f;
    constexpr float MIN_COS_TILT = 0.86f;
    constexpr float MAX_DH_M = 0.30f;
    constexpr float MAX_VZ_MPS = 5.0f;
    constexpr float MAX_DT_S = 0.20f;

    if (range_m < MIN_RANGE_M || range_m > MAX_RANGE_M)
    {
        return FloatWithTrust{0.0f, 0.0f};
    }

    const Vec3 sensor_down_body{0.0f, 0.0f, 1.0f};
    const Vec3 world_down{0.0f, 0.0f, 1.0f};

    const Vec3 sensor_down_world = q * sensor_down_body * q.T();

    const float cos_tilt = constrain(
        dot(sensor_down_world, world_down),
        -1.0f,
        1.0f);

    if (cos_tilt < MIN_COS_TILT)
    {
        return FloatWithTrust{0.0f, 0.0f};
    }

    const float height_sensor_m = range_m * cos_tilt;

    const Vec3 r_g_to_flow_world = q * R_G_TO_FLOW * q.T();

    // AGL height of COM. Increases when drone climbs.
    const float height_com_m = height_sensor_m + r_g_to_flow_world.z;

    const uint32_t now = micros();

    if (!range_vz_initialized_)
    {
        range_vz_initialized_ = true;
        last_height_m_ = height_com_m;
        last_range_update_us_ = now;
        last_range_vz = 0.0f;
        return FloatWithTrust{0.0f, 0.0f};
    }

    const float dt = (uint32_t)(now - last_range_update_us_) * 1e-6f;

    if (dt <= 0.0f || dt > MAX_DT_S)
    {
        last_height_m_ = height_com_m;
        last_range_update_us_ = now;
        last_range_vz = 0.0f;
        return FloatWithTrust{0.0f, 0.0f};
    }

    const float dh = height_com_m - last_height_m_;
    const float vz_down = -dh / dt;
    const float range_accel_down = (vz_down - last_range_vz) / dt;

    // Update baseline after derivative is computed.
    last_height_m_ = height_com_m;
    last_range_update_us_ = now;

    if (fabsf(dh) > MAX_DH_M || fabsf(vz_down) > MAX_VZ_MPS)
    {
        last_range_vz = vz_down;
        return FloatWithTrust{0.0f, 0.0f};
    }

    const float accel_mag_mps2 = 9.81f * sqrtf(dot(accel, accel));

    // Range acceleration is noisy, so allow a generous margin.
    constexpr float ACCEL_GATE_MULT = 3.0f;
    constexpr float ACCEL_GATE_BIAS = 3.0f; // m/s^2 extra slack

    if (fabsf(range_accel_down) >
        ACCEL_GATE_MULT * accel_mag_mps2 + ACCEL_GATE_BIAS)
    {
        last_range_vz = vz_down;
        return FloatWithTrust{0.0f, 0.0f};
    }

    last_range_vz = vz_down;
    return FloatWithTrust{vz_down, 1.0f};
}

Vec3WithTrust OpticalFlow::get_raw_flow_with_trust(const MTF02Data &flowdata, Vec3 gyro)
{
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

    if (range_m < 0.004f || range_m > 5.0f ||
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
    constexpr float GYRO_NOISE_GAIN = 0.35f;
    constexpr float MAX_GYRO_SCALE = 1.6f;

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
