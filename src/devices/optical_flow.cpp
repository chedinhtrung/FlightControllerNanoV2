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

    gyro_lpf = Vec3LPF(0.08);
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
    return driver_.read(out);
}

Vec3 OpticalFlow::get_compensated_v1frame_vxy(const MTF02Data &flowdata, const Vec3 gyro, const Quaternion &q)

{
    constexpr float FLOW_RAD_PER_CENTIRAD= 0.01f;
    constexpr float FLOW_QUALITY_MIN = 20.0f;
    constexpr float FLOW_QUALITY_MAX = 255.0f;
    constexpr float GYRO_MULTIPLIER_MAX = 3.6f;

    const float range_m = 0.001f * static_cast<float>(flowdata.dist_mm);

    if (range_m < 0.02f || range_m > 3.0f ||
        flowdata.flow_status != 1 ||
        flowdata.dist_status != 1)
    {
        return Vec3{0.0f, 0.0f, 0.0f};
    }

    // Optical flow in sensor angular rate units (rad/s).
    Vec3 compensated_flow{
        static_cast<float>(flowdata.flow_x) * FLOW_RAD_PER_CENTIRAD,
        static_cast<float>(flowdata.flow_y) * FLOW_RAD_PER_CENTIRAD,
        0.0f
    };

    compensated_flow = flow_lpf.update(compensated_flow);
    Vec3 filtered_gyro = gyro_lpf.update(gyro);

    // gate compensation and multiplier according to quality
    float gyro_multiplier = 0.0f;
    if (flowdata.flow_quality >= FLOW_QUALITY_MIN)
    {
        const float q_norm = (static_cast<float>(flowdata.flow_quality) - FLOW_QUALITY_MIN) /
                             (FLOW_QUALITY_MAX - FLOW_QUALITY_MIN);
        const float q_clamped = constrain(q_norm, 0.0f, 1.0f);
        gyro_multiplier = q_clamped * GYRO_MULTIPLIER_MAX;
    }

    const Vec3 scaled_gyro = filtered_gyro * gyro_multiplier;

    // Compensate for rotation-induced flow using quality-scaled gyro.
    compensated_flow = Vec3{
        compensated_flow.x - scaled_gyro.y,
        compensated_flow.y + scaled_gyro.x,
        0.0f
    };

    // Corrected COM velocity, expressed in body/sensor axes.
    Vec3 v_body = compensated_flow * range_m;
    v_body -= cross(scaled_gyro, R_G_TO_FLOW);

    

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

    return v_v1;
}
