#include "velocity_kf.h"
#include "mathutils.h"

VelKF2::VelKF2(float accel_sigma_,
               float flow_sigma_,
               float initial_P_,
               float v_range_sigma_,
               float v_baro_sigma_)
    : v{0.0f, 0.0f, 0.0f},
      P{initial_P_, initial_P_, initial_P_},
      accel_sigma(accel_sigma_),
      flow_sigma(flow_sigma_),
      v_range_sigma(v_range_sigma_),
      v_baro_sigma(v_baro_sigma_)
{
}

void VelKF2::reset(const Vec3 &v0, float initial_P_)
{
    v[0] = v0.x;
    v[1] = v0.y;

    P[0] = initial_P_;
    P[1] = initial_P_;

    v[2] = v0.z;
    P[2] = initial_P_;
}

void VelKF2::predict(const Vec3 &accel, const Quaternion &q)
{
    // Rotate accel into world and subtract gravity
    Vec3 accel_world = q * accel * q.T();

    accel_world += Vec3{0, 0, 1};
    accel_world *= 9.81;

    EulerAngle e = quaternionToEuler(q);
    e.pitch = 0;
    e.roll = 0;

    Quaternion q_v1_to_world = eulerToQuaternion(e);

    Vec3 accel_v1 = q_v1_to_world.T() * accel_world * q_v1_to_world;

    // State:
    // x = [vx, vy]^T
    //
    // System model:
    // x_k = F x_{k-1} + B u_k + w
    //
    // Since state is velocity only:
    // F = I
    // B = dt * I
    // u = [ax, ay]^T
    uint32_t now = micros();

    float dt = (uint32_t)(now - last_update_us) * 1e-6f;
    last_update_us = now;

    // reject integration if dt is too stale
    if (dt <= 0.0f || dt > 0.05f)
    {
        P[0] += 0.05f;
        P[1] += 0.05f;
        P[2] += 0.05f;
        return;
    }

    v[0] += accel_v1.x * dt;
    v[1] += accel_v1.y * dt;
    v[2] += accel_v1.z * dt;

    const float Q = accel_sigma * accel_sigma * dt * dt;

    P[0] += Q;
    P[1] += Q;
    P[2] += Q;
}

void VelKF2::updateFlow(const Vec3 &v_v1, const Vec3& gyro, float quality, float range_m)
{
    // Measurement model:
    //
    // z = H x + v
    //
    // z = [vx_flow, vy_flow]^T
    // H = I
    // R = sigma_flow^2 I

    float q_norm = quality / 255.0f;
    q_norm = constrain(q_norm, 0.0f, 1.0f);

    // Low quality = low confidence = high sigma
    float quality_scale = 1.0f / (0.15f + 0.85f * q_norm);

    // gyro magnitude, rad/s

    // More rotation -> less confidence.
    // Tune GYRO_REF. At gyro_mag == GYRO_REF,
    // sigma roughly doubles from this term.
    constexpr float GYRO_REF = 1.0f; // rad/s
    float gyro_scale_x = 1.0f + sqrt(gyro.x * gyro.x + gyro.z * gyro.z * 0.09) / GYRO_REF;
    float gyro_scale_y = 1.0f + fabs(gyro.y) / GYRO_REF;

     // Height-based trust reduction.
    //
    // Below 1.2m: full trust.
    // From 1.2m to 2.7m: gradually trust less.
    // Above 2.7m: very weak trust.
    constexpr float FULL_TRUST_H = 1.2f;
    constexpr float WEAK_TRUST_H = 2.7f;
    constexpr float MAX_HEIGHT_SCALE = 4.0f;

    float height_scale = 1.0f;

    if (range_m > FULL_TRUST_H) {
        float t = (range_m - FULL_TRUST_H) / (WEAK_TRUST_H - FULL_TRUST_H);
        t = constrain(t, 0.0f, 1.0f);

        // sigma grows smoothly from 1x to 4x.
        height_scale = 1.0f + t * (MAX_HEIGHT_SCALE - 1.0f);
    }

    float sigma_eff_x = flow_sigma * quality_scale * gyro_scale_y * height_scale;
    float sigma_eff_y = flow_sigma * quality_scale * gyro_scale_x * height_scale;

    // Optional hard cap so R does not explode numerically.
    sigma_eff_x = constrain(sigma_eff_x, flow_sigma, 3.0f);
    sigma_eff_y = constrain(sigma_eff_y, flow_sigma, 3.0f);

    const float Rx = sigma_eff_x * sigma_eff_x;
    const float Ry = sigma_eff_y * sigma_eff_y;

    update1D(v[0], P[0], v_v1.x, Rx);
    update1D(v[1], P[1], v_v1.y, Ry);
}

Vec3 VelKF2::velocity() const
{
    return Vec3{v[0], v[1], v[2]};
}

void VelKF2::update1D(float &xi, float &Pi, float zi, float Ri)
{
    // Innovation:
    // y = z - Hx
    // H = 1
    const float y = zi - xi;

    // Innovation covariance:
    // S = HPH^T + R
    // H = 1
    const float S = Pi + Ri;

    if (S <= 1e-9f) {return;}

    // Innovation gate: reject measurement if too surprising.
    // 3-sigma gate.
    const float gate = 3.0f * sqrtf(S);
    if (fabsf(y) > gate) {
        // Optional: increase covariance slightly so filter can recover.
        Pi += 0.02f;
        return;
    }

    // Kalman gain:
    // K = P H^T S^-1
    // H = 1
    const float K = Pi / S;

    // State update:
    // x = x + K y
    xi += K * y;

    // Covariance update:
    // P = (I - K H) P
    // H = 1
    Pi *= (1.0f - K);
}