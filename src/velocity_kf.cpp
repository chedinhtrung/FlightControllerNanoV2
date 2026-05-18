#include "velocity_kf.h"
#include "mathutils.h"

VelKF2::VelKF2(float accel_sigma_,
               float flow_sigma_,
               float initial_P_)
    : x{0.0f, 0.0f},
      P{initial_P_, initial_P_},
      accel_sigma(accel_sigma_),
      flow_sigma(flow_sigma_)
{
}

void VelKF2::reset(const Vec3 &v0, float initial_P_)
{
    x[0] = v0.x;
    x[1] = v0.y;

    P[0] = initial_P_;
    P[1] = initial_P_;
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
        return;
    }

    x[0] += accel_v1.x * dt;
    x[1] += accel_v1.y * dt;

    const float Q = accel_sigma * accel_sigma * dt * dt;

    P[0] += Q;
    P[1] += Q;
}

void VelKF2::updateFlow(const Vec3 &v_v1, Vec3 gyro, float quality)
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
    float gyro_mag = sqrtf(
        gyro.x * gyro.x +
        gyro.y * gyro.y +
        gyro.z * gyro.z);

    // More rotation -> less confidence.
    // Tune GYRO_REF. At gyro_mag == GYRO_REF,
    // sigma roughly doubles from this term.
    constexpr float GYRO_REF = 0.7f; // rad/s
    float gyro_scale = 1.0f + gyro_mag / GYRO_REF;

    float sigma_eff = flow_sigma * quality_scale * gyro_scale;

    // Optional hard cap so R does not explode numerically.
    sigma_eff = constrain(sigma_eff, flow_sigma, 2.0f);

    const float R = sigma_eff * sigma_eff;

    update1D(x[0], P[0], v_v1.x, R);
    update1D(x[1], P[1], v_v1.y, R);
}

Vec3 VelKF2::velocity() const
{
    return Vec3{x[0], x[1], 0.0f};
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