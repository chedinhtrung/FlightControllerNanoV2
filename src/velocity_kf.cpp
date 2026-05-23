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

void VelKF2::updateFlow(const Vec3WithTrust& flow_v1)
{
    // Measurement model:
    //
    // z = H x + v
    //
    // z = [vx_flow, vy_flow]^T
    // H = I
    // R = sigma_flow^2 I

    if (flow_v1.trust.x <= 0.0f || flow_v1.trust.y <= 0.0f)
    {
        return;
    }
    const float sigma_eff_x = constrain(flow_sigma / flow_v1.trust.x, flow_sigma, 3.0f);
    const float sigma_eff_y = constrain(flow_sigma / flow_v1.trust.y, flow_sigma, 3.0f);

    const float Rx = sigma_eff_x * sigma_eff_x;
    const float Ry = sigma_eff_y * sigma_eff_y;

    update1D(v[0], P[0], flow_v1.value.x, Rx);
    update1D(v[1], P[1], flow_v1.value.y, Ry);
}

void VelKF2::updateRange(const FloatWithTrust &vz_range_down)
{
    if (vz_range_down.trust <= 0.0f)
    {
        return;
    }
    const float sigma_eff = constrain(v_range_sigma / vz_range_down.trust, v_range_sigma, 3.0f);
    const float R = sigma_eff * sigma_eff;
    update1D(v[2], P[2], vz_range_down.value, R);
}

void VelKF2::updateBaro(const FloatWithTrust &vz_baro)
{
    if (vz_baro.trust <= 0.0f)
    {
        return;
    }
    const float sigma_eff = constrain(v_baro_sigma / vz_baro.trust, v_range_sigma, 3.0f);
    const float R = sigma_eff * sigma_eff;
    update1D(v[2], P[2], vz_baro.value, R);
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

    if (S <= 1e-9f)
    {
        return;
    }

    // Innovation gate: reject measurement if too surprising.
    // 3-sigma gate.
    const float gate = 3.0f * sqrtf(S);
    if (fabsf(y) > gate)
    {
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