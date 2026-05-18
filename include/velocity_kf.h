#pragma once

#include "datastructs.h"

struct VelKF2
{
    // State:
    // x = [vx, vy]^T
    float x[2];

    // Covariance:
    // P = 2x2, kept diagonal for two independent 1D filters.
    float P[2];

    float accel_sigma; // process acceleration noise, m/s^2
    float flow_sigma;  // optical-flow velocity noise, m/s

    uint32_t last_update_us = micros();

    VelKF2(float accel_sigma_ = 2.0f,
           float flow_sigma_ = 0.10f,
           float initial_P_ = 1.0f);

    void reset(const Vec3& v0 = Vec3{0.0f, 0.0f, 0.0f},
               float initial_P_ = 1.0f);

    // Prediction:
    // x_k|k-1 = F x_k-1|k-1 + B a
    // P_k|k-1 = F P F^T + Q
    void predict(const Vec3& accel, const Quaternion& q);

    // Measurement update:
    // z = H x + noise
    // Here z = [vx_flow, vy_flow]^T and H = I.
    void updateFlow(const Vec3& flow_v1, Vec3 gyro, float quality);

    Vec3 velocity() const;

private:
    static void update1D(float& xi, float& Pi, float zi, float Ri);
};