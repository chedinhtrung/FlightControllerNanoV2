#include "madgwick.h"

constexpr Vec3 DOWN{0.0f, 0.0f, 1.0f};

Madgwick::Madgwick(float beta_, const Vec3 &g_body)
{
    beta = beta_;
    q = quatFromTwoUnitVectors(g_body, DOWN);
    q = normalize(q);
    last_update_us = micros();
}

void Madgwick::update(const ImuData &imu)
{
    uint32_t now = micros();

    if ((uint32_t)(now - last_update_us) < PERIOD_US)
    {
        return;
    }

    float dt = (now - last_update_us) * 1e-6f;
    last_update_us = now;

    BLA::Matrix<4, 3> Jt = {
        -2.0f * q.y, 2.0f * q.x, 0.0f,
        2.0f * q.z, 2.0f * q.w, -4.0f * q.x,
        -2.0f * q.w, 2.0f * q.z, -4.0f * q.y,
        2.0f * q.x, 2.0f * q.y, 0.0f};

    BLA::Matrix<3, 1> error = {
        2.0f * (q.x * q.z - q.w * q.y) + imu.accel.x, // + instead of - because accels measure the opposite
        2.0f * (q.w * q.x + q.y * q.z) + imu.accel.y,
        2.0f * (0.5f - q.x * q.x - q.y * q.y) + imu.accel.z};

    BLA::Matrix<4, 1> grad = Jt * error;

    float norm = sqrtf(
        grad(0) * grad(0) +
        grad(1) * grad(1) +
        grad(2) * grad(2) +
        grad(3) * grad(3));

    if (norm > 1e-9f)
    {
        grad /= norm;
    }

    Quaternion correction{
        grad(1), // qx component
        grad(2), // qy component
        grad(3), // qz component
        grad(0), // qw component
    };

    float accel_norm = sqrtf(
        imu.accel.x * imu.accel.x +
        imu.accel.y * imu.accel.y +
        imu.accel.z * imu.accel.z);

    float g_error = fabsf(accel_norm - 1.0f); // assuming accel is in g

    float accel_trust = 1.0f;

    // Full trust near 1g, fade out as error grows.
    constexpr float FULL_TRUST_ERR = 0.05f; // ±5%
    constexpr float ZERO_TRUST_ERR = 0.30f; // ±30%

    if (g_error <= FULL_TRUST_ERR)
    {
        accel_trust = 1.0f;
    }
    else if (g_error >= ZERO_TRUST_ERR)
    {
        accel_trust = 0.0f;
    }
    else
    {
        float t = (g_error - FULL_TRUST_ERR) / (ZERO_TRUST_ERR - FULL_TRUST_ERR);
        accel_trust = 1.0f - t;
    }

    constexpr float BETA_MIN = 0.05f;
    float BETA_MAX = beta;

    Quaternion gq = Quaternion(imu.gyro);

    float beta_eff = BETA_MIN + accel_trust * (BETA_MAX - BETA_MIN);

    Quaternion q_dot = (q * gq) * 0.5 - correction * beta_eff;
    q += q_dot * dt;
    q = normalize(q);
}
