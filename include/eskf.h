#pragma once

#include "BasicLinearAlgebra.h"
#include "datastructs.h"
#include "mathutils.h"
#include "devices/imu.h"

struct NominalState
{
    Vec3 p;       // position
    Vec3 v;       // velocity
    Quaternion q; // orientation quaternion
    Vec3 ab;      // accel bias
    Vec3 wb;      // gyro bias
};

struct ErrorState
{
    Vec3 dp;
    Vec3 dv;
    Vec3 dtheta;
    Vec3 dab;
    Vec3 dwb;
};

class ESKF
{
private:
    BLA::Matrix<15, 15> P;  // Covariance of the estimation
    BLA::Matrix<15, 15> Fx; // Transition

    // accel white noise, m/s^2 / sqrt(Hz)
    float sigma_an = 0.9f;

    // gyro white noise, rad/s / sqrt(Hz)
    float sigma_wn = 0.004f;

    // accel bias random walk, m/s^2 / sqrt(Hz)
    float sigma_aw = 0.002f;

    // gyro bias random walk, rad/s / sqrt(Hz)
    float sigma_ww = 0.00005f;

    // How much we trust gravity direction to reflect
    // drone's orientation
    // unit is radian, meaning we trust the difference from accel to be
    // around 3 degrees relative to the true gravity vector
    // in truth the error is dimensionless (just error between 2 unit vectors)
    // but for small errors, it approximately IS the angle in radian which we use
    // for ease of interpretability
    float gravity_direction_sigma = 3.0f * DEG_TO_RAD;

    // Optical flow uncertainty measured in rad per sec of angular change in the image
    float flow_sigma_radps = 0.15f; // rad/s, tune

    uint32_t last_imu_timestamp = 0;

public:
    NominalState nominal;

    ESKF();

    void setup(Vec3 accel);
    void propagate(const ImuData &imudata);

    void correct_gravity(const Vec3 &accel);
    void correct_flow(const Vec3WithTrust &flow, const Vec3 &gyro, float range_m);
    void correct_range(const FloatWithTrust &range_m);
    void inject(const ErrorState &e);

    void correct_zero_velocity(float sigma_mps = 0.03f);
};

inline BLA::Matrix<16, 1> pack_nominal(const NominalState &s)
{
    return {
        s.p.x, s.p.y, s.p.z,
        s.v.x, s.v.y, s.v.z,
        s.q.x, s.q.y, s.q.z, s.q.w,
        s.ab.x, s.ab.y, s.ab.z,
        s.wb.x, s.wb.y, s.wb.z};
}

inline NominalState unpack_nominal(const BLA::Matrix<16, 1> &x)
{
    NominalState s{};
    s.p = {x(0), x(1), x(2)};
    s.v = {x(3), x(4), x(5)};
    s.q = {x(6), x(7), x(8), x(9)};
    s.ab = {x(10), x(11), x(12)};
    s.wb = {x(13), x(14), x(15)};
    return s;
}

inline BLA::Matrix<15, 1> pack_error(const ErrorState &e)
{
    return {
        e.dp.x, e.dp.y, e.dp.z,
        e.dv.x, e.dv.y, e.dv.z,
        e.dtheta.x, e.dtheta.y, e.dtheta.z,
        e.dab.x, e.dab.y, e.dab.z,
        e.dwb.x, e.dwb.y, e.dwb.z};
}

inline ErrorState unpack_error(const BLA::Matrix<15, 1> &dx)
{
    ErrorState e{};
    e.dp = {dx(0), dx(1), dx(2)};
    e.dv = {dx(3), dx(4), dx(5)};
    e.dtheta = {dx(6), dx(7), dx(8)};
    e.dab = {dx(9), dx(10), dx(11)};
    e.dwb = {dx(12), dx(13), dx(14)};
    return e;
}
