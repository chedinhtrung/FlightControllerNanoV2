#include "eskf.h"

ESKF::ESKF()
{   
    // Setting the initial state uncertainty
    P.Fill(0.0f);

    // position uncertainty
    P(0, 0) = 0.01f; // x, m^2
    P(1, 1) = 0.01f;
    P(2, 2) = 0.01f;

    // velocity uncertainty
    P(3, 3) = 0.01f; // vx, (m/s)^2
    P(4, 4) = 0.01f;
    P(5, 5) = 0.01f;

    // attitude error uncertainty. NOT Euler! But for small angle and 
    // initial guess, it can be.
    P(6, 6) = sqr(5.0f * DEG_TO_RAD);  // roll
    P(7, 7) = sqr(5.0f * DEG_TO_RAD);  // pitch
    P(8, 8) = sqr(30.0f * DEG_TO_RAD); // yaw

    // accel bias uncertainty
    P(9, 9) = sqr(0.2f); // m/s^2
    P(10, 10) = sqr(0.2f);
    P(11, 11) = sqr(0.2f);

    // gyro bias uncertainty
    P(12, 12) = sqr(0.02f); // rad/s
    P(13, 13) = sqr(0.02f);
    P(14, 14) = sqr(0.02f);
}

void ESKF::propagate(Vec3 gyro, Vec3 accel, float dt)
{

    float dt2 = dt * dt;

    Vec3 gyro_u = gyro - nominal.wb;
    Vec3 accel_u = accel - nominal.ab;

    BLA::Matrix<3, 3> R = quatToRotMat(nominal.q);

    Quaternion q = nominal.q;

    Vec3 a_world = q * accel_u * q.T() + Vec3{0, 0, 9.81f};

    nominal.p = nominal.p + nominal.v * dt + a_world * dt * dt * 0.5f;
    nominal.v = nominal.v + a_world * dt;
    nominal.q = normalize(nominal.q * qexp(gyro_u * dt));

    BLA::Matrix<15, 15> Qx;
    Qx.Fill(0.0f);

    const BLA::Eye<3, 3> I3 = BLA::Eye<3, 3>();

    Qx.Submatrix<3, 3>(3, 3) = sigma_an * sigma_an * dt2 * I3;
    Qx.Submatrix<3, 3>(6, 6) = sigma_wn * sigma_wn * dt2 * I3;
    Qx.Submatrix<3, 3>(9, 9) = sigma_aw * sigma_aw * dt * I3;
    Qx.Submatrix<3, 3>(12, 12) = sigma_ww * sigma_ww * dt * I3;

    Fx = BLA::Eye<15, 15>();
    Fx.Submatrix<3, 3>(0, 3) = I3 * dt;
    Fx.Submatrix<3, 3>(3, 6) = -1.0f * R * skewSymmetric(accel_u) * dt;
    Fx.Submatrix<3, 3>(3, 9) = -1.0f * R * dt;
    Fx.Submatrix<3, 3>(6, 6) = ~exp((gyro_u)*dt);
    Fx.Submatrix<3, 3>(6, 12) = -1.0f * I3 * dt;

    P = Fx * P * ~Fx + Qx;
}