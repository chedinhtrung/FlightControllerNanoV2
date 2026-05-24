#include "eskf.h"
#include "debug.h"

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

void ESKF::setup(Vec3 accel){
    // Compute initial orientation estimate based on earth gravity at rest

    // accel in g
    const float n = sqrtf(accel.x * accel.x +
                          accel.y * accel.y +
                          accel.z * accel.z);

    if (n < 1e-6f) {
        nominal.q = Quaternion{0.0f, 0.0f, 0.0f, 1.0f};
    } else {
        Vec3 a = accel * (1.0f/n);

        // TODO: might need bench verify for correct rpy orientations
        float roll  = atan2f(-a.y, -a.z);
        float pitch = atan2f( a.x, sqrtf(a.y * a.y + a.z * a.z));
        float yaw   = 0.0f;

        nominal.q = normalize(eulerToQuaternion(EulerAngle{
            yaw,
            pitch,
            roll
        }));
    }

    nominal.p  = Vec3{0.0f, 0.0f, 0.0f};
    nominal.v  = Vec3{0.0f, 0.0f, 0.0f};
    nominal.ab = Vec3{0.0f, 0.0f, 0.0f}; // m/s^2
    nominal.wb = Vec3{0.0f, 0.0f, 0.0f}; // rad/s
}

void ESKF::propagate(const ImuData& imudata)
{
    // P 60 - 61 Joan Sola

    float dt = (uint32_t)(imudata.timestamp - last_imu_timestamp) * 1e-6f;
    float dt2 = dt * dt;

    Vec3 gyro_u = imudata.gyro - nominal.wb;
    Vec3 accel_u = imudata.accel*G_EARTH_MPS2 - nominal.ab;

    BLA::Matrix<3, 3> R = quatToRotMat(nominal.q);

    Quaternion q = nominal.q;

    Vec3 a_world = q * accel_u * q.T() + GRAVITY_EARTH;

    nominal.p = nominal.p + nominal.v * dt + a_world * dt * dt * 0.5f;
    nominal.v = nominal.v + a_world * dt;
    nominal.q = normalize(nominal.q * qexp(gyro_u * dt));

    BLA::Matrix<15, 15> Qx;
    Qx.Fill(0.0f);

    const BLA::Eye<3, 3> I3 = BLA::Eye<3, 3>();

    // p61. eq. 268-271 Joan Sola

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

    last_imu_timestamp = imudata.timestamp;
}

void ESKF::inject(ErrorState e){

    // p64 - 65 eq. 283 - 288 Joan Sola, skipping angle error term for trivial reset

    nominal.p += e.dp; 
    nominal.v += e.dv;
    nominal.q *= qexp(e.dtheta);
    nominal.ab += e.dab;
    nominal.wb += e.dwb;

    nominal.q = normalize(nominal.q);
}

void ESKF::correct_gravity(Vec3 accel)
{
    const float acc_norm = sqrtf(
        accel.x * accel.x +
        accel.y * accel.y +
        accel.z * accel.z
    );

    if (acc_norm < 1e-6f) {
        return;
    }

    //debug::log(accel);

    const float g_err = fabsf(acc_norm - 1.0f);

    constexpr float FULL_TRUST_ERR = 0.05f;
    constexpr float ZERO_TRUST_ERR = 0.30f;

    if (g_err > ZERO_TRUST_ERR) {
        return;
    }

    float trust = 1.0f;
    if (g_err > FULL_TRUST_ERR) {
        float t = (g_err - FULL_TRUST_ERR) / (ZERO_TRUST_ERR - FULL_TRUST_ERR);
        t = constrain(t, 0.0f, 1.0f);
        t = t * t * (3.0f - 2.0f * t);
        trust = 1.0f - t;
    }

    const Vec3 z = accel * (1.0f/acc_norm);

    const Vec3 f_world{0.0f, 0.0f, -1.0f};
    const Vec3 h = nominal.q.T() * f_world * nominal.q;

    const Vec3 r = z - h;

    BLA::Matrix<3, 15> H;
    H.Fill(0.0f);

    // d (measured gravity) / d \\delta theta
    // here H = d measurement / d error state computed directly 
    // Not via equation 278 and chain rule
    H.Submatrix<3, 3>(0, 6) = 1.0f * skewSymmetric(h);

    constexpr float BASE_SIGMA = 3.0f * DEG_TO_RAD;
    const float sigma = BASE_SIGMA / constrain(trust, 0.1f, 1.0f);

    BLA::Matrix<3, 3> R;
    R.Fill(0.0f);
    R(0, 0) = sigma * sigma;
    R(1, 1) = sigma * sigma;
    R(2, 2) = sigma * sigma;

    const BLA::Matrix<15, 15> I = BLA::Eye<15, 15>();

    BLA::Matrix<15, 3> PHt = P * ~H;
    BLA::Matrix<3, 3> S = H * PHt + R;

    BLA::Matrix<15, 3> K = PHt * Inverse(S);

    BLA::Matrix<3, 1> y = {r.x, r.y, r.z};
    BLA::Matrix<15, 1> dx = K * y;

    BLA::Matrix<15, 15> A = I - K * H;
    P = A * P * ~A + K * R * ~K;

    inject(unpack_error(dx));
}