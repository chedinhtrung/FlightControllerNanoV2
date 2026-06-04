#include "eskf.h"
#include "debug.h"
#include "lpf.h"
#include "devices/optical_flow.h"

ESKF::ESKF()
{
    // Setting the initial state uncertainty
    P.Fill(0.0f);

    // position uncertainty
    P(0, 0) = 0.01f; // x, m^2
    P(1, 1) = 0.01f;
    P(2, 2) = 0.1f;

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

void ESKF::setup(Vec3 accel)
{
    // Compute initial orientation estimate based on earth gravity at rest

    // accel in g
    const float n = sqrtf(accel.x * accel.x +
                          accel.y * accel.y +
                          accel.z * accel.z);

    if (n < 1e-6f)
    {
        nominal.q = Quaternion{0.0f, 0.0f, 0.0f, 1.0f};
    }
    else
    {
        Vec3 a = accel * (1.0f / n);

        // TODO: might need explicit reconfig to ease install on different hardware
        float roll = atan2f(-a.y, -a.z);
        float pitch = atan2f(a.x, sqrtf(a.y * a.y + a.z * a.z));
        float yaw = 0.0f;

        nominal.q = normalize(eulerToQuaternion(EulerAngle{
            yaw,
            pitch,
            roll}));
    }

    nominal.p = Vec3{0.0f, 0.0f, 0.0f};
    nominal.v = Vec3{0.0f, 0.0f, 0.0f};
    nominal.ab = Vec3{0.0f, 0.0f, 0.0f}; // m/s^2
    nominal.wb = Vec3{0.0f, 0.0f, 0.0f}; // rad/s
}

bool ESKF::propagate_core(const ImuData &imudata)
{
    // P 60 - 61 Joan Sola
    if (last_imu_timestamp == 0)
    {
        last_imu_timestamp = imudata.timestamp;
        return false;
    }

    float dt = (uint32_t)(imudata.timestamp - last_imu_timestamp) * 1e-6f;

    if (dt <= 0.0f || dt > 0.02f)
    {
        last_imu_timestamp = imudata.timestamp;
        return false;
    }

    float dt2 = dt * dt;

    Vec3 gyro_u = imudata.gyro - nominal.wb;
    Vec3 accel_u = imudata.accel * G_EARTH_MPS2 - nominal.ab;

    BLA::Matrix<3, 3> R = quatToRotMat(nominal.q);

    Quaternion q = nominal.q;

    Vec3 a_world = q * accel_u * q.T() + GRAVITY_EARTH;

    float accel_q_scale = 1.0f;

    // Basic IMU sanity gate on raw accel norm in g.
    const float accel_norm_g = sqrtf(dot(imudata.accel, imudata.accel));

    bool accel_bad =
        accel_norm_g < 0.1f ||
        accel_norm_g > 5.0f;

    // Clamp net world acceleration used for nominal integration.
    constexpr float MAX_PROP_ACCEL_MPS2 = 25.0f;
    float a_world_norm = sqrtf(dot(a_world, a_world));

    if (accel_bad || a_world_norm > MAX_PROP_ACCEL_MPS2)
    {
        if (a_world_norm > MAX_PROP_ACCEL_MPS2)
        {
            a_world *= MAX_PROP_ACCEL_MPS2 / a_world_norm;
        }

        // This sample was suspicious. Let measurements correct more easily later.
        accel_q_scale = 25.0f; // 5x sigma equivalent
    }

    nominal.p = nominal.p + nominal.v * dt + a_world * dt * dt * 0.5f;
    nominal.v = nominal.v + a_world * dt;
    nominal.q = normalize(nominal.q * qexp(gyro_u * dt));

    // This mess is doint P = FPF^T + Qx, but optimized for the fact that
    // F is mostly zeroes and I
    const BLA::Eye<3, 3> I3 = BLA::Eye<3, 3>();

    const BLA::Matrix<3, 3> Ra_udt =
        R * skewSymmetric(accel_u) * dt;

    const BLA::Matrix<3, 3> Rdt =
        R * dt;

    const BLA::Matrix<3, 3> w_udt =
        ~exp(gyro_u * dt);

    // PFt = P * F.T
    BLA::Matrix<15, 15> PFt;
    PFt.Fill(0.0f);

    // PFt[:, p] = P[:, p] + P[:, v] * dt
    PFt.Submatrix<15, 3>(0, 0) =
        P.Submatrix<15, 3>(0, 0) +
        P.Submatrix<15, 3>(0, 3) * dt;

    // PFt[:, v] = P[:, v] + P[:, theta] * A.T + P[:, ab] * B.T
    PFt.Submatrix<15, 3>(0, 3) =
        P.Submatrix<15, 3>(0, 3) +
        P.Submatrix<15, 3>(0, 6) * ~-Ra_udt +
        P.Submatrix<15, 3>(0, 9) * ~-Rdt;

    // PFt[:, theta] = P[:, theta] * C.T + P[:, wb] * (-dt)
    PFt.Submatrix<15, 3>(0, 6) =
        P.Submatrix<15, 3>(0, 6) * ~w_udt -
        P.Submatrix<15, 3>(0, 12) * dt;

    // PFt[:, ab] = P[:, ab]
    PFt.Submatrix<15, 3>(0, 9) = P.Submatrix<15, 3>(0, 9) * 1.0f;

    // PFt[:, wb] = P[:, wb]
    PFt.Submatrix<15, 3>(0, 12) = P.Submatrix<15, 3>(0, 12) * 1.0f;

    // Pnew = F * PFt + Qx

    BLA::Matrix<15, 15> Pnew;
    Pnew.Fill(0.0f);

    // Pnew[p, :] = PFt[p, :] + PFt[v, :] * dt
    Pnew.Submatrix<3, 15>(0, 0) =
        PFt.Submatrix<3, 15>(0, 0) +
        PFt.Submatrix<3, 15>(3, 0) * dt;

    // Pnew[v, :] = PFt[v, :] + A * PFt[theta, :] + B * PFt[ab, :]
    Pnew.Submatrix<3, 15>(3, 0) =
        PFt.Submatrix<3, 15>(3, 0) -
        Ra_udt * PFt.Submatrix<3, 15>(6, 0) -
        Rdt * PFt.Submatrix<3, 15>(9, 0);

    // Pnew[theta, :] = C * PFt[theta, :] - PFt[wb, :] * dt
    Pnew.Submatrix<3, 15>(6, 0) =
        w_udt * PFt.Submatrix<3, 15>(6, 0) -
        PFt.Submatrix<3, 15>(12, 0) * dt;

    // Pnew[ab, :] = PFt[ab, :]
    Pnew.Submatrix<3, 15>(9, 0) = PFt.Submatrix<3, 15>(9, 0) * 1.0f;

    // Pnew[wb, :] = PFt[wb, :]
    Pnew.Submatrix<3, 15>(12, 0) = PFt.Submatrix<3, 15>(12, 0) * 1.0f;

    // Add Qx. No need to construct full Qx.

    const float q_v = sigma_an_mps2 * sigma_an_mps2 * accel_q_scale * dt2;
    const float q_th = sigma_wn_radps * sigma_wn_radps * dt2;
    const float q_ab = sigma_aw * sigma_aw * dt;
    const float q_wb = sigma_ww_radps * sigma_ww_radps * dt;

    // velocity block, indices 3..5
    Pnew(3, 3) += q_v;
    Pnew(4, 4) += q_v;
    Pnew(5, 5) += q_v;

    // attitude block, indices 6..8
    Pnew(6, 6) += q_th;
    Pnew(7, 7) += q_th;
    Pnew(8, 8) += q_th;

    // accel bias block, indices 9..11
    Pnew(9, 9) += q_ab;
    Pnew(10, 10) += q_ab;
    Pnew(11, 11) += q_ab;

    // gyro bias block, indices 12..14
    Pnew(12, 12) += q_wb;
    Pnew(13, 13) += q_wb;
    Pnew(14, 14) += q_wb;

    // Keep covariance symmetric against float roundoff.
    P = (Pnew + ~Pnew) * 0.5f;

    last_imu_timestamp = imudata.timestamp;

    return true;
}
void ESKF::propagate(const ImuData &imudata)
{
    if(propagate_core(imudata)){
        push_buffer(imudata);
    }
}

void ESKF::inject(const ErrorState &e)
{

    // p64 - 65 eq. 283 - 288 Joan Sola, skipping angle error term for trivial reset

    nominal.p += e.dp;
    nominal.v += e.dv;
    nominal.q *= qexp(e.dtheta);
    nominal.ab += e.dab;
    nominal.wb += e.dwb;

    nominal.q = normalize(nominal.q);
}

void ESKF::correct_gravity(const Vec3 &accel)
{
    const float acc_norm = sqrtf(
        accel.x * accel.x +
        accel.y * accel.y +
        accel.z * accel.z);

    if (acc_norm < 1e-4f)
    {
        return;
    }

    // debug::log(accel);

    const float g_err = fabsf(acc_norm - 1.0f);

    // Trust gravity when it is closer to unit length
    constexpr float FULL_TRUST_ERR = 0.08f;
    constexpr float ZERO_TRUST_ERR = 0.20f;

    if (g_err > ZERO_TRUST_ERR)
    {
        return;
    }

    float trust = 1.0f;
    if (g_err > FULL_TRUST_ERR)
    {
        float t = (g_err - FULL_TRUST_ERR) / (ZERO_TRUST_ERR - FULL_TRUST_ERR);
        t = constrain(t, 0.0f, 1.0f);
        t = t * t * (3.0f - 2.0f * t);
        trust = 1.0f - t;
    }

    const Vec3 z = accel * (1.0f / acc_norm);

    const Vec3 f_world{0.0f, 0.0f, -1.0f};
    const Vec3 h = nominal.q.T() * f_world * nominal.q;

    const Vec3 r = z - h;

    // H = [0 0 Htheta 0 0] (observation assumed to only depend on orientation)
    // In reality it also needs ab (accel bias). Suggestion for further improvement
    // Optimized for blockwise update. This mess is doing K = HP(HPH^T + V)⁻1 + KVK^T
    BLA::Matrix<3, 3> Htheta = skewSymmetric(h);

    const float sigma = gravity_direction_sigma / constrain(trust, 0.1f, 1.0f);

    BLA::Matrix<3, 3> V;
    V.Fill(0.0f);
    V(0, 0) = sigma * sigma;
    V(1, 1) = sigma * sigma;
    V(2, 2) = sigma * sigma;

    // H = [0 0 Htheta 0 0]

    BLA::Matrix<15, 3> PHt =
        P.Submatrix<15, 3>(0, 6) * ~Htheta; // PHt = P * H.T = P[:, theta] * Htheta.T

    BLA::Matrix<3, 3> S =
        Htheta * PHt.Submatrix<3, 3>(6, 0) + V; // S = H * PHt + V = Htheta * PHt[theta, :] + V

    BLA::Matrix<3, 3> S_inv = Inverse(S);
    BLA::Matrix<15, 3> K = PHt * S_inv;

    BLA::Matrix<3, 1> y = {r.x, r.y, r.z};
    BLA::Matrix<15, 1> dx = K * y;

    // Optimized for blockwise update. This mess is doing P = (I-KH) P (I-KH)^T + KVK^T
    BLA::Matrix<15, 15> KHP =
        K * Htheta * P.Submatrix<3, 15>(6, 0); // KHP = K * H * P = K * Htheta * P[theta, :]

    P = P - KHP - ~KHP + K * S * ~K; // P = P - KHP - KHP.T + KSK.T
    P = (P + ~P) * 0.5f;             // Enforce symmetric covariance

    ErrorState e = unpack_error(dx);

    // intentionally make this correction only about attitude, not about position / vel
    e.dp = Vec3{0, 0, 0};
    e.dv = Vec3{0, 0, 0};
    e.dab = Vec3{0, 0, 0};

    inject(e);
}

void ESKF::correct_flow_and_range(const MTF02Data &flowdata)
{
    int buf_idx = get_closest_buf(flowdata.timestamp);
    if (buf_idx == -1)
    {
        return;
    }

    // 1) Restore estimator to delayed time.
    nominal = state_buf[buf_idx].state;
    P = state_buf[buf_idx].P;
    last_imu_timestamp = state_buf[buf_idx].timestamp;

    // correct then save back to buffer
    correct_flow(flowdata, state_buf[buf_idx]);
    correct_range(flowdata, state_buf[buf_idx]);
    state_buf[buf_idx].state = nominal;
    state_buf[buf_idx].P = P;

    replay_from(buf_idx);
}

void ESKF::correct_flow(const MTF02Data &flowdata, const StateBuffer &closest_buf)
{
    // Refer to the flow observation in the docs

    // const StateBuffer *closest_buf = get_closest_buf(flowdata.timestamp);

    const float rho = flowdata.data.dist_mm * 1e-3;

    Vec3WithTrust flow = get_raw_flow_with_trust(flowdata, closest_buf.imudata.gyro);
    const float trust_x_raw = flow.trust.x;
    const float trust_y_raw = flow.trust.y;

    if (trust_x_raw <= 0.0f && trust_y_raw <= 0.0f)
    {
        return;
    }

    if (rho < 0.005f || rho > 5.0f)
    {
        return;
    }

    const float trust_x = constrain(trust_x_raw, 0.05f, 1.0f);
    const float trust_y = constrain(trust_y_raw, 0.05f, 1.0f);

    const Vec3 v_G_B = nominal.q.T() * nominal.v * nominal.q;

    const Vec3 omegr_GP = closest_buf.imudata.gyro - nominal.wb;

    // Earth -> body rotation.
    BLA::Matrix<3, 3> R_EB = quatToRotMat(nominal.q); // body -> earth
    BLA::Matrix<3, 3> R_BE = ~R_EB;                   // earth -> body

    // COM velocity expressed in body frame.
    // const Vec3 v_G_B = nominal.q.T() * nominal.v * nominal.q;

    // Lever arm from sensor to observed ground point, expressed as COM->sensor
    // plus sensor->ground-ray point.
    const Vec3 r_SO_B{0.0f, 0.0f, rho};
    const Vec3 r_B = R_G_TO_FLOW + r_SO_B;

    // Predicted raw optical flow angular rate.
    const Vec3 pred_3d =
        (v_G_B * -1.0f - cross(omegr_GP, r_B)) * (1.0f / rho);

    // Flow sensor might be mounted backwards. negative here if needed
    constexpr float FLOW_SIGN_X = -1.0f;
    constexpr float FLOW_SIGN_Y = -1.0f;

    BLA::Matrix<2, 1> z = {
        FLOW_SIGN_X * flow.value.x,
        FLOW_SIGN_Y * flow.value.y};

    BLA::Matrix<2, 1> h = {
        pred_3d.x,
        pred_3d.y};

    BLA::Matrix<2, 1> y = z - h;

    // debug::plot(v_G_B);

    // debug::plot(Vec3{pred_3d.x, FLOW_SIGN_X * flow.value.x, 0});

    // S = [1 0 0; 0 1 0] because only observe xy
    BLA::Matrix<2, 3> S;
    S = {
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f};

    // construct H = d measure / d error state, refer to doc
    BLA::Matrix<2, 3> Hv =
        (-1.0f / rho) * S * R_BE;

    BLA::Matrix<2, 3> Htheta =
        (-1.0f / rho) * S * skewSymmetric(v_G_B);

    BLA::Matrix<2, 3> Hwb =
        (-1.0f / rho) * S * skewSymmetric(r_B);

    // Measurement noise
    // angular-rate noise in rad/s

    float sigma_x = sigma_flow_radps / trust_x;
    float sigma_y = sigma_flow_radps / trust_y;

    sigma_x = constrain(sigma_x, sigma_flow_radps, 2.5f);
    sigma_y = constrain(sigma_y, sigma_flow_radps, 2.5f);

    BLA::Matrix<2, 2> V;
    V.Fill(0.0f);
    V(0, 0) = sigma_x * sigma_x;
    V(1, 1) = sigma_y * sigma_y;

    // Kalman update.
    // H = [0 Hv Htheta 0 Hwb], optimized to only multiply non zero blocks
    // This mess is just S_innov = HPH^T + V
    BLA::Matrix<15, 2> PHt;
    PHt.Fill(0.0f);
    PHt += P.Submatrix<15, 3>(0, 3) * ~Hv;     // PHt = P[:,v] * Hv.T
    PHt += P.Submatrix<15, 3>(0, 6) * ~Htheta; // + P[:,theta] * Htheta.T
    PHt += P.Submatrix<15, 3>(0, 12) * ~Hwb;   // + P[:,wb] * Hwb.T

    BLA::Matrix<2, 2> S_innov = V;
    S_innov += Hv * PHt.Submatrix<3, 2>(3, 0);     // H_v * PHt[v_rows, :]
    S_innov += Htheta * PHt.Submatrix<3, 2>(6, 0); // H_theta * PHt[theta_rows, :]
    S_innov += Hwb * PHt.Submatrix<3, 2>(12, 0);   // H_wb * PHt[wb_rows, :]
    // end S_innov = HPH^T + V

    BLA::Matrix<2, 2> S_inv = Inverse(S_innov);

    BLA::Matrix<1, 1> nis_m = ~y * S_inv * y;
    float nis = nis_m(0);

    constexpr float FLOW_NIS_GATE = 13.8f;

    if (nis > FLOW_NIS_GATE)
    {
        // Soft reject:
        // Measurement disagrees more than expected, so trust it less
        // instead of completely ignoring it.
        float scale = constrain(nis / FLOW_NIS_GATE, 1.0f, 25.0f);

        // Recompute innovation covariance with inflated measurement noise.
        S_innov -= V;
        V(0, 0) *= scale;
        V(1, 1) *= scale;
        S_innov += V;

        S_inv = Inverse(S_innov);
    }

    BLA::Matrix<15, 2> K = PHt * S_inv;

    BLA::Matrix<15, 1> dx = K * y;

    // Joseph covariance update.
    // Optimized for sparse H. This mess is only doing P = (I-KH) P (I-KH)^T + KVK^T
    BLA::Matrix<15, 15> KHP;
    KHP.Fill(0.0f);

    KHP += K * Hv * P.Submatrix<3, 15>(3, 0);
    KHP += K * Htheta * P.Submatrix<3, 15>(6, 0);
    KHP += K * Hwb * P.Submatrix<3, 15>(12, 0);

    P = P - KHP - ~KHP + K * S_innov * ~K;
    P = (P + ~P) * 0.5f; // ensure correct symmetric covariance

    ErrorState e = unpack_error(dx);

    // Safety gatings
    // Allow limited influence of dtheta. If more than 2 degs, saturate at 2 degs
    constexpr float MAX_DTHETA_CORR = 2.0f * RAD_PER_DEG;
    float dtheta_norm = sqrtf(dot(e.dtheta, e.dtheta));
    if (dtheta_norm > MAX_DTHETA_CORR)
    {
        e.dtheta *= MAX_DTHETA_CORR / dtheta_norm;
    }

    // Allow limited influence on v. If more than 10cm/s, saturate at 10cm/s
    constexpr float MAX_DV_CORR = 0.15f; // m/s per update
    float dv_norm = sqrtf(dot(e.dv, e.dv));
    if (dv_norm > MAX_DV_CORR)
    {
        e.dv *= MAX_DV_CORR / dv_norm;
    }

    // Same with gyro bias: if more than 0.0002, saturate at 0.001 rad/s
    constexpr float MAX_DWB_CORR = 0.0002f; // rad/s per update
    float dwb_norm = sqrtf(dot(e.dwb, e.dwb));
    if (dwb_norm > MAX_DWB_CORR)
    {
        e.dwb *= MAX_DWB_CORR / dwb_norm;
    }

    //e.dwb = Vec3{0, 0, 0};    // temporary gate disallow update gyro bias
    // e.dtheta = Vec3{0, 0, 0}; // temporary gate disallow update angle

    inject(e);
}

void ESKF::correct_range(const MTF02Data &flowdata, const StateBuffer &closest_buf)
{
    const float rho = flowdata.data.dist_mm * 1e-3f;

    if (rho < 0.005f || rho > 5.0f)
    {
        return;
    }

    BLA::Matrix<3, 3> R_EB = quatToRotMat(nominal.q); // body -> earth

    const Vec3 e_z{0.0f, 0.0f, 1.0f};
    const Vec3 _Br_GS = R_G_TO_FLOW;

    Vec3 Rz{
        R_EB(2, 0),
        R_EB(2, 1),
        R_EB(2, 2)};

    float R_22 = R_EB(2, 2);

    if (R_22 < 0.7f)
    {
        return; // too tilted
    }

    float _Er_S = nominal.p.z + dot(Rz, _Br_GS);

    float rho_pred = (-h_terrain - _Er_S) / R_22;

    float Hpz = -1.0f / R_22;

    Vec3 a_B = _Br_GS + e_z * rho_pred;

    BLA::Matrix<1, 3> Htheta = {
        (Rz.y * a_B.z - Rz.z * a_B.y) / R_22,
        (Rz.z * a_B.x - Rz.x * a_B.z) / R_22,
        (Rz.x * a_B.y - Rz.y * a_B.x) / R_22};

    BLA::Matrix<1, 3> Hp = {
        0, 0,
        -1 / R_22};

    BLA::Matrix<1, 1> y = {
        rho - rho_pred};

    constexpr float TERRAIN_JUMP_GATE_M = 0.15f; // tune 0.10-0.20 m

    // if the height suddenly jumps compared to prediction, we
    // probly flew over a terrain bump like a table
    // then we just update h_terrain
    if (fabsf(rho - rho_pred) > TERRAIN_JUMP_GATE_M)
    {
        h_terrain = -_Er_S - rho * R_22;
        return;
    }

    BLA::Matrix<1, 1> V = {
        sigma_range_m * sigma_range_m};

    // Kalman update.
    // H = [Hp 0 Htheta 0 0], optimized to only multiply non zero blocks
    // This mess is just S = HPH^T + V

    BLA::Matrix<15, 1> PHt;
    PHt.Fill(0.0f);

    PHt += P.Submatrix<15, 3>(0, 0) * ~Hp;     // P[:,p] * Hp.T
    PHt += P.Submatrix<15, 3>(0, 6) * ~Htheta; // P[:,theta] * Htheta.T

    BLA::Matrix<1, 1> S = Hp * PHt.Submatrix<3, 1>(0, 0) + Htheta * PHt.Submatrix<3, 1>(6, 0) + V;

    // 1D inverse
    float S_inv = 1.0f / S(0, 0);

    // Optional 1D NIS.
    BLA::Matrix<1, 1> nis_m = ~y * S_inv * y;
    float nis = nis_m(0);

    constexpr float RANGE_NIS_GATE = 10.8f; // 99.9% for 1D-ish

    if (nis > RANGE_NIS_GATE)
    {
        // Soft reject: inflate measurement noise instead of hard-rejecting.
        float scale = constrain(nis / RANGE_NIS_GATE, 1.0f, 25.0f);

        // Replace old V contribution with inflated V contribution.
        S -= V;

        V(0, 0) *= scale;

        S += V;

        // Recompute inverse after soft rejection.
        S_inv = 1.0f / S(0, 0);
    }

    BLA::Matrix<15, 1> K = PHt * S_inv;

    BLA::Matrix<15, 1> dx = K * y;

    // Optimized for sparse H. This mess is only doing:
    // P = (I-KH) P (I-KH)^T + KVK^T
    //
    // Expanded:
    // P = P - KHP - KHP^T + KSK^T
    BLA::Matrix<15, 15> KHP;
    KHP.Fill(0.0f);

    KHP += K * Hp * P.Submatrix<3, 15>(0, 0);
    KHP += K * Htheta * P.Submatrix<3, 15>(6, 0);

    P = P - KHP - ~KHP + K * S * ~K;
    P = (P + ~P) * 0.5f;

    ErrorState e = unpack_error(dx);

    // Safety clamps for dp, dvz and dtheta
    constexpr float MAX_DPZ_CORR = 0.10f; // m per update
    if (fabsf(e.dp.z) > MAX_DPZ_CORR)
    {
        e.dp.z = constrain(e.dp.z, -MAX_DPZ_CORR, MAX_DPZ_CORR);
    }

    constexpr float MAX_DVZ_CORR = 0.15f; // m/s per update
    if (fabsf(e.dv.z) > MAX_DVZ_CORR)
    {
        e.dv.z = constrain(e.dv.z, -MAX_DVZ_CORR, MAX_DVZ_CORR);
    }

    constexpr float MAX_DTHETA_CORR = 1.0f * DEG_TO_RAD;
    float dtheta_norm = sqrtf(dot(e.dtheta, e.dtheta));
    if (dtheta_norm > MAX_DTHETA_CORR)
    {
        e.dtheta *= MAX_DTHETA_CORR / dtheta_norm;
    }

    // Optional: kill effect on accel and gyro bias
    //e.dab = Vec3{0.0f, 0.0f, 0.0f};
    // e.dwb = Vec3{0.0f, 0.0f, 0.0f};

    inject(e);
}

void ESKF::correct_baro(float baro_alt_m, float trust)
{
    if (trust <= 1e-2)
        return;

    // baro altitude positive up; ESKF p.z positive down
    const float z = baro_alt_m - baro_offset_m;
    const float h = -nominal.p.z;
    const float residual = z - h;

    static float residual_int = 0.0f;
    static float baro_sum = 0.0f;
    static uint8_t baro_cnt = 0;

    constexpr float BARO_SPIKE_REJECT_M = 1.2f;       // reject one-off garbage
    constexpr float BARO_RESIDUAL_DEADBAND_M = 0.08f; // ignore normal noise
    constexpr float BARO_RESET_INT_M = 2.5f;          // sample-wise residual integral

    // Huge residual: probably a spike. Ignore it completely.
    if (fabsf(residual) > BARO_SPIKE_REJECT_M)
    {
        return;
    }

    // Normal small residual: reset detector and continue normal baro fusion.
    if (fabsf(residual) < BARO_RESIDUAL_DEADBAND_M)
    {
        residual_int = 0.0f;
        baro_sum = 0.0f;
        baro_cnt = 0;
    }
    else
    {
        // Persistent medium residual: likely baro offset error.
        residual_int += residual;
        baro_sum += baro_alt_m;
        baro_cnt++;

        if (fabsf(residual_int) > BARO_RESET_INT_M)
        {
            reset_baro_offset(baro_sum / static_cast<float>(baro_cnt));

            residual_int = 0.0f;
            baro_sum = 0.0f;
            baro_cnt = 0;
        }

        return;
    }

    // debug::plot(Vec3{z, h, nominal.v.z});

    BLA::Matrix<1, 1> y = {residual};

    BLA::Matrix<1, 3> Hp = {0.0f, 0.0f, -1.0f};

    BLA::Matrix<1, 1> V = {sigma_baro_m * sigma_baro_m};
    V *= 1.0f / (trust * trust);

    BLA::Matrix<15, 1> PHt;
    PHt.Fill(0.0f);

    PHt += P.Submatrix<15, 3>(0, 0) * ~Hp;

    BLA::Matrix<1, 1> S = Hp * PHt.Submatrix<3, 1>(0, 0) + V;

    float S_inv = 1.0f / S(0, 0);

    BLA::Matrix<1, 1> nis_m = ~y * S_inv * y;
    float nis = nis_m(0);

    constexpr float BARO_NIS_GATE = 10.8f;

    if (nis > BARO_NIS_GATE)
    {
        float scale = constrain(nis / BARO_NIS_GATE, 1.0f, 25.0f);

        S -= V;
        V(0, 0) *= scale;
        S += V;

        S_inv = 1.0f / S(0, 0);
    }

    BLA::Matrix<15, 1> K = PHt * S_inv;
    BLA::Matrix<15, 1> dx = K * y;

    BLA::Matrix<15, 15> KHP;
    KHP.Fill(0.0f);

    KHP += K * Hp * P.Submatrix<3, 15>(0, 0);

    P = P - KHP - ~KHP + K * S * ~K;
    P = (P + ~P) * 0.5f;

    ErrorState e = unpack_error(dx);

    // Baro is slow/noisy. Keep it gentle.
    constexpr float MAX_DPZ_CORR = 0.05f; // m/update
    constexpr float MAX_DVZ_CORR = 0.05f; // m/s/update

    e.dp.x = 0.0f;
    e.dp.y = 0.0f;
    e.dv.x = 0.0f;
    e.dv.y = 0.0f;

    e.dtheta = Vec3{0, 0, 0};
    e.dab = Vec3{0, 0, 0};
    e.dwb = Vec3{0, 0, 0};

    e.dp.z = constrain(e.dp.z, -MAX_DPZ_CORR, MAX_DPZ_CORR);
    e.dv.z = constrain(e.dv.z, -MAX_DVZ_CORR, MAX_DVZ_CORR);

    inject(e);

    int newest = newest_buf_index();
    if (newest >= 0)
    {
        state_buf[newest].state = nominal;
        state_buf[newest].P = P;
    }
}

void ESKF::reset_zero_vxy(float sigma_mps)
{

    // used to zero velocity and avoid drift when no update is available
    // and drone is disarmed. Then we update v = 0 with low uncertainty
    if (sigma_mps <= 0.0f)
    {
        return;
    }

    // Measurement:
    //   z = [0, 0, 0]
    //   h = nominal.v
    // Residual:
    //   y = z - h = -nominal.v
    BLA::Matrix<3, 1> y = {
        -nominal.v.x,
        -nominal.v.y,
        -nominal.v.z};

    BLA::Matrix<3, 3> V;
    V.Fill(0.0f);
    V(0, 0) = sigma_mps * sigma_mps;
    V(1, 1) = sigma_mps * sigma_mps;
    V(2, 2) = sigma_mps * sigma_mps;

    // K = PH^T(HPH^T + V)⁻1 but optimized for
    // H = [0 I 0 0 0] to avoid zero and 1 multiplications
    // PHt = P * H.T = P[:, v]
    BLA::Matrix<15, 3> PHt = P.Submatrix<15, 3>(0, 3);

    // S = H * PHt + V = P[v, v] + V
    BLA::Matrix<3, 3> S = P.Submatrix<3, 3>(3, 3) + V;

    BLA::Matrix<3, 3> S_inv = Inverse(S);
    BLA::Matrix<15, 3> K = PHt * S_inv;

    BLA::Matrix<15, 1> dx = K * y;

    // Joseph covariance update.
    // Optimized for sparse H. This mess is only doing P = (I-KH) P (I-KH)^T + KVK^T
    BLA::Matrix<15, 15> KHP = K * P.Submatrix<3, 15>(3, 0);
    P = P - KHP - ~KHP + K * S * ~K;
    P = (P + ~P) * 0.5f; // Enforce symmetric covariance

    ErrorState e = unpack_error(dx);

    // Safety clamp: zero-velocity update should not inject crazy corrections.
    constexpr float MAX_DV_CORR = 0.50f; // m/s per update
    float dv_norm = sqrtf(dot(e.dv, e.dv));
    if (dv_norm > MAX_DV_CORR)
    {
        e.dv *= MAX_DV_CORR / dv_norm;
    }

    inject(e);
}

void ESKF::reset_baro_offset(float baro_alt_m)
{
    baro_offset_m = baro_alt_m + nominal.p.z;
}

void ESKF::push_buffer(const ImuData &imudata)
{
    state_buf[buf_head] = StateBuffer{
        imudata,
        nominal,
        P,
        imudata.timestamp};

    buf_head = (buf_head + 1) % STATE_BUF_SIZE;
    if (buf_count < STATE_BUF_SIZE)
    {
        buf_count++;
    }
}

int ESKF::get_closest_buf(uint32_t timestamp) const
{
    if (buf_count == 0)
    {
        return -1;
    }

    int closest_buf_idx = -1;
    uint32_t best_err = UINT32_MAX;

    // line search is fine, we have only max 50 or so
    for (uint32_t i = 0; i < buf_count; i++)
    {
        const StateBuffer &s = state_buf[i];
        uint32_t err = time_abs_diff_us(s.timestamp, timestamp);

        if (err < best_err)
        {
            best_err = err;
            closest_buf_idx = i;
        }
    }

    return closest_buf_idx;
}

int ESKF::newest_buf_index() const
{
    if (buf_count == 0)
    {
        return -1;
    }

    return (buf_head + STATE_BUF_SIZE - 1) % STATE_BUF_SIZE;
}

void ESKF::replay_from(int buf_idx)
{
    int newest = newest_buf_index();
    if (newest < 0 || buf_idx < 0)
    {
        return;
    }

    int cur = buf_idx;

    while (cur != newest)
    {
        int next = (cur + 1) % STATE_BUF_SIZE;

        if (propagate_core(state_buf[next].imudata))
        {
            //correct_gravity(state_buf[next].imudata.accel);
            state_buf[next].state = nominal;
            state_buf[next].P = P;
            state_buf[next].timestamp = state_buf[next].imudata.timestamp;
        }

        cur = next;
    }
}