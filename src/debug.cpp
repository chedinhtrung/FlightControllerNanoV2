#include "debug.h"


void debug::log(const Vec3 &value, const char *label) {
    char buf[96];
    sprintf(buf, "%s x=%.4f y=%.4f z=%.4f", label, value.x, value.y, value.z);
    Serial.println(buf);
}

void debug::log(const VectInt16 &value, const char *label) {
    char buf[96];
    sprintf(buf, "%s x=%d y=%d z=%d", label, value.x, value.y, value.z);
    Serial.println(buf);
}

void debug::log(const EulerAngle &value, const char *label) {
    char buf[112];
    sprintf(buf, "%s yaw=%.4f pitch=%.4f roll=%.4f", label, value.yaw, value.pitch, value.roll);
    Serial.println(buf);
}

void debug::log(const Quaternion &value, const char *label) {
    char buf[128];
    sprintf(buf, "%s x=%.4f y=%.4f z=%.4f w=%.4f", label, value.x, value.y, value.z, value.w);
    Serial.println(buf);
}

void debug::log(const RawImuData &value, const char *label) {
    char header[64];
    sprintf(header, "%s", label);
    Serial.println(header);
    debug::log(value.accel, "  accel(raw)");
    debug::log(value.gyro, "  gyro(raw)");
}

void debug::log(const ImuData &value, const char *label) {
    char header[64];
    sprintf(header, "%s", label);
    Serial.println(header);
    debug::log(value.accel, "  accel(g)");
    debug::log(value.gyro, "  gyro(dps)");
}

void debug::log(const ReceiverData &value, const char *label) {
    char buf[220];
    sprintf(
        buf,
        "%s thr=%.2f roll=%.2f pitch=%.2f yaw=%.2f aux5=%.2f aux6=%.2f",
        label,
        value.ThrottleIn,
        value.RollIn,
        value.PitchIn,
        value.YawIn,
        value.AuxChannel5In,
        value.AuxChannel6In
    );
    Serial.println(buf);
}

void debug::log(const MTF02Data &value, const char *label) {
    char buf[220];
    sprintf(
        buf,
        "%s t_ms=%lu dist_mm=%lu strength=%u precision=%u dstat=%u flow_x=%d flow_y=%d flow_q=%u flow_s=%u",
        label,
        (unsigned long)value.time_ms,
        (unsigned long)value.distance_mm,
        (unsigned int)value.strength,
        (unsigned int)value.precision,
        (unsigned int)value.distance_status,
        (int)value.flow_vel_x,
        (int)value.flow_vel_y,
        (unsigned int)value.flow_quality,
        (unsigned int)value.flow_status
    );
    Serial.println(buf);
}
