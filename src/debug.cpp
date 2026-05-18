#include "debug.h"

static const unsigned long DEBUG_LOG_PERIOD_US = 20000;

void debug::log(const Vec3 &value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    char buf[96];
    sprintf(buf, "%s x=%.4f y=%.4f z=%.4f", label, value.x, value.y, value.z);
    Serial.println(buf);
}

void debug::log(const VectInt16 &value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    char buf[96];
    sprintf(buf, "%s x=%d y=%d z=%d", label, value.x, value.y, value.z);
    Serial.println(buf);
}

void debug::log(const EulerAngle &value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    char buf[112];
    sprintf(buf, "%s yaw=%.4f pitch=%.4f roll=%.4f", label, value.yaw, value.pitch, value.roll);
    Serial.println(buf);
}

void debug::log(const Quaternion &value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    char buf[128];
    sprintf(buf, "%s x=%.4f y=%.4f z=%.4f w=%.4f", label, value.x, value.y, value.z, value.w);
    Serial.println(buf);
}

void debug::log(const RawImuData &value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    char header[64];
    sprintf(header, "%s", label);
    Serial.println(header);
    debug::log(value.accel, "  accel(raw)");
    debug::log(value.gyro, "  gyro(raw)");
}

void debug::log(const ImuData &value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    char header[64];
    sprintf(header, "%s", label);
    Serial.println(header);
    debug::log(value.accel, "  accel(g)");
    debug::log(value.gyro, "  gyro(dps)");
}

void debug::log(const PPMCommand &value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    char buf[220];
    sprintf(
        buf,
        "%s C1=%.2f C2=%.2f C3=%.2f C4=%.2f C5=%.2f C6=%.2f",
        label,
        value.C1,
        value.C2,
        value.C3,
        value.C4,
        value.C5,
        value.C6
    );
    Serial.println(buf);
}

void debug::log(const MTF02Data &value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    char buf[220];
    sprintf(
        buf,
        "%s t_ms=%lu dist_mm=%lu strength=%u precision=%u dstat=%u flow_x=%d flow_y=%d flow_q=%u flow_s=%u",
        label,
        (unsigned long)value.time_ms,
        (unsigned long)value.dist_mm,
        (unsigned int)value.strength,
        (unsigned int)value.precision,
        (unsigned int)value.dist_status,
        (int)value.flow_x,
        (int)value.flow_y,
        (unsigned int)value.flow_quality,
        (unsigned int)value.flow_status
    );
    Serial.println(buf);
}

void debug::plot(const Vec3 &value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    char buf[128];
    sprintf(
        buf,
        ">%s_x:%.4f,%s_y:%.4f,%s_z:%.4f",
        label,
        value.x,
        label,
        value.y,
        label,
        value.z
    );
    Serial.println(buf);
}

void debug::plot(const EulerAngle &value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    char buf[160];
    sprintf(
        buf,
        ">%s_yaw:%.4f,%s_pitch:%.4f,%s_roll:%.4f",
        label,
        value.yaw,
        label,
        value.pitch,
        label,
        value.roll
    );
    Serial.println(buf);
}
