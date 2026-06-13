#include "debug.h"

static const unsigned long DEBUG_LOG_PERIOD_US = 20000;

void debug::log(const Vec3 &value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    Serial.print(label);
    Serial.print(" x=");
    Serial.print(value.x, 4);
    Serial.print(" y=");
    Serial.print(value.y, 4);
    Serial.print(" z=");
    Serial.println(value.z, 4);
}

void debug::log(const VectInt16 &value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    Serial.print(label);
    Serial.print(" x=");
    Serial.print(value.x);
    Serial.print(" y=");
    Serial.print(value.y);
    Serial.print(" z=");
    Serial.println(value.z);
}

void debug::log(const EulerAngle &value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    Serial.print(label);
    Serial.print(" yaw=");
    Serial.print(value.yaw, 4);
    Serial.print(" pitch=");
    Serial.print(value.pitch, 4);
    Serial.print(" roll=");
    Serial.println(value.roll, 4);
}

void debug::log(const Quaternion &value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    Serial.print(label);
    Serial.print(" x=");
    Serial.print(value.x, 4);
    Serial.print(" y=");
    Serial.print(value.y, 4);
    Serial.print(" z=");
    Serial.print(value.z, 4);
    Serial.print(" w=");
    Serial.println(value.w, 4);
}

void debug::log(const RawImuData &value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    Serial.println(label);
    debug::log(value.accel, "  accel(raw)");
    debug::log(value.gyro, "  gyro(raw)");
}

void debug::log(const ImuData &value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    Serial.println(label);
    debug::log(value.accel, "  accel(g)");
    debug::log(value.gyro, "  gyro(dps)");
}

void debug::log(const PPMCommand &value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    Serial.print(label);
    Serial.print(" C1=");
    Serial.print(value.C1, 2);
    Serial.print(" C2=");
    Serial.print(value.C2, 2);
    Serial.print(" C3=");
    Serial.print(value.C3, 2);
    Serial.print(" C4=");
    Serial.print(value.C4, 2);
    Serial.print(" C5=");
    Serial.print(value.C5, 2);
    Serial.print(" C6=");
    Serial.print(value.C6, 2);
    Serial.print(" C7=");
    Serial.print(value.C7, 2);
    Serial.print(" C8=");
    Serial.print(value.C8, 2);
    Serial.print(" C9=");
    Serial.print(value.C9, 2);
    Serial.print(" C10=");
    Serial.println(value.C10, 2);
}

void debug::log(const MTF02Data &value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    Serial.print(label);
    Serial.print(" t_ms=");
    Serial.print((unsigned long)value.data.time_ms);
    Serial.print(" dist_mm=");
    Serial.print((unsigned long)value.data.dist_mm);
    Serial.print(" strength=");
    Serial.print((unsigned int)value.data.strength);
    Serial.print(" precision=");
    Serial.print((unsigned int)value.data.precision);
    Serial.print(" dstat=");
    Serial.print((unsigned int)value.data.dist_status);
    Serial.print(" flow_x=");
    Serial.print((int)value.data.flow_x);
    Serial.print(" flow_y=");
    Serial.print((int)value.data.flow_y);
    Serial.print(" flow_q=");
    Serial.print((unsigned int)value.data.flow_quality);
    Serial.print(" flow_s=");
    Serial.println((unsigned int)value.data.flow_status);
}

void debug::log(float value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    Serial.print(label);
    Serial.print(" ");
    Serial.println(value, 4);
}

void debug::log(const BLA::Matrix<3, 3> &value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    Serial.println(label);
    Serial.print("  [");
    Serial.print(value(0, 0), 4);
    Serial.print(" ");
    Serial.print(value(0, 1), 4);
    Serial.print(" ");
    Serial.print(value(0, 2), 4);
    Serial.println("]");
    Serial.print("  [");
    Serial.print(value(1, 0), 4);
    Serial.print(" ");
    Serial.print(value(1, 1), 4);
    Serial.print(" ");
    Serial.print(value(1, 2), 4);
    Serial.println("]");
    Serial.print("  [");
    Serial.print(value(2, 0), 4);
    Serial.print(" ");
    Serial.print(value(2, 1), 4);
    Serial.print(" ");
    Serial.print(value(2, 2), 4);
    Serial.println("]");
}

void debug::plot(const Vec3 &value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    Serial.print(">");
    Serial.print(label);
    Serial.print("_x:");
    Serial.print(value.x, 4);
    Serial.print(",");
    Serial.print(label);
    Serial.print("_y:");
    Serial.print(value.y, 4);
    Serial.print(",");
    Serial.print(label);
    Serial.print("_z:");
    Serial.println(value.z, 4);
}

void debug::plot(float value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    Serial.print(">");
    Serial.print(label);
    Serial.print(":");
    Serial.println(value, 4);
}

void debug::plot(const EulerAngle &value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    Serial.print(">");
    Serial.print(label);
    Serial.print("_yaw:");
    Serial.print(value.yaw, 4);
    Serial.print(",");
    Serial.print(label);
    Serial.print("_pitch:");
    Serial.print(value.pitch, 4);
    Serial.print(",");
    Serial.print(label);
    Serial.print("_roll:");
    Serial.println(value.roll, 4);
}
