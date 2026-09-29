#include "debug.h"

static const unsigned long DEBUG_LOG_PERIOD_US = 20000;

void debug::log(const Vec3 &value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    DBG_SERIAL.print(label);
    DBG_SERIAL.print(" x=");
    DBG_SERIAL.print(value.x, 4);
    DBG_SERIAL.print(" y=");
    DBG_SERIAL.print(value.y, 4);
    DBG_SERIAL.print(" z=");
    DBG_SERIAL.println(value.z, 4);
}

void debug::log(const VectInt16 &value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    DBG_SERIAL.print(label);
    DBG_SERIAL.print(" x=");
    DBG_SERIAL.print(value.x);
    DBG_SERIAL.print(" y=");
    DBG_SERIAL.print(value.y);
    DBG_SERIAL.print(" z=");
    DBG_SERIAL.println(value.z);
}

void debug::log(const EulerAngle &value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    DBG_SERIAL.print(label);
    DBG_SERIAL.print(" yaw=");
    DBG_SERIAL.print(value.yaw, 4);
    DBG_SERIAL.print(" pitch=");
    DBG_SERIAL.print(value.pitch, 4);
    DBG_SERIAL.print(" roll=");
    DBG_SERIAL.println(value.roll, 4);
}

void debug::log(const Quaternion &value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    DBG_SERIAL.print(label);
    DBG_SERIAL.print(" x=");
    DBG_SERIAL.print(value.x, 4);
    DBG_SERIAL.print(" y=");
    DBG_SERIAL.print(value.y, 4);
    DBG_SERIAL.print(" z=");
    DBG_SERIAL.print(value.z, 4);
    DBG_SERIAL.print(" w=");
    DBG_SERIAL.println(value.w, 4);
}

void debug::log(const RawImuData &value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    DBG_SERIAL.println(label);
    debug::log(value.accel, "  accel(raw)");
    debug::log(value.gyro, "  gyro(raw)");
}

void debug::log(const ImuData &value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    DBG_SERIAL.println(label);
    debug::log(value.accel, "  accel(g)");
    debug::log(value.gyro, "  gyro(dps)");
}

void debug::log(const PPMCommand &value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    DBG_SERIAL.print(label);
    DBG_SERIAL.print(" C1=");
    DBG_SERIAL.print(value.C1, 2);
    DBG_SERIAL.print(" C2=");
    DBG_SERIAL.print(value.C2, 2);
    DBG_SERIAL.print(" C3=");
    DBG_SERIAL.print(value.C3, 2);
    DBG_SERIAL.print(" C4=");
    DBG_SERIAL.print(value.C4, 2);
    DBG_SERIAL.print(" C5=");
    DBG_SERIAL.print(value.C5, 2);
    DBG_SERIAL.print(" C6=");
    DBG_SERIAL.print(value.C6, 2);
    DBG_SERIAL.print(" C7=");
    DBG_SERIAL.print(value.C7, 2);
    DBG_SERIAL.print(" C8=");
    DBG_SERIAL.print(value.C8, 2);
    DBG_SERIAL.print(" C9=");
    DBG_SERIAL.print(value.C9, 2);
    DBG_SERIAL.print(" C10=");
    DBG_SERIAL.println(value.C10, 2);
}

void debug::log(const MTF02Data &value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    DBG_SERIAL.print(label);
    DBG_SERIAL.print(" t_ms=");
    DBG_SERIAL.print((unsigned long)value.data.time_ms);
    DBG_SERIAL.print(" dist_mm=");
    DBG_SERIAL.print((unsigned long)value.data.dist_mm);
    DBG_SERIAL.print(" strength=");
    DBG_SERIAL.print((unsigned int)value.data.strength);
    DBG_SERIAL.print(" precision=");
    DBG_SERIAL.print((unsigned int)value.data.precision);
    DBG_SERIAL.print(" dstat=");
    DBG_SERIAL.print((unsigned int)value.data.dist_status);
    DBG_SERIAL.print(" flow_x=");
    DBG_SERIAL.print((int)value.data.flow_x);
    DBG_SERIAL.print(" flow_y=");
    DBG_SERIAL.print((int)value.data.flow_y);
    DBG_SERIAL.print(" flow_q=");
    DBG_SERIAL.print((unsigned int)value.data.flow_quality);
    DBG_SERIAL.print(" flow_s=");
    DBG_SERIAL.println((unsigned int)value.data.flow_status);
}

void debug::log(float value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    DBG_SERIAL.print(label);
    DBG_SERIAL.print(" ");
    DBG_SERIAL.println(value, 4);
}

void debug::log(const BLA::Matrix<3, 3> &value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    DBG_SERIAL.println(label);
    DBG_SERIAL.print("  [");
    DBG_SERIAL.print(value(0, 0), 4);
    DBG_SERIAL.print(" ");
    DBG_SERIAL.print(value(0, 1), 4);
    DBG_SERIAL.print(" ");
    DBG_SERIAL.print(value(0, 2), 4);
    DBG_SERIAL.println("]");
    DBG_SERIAL.print("  [");
    DBG_SERIAL.print(value(1, 0), 4);
    DBG_SERIAL.print(" ");
    DBG_SERIAL.print(value(1, 1), 4);
    DBG_SERIAL.print(" ");
    DBG_SERIAL.print(value(1, 2), 4);
    DBG_SERIAL.println("]");
    DBG_SERIAL.print("  [");
    DBG_SERIAL.print(value(2, 0), 4);
    DBG_SERIAL.print(" ");
    DBG_SERIAL.print(value(2, 1), 4);
    DBG_SERIAL.print(" ");
    DBG_SERIAL.print(value(2, 2), 4);
    DBG_SERIAL.println("]");
}

void debug::plot(const Vec3 &value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    DBG_SERIAL.print(">");
    DBG_SERIAL.print(label);
    DBG_SERIAL.print("_x:");
    DBG_SERIAL.print(value.x, 4);
    DBG_SERIAL.print(",");
    DBG_SERIAL.print(label);
    DBG_SERIAL.print("_y:");
    DBG_SERIAL.print(value.y, 4);
    DBG_SERIAL.print(",");
    DBG_SERIAL.print(label);
    DBG_SERIAL.print("_z:");
    DBG_SERIAL.println(value.z, 4);
}

void debug::plot(float value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    DBG_SERIAL.print(">");
    DBG_SERIAL.print(label);
    DBG_SERIAL.print(":");
    DBG_SERIAL.println(value, 4);
}

void debug::plot(const EulerAngle &value, const char *label) {
    static unsigned long last_log_us = 0;
    unsigned long now = micros();
    if ((unsigned long)(now - last_log_us) < DEBUG_LOG_PERIOD_US) return;
    last_log_us = now;

    DBG_SERIAL.print(">");
    DBG_SERIAL.print(label);
    DBG_SERIAL.print("_yaw:");
    DBG_SERIAL.print(value.yaw, 4);
    DBG_SERIAL.print(",");
    DBG_SERIAL.print(label);
    DBG_SERIAL.print("_pitch:");
    DBG_SERIAL.print(value.pitch, 4);
    DBG_SERIAL.print(",");
    DBG_SERIAL.print(label);
    DBG_SERIAL.print("_roll:");
    DBG_SERIAL.println(value.roll, 4);
}
