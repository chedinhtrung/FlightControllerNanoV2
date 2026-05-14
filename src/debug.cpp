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
