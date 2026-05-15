#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>
#include "datastructs.h"
#include "drivers/mtf02.h"
#include "drivers/mpu9250.h"
#include "drivers/receiver.h"

namespace debug {
void log(const Vec3 &value, const char *label = "Vect");
void log(const VectInt16 &value, const char *label = "VectInt16");
void log(const EulerAngle &value, const char *label = "EulerAngle");
void log(const Quaternion &value, const char *label = "Quaternion");
void log(const RawImuData &value, const char *label = "RawImuData");
void log(const ImuData &value, const char *label = "ImuData");
void log(const ReceiverData &value, const char *label = "Control");
void log(const MTF02Data &value, const char *label = "MTF02");
}

#endif
