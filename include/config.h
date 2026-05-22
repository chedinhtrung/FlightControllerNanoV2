#ifndef CONFIG 
#define CONFIG

#define DEBUG 1
#include "datastructs.h"

constexpr int LOOP_HZ = 250;

constexpr int PERIOD_US = 1000000UL / LOOP_HZ;
constexpr float DT = 1.0/LOOP_HZ;

constexpr int LOG_RATE_HZ = 30;
constexpr int PERIOD_LOG_US = 1000000UL / LOG_RATE_HZ;

constexpr Vec3 R_G_TO_FLOW = {-0.054, -0.005, 0.002};  // drone COM to sensor 

#define RAD_PER_DEG 0.0174533f
#define DEG_PER_RAD 57.295779f
#define MW_BETA 0.08f


#endif