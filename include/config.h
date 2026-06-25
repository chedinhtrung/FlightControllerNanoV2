#ifndef CONFIG 
#define CONFIG

#define DEBUG 1
#include "datastructs.h"

// Configurations 

constexpr int LOOP_HZ = 250;

constexpr int PERIOD_US = 1000000UL / LOOP_HZ;
constexpr float DT = 1.0/LOOP_HZ;

constexpr int LOG_RATE_HZ = 30;
constexpr int PERIOD_LOG_US = 1000000UL / LOG_RATE_HZ;

constexpr Vec3 R_G_TO_FLOW = {-0.073, -0.004, 0.002};  // drone COM to sensor 

// Constants

constexpr Vec3 GRAVITY_EARTH {0, 0, 9.81};

constexpr float G_EARTH_MPS2 = 9.81f;

#define RAD_PER_DEG 0.0174533f
#define DEG_PER_RAD 57.295779f
#define MW_BETA 0.08f


#endif