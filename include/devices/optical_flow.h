#ifndef DEVICE_OPTICAL_FLOW_H
#define DEVICE_OPTICAL_FLOW_H

#include "drivers/mtf02.h"
#include "interfaces.h"
#include "datastructs.h"
#include "lpf.h"

class OpticalFlow {
public:
    explicit OpticalFlow(OpticalFlowDriver &driver);

    bool setup();
    bool kick();
    bool has_bytes() const;
    bool read(MTF02Data &out);

    Vec3WithTrust get_compensated_v1frame_vxy(const MTF02Data &flowdata, const Vec3 gyro, const Quaternion& q);
    FloatWithTrust get_compensated_vz(float range_m, const Vec3& accel, const Quaternion &q);
    
    Vec3LPF gyro_lpf;
    Vec3LPF flow_lpf;

private:
    OpticalFlowDriver &driver_;
    Vec3 last_flow_radps_{0.0f, 0.0f, 0.0f};
    uint32_t last_flow_ms_ = 0;

    bool flow_reject_initialized_ = false;
    bool range_vz_initialized_ = false;
    float last_height_m_ = 0.015f;
    float last_range_vz = 0.0f;
    uint32_t last_range_update_us_ = 0;
};

#endif
