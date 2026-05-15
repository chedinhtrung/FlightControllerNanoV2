#ifndef DEVICE_OPTICAL_FLOW_H
#define DEVICE_OPTICAL_FLOW_H

#include "drivers/mtf02.h"
#include "interfaces.h"

class OpticalFlow {
public:
    explicit OpticalFlow(OpticalFlowDriver &driver);

    bool setup();
    bool parse();
    bool has_bytes() const;
    bool read(MTF02Data &out);

private:
    OpticalFlowDriver &driver_;
};

#endif
