#include "devices/optical_flow.h"

OpticalFlow::OpticalFlow(OpticalFlowDriver &driver) : driver_(driver) {}

bool OpticalFlow::setup() {
    if (!driver_.setup()) {
        // Placeholder: optional optical flow setup error handling.
        return false;
    }
    return true;
}

bool OpticalFlow::kick() {
    return driver_.parse();
}

bool OpticalFlow::has_bytes() const {
    return driver_.has_bytes();
}

bool OpticalFlow::read(MTF02Data &out) {
    return driver_.read(out);
}
