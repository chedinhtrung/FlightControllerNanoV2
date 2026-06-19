#include "devices/optical_flow.h"
#include "config.h"
#include "mathutils.h"
#include "debug.h"

OpticalFlow::OpticalFlow(OpticalFlowDriver &driver) : driver_(driver) {}

bool OpticalFlow::setup()
{
    if (!driver_.setup())
    {
        // Placeholder: optional optical flow setup error handling.
        return false;
    }

    return true;
}

bool OpticalFlow::kick()
{
    return driver_.parse();
}

bool OpticalFlow::has_bytes() const
{
    return driver_.has_bytes();
}

bool OpticalFlow::read(MTF02Data &out)
{
    if (driver_.read(out))
    {
        out.timestamp -= FLOW_DELAY_US; // adjust back to real world timestamp of the measured state
        return true;
    }
    return false;
}
