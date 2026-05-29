#include "statemachine.h"

void StateMachine::update(const PPMCommand &cmd, const NominalState &state, float h_terrain)
{
    // Hard disarm / motor lock.
    // Since C3 is normalized throttle/stick, near-zero means motors are not allowed.
    if (cmd.C3 < 0.005f)
    {
        flightstate = DISARMED;
        return;
    }

    /*
    // Decode flight mode from a 3-position switch.
    // TODO: flightmode switch channel
    if (cmd.C6 < -0.33f)
    {
        flightmode = ANGLE;
    }
    else if (cmd.C6 < 0.33f)
    {
        flightmode = VXY;
    }
    else
    {
        flightmode = VXYZ;
    }
    */

    // These thresholds are deliberately conservative.
    // ESKF z is positive down, so above takeoff point means p.z is negative.
    constexpr float TAKEOFF_CLEAR_HEIGHT_M = 0.03f;
    constexpr float LAND_STICK_LOW = 0.05f;
    constexpr float LAND_CANCEL_STICK = 0.10f;
    constexpr float LAND_VZ_STOPPED_MPS = 0.03f;
    constexpr uint32_t LAND_STOPPED_COUNT_NEEDED = 250;

    static uint32_t landed_counter = 0;

    const bool height_cleared_ground =
        -state.p.z - h_terrain > TAKEOFF_CLEAR_HEIGHT_M;

    const bool landing_requested =
        cmd.C3 < LAND_STICK_LOW;

    const bool landing_cancel_requested =
        cmd.C3 > LAND_CANCEL_STICK;

    const bool vz_stopped =
        fabsf(state.v.z) < LAND_VZ_STOPPED_MPS;

    switch (flightstate)
    {
    case DISARMED:
        // We already know C3 is not near zero, so arm.
        // In ARMED, throttle is manual until takeoff (ground clearance).
        flightstate = ARMED;
        landed_counter = 0;
        break;

    case ARMED:
        if (height_cleared_ground)
        {
            flightstate = AIRBORNE;
            landed_counter = 0;
        }
        break;

    case TAKEOFF:
        // Not actively used
        if (height_cleared_ground)
        {
            flightstate = AIRBORNE;
            landed_counter = 0;
        }
        break;

    case AIRBORNE:
        // In VXYZ, low throttle stick means request controlled landing.
        if (flightmode == VXYZ && landing_requested)
        {
            flightstate = LANDING;
            landed_counter = 0;
        }
        break;

    case LANDING:
        // Raising stick cancels landing and returns to normal airborne VZ control.
        if (landing_cancel_requested)
        {
            flightstate = AIRBORNE;
            landed_counter = 0;
            break;
        }

        // If we keep asking to go down but vertical speed has stopped,
        // assume touchdown after debounce.
        if (landing_requested && vz_stopped)
        {
            if (landed_counter < LAND_STOPPED_COUNT_NEEDED)
            {
                landed_counter++;
            }

            if (landed_counter >= LAND_STOPPED_COUNT_NEEDED)
            {
                flightstate = ARMED;
                landed_counter = 0;
            }
        }
        else
        {
            landed_counter = 0;
        }
        break;
    }
}