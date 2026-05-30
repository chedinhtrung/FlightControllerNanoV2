#include "statemachine.h"

void StateMachine::update(const PPMCommand &cmd,
                          const NominalState &state,
                          float h_terrain)
{
    // Hard disarm / motor lock.
    // Your normal lowest stick is around 0.04, while the disarm switch forces 0.
    if (cmd.C3 < 0.005f)
    {
        flightstate = DISARMED;
        return;
    }

    // TODO: Decode flight mode from switch later.
    // For now keep whatever flightmode was already set, default VXYZ.

    constexpr float TAKEOFF_HEIGHT_AGL_M = 0.04f;
    constexpr float TAKEOFF_THROTTLE_INTENT = 0.15f;

    constexpr float LAND_HEIGHT_AGL_M = 0.10f;
    constexpr float LAND_STICK_LOW = 0.10f;
    constexpr float LAND_CANCEL_STICK = 0.10f;

    constexpr float LAND_VZ_STOPPED_MPS = 0.05f;
    constexpr uint32_t LAND_STOPPED_COUNT_NEEDED = 10;

    static uint32_t landed_counter = 0;

    const float height_agl = -state.p.z - h_terrain;

    const bool takeoff_detected =
        height_agl > TAKEOFF_HEIGHT_AGL_M &&
        cmd.C3 > TAKEOFF_THROTTLE_INTENT;

    const bool landing_requested =
        cmd.C3 < LAND_STICK_LOW &&
        height_agl < LAND_HEIGHT_AGL_M;

    const bool landing_cancel_requested =
        cmd.C3 > LAND_CANCEL_STICK;

    const bool vz_stopped =
        fabsf(state.v.z) < LAND_VZ_STOPPED_MPS;

    switch (flightstate)
    {
    case DISARMED:
        // C3 is no longer zero, so motors are allowed.
        // Stay manual throttle until takeoff is detected.
        flightstate = ARMED;
        landed_counter = 0;
        break;

    case ARMED:
        if (takeoff_detected)
        {
            flightstate = AIRBORNE;
            landed_counter = 0;
        }
        break;

    case TAKEOFF:
        // Not used in this simplified version.
        flightstate = AIRBORNE;
        landed_counter = 0;
        break;

    case AIRBORNE:
        // Do NOT return to ARMED just because height estimate/range changes.
        // Only enter LANDING by explicit low-stick intent near ground.
        if (flightmode == VXYZ && landing_requested)
        {
            flightstate = LANDING;
            landed_counter = 0;
        }
        break;

    case LANDING:
        // Raising stick above 0.1 cancels landing and returns to airborne.
        if (landing_cancel_requested)
        {
            flightstate = AIRBORNE;
            landed_counter = 0;
            break;
        }

        // If low stick persists and vertical speed is stopped,
        // declare landed after 10 iterations.
        if (cmd.C3 < LAND_STICK_LOW && vz_stopped)
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