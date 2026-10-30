#pragma once 
#include "datastructs.h"
#include "config.h"
#include "devices/receiver.h"
#include "eskf.h"

enum FlightMode{
    ANGLE,
    VXY,
    VXYZ
};

enum FlightState {
    DISARMED,
    ARMED, 
    TAKEOFF,
    AIRBORNE,
    LANDING
};


class StateMachine {
    private:
        FlightMode flightmode = VXYZ;
        FlightState flightstate = DISARMED;
    
    public: 
        void update(const PPMCommand& cmd, const NominalState& state, float h_terrain,
                    bool new_valid_cmd);
        inline FlightState get_flightstate(){return flightstate;}
        inline FlightMode get_flightmode(){return flightmode;}

        inline bool is_motors_allowed() const {
            return flightstate != DISARMED;
        }

        inline bool is_ground_state() const {
            return flightstate == DISARMED || flightstate == ARMED;
        }

        inline bool is_manual_attitude() const {
            return flightmode == ANGLE || is_ground_state() || flightstate == TAKEOFF;
        }

        inline bool is_manual_throttle() const {
            return is_ground_state() || flightstate == TAKEOFF || flightmode != VXYZ;
        }

        inline bool use_vz_control() const {
            return is_motors_allowed() && !is_manual_throttle();
        }

};
