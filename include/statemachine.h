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
        void update(const PPMCommand& cmd, const NominalState& state, float h_terrain);
        FlightState get_flightstate(){return flightstate;}
        FlightMode get_flightmode(){return flightmode;}

};