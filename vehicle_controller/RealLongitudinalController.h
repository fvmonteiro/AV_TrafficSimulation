/*==========================================================================*/
/* OriginLaneLongitudinalController.h    									*/
/* Origin Lane (usual) Adaptive Cruise controller using the constant time   */
/* headway policy                                                           */
/*                                                                          */
/* Version of 2021-xx-xx                             Fernando V. Monteiro   */
/*==========================================================================*/

#pragma once
#include "SwitchedLongitudinalController.h"

/* Longitudinal controller to follow "real" leaders, that is, vehicles
on the same lane as the ego vehicle. */
class RealLongitudinalController :
    public SwitchedLongitudinalController
{

public:
    RealLongitudinalController();
    RealLongitudinalController(const EgoVehicle& ego_vehicle,
        VelocityControllerGains velocity_controller_gains,
        AutonomousGains autonomous_gains, ConnectedGains connected_gains,
        double velocity_filter_gain, double time_headway_filter_gain,
        std::unordered_map<State, color_t> state_to_color_map,
        bool verbose);
    /*RealLongitudinalController(const EgoVehicle& ego_vehicle,
        VelocityControllerGains velocity_controller_gains,
        AutonomousGains autonomous_gains, ConnectedGains connected_gains,
        double velocity_filter_gain, double time_headway_filter_gain);*/

    /* To be used ONLY when the compute_desired_acceleration method is not
    called. So far, only necessary when we allow VISSIM to take control 
    of the vehicle. */
    void update_leader_velocity_filter(double leader_velocity);

    /* Determines and sets the current state of the longitudinal controller */
    void determine_controller_state(const EgoVehicle& ego_vehicle,
        const NearbyVehicle* leader,
        double reference_velocity, double gap_control_input) override;

};
