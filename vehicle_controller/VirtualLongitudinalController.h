/*==========================================================================*/
/* DestinationLaneLongitudinalController.h     								*/
/* Controller to perform longitudinal adjustments before lane changes based */
/* on the constant time headway policy                                      */
/*                                                                          */
/* Version of 2021-xx-xx                             Fernando V. Monteiro   */
/*==========================================================================*/

#pragma once
#include "SwitchedLongitudinalController.h"

/* Longitudinal controller to follow a virtual leader, that is,
a vehicle on a different lane. This controller is not as aggressive as the
RealLongitudinalController, since it is not safety critical, and it
is deactivated if there is no virtual leader. */
class VirtualLongitudinalController :
    public SwitchedLongitudinalController
{
public:

    VirtualLongitudinalController() = default;
    VirtualLongitudinalController(
        const EgoVehicle* ego_vehicle,
        std::unordered_map<State, color_t> state_to_color_map,
        bool verbose);

    /* Changes the accepted risk if necessary and returns 
    true if it made any changes*/
    //bool update_accepted_risk(double time, const EgoVehicle& ego_vehicle);
    /* Computes the risk at which the lane change headway becomes equal to
    the vehicle following headway. */
    /*void compute_intermediate_risk_to_leader(double lambda_1, 
        double lane_change_lambda_1, double max_brake_no_lane_change,
        double leader_max_brake);*/
    //void compute_max_risk_to_follower(double follower_max_brake);

private:

    double get_max_accepted_brake() const override;
    void determine_controller_state(const NearbyVehicle* leader,
        double reference_velocity) override;
    bool implement_is_velocity_reference_outdated() const override;
};

