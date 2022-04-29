/*==========================================================================*/
/* DestinationLaneLongitudinalController.h     								*/
/* Controller to perform longitudinal adjustments before lane changes based */
/* on the constant time headway policy                                      */
/*                                                                          */
/* Version of 2021-xx-xx                             Fernando V. Monteiro   */
/*==========================================================================*/

#pragma once
#include "LongitudinalController.h"

/* Longitudinal controller to follow a virtual leader, that is,
a vehicle on a different lane. This controller is not as aggressive as the
RealLongitudinalController, since it is not safety critical, and it
is deactivated if there is no virtual leader. */
class VirtualLongitudinalController :
    public LongitudinalController
{
public:

    VirtualLongitudinalController();
    VirtualLongitudinalController(
        const VehicleParameters& ego_parameters,
        VelocityControllerGains velocity_controller_gains,
        AutonomousGains autonomous_gains, ConnectedGains connected_gains, 
        bool verbose);
    VirtualLongitudinalController(
        const VehicleParameters& ego_parameters, 
        VelocityControllerGains velocity_controller_gains,
        AutonomousGains autonomous_gains, ConnectedGains connected_gains);

    double get_follower_time_headway() const {
        return follower_time_headway;
    };
    
    void set_follower_time_headway(double h) {
        this->follower_time_headway = h;
    };
    void set_timer_start(double time) {
        this->timer_start = time;
    };

    //void set_reference_velocity(double reference_velocity, double ego_velocity);
    //void set_reference_velocity(double ego_velocity, double adjustment_speed_factor);

    virtual void determine_controller_state(const EgoVehicle& ego_vehicle,
        const std::shared_ptr<NearbyVehicle> leader,
        double reference_velocity) override;
    bool is_active() const;
    /* Checks whether the filtered reference velocity is greater than
    the ego velocity. Method should only be called when the 
    destination lane controller is NOT the active one. */
    bool is_outdated(double ego_velocity) const;
    /*void estimate_follower_time_headway(const NearbyVehicle& follower,
        double ego_max_brake, double follower_free_flow_velocity);*/
    /* Changes the accepted risk if necessary and returns 
    true if it made any changes*/
    bool update_accepted_risk(double time, const EgoVehicle& ego_vehicle);
    /* Computes the risk at which the lane change headway becomes equal to
    the vehicle following headway. */
    /*void compute_intermediate_risk_to_leader(double lambda_1, 
        double lane_change_lambda_1, double max_brake_no_lane_change,
        double leader_max_brake);*/
    void reset_accepted_risks() override;
    void compute_max_risk_to_follower(double follower_max_brake);

private:
    /* Estimated value of the time headway used by the follower at
    the destination lane. Used when computing lane change safe gaps. */
    double follower_time_headway{ 0.0 };

    /* Risk at which the lane change headway becomes equal to the
    vehicle following headway. */
    double intermediate_risk_to_leader{ 0.0 }; // [m/s] 
    double accepted_risk_to_follower{ initial_risk }; // [m/s]
    double max_risk_to_follower{ 0.0 }; // [m/s]
};

