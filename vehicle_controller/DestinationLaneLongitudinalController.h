/*==========================================================================*/
/* DestinationLaneLongitudinalController.h     								*/
/* Controller to perform longitudinal adjustments before lane changes based */
/* on the constant time headway policy                                      */
/*                                                                          */
/* Version of 2021-xx-xx                             Fernando V. Monteiro   */
/*==========================================================================*/

#pragma once
#include "LongitudinalController.h"
class DestinationLaneLongitudinalController :
    public LongitudinalController
{
public:

    DestinationLaneLongitudinalController();
    DestinationLaneLongitudinalController(const EgoVehicle& ego_vehicle,
        bool verbose);
    DestinationLaneLongitudinalController(const EgoVehicle& ego_vehicle);

    double get_follower_time_headway() {
        return destination_lane_follower_time_headway;
    }
    
    void set_timer_start(double time) {
        this->timer_start = time;
    }

    void set_reference_velocity(double ego_velocity, double adjustment_speed_factor);

    virtual void determine_controller_state(double ego_velocity,
        const std::shared_ptr<NearbyVehicle> leader) override;
    bool is_active() const;
    /* Checks whether the filtered reference velocity is greater than
    the ego velocity. Method should only be called when the 
    destination lane controller is NOT the active one. */
    bool is_outdated(double ego_velocity) const;
    void estimate_follower_time_headway(const NearbyVehicle& follower,
        double ego_max_brake, double follower_free_flow_velocity);
    /* Changes the accepted risk if necessary and returns 
    true if it made any changes*/
    bool update_accepted_risk(double time, const EgoVehicle& ego_vehicle);

private:
    /* Estimated value of the time headway used by the follower at
    the destination lane. Used when computing lane change safe gaps. */
    double destination_lane_follower_time_headway{ 0.0 };
};

