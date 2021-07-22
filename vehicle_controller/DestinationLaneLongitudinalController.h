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
    DestinationLaneLongitudinalController(const Vehicle& ego_vehicle,
        bool verbose);
    DestinationLaneLongitudinalController(const Vehicle& ego_vehicle);

    virtual void determine_controller_state(double ego_velocity,
        const NearbyVehicle* leader) override;

    bool is_active();

    double estimate_follower_time_headway(const NearbyVehicle& follower,
        double ego_max_brake, double ego_desired_velocity);
};

