#pragma once

#include "LongitudinalController.h"
#include "VanAremGapController.h"

class TalebpourALC:
	public LongitudinalController
{
public:
    TalebpourALC(const EgoVehicle& ego_vehicle,
        double velocity_controller_gain,
        ConnectedGains connected_gains,
        double velocity_filter_gain, bool verbose);

private:
    VelocityController velocity_controller;
    VanAremGapController gap_controller;

    /* Determines and sets the current state of the longitudinal controller */
    double compute_desired_acceleration(const EgoVehicle& ego_vehicle,
        const std::shared_ptr<NearbyVehicle> leader,
        double velocity_reference) override;
    double compute_max_safe_speed(const EgoVehicle& ego_vehicle,
        const std::shared_ptr<NearbyVehicle> leader);
};

