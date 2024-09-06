#pragma once

#include "LongitudinalController.h"
#include "VanAremGapController.h"

class TalebpourALC:
	public LongitudinalController
{
public:
    TalebpourALC() = default;
    TalebpourALC(const EgoVehicle& ego_vehicle,
        double velocity_controller_gain,
        ConnectedGains connected_gains,
        double velocity_filter_gain, bool verbose);

private:
    VelocityController velocity_controller;
    VanAremGapController gap_controller;

    double max_jerk{ VIRDI_MAX_JERK }; // [m/s^3]
    double max_accel{ VIRDI_MAX_ACCEL };// [m/s^2]
    double min_accel{ VIRDI_MIN_ACCEL };// [m/s^2]

    /* Determines and sets the current state of the longitudinal controller */
    double compute_desired_acceleration(const EgoVehicle& ego_vehicle,
        const std::shared_ptr<NearbyVehicle> leader,
        double velocity_reference) override;
    double compute_max_safe_speed(const EgoVehicle& ego_vehicle,
        const std::shared_ptr<NearbyVehicle> leader);
    double filter_accel(double current_accel, double next_accel,
        double time_step);
};

