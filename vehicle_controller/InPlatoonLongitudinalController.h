#pragma once
#include "LongitudinalController.h"

/* Controller for vehicles in a platoon (not platoon leader) 
NOT BEING USED */
class InPlatoonLongitudinalController :
    public LongitudinalController
{
public:
    InPlatoonLongitudinalController() = default;
	InPlatoonLongitudinalController(
		const VelocityControllerGains& velocity_controller_gains,
		const ConnectedGains& gains,
		double velocity_filter_gain, double time_headway_filter_gain,
		double filter_brake_limit, double comfortable_acceleration,
		double simulation_time_step,
		std::unordered_map<State, color_t> state_to_color_map,
		bool verbose);

private:
	GapController gap_controller;
    double implement_compute_desired_acceleration(const EgoVehicle& ego_vehicle,
		const NearbyVehicle*, double velocity_reference) override;
};

