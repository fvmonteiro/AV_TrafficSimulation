#pragma once
#include "CAVController.h"
#include "SimpleLongitudinalController.h"

class PlatoonVehicle;

/* This class'implementation is significantly different from previous 
controllers because it implements controllers for a different paper. */

class PlatoonVehicleController : public CAVController
{
public:

	PlatoonVehicleController::PlatoonVehicleController(
		const PlatoonVehicle* platoon_vehicle, bool verbose);

	//bool get_destination_lane_desired_acceleration_when_in_platoon(
	//	std::unordered_map<ALCType, double>& possible_accelerations);

protected:
	//PlatoonVehicleController::PlatoonVehicleController(bool verbose)
	//	: CAVController(verbose) {};

private:
	const PlatoonVehicle* platoon_vehicle{ nullptr };
	SimpleLongitudinalController real_leader_controller;
	SimpleLongitudinalController virtual_leader_controller;

	double platoon_velocity_filter_gain{ 10.0 };     
	double platoon_time_headway_filter_gain{ 0.0 }; /* pass all */
	AutonomousGains platoon_vehicle_autonomous_gains{ 0.2, 0.5 };
	ConnectedGains platoon_vehicle_connected_gains{ 0.2, 0.5, 0.0, 0.0 };
	VelocityControllerGains platoon_vehicle_velocity_gains{ 0.5, 0.0, 0.0 };

	std::unordered_map<LongitudinalController::State, color_t>
		orig_lane_colors =
	{
		{ LongitudinalController::State::velocity_control, GREEN },
		{ LongitudinalController::State::vehicle_following, DARK_GREEN },
	};

	void implement_add_internal_controllers() override;
	double implement_get_desired_acceleration() override;
	//void implement_update_origin_lane_controller(
	//	const NearbyVehicle& real_leader) override;
};
