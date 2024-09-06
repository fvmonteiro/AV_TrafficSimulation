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
	std::shared_ptr<SimpleLongitudinalController> real_leader_controller;
	std::shared_ptr<SimpleLongitudinalController> virtual_leader_controller;

	double platoon_velocity_filter_gain{ 10.0 };     
	double platoon_time_headway_filter_gain{ 1000.0 }; /* pass all */
	AutonomousGains platoon_vehicle_autonomous_gains{ 0.2, 0.5 };
	ConnectedGains platoon_vehicle_connected_gains{ 0.2, 0.5, 0.0, 0.0 };
	VelocityControllerGains platoon_vehicle_velocity_gains{ 0.5, 0.0, 0.0 };

	std::unordered_map<LongitudinalController::State, color_t>
		real_leader_controller_colors =
	{
		{ LongitudinalController::State::velocity_control, GREEN },
		{ LongitudinalController::State::vehicle_following, DARK_GREEN },
	};

	std::unordered_map<LongitudinalController::State, color_t>
		virtual_leader_controller_colors =
	{
		{ LongitudinalController::State::velocity_control, LIGHT_BLUE },
		{ LongitudinalController::State::vehicle_following, BLUE },
	};

	void implement_add_internal_controllers() override;
	double implement_get_desired_acceleration() override;
	void implement_update_origin_lane_controller(
		const NearbyVehicle& real_leader) override;
	/* This is a misnomer: we are updating the virtual leader controller,
	which follows either the desired dest lane leader or the assisted
	vehicle. */
	virtual void implement_update_destination_lane_controller(
		const NearbyVehicle& virtual_leader);
	double compute_destination_lane_follower_time_headway(
		NearbyVehicle& dest_lane_follower) override;
};
