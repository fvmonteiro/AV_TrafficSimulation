#pragma once
#include "CAVController.h"

class PlatoonVehicle;

class PlatoonVehicleController : public CAVController
{
public:

	PlatoonVehicleController::PlatoonVehicleController(
		const PlatoonVehicle* platoon_vehicle, bool verbose);

	//const RealLongitudinalController* get_real_leader_controller() const {
	//	return real_leader_controller;
	//}
	//const RealLongitudinalController& get_virtual_leader_controller() const {
	//	return virtual_leader_controller;
	//}

	bool get_destination_lane_desired_acceleration_when_in_platoon(
		std::unordered_map<ALCType, double>& possible_accelerations);

protected:
	//PlatoonVehicleController::PlatoonVehicleController(bool verbose)
	//	: CAVController(verbose) {};

private:
	const PlatoonVehicle* platoon_vehicle{ nullptr };
	/*RealLongitudinalController* real_leader_controller{ nullptr };
	RealLongitudinalController virtual_leader_controller;*/

	double platoon_velocity_filter_gain{ 10.0 };     
	double platoon_time_headway_filter_gain{ 0.0 }; /* pass all */
	AutonomousGains platoon_vehicle_autonomous_gains{ 0.2, 0.5 };
	ConnectedGains platoon_vehicle_connected_gains{ 0.2, 0.5, 0.0, 0.0 };
	VelocityControllerGains platoon_vehicle_velocity_gains{ 0.5, 0.0, 0.0 };

	std::unordered_map<LongitudinalController::State, color_t>
		in_platoon_colors =
	{
		{ LongitudinalController::State::velocity_control, DARK_GRAY },
		{ LongitudinalController::State::vehicle_following, WHITE},
	};

	/* Jan 24, 24 tests: keep the CAV controllers and check only coordinated
	lane changing */
	//void implement_add_internal_controllers() override;
	//double implement_get_desired_acceleration() override;
	//void implement_update_origin_lane_controller(
	//	const NearbyVehicle& real_leader) override;
};
