#pragma once
#include "CAVController.h"

class PlatoonVehicle;

class PlatoonVehicleController : public CAVController
{
public:
	PlatoonVehicleController::PlatoonVehicleController(
		const PlatoonVehicle& platoon_vehicle, bool verbose);

	double get_desired_acceleration(const PlatoonVehicle& platoon_vehicle);

	/* TODO: instead of overloading the method, we should deal with this
	via polymorphism, but that'll be a long refactoring process */
	void update_origin_lane_controller(const PlatoonVehicle& platoon_vehicle,
		const NearbyVehicle& real_leader);

	const RealLongitudinalController* get_real_leader_controller() const {
		return real_leader_controller;
	}
	const RealLongitudinalController& get_virtual_leader_controller() const {
		return virtual_leader_controller;
	}

	bool get_destination_lane_desired_acceleration_when_in_platoon(
		const PlatoonVehicle& platoon_vehicle,
		std::unordered_map<ALCType, double>& possible_accelerations);

protected:
	PlatoonVehicleController::PlatoonVehicleController(bool verbose)
		: CAVController(verbose) {};

private:
	RealLongitudinalController* real_leader_controller{ nullptr }; /* copy of
	origin_lane_controller, but we want a different name in the platoon
	scenario */
	RealLongitudinalController virtual_leader_controller;
	/*^^^^ Not a mistake. The virtual leader controller in the platoon
	LC paper has the RealLongitudinalController behavior*/

	double platoon_velocity_filter_gain{ 0.0 };     /* hoping to make them */
	double platoon_time_headway_filter_gain{ 0.0 }; /* pass all filters    */
	AutonomousGains platoon_vehicle_autonomous_gains{ 0.2, 0.5 };
	ConnectedGains platoon_vehicle_connected_gains{ 0.2, 0.5, 0.0, 0.0 };
	VelocityControllerGains platoon_vehicle_velocity_gains{ 0.5, 0.0, 0.0 };

	std::unordered_map<LongitudinalController::State, color_t>
		in_platoon_colors =
	{
		{ LongitudinalController::State::velocity_control, DARK_GRAY },
		{ LongitudinalController::State::vehicle_following, WHITE},
	};
};

