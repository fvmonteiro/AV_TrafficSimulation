#pragma once

#include "RealLongitudinalController.h"
#include "VehicleController.h"

class ACCVehicle;

class ACCVehicleController : public VehicleController
{
public:

	ACCVehicleController() = default;
	ACCVehicleController(const ACCVehicle* acc_vehicle, bool verbose);

protected:
	std::shared_ptr<SwitchedLongitudinalController> end_of_lane_controller;

	/* Initializing controllers */
	void add_vissim_controller();
	void add_origin_lane_controllers();

	void activate_end_of_lane_controller(double time_headway);

	/* VISSIM's suggested acceleration */
	double get_vissim_desired_acceleration();
	/* Desired acceleration relative to the current lane.
	Returns true if the computed acceleration was added to the map */
	bool get_origin_lane_desired_acceleration(
		std::unordered_map<ALCType, double>& possible_accelerations);
	/* Desired acceleration to wait at the end of the lane while
	looking for an appropriate lane change gap. Without this,
	vehicles might miss a desired exit.
	Returns true if the computed acceleration was added to the map */
	bool get_end_of_lane_desired_acceleration(
		std::unordered_map<ALCType, double>& possible_accelerations);

	double get_desired_time_headway_gap_to_leader() const;

private:
	const ACCVehicle* acc_vehicle{ nullptr };
	VissimLongitudinalController vissim_controller;

	/* ------------------------- Control Parameters ----------------------- */
	AutonomousGains autonomous_real_following_gains{ 0.2, 1.0 };
	// TODO organization: connected gains do not belong here
	ConnectedGains connected_real_following_gains{ 0.2, 2.3, 0.13, 1.3 };
	VelocityControllerGains desired_velocity_controller_gains{
		0.5, 0.1, 0.03 };
	/* The time headway used by the end-of-lane controller */
	double time_headway_to_end_of_lane{ 0.5 };
	
	std::unordered_map<LongitudinalController::State, color_t>
		vissim_colors =
	{
		{LongitudinalController::State::uninitialized, BLACK}
	};
	std::unordered_map<LongitudinalController::State, color_t>
		orig_lane_colors =
	{
		{ LongitudinalController::State::velocity_control, GREEN },
		{ LongitudinalController::State::vehicle_following, DARK_GREEN },
	};
	std::unordered_map<LongitudinalController::State, color_t>
		end_of_lane_colors =
	{
		{ LongitudinalController::State::velocity_control, MAGENTA },
		{ LongitudinalController::State::vehicle_following, DARK_MAGENTA },
	};

	void implement_add_internal_controllers() override;
	/* Active ACC during lane keeping; human (vissim) control if there is
	lane change intention*/
	double implement_get_desired_acceleration() override;
	void implement_update_origin_lane_controller(
		const NearbyVehicle& real_leader) override;
	double implement_get_desired_time_headway_gap(
		const NearbyVehicle& real_leader) const override;
};
