#pragma once
#include "VehicleController.h"

class AutonomousVehicle;

class AVController : public VehicleController
{
public:
	AVController(const AutonomousVehicle* autonomous_vehicle, bool verbose);

	const VirtualLongitudinalController& get_destination_lane_controller()
		const {
		return destination_lane_controller;
	};

	const LateralController& get_lateral_controller() const {
		return lateral_controller;
	};

	/* Computes ACC desired acceleration plus the acceleration during lane
	change adjustments and lateral movement. Gives control to human (vissim)
	if the vehicle is waiting for too long to find a gap. */
	double get_desired_acceleration(const AutonomousVehicle&
		autonomous_vehicle);

	void add_lane_change_adjustment_controller(
		const AutonomousVehicle& autonomous_vehicle);

	/* Resets the destination lane controller's velocity and time headway filters
	and sets the time headway*/
	void activate_destination_lane_controller(
		const EgoVehicle& ego_vehicle, const NearbyVehicle& virtual_leader);
	/* Sets a new time headway and the connectivity of the destination lane
	controller, and resets its velocity filter. */
	void update_destination_lane_controller(const EgoVehicle& ego_vehicle,
		const NearbyVehicle& virtual_leader /*double ego_velocity,
		double time_headway, bool is_leader_connected*/);
		//void update_leader_lane_changing_time_headway(double time_headway);
	void update_destination_lane_follower_parameters(
		NearbyVehicle& dest_lane_follower);
	void update_destination_lane_follower_time_headway(double ego_max_brake,
		bool are_vehicles_connected, NearbyVehicle& dest_lane_follower);
	void update_destination_lane_leader_time_headway(double time_headway);

	double compute_accepted_lane_change_gap(const EgoVehicle& ego_vehicle,
		const NearbyVehicle& nearby_vehicle, double accepted_risk);

	double compute_accepted_lane_change_gap_exact(const EgoVehicle& ego_vehicle,
		const NearbyVehicle& nearby_vehicle,
		std::pair<double, double> ego_safe_lane_changing_params,
		double accepted_risk);

	/* Returns the expected gap variation during the lane change */
	double get_gap_variation_during_lane_change(
		const EgoVehicle& ego_vehicle,
		const NearbyVehicle& nearby_vehicle,
		bool will_accelerate) const;

protected:
	VelocityControllerGains adjustment_velocity_controller_gains{
	1, 0.1, 0.03 };
	AutonomousGains autonomous_virtual_following_gains{ 0.4, 1.0 };
	// TODO organization: connected gains do not belong here
	ConnectedGains connected_virtual_following_gains{ 0.4, 2.3, 0.13, 1.3 };
	LateralController lateral_controller;
	VirtualLongitudinalController destination_lane_controller;

	//AVController(bool verbose) : VehicleController(verbose), 
	//	lateral_controller{ LateralController(verbose) } {};

	/* Desired acceleration to adjust to destination lane leader.
	Returns true if the computed acceleration was added to the map */
	bool get_destination_lane_desired_acceleration(
		const AutonomousVehicle& autonomous_vehicles,
		std::unordered_map<ALCType, double>& possible_accelerations);

	/* Computes the velocity reference when adjusting for lane change */
	double determine_low_velocity_reference(double ego_velocity,
		const NearbyVehicle& nearby_vehicle) const;

private:
	const AutonomousVehicle* autonomous_vehicle{ nullptr };

	std::unordered_map<LongitudinalController::State, color_t>
		dest_lane_colors =
	{
		{ LongitudinalController::State::velocity_control, LIGHT_BLUE },
		{ LongitudinalController::State::vehicle_following, BLUE },
	};
	
	void implement_add_internal_controllers() override;
	void implement_update_origin_lane_controller(
		const EgoVehicle& ego_vehicle, const NearbyVehicle& real_leader
	) override;
	double implement_get_desired_time_headway_gap(
		const NearbyVehicle& real_leader) const override;
};

