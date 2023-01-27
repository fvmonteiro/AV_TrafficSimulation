/*==========================================================================*/
/*  ControlManager.h    											        */
/*  State-machine that determines the actual vehicle longitudinal and		*/
/*  lateral inputs															*/
/*                                                                          */
/*  Version of 2021-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#pragma once

#include <unordered_map>
#include <vector>

#include "LateralController.h"
#include "SwitchedLongitudinalController.h"
#include "LongitudinalControllerWithTrafficLights.h"
#include "RealLongitudinalController.h"
#include "Vehicle.h"
#include "VirtualLongitudinalController.h"
#include "VissimLongitudinalController.h"

class EgoVehicle;
class ACCVehicle;
class AutonomousVehicle;
class ConnectedAutonomousVehicle;
class TrafficLightALCVehicle;
class PlatoonVehicle;

class ControlManager 
{
public:

	/* Autonomous Longitudinal Controller Type */
	enum class ALCType {
		origin_lane,
		destination_lane,
		cooperative_gap_generation,
		end_of_lane,
		vissim,
		traffic_light_alc
	};

	ControlManager() = default;
	ControlManager(const EgoVehicle& ego_vehicle, bool verbose);
	//ControlManager(const EgoVehicle& ego_vehicle);

	//std::vector<State> get_states() { return states; };
	//ALCType get_active_alc_type() const {
	//	return active_longitudinal_controller_type;
	//}

	color_t get_longitudinal_controller_color() const;
	/* Safe value depends on whether or not the vehicle has lane
	change intention */
	double get_safe_time_headway() const;
	/* Gets the current time headway, which can be any value between
	the veh following or lane changing time headways if the vehicle
	is transitiong between states. */
	double get_current_desired_time_headway() const;

	LongitudinalController::State
		get_longitudinal_controller_state() const;
	/*LongitudinalControllerWithTrafficLights::State
		get_longitudinal_controller_with_traffic_lights_state();*/
	//void create_destination_lane_controller(const Vehicle& ego_vehicle);

	/* Initializing controllers */
	void add_vissim_controller();
	void add_origin_lane_controllers(const EgoVehicle& ego_vehicle);
	void add_lane_change_adjustment_controller(
		const AutonomousVehicle& autonomous_vehicle);
	void add_cooperative_lane_change_controller(
		const ConnectedAutonomousVehicle& cav);
	void add_traffic_lights_controller();
	void add_in_platoon_controller(const PlatoonVehicle& platoon_vehicle);

	/* DEBUGGING FUNCTIONS --------------------------------------------------- */
	/* These functions are used to easily read data from internal instances and
	methods. */

	/* Each controller should never be accessed directly by external
	functions. */
	const RealLongitudinalController& get_origin_lane_controller() const {
		return origin_lane_controller;
	}
	const VirtualLongitudinalController& get_gap_generation_lane_controller()
		const {
		return gap_generating_controller;
	};
	/* Each controller should never be accessed directly by external
	functions. */
	const VirtualLongitudinalController& get_destination_lane_controller() 
		const {
		return destination_lane_controller;
	};
	/* Each controller should never be accessed directly by external
	functions. */
	const LateralController& get_lateral_controller() const {
		return lateral_controller;
	};
	double get_reference_gap(double ego_velocity) const; /* could be const */

	/* ----------------------------------------------------------------------- */

	double get_gap_error() const;
	/* Computes the deceleration rate to avoid collision */
	double compute_drac(double relative_velocity, double gap);

	/* Resets the origin lane controller's velocity and time headway filters
	and sets the time headway*/
	void activate_origin_lane_controller(double ego_velocity,
		double time_headway, bool is_leader_connected);
	void activate_end_of_lane_controller(double time_headway);
	/* Sets a new time headway and the connectivity of the origin lane
	controller */
	/* Resets the destination lane controller's velocity and time headway filters
	and sets the time headway*/
	void activate_destination_lane_controller(double ego_velocity,
		double leader_velocity,
		double time_headway, bool is_leader_connected);
	//void activate_in_platoon_controller(double ego_velocity,
	//	double time_headway);
	void update_origin_lane_controller(double time_headway,
		bool is_leader_connected);
	/* Sets a new time headway and the connectivity of the destination lane
	controller, and resets its velocity filter. */
	void update_destination_lane_controller(double ego_velocity,
		double time_headway, bool is_leader_connected);
	void update_destination_lane_follower_time_headway(double time_headway);
	void update_gap_generation_controller(double ego_velocity,
		double time_headway);

	void reset_origin_lane_velocity_controller(double ego_velocity);

	bool is_in_free_flow_at_origin_lane() const;

	/* Active ACC during lane keeping; human (vissim) control if there is
	lane change intention*/
	double get_desired_acceleration(const ACCVehicle& acc_vehicle);
	/* Computes ACC desired acceleration plus the acceleration during lane
	change adjustments and lateral movement. Gives control to human (vissim)
	if the vehicle is waiting for too long to find a gap. */
	double get_desired_acceleration(const AutonomousVehicle&
		autonomous_vehicle);
	/* Computes the AV desired acceleration plus the cooperative acceleration
	to help create a gap for an incoming vehicle, and chooses the minimum. */
	double get_desired_acceleration(const ConnectedAutonomousVehicle& cav);
	double get_desired_acceleration(const PlatoonVehicle& platoon_vehicle);
	/* TODO description */
	double get_desired_acceleration(
		const TrafficLightALCVehicle& ego_vehicle,
		const std::unordered_map<int, TrafficLight>& traffic_lights);

	void print_traffic_lights(const EgoVehicle& ego,
		const std::unordered_map<int, TrafficLight>& traffic_lights) const;

	/* Computes the velocity reference when adjusting for lane change */
	double determine_low_velocity_reference(double ego_velocity,
		const NearbyVehicle& nearby_vehicle) const;

	/* Returns the reference gap used by the longitudinal
	controller. It uses a previously defined time headway, which might
	have been computed assuming some accepted risk. */
	double compute_desired_lane_change_gap(
		const AutonomousVehicle& ego_vehicle,
		const NearbyVehicle& nearby_vehicle, 
		bool will_accelerate = false) const;

	/* Returns the accepted lane change gap, which might is different from
	the longitudinal controller's reference gap if the accepted risk is
	greater than zero. This assumes the longitudinal controller uses a
	zero-risk time headway. */
	//double compute_accepted_lane_change_gap(
	//	const AutonomousVehicle& ego_vehicle,
	//	const NearbyVehicle& nearby_vehicle,
	//	bool will_accelerate = false);

	/* Returns the safe time headway gap. */
	double get_desired_time_headway_gap(double ego_velocity,
		const NearbyVehicle& nearby_vehicle) const;
	/* Returns the time headway part of the accepted lane change gap. */
	/*double get_accepted_time_headway_gap(const AutonomousVehicle& ego_vehicle,
		const NearbyVehicle& nearby_vehicle);*/
	/* Returns the expected gap variation during the lane change */
	double get_gap_variation_during_lane_change(
		const AutonomousVehicle& ego_vehicle,
		const NearbyVehicle& nearby_vehicle,
		bool will_accelerate) const;


	/* Printing ----------------------------------------------------------- */
	static std::string ALC_type_to_string(
		ALCType active_longitudinal_controller);

private:
	/* ------------ Control Parameters ------------ */

	double velocity_filter_gain{ 10.0 };
	double time_headway_filter_gain{ 1.0 }; // 0.3 original value for CAVs
	AutonomousGains autonomous_real_following_gains{ 0.2, 1.0 };
	AutonomousGains autonomous_virtual_following_gains{ 0.4, 1.0 };
	// Original values for CAVs
	/*ConnectedGains connected_real_following_gains{ 0.2, 2.3, 0.13, 1.3 };*/
	// Values focused on platoon vehicles TODO: separate controllers
	// gains will probably have to be tuned
	ConnectedGains connected_real_following_gains{ 0.5, 2.2, 0.13, 1.3 };
	ConnectedGains connected_virtual_following_gains{ 0.4, 2.3, 0.13, 1.3 };
	ConnectedGains platoon_following_gains{ 0.2, 2.3, 0.13, 1.3 };
	VelocityControllerGains desired_velocity_controller_gains{
		0.5, 0.1, 0.03 };
	VelocityControllerGains adjustment_velocity_controller_gains{
		1, 0.1, 0.03 };
	/* The time headway used by the end-of-lane controller */
	double time_headway_to_end_of_lane{ 0.5 };

	/* Colors to make debugging visually easier
	General rule: bright colors represent vel control,
	darker colors represent vehicle following. */
	std::unordered_map<LongitudinalController::State, color_t>
		vissim_colors =
	{
		{LongitudinalController::State::uninitialized, CYAN}
	};
	std::unordered_map<LongitudinalController::State, color_t>
		orig_lane_colors =
	{
		{ LongitudinalController::State::velocity_control, GREEN },
		{ LongitudinalController::State::vehicle_following, DARK_GREEN },
	};
	/* TODO [Nov 11, 2022]: still missing implementation */
	color_t orig_lane_max_vel_control_color{ BLUE_GREEN };
	std::unordered_map<LongitudinalController::State, color_t>
		end_of_lane_colors =
	{
		{ LongitudinalController::State::velocity_control, MAGENTA },
		{ LongitudinalController::State::vehicle_following, DARK_MAGENTA },
	};
	std::unordered_map<LongitudinalController::State, color_t>
		dest_lane_colors =
	{
		{ LongitudinalController::State::velocity_control, LIGHT_BLUE },
		{ LongitudinalController::State::vehicle_following, BLUE },
	};
	std::unordered_map<LongitudinalController::State, color_t>
		gap_generation_colors =
	{
		{ LongitudinalController::State::velocity_control, YELLOW },
		{ LongitudinalController::State::vehicle_following, DARK_YELLOW },
	};
	std::unordered_map<LongitudinalController::State, color_t>
		tl_alc_colors =
	{
		{ LongitudinalController::State::comf_accel, BLUE_GREEN },
		{ LongitudinalController::State::velocity_control, GREEN },
		{ LongitudinalController::State::vehicle_following, DARK_GREEN },
		{ LongitudinalController::State::traffic_light, YELLOW},
		{ LongitudinalController::State::too_close, RED },
	};
	std::unordered_map<LongitudinalController::State, color_t>
		in_platoon_colors =
	{
		{ LongitudinalController::State::velocity_control, LIGHT_GRAY },
		{ LongitudinalController::State::vehicle_following, GRAY},
	};
	/* -------------------------------------------- */

	VissimLongitudinalController vissim_controller;
	RealLongitudinalController origin_lane_controller;
	RealLongitudinalController end_of_lane_controller;
	VirtualLongitudinalController destination_lane_controller;
	VirtualLongitudinalController gap_generating_controller;
	LongitudinalControllerWithTrafficLights
		with_traffic_lights_controller;
	RealLongitudinalController in_platoon_controller;

	LateralController lateral_controller;
	/* indicates which controller is active. Used for debugging and
	visualization. */
	ALCType active_longitudinal_controller_type{ ALCType::origin_lane };

	double origin_lane_leader_max_brake{ 0.0 };
	double destination_lane_leader_max_brake{ 0.0 };
	double destination_lane_follower_max_brake{ 0.0 };
	double assisted_vehicle_max_brake{ 0.0 };
	VehicleType origin_lane_leader_type{ VehicleType::undefined };
	VehicleType destination_lane_leader_type{ VehicleType::undefined };
	VehicleType destination_lane_follower_type{ VehicleType::undefined };

	bool verbose{ false };
    bool long_controllers_verbose{ false };

	std::unique_ptr<LongitudinalController>
		get_active_long_controller() const;
	/* Gets the minimum of the accelerations in the map and sets the
	active longitudinal controller. */
	double choose_minimum_acceleration(
		std::unordered_map<ALCType, double>& possible_accelerations);

	NearbyVehicle create_virtual_stopped_vehicle(
		const EgoVehicle& ego_vehicle);
	/* Desired accelerations --------------------- */

	/* VISSIM's suggested acceleration */
	double get_vissim_desired_acceleration(const EgoVehicle& ego_vehicle);
	/* Desired acceleration relative to the current lane.
	Returns true if the computed acceleration was added to the map */
	bool get_origin_lane_desired_acceleration(
		const EgoVehicle& ego_vehicle,
		std::unordered_map<ALCType, double>& possible_accelerations);
	/* Desired acceleration to wait at the end of the lane while
	looking for an appropriate lane change gap. Without this,
	vehicles might miss a desired exit.
	Returns true if the computed acceleration was added to the map */
	bool get_end_of_lane_desired_acceleration(
		const EgoVehicle& ego_vehicle,
		std::unordered_map<ALCType, double>& possible_accelerations);
	/* Desired acceleration to adjust to destination lane leader.
	Returns true if the computed acceleration was added to the map */
	bool get_destination_lane_desired_acceleration(
		const AutonomousVehicle& autonomous_vehicles,
		std::unordered_map<ALCType, double>& possible_accelerations);
	/* Returns true if the computed acceleration was added to the map */
	bool get_cooperative_desired_acceleration(
		const ConnectedAutonomousVehicle& cav,
		std::unordered_map<ALCType, double>& possible_accelerations);
	bool get_destination_lane_desired_acceleration_when_in_platoon(
		const PlatoonVehicle& platoon_vehicle,
		std::unordered_map<ALCType, double>& possible_accelerations);
};
