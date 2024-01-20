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
#include "VanAremLongitudinalController.h"
#include "Vehicle.h"
#include "VirtualLongitudinalController.h"
#include "VissimLongitudinalController.h"

class EgoVehicle;
class ACCVehicle;
class AutonomousVehicle;
class ConnectedAutonomousVehicle;
class TrafficLightALCVehicle;
class PlatoonVehicle;
class VirdiVehicle;

class VehicleController 
{
public:

	/* Autonomous Longitudinal Controller Type */
	enum class ALCType 
	{
		// Coop lane change paper
		origin_lane,
		destination_lane,
		cooperative_gap_generation,
		end_of_lane,
		vissim,
		// Traffic light ALC paper
		traffic_light_alc,
		// Platoon paper
		real_leader,
		virtual_leader
	};

	VehicleController() = default;
	VehicleController(const ACCVehicle& acc_vehicle, bool verbose);
	VehicleController(const AutonomousVehicle& autonomous_vehicle, bool verbose);
	VehicleController(const ConnectedAutonomousVehicle& cav, bool verbose);
	VehicleController(const PlatoonVehicle& platoon_vehicle, bool verbose);
	VehicleController(const TrafficLightALCVehicle& tfalc_vehicle, bool verbose);
	VehicleController(const VirdiVehicle& virdi_vehicle, bool verbose);

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

	void set_verbose(bool value);

	/* Initializing controllers */
	void add_vissim_controller();
	void add_origin_lane_controllers(const EgoVehicle& ego_vehicle);
	void add_lane_change_adjustment_controller(
		const AutonomousVehicle& autonomous_vehicle);
	void add_cooperative_lane_change_controller(
		const ConnectedAutonomousVehicle& cav);
	void add_traffic_lights_controller();
	void add_van_arem_controllers(const VirdiVehicle& virdi_vehicle);
	//void add_in_platoon_controller(const PlatoonVehicle& platoon_vehicle);

	/* Access to internal controllers ---------------------------------------- */

	const RealLongitudinalController& get_origin_lane_controller() const {
		return origin_lane_controller;
	}
	const VirtualLongitudinalController& get_gap_generation_lane_controller()
		const {
		return gap_generating_controller;
	};
	const VirtualLongitudinalController& get_destination_lane_controller() 
		const {
		return destination_lane_controller;
	};
	const LateralController& get_lateral_controller() const {
		return lateral_controller;
	};
	const RealLongitudinalController* get_real_leader_controller() const {
		return real_leader_controller;
	}
	const RealLongitudinalController& get_virtual_leader_controller() const {
		return virtual_leader_controller;
	}
	double get_reference_gap(double ego_velocity) const;

	/* ----------------------------------------------------------------------- */

	double get_gap_error() const;
	/* Computes the deceleration rate to avoid collision */
	double compute_drac(double relative_velocity, double gap);

	/* Resets the origin lane controller's velocity and time headway filters
	and sets the time headway*/
	void activate_origin_lane_controller(const EgoVehicle& ego_vehicle,
		const NearbyVehicle& real_leader);
	void activate_end_of_lane_controller(double time_headway);
	/* Sets a new time headway and the connectivity of the origin lane
	controller */
	/* Resets the destination lane controller's velocity and time headway filters
	and sets the time headway*/
	void activate_destination_lane_controller(
		const EgoVehicle& ego_vehicle, const NearbyVehicle& virtual_leader);
	//void activate_in_platoon_controller(double ego_velocity,
	//	double time_headway);
	void update_origin_lane_controller(const EgoVehicle& ego_vehicle, 
		const NearbyVehicle& real_leader);
	/* TODO: instead of overloading the method, we should deal with this 
	via polymorphism, but that'll be a long refactoring process */
	void update_origin_lane_controller(const PlatoonVehicle& platoon_vehicle,
		const NearbyVehicle& real_leader);
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

	void update_gap_generation_controller(double ego_velocity,
		double time_headway);

	void reset_origin_lane_velocity_controller(double ego_velocity);
	/* Finds which time headway leads to zero gap error. Useful when 
	there are new real or virtual leaders */
	double find_comfortable_time_headway(const EgoVehicle& ego_vehicle,
		const NearbyVehicle& a_leader, double standstill_distance);

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
	double get_desired_acceleration(const VirdiVehicle& virdi_vehicle);
	void print_traffic_lights(const EgoVehicle& ego,
		const std::unordered_map<int, TrafficLight>& traffic_lights) const;

	/* Computes the velocity reference when adjusting for lane change */
	double determine_low_velocity_reference(double ego_velocity,
		const NearbyVehicle& nearby_vehicle) const;

	/* Returns the reference gap used by the longitudinal
	controller. It uses a previously defined time headway, which might
	have been computed assuming some accepted risk. NOT IN USE */
	//double compute_desired_lane_change_gap(
	//	const AutonomousVehicle& ego_vehicle,
	//	const NearbyVehicle& nearby_vehicle, 
	//	bool will_accelerate = false) const;
	double compute_accepted_lane_change_gap(const EgoVehicle& ego_vehicle,
		const NearbyVehicle& nearby_vehicle, double accepted_risk);
	double compute_accepted_lane_change_gap_exact(const EgoVehicle& ego_vehicle,
		const NearbyVehicle& nearby_vehicle, 
		std::pair<double, double> ego_safe_lane_changing_params,
		double accepted_risk);

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


protected:
	VehicleController(bool verbose);

private:
	/* ------------------------- Control Parameters ----------------------- */

	/* Cooperative Lane Changing paper ------------------------------------ */
	double velocity_filter_gain{ 10.0 };
	double time_headway_filter_gain{ 0.3 };
	AutonomousGains autonomous_real_following_gains{ 0.2, 1.0 };
	AutonomousGains autonomous_virtual_following_gains{ 0.4, 1.0 };
	// Values focused on platoon vehicles TODO: separate controllers
	// gains will probably have to be tuned
	ConnectedGains connected_real_following_gains{ 0.2, 2.3, 0.13, 1.3 };
	ConnectedGains connected_virtual_following_gains{ 0.4, 2.3, 0.13, 1.3 };
	
	VelocityControllerGains desired_velocity_controller_gains{
		0.5, 0.1, 0.03 };
	VelocityControllerGains adjustment_velocity_controller_gains{
		1, 0.1, 0.03 };
	/* The time headway used by the end-of-lane controller */
	double time_headway_to_end_of_lane{ 0.5 };

	VissimLongitudinalController vissim_controller;
	RealLongitudinalController origin_lane_controller;
	RealLongitudinalController end_of_lane_controller;
	VirtualLongitudinalController destination_lane_controller;
	VirtualLongitudinalController gap_generating_controller;

	/* "Virdi" paper (for comparison) */
	ConnectedGains van_arem_gap_ctrl_gains{ 0.1, 0.58, 0.0, 1.0 };
	VelocityControllerGains van_arem_vel_ctrl_gains{ 1.0, 0.0, 0.0 };
	double virdi_max_jerk = 0.5;
	std::unordered_map<ALCType, VanAremLongitudinalController>
		van_arem_controllers;  // for comparison

	/* colors */
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
	
	/* Traffic Light ALC paper -------------------------------------------- */
	LongitudinalControllerWithTrafficLights with_traffic_lights_controller;

	std::unordered_map<LongitudinalController::State, color_t>
		tl_alc_colors =
	{
		{ LongitudinalController::State::comf_accel, BLUE_GREEN },
		{ LongitudinalController::State::velocity_control, GREEN },
		{ LongitudinalController::State::vehicle_following, DARK_GREEN },
		{ LongitudinalController::State::traffic_light, YELLOW},
		{ LongitudinalController::State::too_close, RED },
	};

	/* Platoon LC paper --------------------------------------------------- */
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
	RealLongitudinalController* real_leader_controller; /* copy of 
	origin_lane_controller, but we want a different name in the platoon
	scenario */
	RealLongitudinalController virtual_leader_controller;
  /*^^^^ Not a mistake. The virtual leader controller in the platoon
  LC paper has the RealLongitudinalController behavior*/

    /* -------------------------------------------------------------------- */

	/* Used to keep track of which controller is active, which helps
	in debugging (sloppy solution) */
	std::unordered_map<ALCType, const LongitudinalController*>
		available_controllers;

	LateralController lateral_controller;
	/* indicates which controller is active. Used for debugging and
	visualization. */
	ALCType active_longitudinal_controller_type{ ALCType::origin_lane };
	double origin_lane_controller_time_headway{ 10.0 };

	double origin_lane_leader_max_brake{ 0.0 };
	double destination_lane_leader_max_brake{ 0.0 };
	double destination_lane_follower_max_brake{ 0.0 };
	double assisted_vehicle_max_brake{ 0.0 };
	VehicleType origin_lane_leader_type{ VehicleType::undefined };
	VehicleType destination_lane_leader_type{ VehicleType::undefined };
	VehicleType destination_lane_follower_type{ VehicleType::undefined };

	bool verbose{ false };
    bool long_controllers_verbose{ false };

	const LongitudinalController* get_active_long_controller() const;
	/* Gets the minimum of the accelerations in the map and sets the
	active longitudinal controller. */
	double choose_minimum_acceleration(
		std::unordered_map<ALCType, double>& possible_accelerations);

	NearbyVehicle create_virtual_stopped_vehicle(
		const EgoVehicle& ego_vehicle) const;
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
	/* Desired acceleration to wait at the end of the lane while
	looking for an appropriate lane change gap. Without this,
	vehicles might miss a desired exit.	Returns infinity if there is no
	leader */
	double get_end_of_lane_desired_acceleration(
		const VirdiVehicle& virdi_vehicle);
};
