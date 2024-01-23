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
#include "RealLongitudinalController.h"
#include "VanAremLongitudinalController.h"
#include "Vehicle.h"
#include "VirtualLongitudinalController.h"
#include "VissimLongitudinalController.h"
#include "TrafficLight.h"

class EgoVehicle;

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

	color_t get_longitudinal_controller_color() const;
	/* Safe value depends on whether or not the vehicle has lane
	change intention */
	double get_safe_time_headway() const;
	/* Gets the current time headway, which can be any value between
	the veh following or lane changing time headways if the vehicle
	is transitiong between states. */
	double get_current_desired_time_headway() const;
	double get_reference_gap() const;
	double get_gap_error() const;
	/* Returns the safe time headway gap. */
	double get_desired_time_headway_gap(
		const NearbyVehicle& nearby_vehicle) const;
	double get_desired_time_headway_gap_to_leader() const;
	LongitudinalController::State
		get_longitudinal_controller_state() const;
	/* Computes the deceleration rate to avoid collision */
	double compute_drac(double relative_velocity, double gap);

	void set_verbose(bool value);

	void add_internal_controllers();

	/* Resets the origin lane controller's velocity and time headway filters
	and sets the time headway */
	void activate_origin_lane_controller(
		const NearbyVehicle& real_leader);
	void activate_end_of_lane_controller(double time_headway);
	void update_origin_lane_controller(
		const NearbyVehicle& real_leader);

	void reset_origin_lane_velocity_controller(double ego_velocity);
	/* Finds which time headway leads to zero gap error. Useful when 
	there are new real or virtual leaders */
	double find_comfortable_time_headway(const EgoVehicle& ego_vehicle,
		const NearbyVehicle& a_leader, double standstill_distance);

	bool is_in_free_flow_at_origin_lane() const;

	/* Printing ----------------------------------------------------------- */
	void print_traffic_lights(const EgoVehicle& ego,
		const std::unordered_map<int, TrafficLight>& traffic_lights) const;
	static std::string ALC_type_to_string(
		ALCType active_longitudinal_controller);


protected:
	bool verbose{ false };
	/* Used to keep track of which controller is active, which helps
	in debugging (sloppy solution) */
	std::unordered_map<ALCType, const LongitudinalController*>
		available_controllers;
	/* indicates which controller is active. Used for debugging and
	visualization. */
	ALCType active_longitudinal_controller_type{ ALCType::origin_lane };
	double velocity_filter_gain{ 10.0 };
	double time_headway_filter_gain{ 0.3 };
	bool long_controllers_verbose{ false };
	double origin_lane_controller_time_headway{ 10.0 };
	VissimLongitudinalController vissim_controller;
	RealLongitudinalController origin_lane_controller;
	RealLongitudinalController end_of_lane_controller;

	/* Constructor for derived classes */
	VehicleController(const EgoVehicle* ego_vehicle, bool verbose);

	/* Initializing controllers */
	void add_vissim_controller();
	void add_origin_lane_controllers(const EgoVehicle& ego_vehicle);

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

	/* Desired acceleration to wait at the end of the lane while
	looking for an appropriate lane change gap. Without this,
	vehicles might miss a desired exit.	Returns infinity if there is no
	leader */
	/*double get_end_of_lane_desired_acceleration(
		const VirdiVehicle& virdi_vehicle);*/

	/* Gets the minimum of the accelerations in the map and sets the
	active longitudinal controller. */
	double choose_minimum_acceleration(
		std::unordered_map<ALCType, double>& possible_accelerations);

private:
	const EgoVehicle* ego_vehicle{ nullptr };

	/* ------------------------- Control Parameters ----------------------- */

	/* Cooperative Lane Changing paper ------------------------------------ */
	AutonomousGains autonomous_real_following_gains{ 0.2, 1.0 };
	// Values focused on platoon vehicles 
	ConnectedGains connected_real_following_gains{ 0.2, 2.3, 0.13, 1.3 };
	VelocityControllerGains desired_velocity_controller_gains{
		0.5, 0.1, 0.03 };
	/* The time headway used by the end-of-lane controller */
	double time_headway_to_end_of_lane{ 0.5 };

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

    /* -------------------------------------------------------------------- */
	virtual void implement_add_internal_controllers() = 0;
	virtual void implement_update_origin_lane_controller(
		const EgoVehicle& ego_vehicle, const NearbyVehicle& real_leader);
	virtual double implement_get_desired_time_headway_gap(
		const NearbyVehicle& real_leader) const;

	const LongitudinalController* get_active_long_controller() const;

	NearbyVehicle create_virtual_stopped_vehicle(
		const EgoVehicle& ego_vehicle) const;

	/* Parameters for controllers no longer supported */
	/* "Virdi" paper (for comparison) */
	//ConnectedGains van_arem_gap_ctrl_gains{ 0.1, 0.58, 0.0, 1.0 };
	//VelocityControllerGains van_arem_vel_ctrl_gains{ 1.0, 0.0, 0.0 };
	//double virdi_max_jerk = 0.5;
	//std::unordered_map<ALCType, VanAremLongitudinalController>
	//	van_arem_controllers;  // for comparison

	///* Traffic Light ALC paper -------------------------------------------- */
	////LongitudinalControllerWithTrafficLights with_traffic_lights_controller;

	//std::unordered_map<LongitudinalController::State, color_t>
	//	tl_alc_colors =
	//{
	//	{ LongitudinalController::State::comf_accel, BLUE_GREEN },
	//	{ LongitudinalController::State::velocity_control, GREEN },
	//	{ LongitudinalController::State::vehicle_following, DARK_GREEN },
	//	{ LongitudinalController::State::traffic_light, YELLOW},
	//	{ LongitudinalController::State::too_close, RED },
	//};
};
