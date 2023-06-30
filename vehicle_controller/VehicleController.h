/*==========================================================================*/
/*  VehicleController.h    											        */
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
#include "SafetyCriticalGapController.h"
#include "Vehicle.h"
#include "VirtualLongitudinalController.h"
#include "VissimLongitudinalController.h"

class EgoVehicle;

class VehicleController 
{
public:

	/* Autonomous Longitudinal Controller Type 
	[June 2023] Addition of safe controllers - this is getting messy.
	Must refactor. */
	enum class ALCType {
		origin_lane,
		destination_lane,
		cooperative_gap_generation,
		end_of_lane,
		vissim,
		traffic_light_alc,
		safe_origin_lane,
		safe_destination_lane,
		safe_cooperative_gap_generation
	};

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

	void set_traffic_lights(
		const std::unordered_map<int, TrafficLight>& traffic_lights);

	/* DEBUGGING FUNCTIONS --------------------------------------------------- */
	/* Functions used to easily read data from internal instances and
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
	double get_reference_gap() const;

	/* ----------------------------------------------------------------------- */

	double get_gap_error() const;
	/* Computes the deceleration rate to avoid collision */
	double compute_drac(double relative_velocity, double gap);

	void activate_end_of_lane_controller(double time_headway);
	/* Sets a new time headway and the connectivity of the origin lane
	controller */
	/* Resets the destination lane controller's velocity and time headway filters
	and sets the time headway*/
	void activate_destination_lane_controller(
		double leader_velocity,
		double time_headway, bool is_leader_connected);
	//void activate_in_platoon_controller(double ego_velocity,
	//	double time_headway);
	void update_origin_lane_controller(double time_headway,
		bool is_leader_connected);
	/* Sets a new time headway and the connectivity of the destination lane
	controller, and resets its velocity filter. */
	void update_destination_lane_controller(
		double time_headway, bool is_leader_connected);
	void update_destination_lane_follower_time_headway(double time_headway);
	void update_gap_generation_controller(double time_headway);

	void reset_origin_lane_velocity_controller();

	bool is_in_free_flow_at_origin_lane() const;

	double compute_desired_acceleration();

	void print_traffic_lights(const EgoVehicle& ego,
		const std::unordered_map<int, TrafficLight>& traffic_lights) const;

	/* Computes the velocity reference when adjusting for lane change */
	double determine_low_velocity_reference(
		const NearbyVehicle& nearby_vehicle) const;

	/* Returns the reference gap used by the longitudinal
	controller. It uses a previously defined time headway, which might
	have been computed assuming some accepted risk. */
	double compute_desired_lane_change_gap(const NearbyVehicle& nearby_vehicle, 
		bool will_accelerate = false) const;

	/* Returns the safe time headway gap. */
	double get_desired_time_headway_gap(
		const NearbyVehicle& nearby_vehicle) const;

	double get_gap_variation_during_lane_change(
		const NearbyVehicle& nearby_vehicle, bool will_accelerate) const;


	/* Printing ----------------------------------------------------------- */
	static std::string ALC_type_to_string(
		ALCType active_longitudinal_controller);

protected:
	bool verbose{ false };
	bool long_controllers_verbose{ false };

	/* [June 2023] TODO possibly moves all the lower level
	controller to the respective derived classes */
	VissimLongitudinalController vissim_controller;
	RealLongitudinalController origin_lane_controller;
	RealLongitudinalController end_of_lane_controller;
	VirtualLongitudinalController destination_lane_controller;
	VirtualLongitudinalController gap_generating_controller;
	LongitudinalControllerWithTrafficLights
		with_traffic_lights_controller;

	/* Controllers for the CBF-based longitudinal approach */
	/* [June 2023] Have a map of gap controllers ? Requires changing
	organization of ControlMananger and related classes */
	/*VelocityController nominal_controller;
	SafetyCriticalGapController safe_origin_lane_controller;
	SafetyCriticalGapController safe_destination_lane_controller;
	SafetyCriticalGapController safe_gap_generating_controller;*/

	LateralController lateral_controller;
	/* indicates which controller is active. Used for debugging and
	visualization. */
	ALCType active_longitudinal_controller_type{ ALCType::origin_lane };
	
	const std::unordered_map<int, TrafficLight>* traffic_lights {nullptr};

	/* -------------------------------------------- */

	VehicleController() = default;
	VehicleController(const EgoVehicle& ego_vehicle, bool verbose);

	/* Gets the minimum of the accelerations in the map and sets the
	active longitudinal controller. */
	double choose_minimum_acceleration(
		std::unordered_map<ALCType, double>& possible_accelerations);

private:
	const EgoVehicle* ego_vehicle{ nullptr };

	double origin_lane_leader_max_brake{ 0.0 };
	double destination_lane_leader_max_brake{ 0.0 };
	double destination_lane_follower_max_brake{ 0.0 };
	double assisted_vehicle_max_brake{ 0.0 };
	VehicleType origin_lane_leader_type{ VehicleType::undefined };
	VehicleType destination_lane_leader_type{ VehicleType::undefined };
	VehicleType destination_lane_follower_type{ VehicleType::undefined };

	virtual double implement_compute_desired_acceleration() = 0;
	virtual double implement_get_safe_time_headway() const { 
		return -1.0; 
	};
	virtual double implement_get_current_desired_time_headway() const {
		return -1.0; 
	};
	virtual double implement_get_reference_gap() const {
		return -1.0;
	};
	const LongitudinalController* get_active_long_controller() const;
};
