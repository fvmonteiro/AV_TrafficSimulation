/*==========================================================================*/
/*  VehicleController.h    											        */
/*                															*/
/*                                                                          */
/*  Version of xxxx-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#pragma once

#include <unordered_map>

#include "SwitchedLongitudinalController.h"
#include "Vehicle.h"
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
	LongitudinalController::State
		get_longitudinal_controller_state() const;
	/* Computes the deceleration rate to avoid collision */
	double get_desired_acceleration();
	double get_reference_gap() const;
	double get_gap_error() const;
	/* Safe value depends on whether or not the vehicle has lane
	change intention */
	double get_safe_time_headway() const;
	/* Gets the current time headway, which can be any value between
	the veh following or lane changing time headways if the vehicle
	is transitiong between states. */
	double get_current_desired_time_headway() const;
	/* Returns the safe time headway gap. */
	double get_desired_time_headway_gap(
		const NearbyVehicle& nearby_vehicle) const;
	
	double compute_drac(double relative_velocity, double gap);

	void set_verbose(bool value);

	void add_internal_controllers();

	/* Resets the origin lane controller's velocity and time headway filters
	and sets the time headway */
	void activate_origin_lane_controller(const NearbyVehicle& real_leader);
	void update_origin_lane_controller(const NearbyVehicle& real_leader);

	void reset_origin_lane_velocity_controller();

	/* Printing ----------------------------------------------------------- */
	void print_traffic_lights(
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
	
	std::shared_ptr<SwitchedLongitudinalController> origin_lane_controller;

	/* Constructor for derived classes */
	VehicleController(const EgoVehicle* ego_vehicle, bool verbose);

	/* Gets the minimum of the accelerations in the map and sets the
	active longitudinal controller. */
	double choose_minimum_acceleration(
		std::unordered_map<ALCType, double>& possible_accelerations);

	/* Finds which time headway leads to zero gap error. Useful when
	there are new real or virtual leaders */
	double find_comfortable_time_headway(const NearbyVehicle& a_leader,
		double standstill_distance);

	NearbyVehicle create_virtual_stopped_vehicle(
		const EgoVehicle& ego_vehicle) const;

private:
	const EgoVehicle* ego_vehicle{ nullptr };

    /* -------------------------------------------------------------------- */
	virtual void implement_add_internal_controllers() = 0;
	virtual double implement_get_desired_acceleration() = 0;
	virtual void implement_update_origin_lane_controller(
		const NearbyVehicle& real_leader) = 0;
	virtual double implement_get_desired_time_headway_gap(
		const NearbyVehicle& real_leader) const = 0;

	const LongitudinalController* get_active_long_controller() const;
};
