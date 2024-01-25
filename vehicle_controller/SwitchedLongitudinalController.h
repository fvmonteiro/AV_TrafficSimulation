/*==========================================================================*/
/* SwtichedLongitudinalController.h     									*/
/* Base class for longitudinal controllers that do switching between		*/
/* gap and velocity control													*/
/*                                                                          */
/* Version of xxxx-xx-xx                             Fernando V. Monteiro   */
/*==========================================================================*/

#pragma once

#include <memory>
#include <string>

#include "Constants.h"
#include "LongitudinalController.h"
#include "VariationLimitedFilter.h"
#include "VelocityController.h"

// Forward declaration
class NearbyVehicle;
class EgoVehicle;

/* This controller computes inputs for vehicle following and velocity
control. It also determine which of these inputs should be used */
class SwitchedLongitudinalController: public LongitudinalController
{
public:

	SwitchedLongitudinalController() = default;
	SwitchedLongitudinalController(const EgoVehicle* ego_vehicle,
		std::unordered_map<State, color_t> state_to_color_map,
		bool verbose);

	void create_velocity_controller(
		const VelocityControllerGains& velocity_controller_gains,
		double velocity_filter_gain);
	void create_gap_controller(const AutonomousGains& autonomous_gains,
		const ConnectedGains& connected_gains, double velocity_filter_gain,
		double time_headway_filter_gain);

	bool is_initialized() const;
	/* Checks if the reference velocity is 'old'. That may happen when
	* the controller is not active and the vehicle's velocity changed
	* faster than the reference velocity filter.
	TODO [Jan 24, 2024] is this really necessary? Can't we just check
	this condition when the controller becomes active? */
	bool is_velocity_reference_outdated() const;

	/* --------------- Methods related to velocity control ---------------- */
	
	void reset_velocity_controller(double reset_velocity);

	/* ------------------ Methods related to gap control ------------------ */

	void connect_gap_controller(bool is_connected);
	void smooth_start_leader_velocity_filter();
	void reset_leader_velocity_filter(double reset_velocity);
	void reset_time_headway_filter(double time_headway);
	/* Returns the desired (final) time headway. */
	double get_desired_time_headway() const;
	/* Returns the current time headway in use */
	double get_current_time_headway() const;
	double get_time_headway_gap(double time_headway, double velocity) const;
	double get_desired_time_headway_gap() const;
	double get_standstill_distance() const;
	/* Computes desired gap with a possibly varying time headway */
	//double compute_desired_gap(double ego_velocity);
	double get_desired_gap() const;
	void set_desired_time_headway(double time_headway);
	/* To be used ONLY when the compute_desired_acceleration method is not
	called. So far, only necessary when we allow VISSIM to take control
	of the vehicle. */
	void update_leader_velocity_filter(double leader_velocity);

	//void compute_max_risk_to_leader(bool is_lane_changing);


protected:
	VelocityController velocity_controller;
	GapController gap_controller;
	//State state{ State::uninitialized }; // event driven logic variable
	double hysteresis_bias{ 10.0 }; // used to avoid state chattering [m]
	double reference_velocity_margin{ 1.0 }; // [m/s]

	/* Computes the gap threshold to decide whether velocity or vehicle
	following control. Threshold is computed such that, at the switch, 
	the vehicle following input is greater or equal to kg*h*(Vf - v) > 0. */
	double compute_gap_threshold_1(double gap, 
		double diff_to_velocity_reference, double gap_control_input) const;

private:
	virtual double get_max_accepted_brake() const = 0;
	/* Determines and sets the current state of the longitudinal controller */
	virtual void determine_controller_state(
		const NearbyVehicle* leader, double reference_velocity, 
		double gap_control_input) = 0;
	virtual bool implement_is_velocity_reference_outdated() const = 0;

	double implement_get_gap_error() const override;
	double implement_compute_desired_acceleration(
		const NearbyVehicle* leader, double velocity_reference) override;
};

