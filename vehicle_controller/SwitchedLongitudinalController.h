/*==========================================================================*/
/* SwtichedLongitudinalController.h     									*/
/* Base class for longitudinal controllers that do switching between		*/
/* gap and velocity control													*/
/*                                                                          */
/* Version of 2021-xx-xx                             Fernando V. Monteiro   */
/*==========================================================================*/

#pragma once

#include <memory>
#include <string>

#include "Constants.h"
#include "LongitudinalController.h"
#include "VariationLimitedFilter.h"
#include "VelocityController.h"
#include "TimeHeadwayGapController.h"

// Forward declaration
class NearbyVehicle;
class EgoVehicle;

/* This controller computes inputs for vehicle following and velocity
control. It also determine which of these inputs should be used */
class SwitchedLongitudinalController: public LongitudinalController
{
public:

	SwitchedLongitudinalController() = default;
	SwitchedLongitudinalController(
		const VelocityControllerGains& velocity_controller_gains,
		const AutonomousGains& autonomous_gains, 
		const ConnectedGains& connected_gains,
		double velocity_filter_gain, double time_headway_filter_gain,
		double filter_brake_limit, double comfortable_acceleration,
		double simulation_time_step, bool verbose);

	
	/* --------------- Methods related to velocity control ---------------- */
	
	void reset_velocity_controller(double reset_velocity);

	/* ------------------ Methods related to gap control ------------------ */

	void connect_gap_controller(bool is_connected);
	void reset_leader_velocity_filter(double reset_velocity);
	void reset_time_headway_filter(double time_headway);
	/* Returns the desired (final) time headway. */
	double get_desired_time_headway() const;
	/* Returns the current time headway in use */
	double get_current_time_headway() const;
	double get_time_headway_gap(double time_headway, double velocity);
	double get_desired_time_headway_gap(double ego_velocity/*,
		bool has_lane_change_intention*/);
	/* Computes desired gap with a possibly varying time headway */
	//double compute_desired_gap(double ego_velocity);
	double get_desired_gap(double ego_velocity);
	void set_desired_time_headway(double time_headway);

	void compute_max_risk_to_leader(bool is_lane_changing);


protected:
	VelocityController velocity_controller;
	TimeHeadwayGapController gap_controller;

	//State state{ State::uninitialized }; // event driven logic variable
	double hysteresis_bias{ 10.0 }; // used to avoid state chattering [m]

	bool verbose{ false };

	/* Computes the gap threshold to decide whether velocity or vehicle
	following control */
	double compute_gap_threshold(double gap, 
		double diff_to_velocity_reference, double gap_control_input);

private:

	double compute_desired_acceleration(const EgoVehicle& ego_vehicle,
		const std::shared_ptr<NearbyVehicle> leader,
		double velocity_reference) override;

	/* Determines and sets the current state of the longitudinal controller
	TODO: should this class provide a default implementation?*/
	virtual void determine_controller_state(const EgoVehicle& ego_vehicle,
		const std::shared_ptr<NearbyVehicle> leader,
		double reference_velocity, double gap_control_input) = 0;

	//double simulation_time_step{ 0.01 };

	/* Vehicle following and velocity control gains
	(computed in Matlab. Should be computed here?) */
	AutonomousGains autonomous_gains;
	ConnectedGains connected_gains;

};

