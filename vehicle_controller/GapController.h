#pragma once

#include <memory>

#include "VariationLimitedFilter.h"

class EgoVehicle;
class NearbyVehicle;

struct AutonomousGains {
	double kg{ 0.0 }; // relative to gap error.
	double kv{ 0.0 }; // relative to velocity error.
};

struct ConnectedGains {
	double kg{ 0.0 }; // relative to gap error.
	double kv{ 0.0 }; // relative to velocity error.
	double kgd{ 0.0 }; // relative to gap error derivative
	double ka{ 0.0 }; // relative to acceleration error
};

/* Constant time headway based controller */
class GapController
{
public:
	GapController() = default;
	GapController(double simulation_time_step, 
		double velocity_filter_gain,
		double comfortable_acceleration, double filter_brake_limit,
		bool verbose);

	double get_standstill_distance() const { return standstill_distance; };
	bool get_is_connected() const { return is_connected; };

	void set_connexion(bool is_conneced) {
		this->is_connected = is_conneced;
	};
	
	double get_desired_gap(double ego_velocity);

	double compute_desired_acceleration(const EgoVehicle& ego_vehicle, 
		const std::shared_ptr<NearbyVehicle> leader);

	void reset_velocity_filter(double ego_velocity);
	void update_leader_velocity_filter(double leader_velocity);

protected:
	bool verbose{ false };

	/* Computes gap minus reference gap and upper bounds it
	with max_gap_error. */
	double compute_gap_error(double gap, double reference_gap) const;

	/* Computes velocity error "typically": vLeader - vEgo*/
	double compute_velocity_error(double velocity_ego,
		double velocity_leader) const;

	double compute_acceleration_error(
		double acceleration_ego, double acceleration_reference) const;

private:
	VariationLimitedFilter velocity_filter;
	
	double standstill_distance{ 1.0 }; /* standstill distance [m] */

	/* Other controller parameters */
	double max_gap_error{ 30.0 }; // maximum positive gap error in meters
	double max_gap_error_connected{ 20.0 }; /* maximum positive gap 
											error in meters */
	bool is_connected{ false };

	virtual double implement_get_desired_gap(double ego_velocity) = 0;
	/* Updates the time headway and computes desired gap */
	virtual double compute_desired_gap(const EgoVehicle& ego_vehicle) = 0;

	virtual double compute_connected_input(double gap_error,
		double velocity_error, double ego_acceleration,
		double leader_acceleration) = 0;

	virtual double compute_autonomous_input(double gap_error,
		double velocity_error) = 0;
};

