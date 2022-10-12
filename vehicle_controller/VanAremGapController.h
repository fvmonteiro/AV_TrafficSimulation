/* ------------------------------------------------------------------------ */
/* Based on the description in Talebpour, A., Mahmassani, H. S. (2016). 
Influence of connected and autonomous vehicles on traffic flow stability and 
throughput. Transportation Research Part C: Emerging Technologies */
/* ------------------------------------------------------------------------ */

#pragma once
#include "GapController.h"
class VanAremGapController :
    public GapController
{
public:
	VanAremGapController() = default;
	VanAremGapController(double simulation_time_step,
		const ConnectedGains& connected_gains,
		double velocity_filter_gain,
		double comfortable_acceleration, double filter_brake_limit,
		bool verbose);

private:
	ConnectedGains connected_gains;
	double simulation_time_step{ 0.1 }; // [s]
	double gap_min{ 2.0 }; // [m]
	double current_desired_gap{ 0.0 }; // [m]

	double implement_get_desired_gap(double ego_velocity) override;
	double compute_desired_gap(const EgoVehicle& ego_vehicle) override;

	double compute_connected_input(double gap_error,
		double velocity_error, double ego_acceleration,
		double leader_acceleration) override;

	/* Not defined for this controller */
	double compute_autonomous_input(double gap_error,
		double velocity_error) override;
};

