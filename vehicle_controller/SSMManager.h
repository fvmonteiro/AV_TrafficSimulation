#pragma once

#include <unordered_map>
#include <vector>

#include "Vehicle.h"

/* SSMManager aggregates the surrogate safety measurements made at vehicle
level and saves them aggregated by time and link 
ATTENTION: CLASS WILL NOT BE USED. 
It's not possible to save results from the DLL using the simulation
identifier (name, random seed, execution order, etc). Just export UDAs
to vehicle record instead. */
class SSMManager
{
public:
	SSMManager() = default;
	/* SSMs are average at time_frequency*/
	SSMManager(double time_frequency);

	void aggregate_ssms(double new_time, 
		std::unordered_map<long, Vehicle>& vehicles);

private:
	enum class SSMs {
		ttc,
		drac,
		collision_severity_risk,
	};

	double time_frequency{ 0.0 };
	double current_time{ 0.0 };
	double initial_aggregation_time{ 0.0 };
	double time_step_counter{ 0.0 };

	std::unordered_map<SSMs, double> accumulated_ssms;
	std::unordered_map<SSMs, std::vector<double>> ssms;

	double accumulated_ttc{ 0.0 };
	double accumulated_drac{ 0.0 };
	double accumulated_collision_severity_risk{ 0.0 };
	std::vector<double> time;
	std::vector<double> ttc;
	std::vector<double> drac;
	std::vector<double> collision_severity_risk;

	void accumulate_ssms(std::unordered_map<long, Vehicle>& vehicles);
	void average_ssms_over_time_period();
	void reset_accumulated_ssms();
};

