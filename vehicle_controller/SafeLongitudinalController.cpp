#include <iostream>

#include "EgoVehicle.h"
#include "SafeLongitudinalController.h"

SafeLongitudinalController::SafeLongitudinalController(
	double velocity_controller_gain,
	std::unordered_map<State, color_t> state_to_color_map,
	bool verbose) :
	LongitudinalController(state_to_color_map, verbose),
	nominal_controller{ VelocityController(velocity_controller_gain, verbose) },
	gap_controller{ SafetyCriticalGapController() }
{
	if (verbose)
	{
		std::clog << "Creating safe longitudinal controller" << std::endl;
	}
}

double SafeLongitudinalController::implement_get_gap_error() const
{
	return gap_controller.get_gap_error();
}

double SafeLongitudinalController::implement_compute_desired_acceleration(
	const EgoVehicle& ego_vehicle,
	std::shared_ptr<const NearbyVehicle> leader,
	double velocity_reference)
{
	std::unordered_map<State, double> possible_accelerations;

	possible_accelerations[State::comf_accel] = 
		ego_vehicle.get_comfortable_acceleration();
	possible_accelerations[State::velocity_control] =
		nominal_controller.compute_acceleration(ego_vehicle, 
			velocity_reference);
	if (ego_vehicle.has_leader())
	{
		possible_accelerations[State::vehicle_following] =
			gap_controller.compute_safe_acceleration(ego_vehicle,
				ego_vehicle.get_leader());
	}

	return choose_acceleration(ego_vehicle, possible_accelerations);
}

double SafeLongitudinalController::choose_acceleration(
	const EgoVehicle& ego_vehicle,
	std::unordered_map<State, double>& possible_accelerations)
{
	if (verbose) std::clog << "Getting min accel:" << "\n\t";

	double desired_acceleration = 1000; // any high value
	for (const auto& it : possible_accelerations)
	{
		if (verbose) std::clog << state_to_string(it.first)
			<< "=" << it.second << ", ";

		if (it.second < desired_acceleration)
		{
			desired_acceleration = it.second;
			state = it.first;
		}
	}
	if (verbose) std::clog << "\n";

	/* [June 2023] 
	TODO: create check regarding current gap < ref.gap. Difficulty:
	differentiate between gap-creation phase and actual risky moments */

	return desired_acceleration;
}
