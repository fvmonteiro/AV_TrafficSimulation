#pragma once

#include "LongitudinalController.h"
#include "SafetyCriticalGapController.h"
#include "VelocityController.h"

/* Class that contains CBF-based safe gap controllers for lane keeping,
lane changing and cooperation as well as a nominal controller for 
velocity tracking */
class SafeLongitudinalController : public LongitudinalController
{
public:
	SafeLongitudinalController() = default;
	SafeLongitudinalController(double velocity_controller_gain,
		std::unordered_map<State, color_t> state_to_color_map,
		bool verbose);

private:
	VelocityController nominal_controller;
	/* [June 2023] Have a map of gap controllers ? Requires changing
	organization of ControlMananger and related classes */
	SafetyCriticalGapController gap_controller;

	double implement_get_gap_error() const override;
	double implement_compute_desired_acceleration(
		const EgoVehicle& ego_vehicle,
		const NearbyVehicle* leader,
		double velocity_reference) override;

	double choose_acceleration(const EgoVehicle& ego_vehicle,
		std::unordered_map<State, double>& possible_accelerations);
};

