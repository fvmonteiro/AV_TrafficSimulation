#include "EgoVehicle.h"
#include "VissimLongitudinalController.h"

VissimLongitudinalController::VissimLongitudinalController(
	std::unordered_map<State, color_t> state_to_color_map)
	: LongitudinalController(state_to_color_map, false) {}

double VissimLongitudinalController::implement_get_gap_error() const
{
	return 0.0;
}

double VissimLongitudinalController::implement_compute_desired_acceleration(
	const EgoVehicle& ego_vehicle, const NearbyVehicle* leader,
	double velocity_reference)
{
	return ego_vehicle.get_vissim_acceleration();
}