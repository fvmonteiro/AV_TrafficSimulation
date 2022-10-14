#include "EgoVehicle.h"
#include "LongitudinalController.h"
#include "NearbyVehicle.h"

const std::unordered_map<LongitudinalController::State, std::string> 
LongitudinalController::state_to_string_map = {
	{ State::uninitialized, "uninitialized" },
	{ State::velocity_control, "velocity_control" },
	{ State::vehicle_following, "vehicle_following" } 
};

double LongitudinalController::get_desired_acceleration(
	const EgoVehicle& ego_vehicle,
	const std::shared_ptr<NearbyVehicle> leader,
	double velocity_reference)
{
	return compute_desired_acceleration(ego_vehicle, leader, 
		velocity_reference);
}

std::string LongitudinalController::state_to_string(State state)
{
	return state_to_string_map.at(state);
};
