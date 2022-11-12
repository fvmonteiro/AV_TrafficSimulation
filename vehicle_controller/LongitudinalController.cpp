#include "EgoVehicle.h"
#include "LongitudinalController.h"
#include "NearbyVehicle.h"

LongitudinalController::LongitudinalController(
	std::unordered_map<State, color_t> state_to_color_map) :
	state_to_color_map(state_to_color_map) 
{
	state_to_color_map.insert({ State::uninitialized, WHITE });
}

color_t LongitudinalController::get_state_color() const
{
	if (state_to_color_map.find(get_state()) == state_to_color_map.end())
	{
		return WHITE;
	}
	return state_to_color_map.at(get_state());
}

double LongitudinalController::compute_desired_acceleration(
	const EgoVehicle& ego_vehicle,
	const std::shared_ptr<NearbyVehicle> leader,
	double velocity_reference)
{
	return implement_compute_desired_acceleration(ego_vehicle, leader,
		velocity_reference);
}

std::string LongitudinalController::state_to_string(State state)
{
	if (state_to_string_map.find(state) == state_to_string_map.end())
	{
		return "unknown state";
	}
	return state_to_string_map.at(state);
};

const std::unordered_map<LongitudinalController::State, std::string>
LongitudinalController::state_to_string_map = {
	{ State::uninitialized, "uninitialized" },
	{ State::velocity_control, "velocity control" },
	{ State::vehicle_following, "vehicle following" },
	{ State::comf_accel, "comf accel" },
	{ State::traffic_light, "traffic light" },
	{ State::too_close, "too close" }
};