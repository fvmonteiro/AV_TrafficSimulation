#include "TrafficLight.h"

TrafficLight::TrafficLight(int id, int position, int red_duration, 
	int green_duration, int amber_duration, bool starts_on_red) :
	id{id}, position{position}, red_duration{red_duration},
	green_duration{green_duration}, amber_duration{amber_duration},
	starts_on_red{ starts_on_red }{}

TrafficLight::TrafficLight(int id, int position, int red_duration,
	int green_duration, int amber_duration) :
	TrafficLight(id, position, red_duration, 
		green_duration, amber_duration, true) {}

TrafficLight::TrafficLight(std::vector<std::string> ordered_parameters) :
	TrafficLight(std::stoi(ordered_parameters[0]),
		std::stoi(ordered_parameters[1]),
		std::stoi(ordered_parameters[2]),
		std::stoi(ordered_parameters[3]),
		std::stoi(ordered_parameters[4])) {}

double TrafficLight::time_for_next_red(
	double state_start_time, double current_time) 
{
	double time_in_state = current_time - state_start_time;
	switch (current_state)
	{
	case TrafficLight::State::red:
		return red_duration - time_in_state + green_duration
			+ amber_duration;
		break;
	case TrafficLight::State::amber:
		return amber_duration - time_in_state;
		break;
	case TrafficLight::State::green:
		return green_duration - time_in_state + amber_duration;
		break;
	default:
		return 0;
		break;
	}
}

double TrafficLight::time_for_next_green(
	double state_start_time, double current_time)
{
	double time_in_state = current_time - state_start_time;
	switch (current_state)
	{
	case TrafficLight::State::red:
		return red_duration - time_in_state;
		break;
	case TrafficLight::State::amber:
		return amber_duration - time_in_state + red_duration;
		break;
	case TrafficLight::State::green:
		return green_duration - time_in_state + amber_duration
			+ red_duration;
		break;
	default:
		return 0;
		break;
	}
}

std::ostream& operator<<(std::ostream& out,
	const TrafficLight& traffic_light)
{
	out << "id: " << traffic_light.id
		<< ", position: " << traffic_light.position
		<< ", red duration: " << traffic_light.red_duration
		<< ", green duration: " << traffic_light.green_duration
		<< ", amber duration: " << traffic_light.amber_duration;

	return out; // return std::ostream so we can chain calls to operator<<
}

