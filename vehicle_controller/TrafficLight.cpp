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

double TrafficLight::get_time_of_next_red() const
{
	switch (current_state)
	{
	case TrafficLight::State::red:
		return current_state_start_time + red_duration + green_duration
			+ amber_duration;
	case TrafficLight::State::amber:
		return current_state_start_time + amber_duration;
	case TrafficLight::State::green:
		return current_state_start_time + green_duration + amber_duration;
	default:
		return 0;
	}
}

double TrafficLight::get_time_of_last_green() const
{
	switch (current_state)
	{
	case TrafficLight::State::red:
		return current_state_start_time - amber_duration - green_duration;
	case TrafficLight::State::amber:
		return current_state_start_time - green_duration;
	case TrafficLight::State::green:
		return current_state_start_time;
	default:
		return 0;
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

