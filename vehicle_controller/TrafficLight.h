#pragma once

#include <iostream>
#include <string>
#include <vector>

class TrafficLight
{
public:

	enum class State { /* should become enum class after some debugging */
		no_traffic_light= 0,
		red=1,
		amber=2,
		green=3
	};

	TrafficLight() = default;
	TrafficLight(int id, int position, int red_duration, int green_duration,
		int amber_duration, bool starts_on_red);
	TrafficLight(int id, int position, int red_duration, int green_duration,
		int amber_duration);
	TrafficLight(std::vector<std::string> ordered_parameters); /*not sure this 
															   will be used*/

	int get_id() const { return id; };
	double get_position() const { return position; };
	State get_current_state() const { return current_state; };



	/* TODO: are these functions an issue? We don't want vehicles being able
	to set the state of the traffic light. Or does passing them as const 
	always avoid that? */
	void set_current_state(long state) { current_state = State(state); };
	void set_current_state_start_time(double time) { 
		current_state_start_time = time; 
	};

	double get_time_of_next_red() const;
	double get_time_of_last_green() const;
	//double get_time_of_next_green() const;

	friend std::ostream& operator<< (std::ostream& out,
		const TrafficLight& traffic_light);

private:
	// Static parameters
	int id{ 0 }, position{ 0 }, red_duration{ 0 }, green_duration{ 0 }, 
		amber_duration{ 0 };
	bool starts_on_red{ true };
	// State
	State current_state{ State::no_traffic_light };
	double current_state_start_time{ 0.0 };

};

