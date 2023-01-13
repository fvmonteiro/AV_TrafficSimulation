#include <iostream>

#include "EgoVehicle.h"
#include "LongitudinalControllerWithTrafficLights.h"
#include "TrafficLightALCVehicle.h"

LongitudinalControllerWithTrafficLights::
LongitudinalControllerWithTrafficLights(
	std::unordered_map<State, color_t> state_to_color_map,
	bool verbose):
	LongitudinalController(state_to_color_map, verbose)
{
	if (verbose)
	{
		std::clog << "Creating traffic-light acc controller" << std::endl;
	}
}

double LongitudinalControllerWithTrafficLights::
implement_get_gap_error() const
{
	return gap_error;
}

double LongitudinalControllerWithTrafficLights::
implement_compute_desired_acceleration(
	const EgoVehicle& ego_vehicle,
	const std::shared_ptr<NearbyVehicle> leader,
	double velocity_reference)
{
	std::unordered_map<State, double>
		possible_accelerations;

	possible_accelerations[State::comf_accel] = 
		get_nominal_input(ego_vehicle);
	possible_accelerations[State::velocity_control] =
		compute_velocity_control_input(ego_vehicle, velocity_reference);
	if (ego_vehicle.has_leader())
	{
		possible_accelerations[State::vehicle_following] =
			compute_vehicle_following_input(ego_vehicle,
				ego_vehicle.get_leader());
	}
	if (ego_vehicle.has_next_traffic_light())
	{
		possible_accelerations[State::traffic_light] = 
			compute_traffic_light_input(ego_vehicle);
	}

	return choose_acceleration(ego_vehicle, possible_accelerations);
}

double LongitudinalControllerWithTrafficLights::get_nominal_input(
	const EgoVehicle& ego_vehicle)
{
	return ego_vehicle.get_comfortable_acceleration();
}

double LongitudinalControllerWithTrafficLights
::compute_vehicle_following_input(const EgoVehicle& ego_vehicle,
	const std::shared_ptr<NearbyVehicle> leader)
{	
	//std::shared_ptr<NearbyVehicle> leader = ego_vehicle.get_leader();
	double gap = ego_vehicle.compute_gap(leader);
	double ego_vel = ego_vehicle.get_velocity();
	double rel_vel = leader->get_relative_velocity();
	double leader_vel = leader->compute_velocity(ego_vel);
	double safe_gap = time_headway * ego_vel + standstill_distance
		+ (std::pow(ego_vel, 2) - std::pow(leader_vel, 2)) 
		/ 2 / comfortable_braking;
	gap_error = gap - safe_gap;
	//double gap_error = gap - safe_gap;

	double leader_accel = leader->get_acceleration();
	double connected_extra_term = leader_accel / comfortable_braking 
		* leader_vel;

	//if (verbose)
	//{
	//	std::clog << "veh type: " << static_cast<int>(ego_vehicle.get_type())
	//		<< ", is connected? " << ego_vehicle.is_connected()
	//		<< ", leader type: " << static_cast<int>(leader->get_type())
	//		<< ", is connected? " << leader->is_connected()
	//		<< std::endl;
	//}
	double desired_acceleration;
	if (ego_vehicle.get_is_connected() && leader->is_connected())
	{
		//if (verbose) std::clog << "connected" << std::endl;
		desired_acceleration =
			(-rel_vel + veh_foll_gain * gap_error + connected_extra_term)
			* comfortable_braking / (comfortable_braking + ego_vel);
	}
	else
	{
		//if (verbose) std::clog << "not connected" << std::endl;
		desired_acceleration =
			(-rel_vel + veh_foll_gain * gap_error)
			/ (time_headway + ego_vel / comfortable_braking);
	}
	return desired_acceleration;
}

double LongitudinalControllerWithTrafficLights
::compute_velocity_control_input(const EgoVehicle& ego_vehicle,
	double velocity_reference)
{
	//double desired_vel = ego_vehicle.get_desired_velocity();

	double ego_vel = ego_vehicle.get_velocity();
	double vel_error = velocity_reference - ego_vel;
	return vel_control_gain * (vel_error);
}

double LongitudinalControllerWithTrafficLights
::compute_traffic_light_input(const EgoVehicle& ego_vehicle)
{
	// Check is made before calling the function
	//if (!ego_vehicle.has_next_traffic_light()) return false;

	double ego_vel = ego_vehicle.get_velocity();
	// Computation must be done somewhere before calling this function
	//compute_traffic_light_input_parameters(ego_vehicle, traffic_lights);

	if (verbose) std::clog << "beta=" << beta
		<< ", dht=" << dht << ", Vf=" << ego_vel << ", h3=" << h3
		<< std::endl;

	//return (dht - ego_vel + h3) / beta;
	return	comfortable_braking / (beta * comfortable_braking + ego_vel)
		* (dht - ego_vel + h3);
}

double LongitudinalControllerWithTrafficLights::choose_minimum_acceleration(
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

	return desired_acceleration;
}

double LongitudinalControllerWithTrafficLights::choose_acceleration(
	const EgoVehicle& ego_vehicle,
	std::unordered_map<State, double>& possible_accelerations)
{
	double min_from_inputs =
		choose_minimum_acceleration(possible_accelerations);
	if (!ego_vehicle.has_leader())
	{
		return min_from_inputs;
	}

	// TODO: h1 is already computed in the vehicle_following_input method.
	double gap = ego_vehicle.compute_gap(ego_vehicle.get_leader());
	double ego_vel = ego_vehicle.get_velocity();
	double leader_vel = ego_vehicle.get_leader()->compute_velocity(ego_vel);
	double gap_error = gap - time_headway * ego_vel - standstill_distance;
	/*h1 = gap_error
		- (std::pow(ego_vel, 2) - std::pow(leader_vel, 2)) 
		/ 2 / comfortable_braking;*/
	
	double margin = 0.1; // 0 for connected
	if (gap_error >= -margin)
	{
		return min_from_inputs;
	}
	else
	{
		state = State::too_close;
		return std::max(min_from_inputs,
			-ego_vehicle.get_max_brake());
	}
}

void LongitudinalControllerWithTrafficLights
::compute_traffic_light_input_parameters(
	const TrafficLightALCVehicle& ego_vehicle,
	const std::unordered_map<int, TrafficLight>& traffic_lights)
{
	int next_traffic_light_id = ego_vehicle.get_next_traffic_light_id();
	if (next_traffic_light_id == 0) return;

	if (verbose) std::clog << "computing tf acc params" << std::endl;

	/* hx is like the safe gap/ safe distance to the traffic light */
	double hx = compute_gap_error_to_next_traffic_light(
		ego_vehicle.get_distance_to_next_traffic_light(), 
		ego_vehicle.get_velocity());
	
	/* ht is how the safe set varies over time */
	double ht = compute_transient_safe_set(ego_vehicle, traffic_lights);
	h3 = ht + hx;
}

double LongitudinalControllerWithTrafficLights::
compute_transient_safe_set(const TrafficLightALCVehicle& ego_vehicle,
	const std::unordered_map<int, TrafficLight>& traffic_lights)
{
	int next_traffic_light_id = ego_vehicle.get_next_traffic_light_id();

	TrafficLight next_traffic_light =
		traffic_lights.at(next_traffic_light_id);
	double distance_between_traffic_lights;
	int next_next_traffic_light_id = next_traffic_light_id + 1;
	if (traffic_lights.find(next_next_traffic_light_id) !=
		traffic_lights.end())
	{
		distance_between_traffic_lights =
			traffic_lights.at(next_next_traffic_light_id).get_position()
			- next_traffic_light.get_position();
	}
	else
	{
		//Any large value
		distance_between_traffic_lights = 1000;
	}

	double ht;
	if (next_traffic_light.get_current_state() == TrafficLight::State::red)
	{
		ht = 0;
		dht = 0;
	}
	else
	{
		double lambda0 = beta * comfortable_braking;
		double time = ego_vehicle.get_time();
		double next_red_time = next_traffic_light.get_time_of_next_red();
		ht = -lambda0 * (time - next_red_time);
		dht = -lambda0;
		if (ht > distance_between_traffic_lights)
		{
			ht = distance_between_traffic_lights;
			dht = 0;
		}
	}
	return ht;
}

double LongitudinalControllerWithTrafficLights::
compute_gap_error_to_next_traffic_light(double distance_to_traffic_light,
	double ego_vel)
{
	/* hx is like the safe gap/ safe distance to the traffic light */
	double hx = distance_to_traffic_light - beta * ego_vel
		- standstill_distance
		- std::pow(ego_vel, 2) / 2 / comfortable_braking;
	return hx;
}

double LongitudinalControllerWithTrafficLights::
compute_transient_safe_set_amber_light(
	const TrafficLightALCVehicle& ego_vehicle,
	const std::unordered_map<int, TrafficLight>& traffic_lights)
{
	int next_traffic_light_id = ego_vehicle.get_next_traffic_light_id();

	TrafficLight next_traffic_light =
		traffic_lights.at(next_traffic_light_id);
	double distance_between_traffic_lights;
	int next_next_traffic_light_id = next_traffic_light_id + 1;
	if (traffic_lights.find(next_next_traffic_light_id) !=
		traffic_lights.end())
	{
		distance_between_traffic_lights =
			traffic_lights.at(next_next_traffic_light_id).get_position()
			- next_traffic_light.get_position();
	}
	else
	{
		//Any large value
		distance_between_traffic_lights = 1000;
	}

	double ht;
	if (next_traffic_light.get_current_state() == TrafficLight::State::red)
	{
		ht = 0;
		dht = 0;
	}
	else if (next_traffic_light.get_current_state() == TrafficLight::State::green)
	{
		double amber_duration = next_traffic_light.get_amber_duration();
		ht = beta * comfortable_braking * amber_duration;
		dht = 0;
	}
	else // amber light
	{
		double time = ego_vehicle.get_time();
		double lambda0 = beta * comfortable_braking;
		double time_crossed_last_traffic_light =
			ego_vehicle.get_time_crossed_last_traffic_light();
		double last_time_next_traffic_light_became_green =
			next_traffic_light.get_time_of_last_green();
		double last_time_next_traffic_light_became_amber =
			next_traffic_light.get_time_of_last_amber();
		double t0 = std::max(time_crossed_last_traffic_light,
			last_time_next_traffic_light_became_green);
		double lambda = distance_between_traffic_lights /
			std::max(t0, last_time_next_traffic_light_became_amber);
		double min_lambda = std::min(lambda, lambda0);
		double next_red_time = next_traffic_light.get_time_of_next_red();
		ht = -min_lambda * (time - next_red_time);
		dht = -min_lambda;
	}

	/* Tentative change */
	if (ht > (beta * comfortable_braking
		* next_traffic_light.get_amber_duration()))
	{
		ht = (beta * comfortable_braking
			* next_traffic_light.get_amber_duration());
		dht = 0;
	}

	return ht;
}

double LongitudinalControllerWithTrafficLights::
compute_transient_safe_set_all_space(
	const TrafficLightALCVehicle& ego_vehicle,
	const std::unordered_map<int, TrafficLight>& traffic_lights)
{
	int next_traffic_light_id = ego_vehicle.get_next_traffic_light_id();

	TrafficLight next_traffic_light =
		traffic_lights.at(next_traffic_light_id);
	double distance_between_traffic_lights;
	int next_next_traffic_light_id = next_traffic_light_id + 1;
	if (traffic_lights.find(next_next_traffic_light_id) !=
		traffic_lights.end())
	{
		distance_between_traffic_lights =
			traffic_lights.at(next_next_traffic_light_id).get_position()
			- next_traffic_light.get_position();
	}
	else
	{
		//Any large value
		distance_between_traffic_lights = 1000;
	}

	double ht;
	if (next_traffic_light.get_current_state() == TrafficLight::State::red)
	{
		ht = 0;
		dht = 0;
	}
	else
	{
		double lambda0 = beta * comfortable_braking;
		double time = ego_vehicle.get_time();
		double time_crossed_last_traffic_light =
			ego_vehicle.get_time_crossed_last_traffic_light();
		double last_time_next_traffic_light_became_green =
			next_traffic_light.get_time_of_last_green();
		double next_red_time = next_traffic_light.get_time_of_next_red();
		double t0 = std::max(time_crossed_last_traffic_light,
			last_time_next_traffic_light_became_green);
		double t01 = next_red_time - t0; /* available time to converge to
										 safe set of next traffic light */
		double lambda = distance_between_traffic_lights / t01;
		double min_lambda = std::min(lambda, lambda0);
		ht = -min_lambda * (time - next_red_time);
		dht = -min_lambda;
	}

	/* Tentative change */
	/*if (ht > (beta * comfortable_braking
			  * next_traffic_light.get_amber_duration()))
	{
		ht = (beta * comfortable_braking
			* next_traffic_light.get_amber_duration());
		dht = 0;
	}*/

	return ht;
}
