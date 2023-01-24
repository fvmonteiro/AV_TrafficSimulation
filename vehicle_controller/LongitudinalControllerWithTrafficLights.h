#pragma once

#include <unordered_map>

#include "Constants.h"
#include "LongitudinalController.h"
#include "TrafficLight.h"

/* Forward declaration */
class EgoVehicle;
class TrafficLightALCVehicle;

class LongitudinalControllerWithTrafficLights: public LongitudinalController
{
public:
	LongitudinalControllerWithTrafficLights() = default;
	LongitudinalControllerWithTrafficLights(
		std::unordered_map<State, color_t> state_to_color_map,
		bool verbose);

	void compute_traffic_light_input_parameters(
		const TrafficLightALCVehicle& ego_vehicle,
		const std::unordered_map<int, TrafficLight>& traffic_lights);

private:
	double time_headway{ 1.0 }; // [s]
	double standstill_distance{ 3.0 };  // [m]
	//double max_speed{ 30.0 };
	double max_accel{ COMFORTABLE_ACCELERATION }; // [m/s2]
	double comfortable_braking{ 4.0 }; // [m/s2] absolute value
	double veh_foll_gain{ 2.0 };
	double vel_control_gain{ 1.0 };
	double gap_error{ 0.0 };  // [m] "gap error" considering relative velocity
	double beta{ 4.0 }; // 2.0, desired_vel / comfortable_braking + 1
	double h3{ 0.0 }, dht{ 0.0 }, dhx{ 0.0 };

	double implement_get_gap_error() const override;
	double implement_compute_desired_acceleration(
		const EgoVehicle& ego_vehicle,
		std::shared_ptr<const NearbyVehicle> leader,
		double velocity_reference) override;

	double get_nominal_input(const EgoVehicle& ego_vehicle);
	double compute_vehicle_following_input(const EgoVehicle& ego_vehicle,
		std::shared_ptr<const NearbyVehicle> leader);
	double compute_velocity_control_input(const EgoVehicle& ego_vehicle,
		double velocity_reference);
	double compute_traffic_light_input(
		const EgoVehicle& ego_vehicle);

	double choose_acceleration(const EgoVehicle& ego_vehicle,//TODO: rename
		std::unordered_map<State, double>& possible_accelerations);
	double choose_minimum_acceleration(
		std::unordered_map<State, double>& possible_accelerations);

	double compute_gap_error_to_next_traffic_light(
		double distance_to_traffic_light, double ego_vel);
	double compute_transient_safe_set(const TrafficLightALCVehicle& ego_vehicle,
		const std::unordered_map<int, TrafficLight>& traffic_lights);

	/* Some version before arriving at the chosen 
	compute_transient_safe_set method above. */
	double compute_transient_safe_set_all_space(
		const TrafficLightALCVehicle& ego_vehicle,
		const std::unordered_map<int, TrafficLight>& traffic_lights);
	double compute_transient_safe_set_amber_light(
		const TrafficLightALCVehicle& ego_vehicle,
		const std::unordered_map<int, TrafficLight>& traffic_lights);
};

