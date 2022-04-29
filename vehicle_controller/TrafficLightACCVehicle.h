#pragma once

#include "EgoVehicle.h"

class TrafficLightACCVehicle : public EgoVehicle
{
public:

	TrafficLightACCVehicle(long id, double desired_velocity,
		double simulation_time_step, double creation_time,
		bool verbose = false) :
		EgoVehicle(id, VehicleType::traffic_light_acc_car, desired_velocity,
			AUTONOMOUS_BRAKE_DELAY, true, false,
			simulation_time_step, creation_time, verbose) {}

	/* The "autonomous lane change" of this vehicle is never lane changing.
	This should be better addressed with polymorphism. */

	int get_next_traffic_light_id() const {
		return next_traffic_light_id;
	};
	double get_time_crossed_last_traffic_light() const {
		return time_crossed_last_traffic_light;
	};
	double get_distance_to_next_traffic_light() const {
		return distance_to_next_traffic_light;
	};

	bool has_next_traffic_light() const;

protected:
	TrafficLightACCVehicle(long id, VehicleType type,
		double desired_velocity, bool is_connected, 
		double simulation_time_step,
		double creation_time, bool verbose = false) :
		EgoVehicle(id, type, desired_velocity,
			AUTONOMOUS_BRAKE_DELAY, true, is_connected,
			simulation_time_step, creation_time, verbose) {}

private:
	double compute_desired_acceleration(
		const std::unordered_map<int, TrafficLight>& traffic_lights) override;
	bool give_lane_change_control_to_vissim() const override
	{
		return false;
	};
	void set_desired_lane_change_direction() override;
	bool can_start_lane_change() override;

	/* Traffic lights -------------------------------------------------------- */
	void TrafficLightACCVehicle::set_traffic_light_information(
		int traffic_light_id, double distance) override;
	double time_crossed_last_traffic_light{ 0.0 };
	int next_traffic_light_id{ 0 };
	double distance_to_next_traffic_light{ 0.0 };
};

class TrafficLightCACCVehicle : public TrafficLightACCVehicle
{
public:

	TrafficLightCACCVehicle(long id, double desired_velocity,
		double simulation_time_step, double creation_time,
		bool verbose = false) :
		TrafficLightACCVehicle(id, VehicleType::traffic_light_cacc_car,
			desired_velocity, true, simulation_time_step, creation_time, 
			verbose) {}

	/* The "autonomous lane change" of this vehicle is never lane changing.
	This should be better addressed with polymorphism. */
};

