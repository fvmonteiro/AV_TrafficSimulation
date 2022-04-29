#pragma once

#include "EgoVehicle.h"

class PlatoonVehicle : public EgoVehicle
{
public:

	PlatoonVehicle(long id, double desired_velocity,
		double simulation_time_step, double creation_time,
		bool verbose = false) :
		EgoVehicle(id, VehicleType::platoon_car, desired_velocity,
			AUTONOMOUS_BRAKE_DELAY, true, true,
			simulation_time_step, creation_time, verbose) {}

	/* The connected vehicle brake delay also depends on the other vehicle
	type. If the other vehicle is not connected, the ego connected vehicle
	behaves like an autonomous vehicle. This issue is addressed in parts
	of the code after the ego vehicle identifies its
	surrouding vehicles. */

private:
	void find_relevant_nearby_vehicles() override;
	double compute_desired_acceleration(
		const std::unordered_map<int, TrafficLight>& traffic_lights) override;
	bool give_lane_change_control_to_vissim() const override
	{
		return false;
	};
	bool can_start_lane_change() override;
};