#pragma once

#include "EgoVehicle.h"

/* Vehicle with autonomous longitudinal control during lane keeping.
Whenever there's lane change intention, the control is handed over
to VISSIM */
class ACCVehicle : public EgoVehicle
{
public:

	ACCVehicle(long id, double desired_velocity,
		double simulation_time_step, double creation_time,
		bool verbose = false) :
		EgoVehicle(id, VehicleType::acc_car, desired_velocity,
			AUTONOMOUS_BRAKE_DELAY, false, false,
			simulation_time_step, creation_time, verbose) {}

private:
	double compute_desired_acceleration(
		const std::unordered_map<int, TrafficLight>& traffic_lights) override;
	double compute_lane_changing_desired_time_headway(
		const NearbyVehicle& nearby_vehicle) const override;
	bool give_lane_change_control_to_vissim() const override
	{
		return true;
	};
	/* Follows VISSIM's recommendation */
	bool can_start_lane_change() override;
};