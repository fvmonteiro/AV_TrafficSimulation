/* ------------------------------------------------------------------------ */
/* 
Based on Virdi, N., Grzybowska, H., Waller, S. T., Dixit, V. (2019). 
A safety assessment of mixed fleets with Connected and Autonomous Vehicles 
using the Surrogate Safety Assessment Module. Accident Analysis and 
Prevention (December 2018)
*/
/* ------------------------------------------------------------------------ */
#pragma once

#include "ConnectedAutonomousVehicle.h"

class VirdiVehicle : public ConnectedAutonomousVehicle
{
public:
	VirdiVehicle(long id, double desired_velocity,
		double simulation_time_step, double creation_time,
		bool verbose);

private:
	void find_relevant_nearby_vehicles() override;
	double compute_desired_acceleration(
		const std::unordered_map<int, TrafficLight>& traffic_lights) override;
	
	bool can_start_lane_change() override;

	double compute_accepted_lane_change_gap(
		std::shared_ptr<NearbyVehicle> nearby_vehicle) override;
	
	/* Methods not used by this vehicle */
	double compute_lane_changing_desired_time_headway(
		const NearbyVehicle& nearby_vehicle) const override
	{
		return 0.0;
	};
	void implement_set_accepted_lane_change_risk_to_leaders(
		double value) override {};
	void implement_set_accepted_lane_change_risk_to_follower(
		double value) override {};
	void implement_set_use_linear_lane_change_gap(long value) override {};

	bool is_lane_change_gap_safe(
		std::shared_ptr<NearbyVehicle>& nearby_vehicle);
};