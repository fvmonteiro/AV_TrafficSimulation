#pragma once

#include "EgoVehicle.h"
#include "LaneChangeGapsSafety.h"

class VirdiVehicle : public EgoVehicle
{
public:

	VirdiVehicle(long id, double desired_velocity,
		double simulation_time_step, double creation_time,
		bool verbose);

private:
	/* Vehicle immediately ahead at the destination lane */
	std::shared_ptr<NearbyVehicle> destination_lane_leader{ nullptr };
	/* Vehicle immediately behind at the destination lane */
	std::shared_ptr<NearbyVehicle> destination_lane_follower{ nullptr };
	/* Vehicle in front of the destination lane leader */
	std::shared_ptr<NearbyVehicle> destination_lane_leader_leader{ nullptr };
	/* Vehicle behind which we want to merge (not necessarily the same as
	the destination lane leader) */
	std::shared_ptr<NearbyVehicle> virtual_leader{ nullptr };
	std::shared_ptr<NearbyVehicle> assisted_vehicle{ nullptr };
	long lane_change_request{ 0 };

	double min_lane_change_gap{ 0.5 };
	LaneChangeGapsSafety lane_change_gaps_safety;

	void implement_create_controller() override;
	double implement_compute_desired_acceleration(
		const std::unordered_map<int, TrafficLight>& traffic_lights) override;
	void implement_analyze_nearby_vehicles() override;
	bool implement_check_lane_change_gaps() override;
	long implement_get_lane_change_request() const override;
	double compute_accepted_lane_change_gap(
		std::shared_ptr<const NearbyVehicle> nearby_vehicle) const override;
	std::shared_ptr<NearbyVehicle>
		implement_get_destination_lane_leader() const override;
	std::shared_ptr<NearbyVehicle>
		implement_get_destination_lane_follower() const override;
	std::shared_ptr<NearbyVehicle>
		implement_get_assisted_vehicle() const override;
	long implement_get_virtual_leader_id() const override;

	double compute_lane_changing_desired_time_headway(
		const NearbyVehicle& nearby_vehicle) const override
	{
		return 0.0;
	};
	bool give_lane_change_control_to_vissim() const override
	{
		return false;
	};
	void implement_set_accepted_lane_change_risk_to_leaders(
		double value) override {};
	void implement_set_accepted_lane_change_risk_to_follower(
		double value) override {};
	void implement_set_use_linear_lane_change_gap(long value) override {};

	bool is_lane_change_gap_safe(
		std::shared_ptr<const NearbyVehicle> nearby_vehicle) const;
};

