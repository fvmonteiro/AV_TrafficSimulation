#pragma once

#include <unordered_map>

class PlatoonVehicle;

class PlatoonLaneChangeStrategy
{
public:
	enum class LaneChangeState
	{
		lane_keeping,
		long_adjustment,
		lateral_maneuver,
		create_gap,
		close_gap
	};

	//PlatoonLaneChangeStrategy();

	void update_vehicle_lane_change_state(
		const PlatoonVehicle& platoon_vehicle, bool should_change_lane,
		std::unordered_map<long, LaneChangeState>& state_map) const;
	bool can_vehicle_start_lane_change(const PlatoonVehicle& platoon_vehicle,
		const std::unordered_map<long, bool> vehicles_lane_change_gap_status)
		const;
	bool can_vehicle_leave_platoon(long veh_id) const;

private:
	virtual void implement_update_vehicle_lane_change_state(
		const PlatoonVehicle& platoon_vehicle, bool should_change_lane, 
		std::unordered_map<long, LaneChangeState>& state_map) const = 0;
	virtual bool implement_can_vehicle_start_lane_change(
		const PlatoonVehicle& platoon_vehicle,
		std::unordered_map<long, bool> vehicles_lane_change_gap_status)
		const = 0;
	virtual bool implement_can_vehicle_leave_platoon(long veh_id) const = 0;
};

class NoStrategy : public PlatoonLaneChangeStrategy
{
private:
	void implement_update_vehicle_lane_change_state(
		const PlatoonVehicle& platoon_vehicle, bool should_change_lane,
		std::unordered_map<long, LaneChangeState>& state_map) const override;
	bool implement_can_vehicle_start_lane_change(
		const PlatoonVehicle& platoon_vehicle,
		const std::unordered_map<long, bool> vehicles_lane_change_gap_status) 
		const override;
	bool implement_can_vehicle_leave_platoon(long veh_id) const override;
};

class SynchronousStrategy: public PlatoonLaneChangeStrategy
{
private:
	void implement_update_vehicle_lane_change_state(
		const PlatoonVehicle& platoon_vehicle, bool should_change_lane,
		std::unordered_map<long, LaneChangeState>& state_map) const override;
	bool implement_can_vehicle_start_lane_change(
		const PlatoonVehicle& platoon_vehicle,
		std::unordered_map<long, bool> vehicles_lane_change_gap_status)
		const override;
	bool implement_can_vehicle_leave_platoon(long veh_id) const override;
};

class LeaderFirstStrategy: public PlatoonLaneChangeStrategy
{
private:
	void implement_update_vehicle_lane_change_state(
		const PlatoonVehicle& platoon_vehicle, bool should_change_lane,
		std::unordered_map<long, LaneChangeState>& state_map) const override;
	bool implement_can_vehicle_start_lane_change(
		const PlatoonVehicle& platoon_vehicle,
		std::unordered_map<long, bool> vehicles_lane_change_gap_status)
		const override;
	bool implement_can_vehicle_leave_platoon(long veh_id) const override;
};

//class LastVehicleFirstStrategy : public PlatoonLaneChangeStrategy
//{
//private:
//	LaneChangeState implement_update_vehicle_lane_change_state(
//		long veh_id, bool should_change_lane) const override;
//	bool implement_can_vehicle_start_lane_change(
//		const PlatoonVehicle& platoon_vehicle,
//		std::unordered_map<long, bool> vehicles_lane_change_gap_status)
//		const override;
//	bool implement_can_vehicle_leave_platoon(long veh_id) const override;
//};
//
//class LeaderFirstAndInvertStrategy : public PlatoonLaneChangeStrategy
//{
//private:
//	LaneChangeState implement_update_vehicle_lane_change_state(
//		long veh_id, bool should_change_lane) const override;
//	bool implement_can_vehicle_start_lane_change(
//		const PlatoonVehicle& platoon_vehicle,
//		std::unordered_map<long, bool> vehicles_lane_change_gap_status)
//		const override;
//	bool implement_can_vehicle_leave_platoon(long veh_id) const override;
//};