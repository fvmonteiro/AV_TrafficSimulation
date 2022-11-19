#pragma once

#include <unordered_map>

#include "VehicleState.h"

class PlatoonVehicle;
class Platoon;

class PlatoonLaneChangeStrategy
{
public:
	//enum class LaneChangeState
	//{
	//	lane_keeping,
	//	long_adjustment,
	//	lateral_maneuver,
	//	create_gap,
	//	close_gap
	//};

	void set_platoon(Platoon* platoon) { this->platoon = platoon; };
	/*void update_vehicle_lane_change_state(
		const PlatoonVehicle& platoon_vehicle, bool should_change_lane,
		std::unordered_map<long, LaneChangeState>& state_map) const;*/
	/*bool can_vehicle_start_lane_change(const PlatoonVehicle& platoon_vehicle,
		const std::unordered_map<long, bool> vehicles_lane_change_gap_status)
		const;*/
	bool can_vehicle_leave_platoon(long veh_id) const;
	std::unique_ptr<VehicleState> get_new_lane_keeping_state() const;
	void set_state_of_all_vehicles();

protected:
	//PlatoonLaneChangeStrategy(Platoon* platoon);
	
	/* Check if we can make this a shared pointer */
	Platoon* platoon{ nullptr };
	
	/* We call "responsible vehicle" the platoon vehicle that
	initiates the lane change process */
	/*LaneChangeState determine_responsible_vehicle_state(
		const PlatoonVehicle& platoon_vehicle, bool should_change_lane) const;*/

private:
	/*virtual void implement_update_vehicle_lane_change_state(
		const PlatoonVehicle& platoon_vehicle, bool should_change_lane, 
		std::unordered_map<long, LaneChangeState>& state_map) const = 0;
	virtual bool implement_can_vehicle_start_lane_change(
		const PlatoonVehicle& platoon_vehicle,
		std::unordered_map<long, bool> vehicles_lane_change_gap_status)
		const = 0;*/
	virtual bool implement_can_vehicle_leave_platoon(long veh_id) const = 0;
	virtual std::unique_ptr<VehicleState> 
		implement_get_new_lane_keeping_state() const = 0;
};

class NoStrategy : public PlatoonLaneChangeStrategy
{
private:
	/*void implement_update_vehicle_lane_change_state(
		const PlatoonVehicle& platoon_vehicle, bool should_change_lane,
		std::unordered_map<long, LaneChangeState>& state_map) const override;
	bool implement_can_vehicle_start_lane_change(
		const PlatoonVehicle& platoon_vehicle,
		const std::unordered_map<long, bool> vehicles_lane_change_gap_status) 
		const override;*/
	bool implement_can_vehicle_leave_platoon(long veh_id) const override;
	std::unique_ptr<VehicleState> 
		implement_get_new_lane_keeping_state() const override;
};

class SynchronousStrategy: public PlatoonLaneChangeStrategy
{
private:
	//void implement_update_vehicle_lane_change_state(
	//	const PlatoonVehicle& platoon_vehicle, bool should_change_lane,
	//	std::unordered_map<long, LaneChangeState>& state_map) const override;
	//bool implement_can_vehicle_start_lane_change(
	//	const PlatoonVehicle& platoon_vehicle,
	//	std::unordered_map<long, bool> vehicles_lane_change_gap_status)
	//	const override;
	bool implement_can_vehicle_leave_platoon(long veh_id) const override;
	std::unique_ptr<VehicleState>
		implement_get_new_lane_keeping_state() const override;
};

//class LeaderFirstStrategy: public PlatoonLaneChangeStrategy
//{
//private:
//	void implement_update_vehicle_lane_change_state(
//		const PlatoonVehicle& platoon_vehicle, bool should_change_lane,
//		std::unordered_map<long, LaneChangeState>& state_map) const override;
//	bool implement_can_vehicle_start_lane_change(
//		const PlatoonVehicle& platoon_vehicle,
//		std::unordered_map<long, bool> vehicles_lane_change_gap_status)
//		const override;
//	bool implement_can_vehicle_leave_platoon(long veh_id) const override;
//};

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