#include "PlatoonLaneChangeStrategy.h"
#include "PlatoonVehicle.h"

/* BASE CLASS ------------------------------------------------------------- */

//PlatoonLaneChangeStrategy::PlatoonLaneChangeStrategy()
//{
//	std::clog << "Creating a platoon lc strat obj\n";
//}

void PlatoonLaneChangeStrategy::update_vehicle_lane_change_state(
	const PlatoonVehicle& platoon_vehicle,
	bool should_change_lane,
	std::unordered_map<long, LaneChangeState>& state_map) const
{
	implement_update_vehicle_lane_change_state(
		platoon_vehicle, should_change_lane, state_map);
}

bool PlatoonLaneChangeStrategy::can_vehicle_start_lane_change(
	const PlatoonVehicle& platoon_vehicle,
	const std::unordered_map<long, bool> vehicles_lane_change_gap_status) 
	const
{
	return implement_can_vehicle_start_lane_change(platoon_vehicle,
		vehicles_lane_change_gap_status);
}

bool PlatoonLaneChangeStrategy::can_vehicle_leave_platoon(long veh_id) const
{
	return implement_can_vehicle_leave_platoon(veh_id);
}

/* ------------------------------------------------------------------------ */
/* No Strategy ------------------------------------------------------------ */

void NoStrategy::implement_update_vehicle_lane_change_state(
	const PlatoonVehicle& platoon_vehicle, bool should_change_lane, 
	std::unordered_map<long, LaneChangeState>& state_map) const
{
	long veh_id = platoon_vehicle.get_id();
	if (platoon_vehicle.is_lane_changing())
	{
		state_map[veh_id] = LaneChangeState::lateral_maneuver;
	}
	else if (should_change_lane)
	{
		state_map[veh_id] = LaneChangeState::long_adjustment;
	}
	else
	{
		state_map[veh_id] = LaneChangeState::lane_keeping;
	}
}

bool NoStrategy::implement_can_vehicle_start_lane_change(
	const PlatoonVehicle& platoon_vehicle,
	const std::unordered_map<long, bool> vehicles_lane_change_gap_status) 
	const
{
	return vehicles_lane_change_gap_status.at(platoon_vehicle.get_id());
}

bool NoStrategy::implement_can_vehicle_leave_platoon(long veh_id) const
{
	return true;
}

/* ------------------------------------------------------------------------ */
/* Synchronous Strategy --------------------------------------------------- */

void SynchronousStrategy::implement_update_vehicle_lane_change_state(
	const PlatoonVehicle& platoon_vehicle, bool should_change_lane, 
	std::unordered_map<long, LaneChangeState>& state_map) const
{
	long veh_id = platoon_vehicle.get_id();
	if (should_change_lane)
	{
		state_map[veh_id] = LaneChangeState::long_adjustment;
	}
	else
	{
		state_map[veh_id] = LaneChangeState::lane_keeping;
	}
}

bool SynchronousStrategy::implement_can_vehicle_start_lane_change(
	const PlatoonVehicle& platoon_vehicle,
	const std::unordered_map<long, bool> vehicles_lane_change_gap_status)
	const
{
	for (auto const& entry : vehicles_lane_change_gap_status)
	{
		if (!entry.second) return false;
	}
	return true;
}

bool SynchronousStrategy::implement_can_vehicle_leave_platoon(
	long veh_id) const
{
	/* TODO: check if a vehicle inserted itself between a platoon
	vehicle and the vehicle ahead */
	return false;
}

/* ------------------------------------------------------------------------ */
/* Leader First Strategy -------------------------------------------------- */

void LeaderFirstStrategy::implement_update_vehicle_lane_change_state(
	const PlatoonVehicle& platoon_vehicle, bool should_change_lane, 
	std::unordered_map<long, LaneChangeState>& state_map) const
{
	long veh_id = platoon_vehicle.get_id();
	if (should_change_lane)
	{
		state_map[veh_id] = LaneChangeState::long_adjustment;
	}
	else
	{
		state_map[veh_id] = LaneChangeState::lane_keeping;
	}
}

bool LeaderFirstStrategy::implement_can_vehicle_start_lane_change(
	const PlatoonVehicle& platoon_vehicle,
	const std::unordered_map<long, bool> vehicles_lane_change_gap_status)
	const
{
	long veh_id = platoon_vehicle.get_id();
	if (platoon_vehicle.is_platoon_leader())
	{
		return vehicles_lane_change_gap_status.at(veh_id);
	}
	std::shared_ptr<NearbyVehicle> leader = platoon_vehicle.get_leader();
	bool leader_started_lane_change =
		(leader->get_relative_lane()
			== platoon_vehicle.get_desired_lane_change_direction())
		|| leader->is_lane_changing();
	return leader_started_lane_change
		&& vehicles_lane_change_gap_status.at(veh_id);
}

bool LeaderFirstStrategy::implement_can_vehicle_leave_platoon(
	long veh_id) const
{
	/* TODO: same as synchronous plus: 
	The vehicle must leave the platoon if it's leader
	has changed lanes, but the vehicle got stuck in the original
	lane. */
	return false;
}

/* ------------------------------------------------------------------------ */
/* Last Vehicle First Strategy -------------------------------------------- */

/* ------------------------------------------------------------------------ */
/* Leader First and Invert Strategy --------------------------------------- */