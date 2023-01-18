#include "Platoon.h"
#include "PlatoonLaneChangeStrategy.h"
#include "PlatoonVehicle.h"
#include "PlatoonVehicleState.h"
#include "SynchronousStates.h"
#include "LeaderFirstStates.h"
#include "LeaderFirstAndInvertStates.h"
#include "LastVehicleFirstStates.h"
#include "NearbyVehicle.h"

/* BASE CLASS ------------------------------------------------------------- */

//PlatoonLaneChangeStrategy::PlatoonLaneChangeStrategy(Platoon* platoon)
//{
//	this->platoon = platoon;//std::make_shared<Platoon>(platoon);
//}

bool PlatoonLaneChangeStrategy::can_vehicle_leave_platoon(
	const PlatoonVehicle& platoon_vehicle) const
{
	return implement_can_vehicle_leave_platoon(platoon_vehicle);
}

std::unique_ptr<VehicleState> PlatoonLaneChangeStrategy
::get_new_lane_keeping_state() const
{
	return implement_get_new_lane_keeping_state();
}

long PlatoonLaneChangeStrategy::get_assisted_vehicle_id(
	const PlatoonVehicle& platoon_vehicle) const
{
	return implement_get_assisted_vehicle_id(platoon_vehicle);
}

bool PlatoonLaneChangeStrategy::can_adjust_to_dest_lane_leader(
	const PlatoonVehicle& platoon_vehicle) const
{
	return implement_can_adjust_to_dest_lane_leader(platoon_vehicle);
}

void PlatoonLaneChangeStrategy::set_state_of_all_vehicles() const
{
	for (auto& item : platoon->get_vehicles_by_position())
	{
		item.second->set_state(implement_get_new_lane_keeping_state());
	}
}

long PlatoonLaneChangeStrategy::create_lane_change_request_for_vehicle(
	long veh_id) const
{
	// TODO
	return 0;
}

/* ------------------------------------------------------------------------ */
/* No Strategy ------------------------------------------------------------ */

bool NoStrategy::implement_can_vehicle_leave_platoon(
	const PlatoonVehicle& platoon_vehicle) const
{
	return true;
}

std::unique_ptr<VehicleState> NoStrategy
::implement_get_new_lane_keeping_state() const
{
	return std::make_unique<SingleVehicleLaneKeepingState>();
}

/* ------------------------------------------------------------------------ */
/* Synchronous Strategy --------------------------------------------------- */

bool SynchronousStrategy::implement_can_vehicle_leave_platoon(
	const PlatoonVehicle& platoon_vehicle) const
{
	/* If something unexepected happens, the vehicle might be 
	stuck at the origin lane with no leader, while its platoon
	preceding vehicle has already changed lanes */
	if (platoon_vehicle.is_platoon_leader()
		|| !platoon_vehicle.has_leader()
		|| *platoon_vehicle.get_state() == SynchronousLaneChangingState())
	{
		return false;
	}
	std::shared_ptr<NearbyVehicle> leader = platoon_vehicle.get_leader();
	const PlatoonVehicle* precending_platoon_vehicle =
		platoon_vehicle.get_preceding_vehicle_in_platoon();
	return *leader != *precending_platoon_vehicle;
}

std::unique_ptr<VehicleState> SynchronousStrategy
::implement_get_new_lane_keeping_state() const
{
	return std::make_unique<SynchronousLaneKeepingState>();
}

bool SynchronousStrategy::implement_can_adjust_to_dest_lane_leader(
	const PlatoonVehicle& platoon_vehicle) const
{
	return platoon_vehicle.is_platoon_leader();
}

/* ------------------------------------------------------------------------ */
/* Leader First Strategy -------------------------------------------------- */

bool LeaderFirstStrategy::implement_can_vehicle_leave_platoon(
	const PlatoonVehicle& platoon_vehicle) const
{
	if (platoon_vehicle.is_platoon_leader()
		|| *platoon_vehicle.get_state() == LeaderFirstLaneChangingState())
	{
		return false;
	}

	long precending_platoon_vehicle_id =
		platoon_vehicle.get_preceding_vehicle_in_platoon()->get_id();
	std::shared_ptr<NearbyVehicle> preceding_platoon_veh =
		platoon_vehicle.get_nearby_vehicle_by_id(
			precending_platoon_vehicle_id
		);

	/* Leave the platoon if the supposed platoon preceding vehicle 
	is not immediately ahead of us (regardless of which lane) */
	return preceding_platoon_veh == nullptr ||
		preceding_platoon_veh->get_relative_position() != 1;
	//std::shared_ptr<PlatoonVehicle> precending_platoon_vehicle =
	//	platoon_vehicle.get_preceding_vehicle_in_platoon();
	//// Careful! leaders could be nullptr
	//std::shared_ptr<NearbyVehicle> orig_lane_leader = 
	//	platoon_vehicle.get_leader();
	//std::shared_ptr<NearbyVehicle> dest_lane_leader =
	//	platoon_vehicle.get_destination_lane_leader();

	//bool next_platoon_veh_is_orig_lane_leader =
	//	platoon_vehicle.has_leader()
	//	&& *orig_lane_leader == *precending_platoon_vehicle;
	//bool next_platoon_veh_is_dest_lane_leader =
	//	platoon_vehicle.has_destination_lane_leader()
	//	&& *dest_lane_leader == *precending_platoon_vehicle;
	//return !next_platoon_veh_is_orig_lane_leader
	//    && !next_platoon_veh_is_dest_lane_leader;
}

std::unique_ptr<VehicleState> LeaderFirstStrategy
::implement_get_new_lane_keeping_state() const
{
	return std::make_unique<LeaderFirstLaneKeepingState>();
}

bool LeaderFirstStrategy::implement_can_adjust_to_dest_lane_leader(
	const PlatoonVehicle& platoon_vehicle) const
{
	if (platoon_vehicle.is_platoon_leader()) return true;
	bool leader_is_done_lane_changing =
		*platoon_vehicle.get_preceding_vehicle_in_platoon()->get_state()
		>= LeaderFirstLaneChangingState();
	return leader_is_done_lane_changing;
}

/* ------------------------------------------------------------------------ */
/* Last Vehicle First Strategy -------------------------------------------- */

bool LastVehicleFirstStrategy::implement_can_vehicle_leave_platoon(
	const PlatoonVehicle& platoon_vehicle) const
{
	if (platoon_vehicle.is_platoon_leader()
		|| (*platoon_vehicle.get_state()
			== LastVehicleFirstLaneChangingState()))
	{
		return false;
	}

	long precending_platoon_vehicle_id =
		platoon_vehicle.get_preceding_vehicle_in_platoon()->get_id();
	std::shared_ptr<NearbyVehicle> preceding_platoon_veh =
		platoon_vehicle.get_nearby_vehicle_by_id(
			precending_platoon_vehicle_id
		);
	
	/* Leave the platoon if the supposed platoon preceding vehicle
	is not immediately ahead of or behind us (regardless of which lane) */
	return preceding_platoon_veh == nullptr ||
		std::abs(preceding_platoon_veh->get_relative_position()) > 1;
}

std::unique_ptr<VehicleState> LastVehicleFirstStrategy
::implement_get_new_lane_keeping_state() const
{
	return std::make_unique<LastVehicleFirstLaneKeepingState>();
}

long LastVehicleFirstStrategy::implement_get_assisted_vehicle_id(
	const PlatoonVehicle& platoon_vehicle) const
{
	if (!platoon_vehicle.is_platoon_leader() 
		&& *platoon_vehicle.get_state() == LastVehicleFirstCreatingGapState())
	{
		const PlatoonVehicle* leader =
			platoon_vehicle.get_preceding_vehicle_in_platoon();
		if (*leader->get_state()
			== LastVehicleFirstLongidutinalAdjustmentState())
		{
			return leader->get_id();
		}
	}
	return 0;
}

/* ------------------------------------------------------------------------ */
/* Leader First and Invert Strategy --------------------------------------- */

bool LeaderFirstAndInvertStrategy::implement_can_vehicle_leave_platoon(
	const PlatoonVehicle& platoon_vehicle) const
{
	if (platoon_vehicle.is_platoon_leader()
		|| (*platoon_vehicle.get_state()
			== LastVehicleFirstLaneChangingState()))
	{
		return false;
	}

	long precending_platoon_vehicle_id =
		platoon_vehicle.get_preceding_vehicle_in_platoon()->get_id();
	std::shared_ptr<NearbyVehicle> preceding_platoon_veh =
		platoon_vehicle.get_nearby_vehicle_by_id(
			precending_platoon_vehicle_id
		);
	/* Leave the platoon if the supposed platoon preceding vehicle
	is "out of sight" */
	return preceding_platoon_veh == nullptr;
}

std::unique_ptr<VehicleState> LeaderFirstAndInvertStrategy
::implement_get_new_lane_keeping_state() const
{
	return std::make_unique<LeaderFirstAndInvertLaneKeepingState>();
}

long LeaderFirstAndInvertStrategy::implement_get_assisted_vehicle_id(
	const PlatoonVehicle& platoon_vehicle) const
{
	if (!platoon_vehicle.is_last_platoon_vehicle()
		&& (*platoon_vehicle.get_state() 
			== LeaderFirstAndInvertCreatingGapState()))
	{
		const PlatoonVehicle* follower =
			platoon_vehicle.get_following_vehicle_in_platoon();
		if (*follower->get_state()
			== LeaderFirstAndInvertLongidutinalAdjustmentState())
		{
			return follower->get_id();
		}
	}
	return 0;
}

bool LeaderFirstAndInvertStrategy::implement_can_adjust_to_dest_lane_leader(
	const PlatoonVehicle& platoon_vehicle) const
{
	/* We only adjust to the destination lane leader if it is not 
	in the platoon */
	return platoon->get_vehicle_by_id(
		platoon_vehicle.get_dest_lane_leader_id()) == nullptr;
}