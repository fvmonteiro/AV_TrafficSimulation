#include "Platoon.h"
#include "PlatoonLaneChangeStrategy.h"
#include "PlatoonVehicle.h"
#include "PlatoonVehicleState.h"
#include "SynchronousStates.h"
#include "LeaderFirstStates.h"
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

void PlatoonLaneChangeStrategy::set_state_of_all_vehicles()
{
	for (auto& item : platoon->get_vehicles())
	{
		item.second->set_state(implement_get_new_lane_keeping_state());
	}
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
	std::shared_ptr<PlatoonVehicle> precending_platoon_vehicle =
		platoon_vehicle.get_preceding_vehicle_in_platoon();
	return *leader != *precending_platoon_vehicle;
}

std::unique_ptr<VehicleState> SynchronousStrategy
::implement_get_new_lane_keeping_state() const
{
	return std::make_unique<SynchronousLaneKeepingState>();
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

	std::shared_ptr<PlatoonVehicle> precending_platoon_vehicle =
		platoon_vehicle.get_preceding_vehicle_in_platoon();
	// Careful! leaders could be nullptr
	std::shared_ptr<NearbyVehicle> orig_lane_leader = 
		platoon_vehicle.get_leader();
	std::shared_ptr<NearbyVehicle> dest_lane_leader =
		platoon_vehicle.get_destination_lane_leader();

	bool next_platoon_veh_is_orig_lane_leader =
		platoon_vehicle.has_leader()
		&& *orig_lane_leader == *precending_platoon_vehicle;
	bool next_platoon_veh_is_dest_lane_leader =
		platoon_vehicle.has_destination_lane_leader()
		&& *dest_lane_leader == *precending_platoon_vehicle;
	return !next_platoon_veh_is_orig_lane_leader
	    && !next_platoon_veh_is_dest_lane_leader;
}

std::unique_ptr<VehicleState> LeaderFirstStrategy
::implement_get_new_lane_keeping_state() const
{
	return std::make_unique<LeaderFirstLaneKeepingState>();
}

/* ------------------------------------------------------------------------ */
/* Last Vehicle First Strategy -------------------------------------------- */

bool LastVehicleFirstStrategy::implement_can_vehicle_leave_platoon(
	const PlatoonVehicle& platoon_vehicle) const
{
	// TODO
	return false;
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
		std::shared_ptr<PlatoonVehicle> leader = 
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