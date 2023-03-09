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

bool PlatoonLaneChangeStrategy::can_vehicle_leave_platoon(
	const PlatoonVehicle& platoon_vehicle) const
{
	//std::clog << "\t[PlatoonStrategy] can vehicle leave platoon method\n";
	return implement_can_vehicle_leave_platoon(platoon_vehicle);
}

std::unique_ptr<VehicleState> PlatoonLaneChangeStrategy
::get_new_lane_keeping_state() const
{
	return implement_get_new_lane_keeping_state();
}

void PlatoonLaneChangeStrategy::reset_state_of_all_vehicles() const
{
	for (auto& item : platoon->get_vehicles_by_position())
	{
		item.second->reset_state(implement_get_new_lane_keeping_state());
	}
}

long PlatoonLaneChangeStrategy::create_platoon_lane_change_request(
	const PlatoonVehicle& platoon_vehicle) const
{
	return implement_create_platoon_lane_change_request(platoon_vehicle);
}

std::shared_ptr<NearbyVehicle> PlatoonLaneChangeStrategy
::define_virtual_leader(const PlatoonVehicle& platoon_vehicle) const
{
	return implement_define_virtual_leader(platoon_vehicle);
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

long NoStrategy::implement_create_platoon_lane_change_request(
	const PlatoonVehicle& platoon_vehicle) const
{
	return platoon_vehicle.get_destination_lane_follower_id();
}

std::shared_ptr<NearbyVehicle> NoStrategy
::implement_define_virtual_leader(const PlatoonVehicle& platoon_vehicle) const
{
	return platoon_vehicle.define_virtual_leader_when_alone();
}

/* ------------------------------------------------------------------------ */
/* Synchronous Strategy --------------------------------------------------- */

bool SynchronousStrategy::implement_can_vehicle_leave_platoon(
	const PlatoonVehicle& platoon_vehicle) const
{
	/* The preceding platoon vehicle must always be the real leader.
	Otherwise, the vehicle leaves the platoon */
	return !platoon_vehicle.is_platoon_leader()
		&& *platoon_vehicle.get_state() != SynchronousLaneChangingState()
		&& (platoon_vehicle.get_preceding_vehicle_id() 
			!= platoon_vehicle.get_leader_id());
}

std::unique_ptr<VehicleState> SynchronousStrategy
::implement_get_new_lane_keeping_state() const
{
	return std::make_unique<SynchronousLaneKeepingState>();
}

long SynchronousStrategy::implement_create_platoon_lane_change_request(
	const PlatoonVehicle& platoon_vehicle) const
{
	/* Heuristic choice */
	return platoon->is_stuck() ?
		platoon->get_destination_lane_vehicle_behind_last_vehicle()
		: platoon->get_destination_lane_vehicle_behind_the_leader();
	//if (platoon->is_stuck())
	//{
	//	return platoon->get_destination_lane_vehicle_behind_last_vehicle();
	//}
	//return platoon->get_destination_lane_vehicle_behind_the_leader();
}

std::shared_ptr<NearbyVehicle> SynchronousStrategy
::implement_define_virtual_leader(const PlatoonVehicle& platoon_vehicle) const
{
	return platoon_vehicle.is_platoon_leader() ?
		platoon_vehicle.define_virtual_leader_when_alone() : nullptr;
}

/* ------------------------------------------------------------------------ */
/* Leader First Strategy -------------------------------------------------- */

bool LeaderFirstStrategy::implement_can_vehicle_leave_platoon(
	const PlatoonVehicle& platoon_vehicle) const
{
	/* [Feb 9, 2023] Old ideas focused on mandatory LC with 
	* blocked lane */
	/* Leave the platoon if the supposed platoon preceding vehicle is 
	not any of the following:
	- the real (same lane) leader
	- the destination lane leader
	- ahead of the destination lane leader 
	*/
	/*long precending_platoon_veh_id =
		platoon_vehicle.get_preceding_vehicle_id();*/
	/*return !platoon_vehicle.is_platoon_leader()
		&& *platoon_vehicle.get_state() != LeaderFirstLaneChangingState()
		&& precending_platoon_veh_id != platoon_vehicle.get_leader_id()
		&& (precending_platoon_veh_id
			!= platoon_vehicle.get_dest_lane_leader_id())
		&& (precending_platoon_veh_id
			!= platoon_vehicle.get_destination_lane_leader_leader_id());*/

	if (platoon_vehicle.is_platoon_leader()) return false;
	const PlatoonVehicle* preceding_vehicle = 
		platoon_vehicle.get_preceding_vehicle_in_platoon();
	/* Vehicle between us and the preceding vehicle */
	if (preceding_vehicle->get_lane() == platoon_vehicle.get_lane()
		&& platoon_vehicle.has_leader()
		&& platoon_vehicle.get_leader_id() != preceding_vehicle->get_id())
	{
		return true;
	}
	return false;
}

std::unique_ptr<VehicleState> LeaderFirstStrategy
::implement_get_new_lane_keeping_state() const
{
	return std::make_unique<LeaderFirstLaneKeepingState>();
}

long LeaderFirstStrategy::implement_create_platoon_lane_change_request(
	const PlatoonVehicle& platoon_vehicle) const
{
	if (*platoon_vehicle.get_state() 
		< LeaderFirstLaneChangingState())
	{
		/* Heuristic choice */
		return platoon->is_stuck() ?
			platoon->get_destination_lane_vehicle_behind_last_vehicle()
			: platoon->get_destination_lane_vehicle_behind_the_leader();
		//return platoon->get_destination_lane_vehicle_behind_the_leader();
	}
	return 0;
}

std::shared_ptr<NearbyVehicle> LeaderFirstStrategy
::implement_define_virtual_leader(const PlatoonVehicle& platoon_vehicle) const
{
	if (platoon_vehicle.is_platoon_leader())
	{
		return platoon_vehicle.define_virtual_leader_when_alone();
	}
	if (!platoon_vehicle.has_leader()
		|| (platoon_vehicle.get_leader_id()
			!= platoon_vehicle.get_preceding_vehicle_id()))
	{
		return platoon_vehicle.get_nearby_vehicle_by_id(
			platoon_vehicle.get_preceding_vehicle_id());
	}
	else
	{
		return nullptr;
	}
	/*const VehicleState& veh_state = *platoon_vehicle.get_state();
	bool is_trying_to_change_lanes = 
		veh_state > LeaderFirstLaneKeepingState();
	bool is_done_lane_changing =
		veh_state > LeaderFirstLaneChangingState();
	bool has_leader_started_lane_change =
		*platoon_vehicle.get_preceding_vehicle_state()
		>= LeaderFirstLaneChangingState();
	if ((is_trying_to_change_lanes && !is_done_lane_changing)
		&& has_leader_started_lane_change)
	{
		return platoon_vehicle.get_nearby_vehicle_by_id(
			platoon_vehicle.get_preceding_vehicle_id());
	}
	else
	{
		return nullptr;
	}*/
}

/* ------------------------------------------------------------------------ */
/* Last Vehicle First Strategy -------------------------------------------- */

bool LastVehicleFirstStrategy::implement_can_vehicle_leave_platoon(
	const PlatoonVehicle& platoon_vehicle) const
{
	/* Leave the platoon if the supposed platoon preceding vehicle:
	- is out of sight
	- is not ahead of us (regardless of which lane) */

	long precending_platoon_veh_id =
		platoon_vehicle.get_preceding_vehicle_id();
	const auto& preceding_platoon_veh =
		platoon_vehicle.get_nearby_vehicle_by_id(precending_platoon_veh_id);
	return !platoon_vehicle.is_platoon_leader()
		&& *platoon_vehicle.get_state() != LastVehicleFirstLaneChangingState()
		&& precending_platoon_veh_id != platoon_vehicle.get_leader_id()
		&& (!platoon_vehicle.is_vehicle_in_sight(precending_platoon_veh_id)
			|| preceding_platoon_veh->get_relative_position() < 0);
}

std::unique_ptr<VehicleState> LastVehicleFirstStrategy
::implement_get_new_lane_keeping_state() const
{
	return std::make_unique<LastVehicleFirstLaneKeepingState>();
}

long LastVehicleFirstStrategy::implement_create_platoon_lane_change_request(
	const PlatoonVehicle& platoon_vehicle) const
{
	long desired_future_follower_id = 0;
	long veh_id = platoon_vehicle.get_id();
	if (platoon->get_last_veh_id() == veh_id)
	{
		desired_future_follower_id =
			platoon->get_last_vehicle()->get_destination_lane_follower_id();
	}
	else
	{
		const VehicleState& veh_state = 
			*platoon_vehicle.get_state();
		if (veh_state > LastVehicleFirstLaneKeepingState()
			&& veh_state <= LastVehicleFirstLaneChangingState())
		{
			desired_future_follower_id =
				platoon->get_following_vehicle_id(veh_id);
		}
	}
	return desired_future_follower_id;	
}

std::shared_ptr<NearbyVehicle> LastVehicleFirstStrategy
::implement_define_virtual_leader(const PlatoonVehicle& platoon_vehicle) const
{
	if (platoon_vehicle.is_last_platoon_vehicle())
	{
		return platoon_vehicle.define_virtual_leader_when_alone();
	}

	long virtual_leader_id = 0;
	bool is_done_lane_changing =
		*platoon_vehicle.get_state() > LastVehicleFirstLaneChangingState();
	bool follower_is_done_lane_changing = 
		*platoon_vehicle.get_following_vehicle_state()
		> LastVehicleFirstLaneChangingState();
	if (!is_done_lane_changing
		&& follower_is_done_lane_changing)
	{
		/* Once the follower has changed lanes, we try to merge between
		the follower and its real leader. */
		long follower_real_leader_id = 
			platoon_vehicle.get_following_vehicle_in_platoon()
			->get_leader_id();
		/* We must take into account the few moments where we are "half"
		in the destination lane */
		virtual_leader_id =
			follower_real_leader_id == platoon_vehicle.get_id() ?
			platoon_vehicle.get_destination_lane_leader_id()
			: follower_real_leader_id;
	}
	else if (platoon_vehicle.is_platoon_leader()
		&& (*platoon->get_last_vehicle()->get_state()
			> LastVehicleFirstLaneKeepingState()))
	{
		/* We want to prevent the platoon from "running away" */
		virtual_leader_id = 0;
			//platoon_vehicle.get_destination_lane_leader_id();
	}
	return platoon_vehicle.get_nearby_vehicle_by_id(virtual_leader_id);
}

/* ------------------------------------------------------------------------ */
/* Leader First and Invert Strategy --------------------------------------- */

bool LeaderFirstAndInvertStrategy::implement_can_vehicle_leave_platoon(
	const PlatoonVehicle& platoon_vehicle) const
{
	/* Leave the platoon if the supposed platoon preceding vehicle
	- is out of sight
	- can no longer see us */

	long precending_platoon_veh_id =
		platoon_vehicle.get_preceding_vehicle_id();
	const auto& preceding_platoon_veh =
		platoon_vehicle.get_preceding_vehicle_in_platoon();
	return !platoon_vehicle.is_platoon_leader()
		&& (*platoon_vehicle.get_state()
			!= LeaderFirstAndInvertLaneChangingState())
		&& (!platoon_vehicle.is_vehicle_in_sight(precending_platoon_veh_id)
			|| !preceding_platoon_veh->is_vehicle_in_sight(
				platoon_vehicle.get_id()));
}

std::unique_ptr<VehicleState> LeaderFirstAndInvertStrategy
::implement_get_new_lane_keeping_state() const
{
	return std::make_unique<LeaderFirstAndInvertLaneKeepingState>();
}

long LeaderFirstAndInvertStrategy
::implement_create_platoon_lane_change_request(
	const PlatoonVehicle& platoon_vehicle) const
{
	long desired_future_follower_id = 0;
	long veh_id = platoon_vehicle.get_id();
	if (platoon->get_leader_id() == veh_id)
	{
		desired_future_follower_id = 
			platoon->get_platoon_leader()->get_destination_lane_follower_id();
	}
	else if (*platoon_vehicle.get_state()
			== LeaderFirstAndInvertLookingForSafeGapState())
	{
		desired_future_follower_id = 
			platoon->get_preceding_vehicle_id(veh_id);
	}
	return desired_future_follower_id;
}

std::shared_ptr<NearbyVehicle> LeaderFirstAndInvertStrategy
::implement_define_virtual_leader(const PlatoonVehicle& platoon_vehicle) const
{
	if (platoon_vehicle.is_platoon_leader())
	{
		return platoon_vehicle.define_virtual_leader_when_alone();
	}

	/*const VehicleState& leader_state =
		*platoon_vehicle.get_preceding_vehicle_state();
	bool leader_is_done_lane_changing = leader_state
		> LeaderFirstAndInvertLaneChangingState();*/
	
	/* As soon as the platoon preceding vehicle is no longer
	our real leader, we start to overtaking it. The goal is to
	merge merge between the preceding vehicle and its real leader. */
	const auto& preceding_vehicle =
		*platoon_vehicle.get_preceding_vehicle_in_platoon();
	std::shared_ptr<NearbyVehicle> virtual_leader{ nullptr };
	if (!platoon_vehicle.has_leader()
		|| (platoon_vehicle.get_leader_id()
			!= preceding_vehicle.get_id()))
	{
		long preceding_veh_leader_id = preceding_vehicle.get_leader_id();
		if (preceding_veh_leader_id == platoon_vehicle.get_id())
		{
			// We already overtook the preceding vehicle
			virtual_leader = platoon_vehicle.get_nearby_vehicle_by_id(
				platoon_vehicle.get_destination_lane_leader_id());
		}
		else
		{
			virtual_leader = platoon_vehicle.get_nearby_vehicle_by_id(
				preceding_veh_leader_id);
			if (virtual_leader == nullptr)
			{
				/* We didn't find the preceding vehicle's leader in the
				platoon vehicle's nearby vehicle list*/
				virtual_leader =
					platoon_vehicle.create_nearby_vehicle_from_another(
						preceding_vehicle, preceding_veh_leader_id);
			}
		}
		//return virtual_leader;
	}
	return virtual_leader;
	/*else
	{
		return nullptr;
	}*/
}
