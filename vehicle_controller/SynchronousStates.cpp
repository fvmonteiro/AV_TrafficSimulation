#include "SynchronousStates.h"
#include "Platoon.h"
#include "PlatoonVehicle.h"

void SynchronousLaneKeepingState
::implement_handle_lane_keeping_intention() {}

void SynchronousLaneKeepingState
::implement_handle_lane_change_intention()
{
	const VehicleState* leader_state = 
		platoon_vehicle->get_preceding_vehicle_state();
	if (leader_state == nullptr // platoon leader
		|| *leader_state > SynchronousIncreasingGapState())
	{
		platoon_vehicle->update_origin_lane_controller();
		platoon_vehicle->set_state(
			std::make_unique<SynchronousIncreasingGapState>());
	}
}

/* ------------------------------------------------------------------------ */

void SynchronousIncreasingGapState
::implement_handle_lane_keeping_intention()
{
	/* This case won't happen in our simulations.
	* For completion, this is coded assuming only the leader can
	abort the maneuver. */
	if (!platoon_vehicle->get_platoon()
		->get_platoon_leader()->has_lane_change_intention())
	{
		platoon_vehicle->reset_lane_change_waiting_time();
		platoon_vehicle->update_origin_lane_controller();
		platoon_vehicle->reset_origin_lane_velocity_controller();
		platoon_vehicle->set_state(
			std::make_unique<SynchronousLaneKeepingState>());
	}
}

void SynchronousIncreasingGapState
::implement_handle_lane_change_intention()
{
	if (platoon_vehicle->has_finished_increasing_gap())
	{
		platoon_vehicle->set_state(
			std::make_unique<SynchronousLookingForSafeGapState>());
	}
}

/* ------------------------------------------------------------------------ */

void SynchronousLookingForSafeGapState
::implement_handle_lane_keeping_intention()
{
	/* This case won't happen in our simulations.
	* For completion, this is coded assuming only the leader can
	abort the maneuver. */
	if (!platoon_vehicle->get_platoon()
		->get_platoon_leader()->has_lane_change_intention())
	{
		platoon_vehicle->reset_lane_change_waiting_time();
		platoon_vehicle->update_origin_lane_controller();
		platoon_vehicle->reset_origin_lane_velocity_controller();
		platoon_vehicle->set_state(
			std::make_unique<SynchronousLaneKeepingState>());
	}
}

void SynchronousLookingForSafeGapState
::implement_handle_lane_change_intention()
{
	/* Check if my gaps are safe and store the info */
	platoon_vehicle->check_lane_change_gaps();
	/* Check if everyone's gaps are safe */
	bool can_start_lane_change = true;
	for (auto& item 
		: platoon_vehicle->get_platoon()->get_vehicles_by_position())
	{
		LaneChangeGapsSafety& lcgs =
			item.second->get_lane_change_gaps_safety();
		if (!lcgs.is_lane_change_safe())
		{
			can_start_lane_change = false;
			break;
		}
		/* v1: Mid platoon vehicles don't need to distance themselves from
		their leaders*/
		//bool gap1_is_safe = lcgs.dest_lane_follower_gap;
		//bool gap2_is_safe = lcgs.dest_lane_leader_gap;
		//bool no_conflict = lcgs.no_conflict;
		//if (item.second->is_platoon_leader() && !lcgs.is_lane_change_safe())
		//{
		//	can_start_lane_change = false;
		//	break;
		//}
		//else if (!(gap1_is_safe && gap2_is_safe && no_conflict))
		//{
		//	can_start_lane_change = false;
		//	break;
		//}
	}

	if (can_start_lane_change)
	{
		if (platoon_vehicle->is_platoon_leader())
		{
			platoon_vehicle->get_platoon()->set_velocity_at_lane_change_start(
				platoon_vehicle->get_velocity()
			);
		}
		platoon_vehicle->set_state(
			std::make_unique<SynchronousLaneChangingState>());
	}
	else
	{
		platoon_vehicle->update_lane_change_waiting_time();
	}
	/* Note: this is not a perfectly synchronous maneuver. The last vehicle
	to check safety at the time step where everyone got safe gaps will
	start the maneuver one sampling time before others. */
}

/* ------------------------------------------------------------------------ */

void SynchronousLaneChangingState
::implement_handle_lane_keeping_intention()
{
	if (!platoon_vehicle->is_lane_changing())
	{
		platoon_vehicle->set_lane_change_direction(RelativeLane::same);
		platoon_vehicle->reset_lane_change_waiting_time();
		/*platoon_vehicle->update_origin_lane_controller();*/
		platoon_vehicle->reset_origin_lane_velocity_controller();
		platoon_vehicle->set_state(
			std::make_unique<SynchronousWaitingOthersState>());
	}
}

void SynchronousLaneChangingState
::implement_handle_lane_change_intention()
{
	// NOTE: this state-action pair is the same for all strategies
	if (platoon_vehicle->is_lane_changing())
	{
		platoon_vehicle->set_lane_change_direction(
			platoon_vehicle->get_active_lane_change_direction());
	}
	else
	{
		platoon_vehicle->set_lane_change_direction(
			platoon_vehicle->get_desired_lane_change_direction());
	}
}

/* ------------------------------------------------------------------------ */

void SynchronousWaitingOthersState
::implement_handle_lane_keeping_intention()
{
	bool change_state = false;
	auto follower = platoon_vehicle->get_platoon()
		->get_following_vehicle(platoon_vehicle->get_id());
	
	if (follower == nullptr // last platoon vehicle
		|| (*follower->get_state()) == SynchronousLaneKeepingState())
	{
		if (platoon_vehicle->is_platoon_leader())
		{
			platoon_vehicle->set_desired_velocity(
				platoon_vehicle->get_platoon()->get_desired_velocity());
		}
		platoon_vehicle->update_origin_lane_controller();
		change_state = true;
	}
	else if (platoon_vehicle->is_platoon_leader())
	{
		double desired_velocity = waiting_velocity_fraction
			* (platoon_vehicle->get_platoon()
				->get_desired_velocity());
		platoon_vehicle->set_desired_velocity(desired_velocity);
	}

	if (change_state)
	{
		platoon_vehicle->set_state(
			std::make_unique<SynchronousClosingGapState>());
	}
}

/* Platoon can only change lanes when all vehicles completed the
previous lane change */
void SynchronousWaitingOthersState
::implement_handle_lane_change_intention() 
{
	unexpected_transition_message(this, true);
	implement_handle_lane_keeping_intention();
}

/* ------------------------------------------------------------------------ */

void SynchronousClosingGapState
::implement_handle_lane_keeping_intention()
{
	if (platoon_vehicle->has_finished_closing_gap())
	{
		platoon_vehicle->set_state(
			std::make_unique<SynchronousLaneKeepingState>());
	}
}

/* Platoon can only change lanes when all vehicles completed the
previous lane change */
void SynchronousClosingGapState
::implement_handle_lane_change_intention() 
{
	unexpected_transition_message(this, true);
	implement_handle_lane_keeping_intention();
}