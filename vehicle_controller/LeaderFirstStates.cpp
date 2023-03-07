/* "Leader First" States -------------------------------------------------- */
#include "LeaderFirstStates.h"
#include "Platoon.h"
#include "PlatoonVehicle.h"

void LeaderFirstLaneKeepingState
::implement_handle_lane_keeping_intention() {}

void LeaderFirstLaneKeepingState
::implement_handle_lane_change_intention()
{
	/* [Jan 24, 2023] IMPORTANT NOTE:
	This is a heuristic solution. Without an optimization step, there's
	no way to know when the vehicles should start increasing the gap
	to their leaders. Is it once the leader is done increasing gap, or
	once it has started/finished its own lane change? */

	const VehicleState* leader_state = 
		platoon_vehicle->get_preceding_vehicle_state();
	if (leader_state == nullptr // this is the platoon leader
		|| *leader_state > LeaderFirstIncreasingGapState())
	{
		platoon_vehicle->update_time_headway_to_leader();
		platoon_vehicle->set_state(
			std::make_unique<LeaderFirstIncreasingGapState>());
	}
}

/* ------------------------------------------------------------------------ */

void LeaderFirstIncreasingGapState
::implement_handle_lane_keeping_intention()
{
	/* This case won't happen in our simulations.
	* For completion, this is coded assuming only the leader can
	abort the maneuver. */
	if (!platoon_vehicle->get_platoon()
		->get_platoon_leader()->has_lane_change_intention())
	{
		platoon_vehicle->reset_lane_change_waiting_time();
		platoon_vehicle->update_time_headway_to_leader();
		platoon_vehicle->reset_origin_lane_velocity_controller();
		platoon_vehicle->set_state(
			std::make_unique<LeaderFirstLaneKeepingState>());
	}
}

void LeaderFirstIncreasingGapState
::implement_handle_lane_change_intention()
{
	if (platoon_vehicle->has_finished_increasing_gap())
	{
		platoon_vehicle->set_state(
			std::make_unique<LeaderFirstLookingForSafeGapState>());
	}
}

/* ------------------------------------------------------------------------ */

void LeaderFirstLookingForSafeGapState
::implement_handle_lane_keeping_intention()
{
	/* This case won't happen in our simulations.
	* For completion, this is coded assuming only the leader can
	abort the maneuver. */
	if (!platoon_vehicle->get_platoon()
		->get_platoon_leader()->has_lane_change_intention())
	{
		platoon_vehicle->reset_lane_change_waiting_time();
		platoon_vehicle->update_time_headway_to_leader();
		platoon_vehicle->reset_origin_lane_velocity_controller();
		platoon_vehicle->set_state(
			std::make_unique<LeaderFirstLaneKeepingState>());
	}
}

void LeaderFirstLookingForSafeGapState
::implement_handle_lane_change_intention()
{
	const VehicleState* leader_state =
		platoon_vehicle->get_preceding_vehicle_state();
	long leader_id = platoon_vehicle->get_preceding_vehicle_id();
	/* We check if there's a vehicle longitudinally between us and 
	the preceding platoon vehicle */
	bool will_merge_behind_the_leader = 
		leader_id == platoon_vehicle->get_destination_lane_leader_id()
		|| leader_id == platoon_vehicle->get_leader_id();
	if ((platoon_vehicle->is_platoon_leader()
		|| /*(*leader_state >= LeaderFirstLaneChangingState()
			&&*/ will_merge_behind_the_leader)
		&& platoon_vehicle->check_lane_change_gaps())
	{
		platoon_vehicle->set_state(
			std::make_unique<LeaderFirstLaneChangingState>());
	}
}

/* ------------------------------------------------------------------------ */

void LeaderFirstLaneChangingState
::implement_handle_lane_keeping_intention()
{
	// Same as single vehicle case except for the next state
	if (!platoon_vehicle->is_lane_changing())
	{
		platoon_vehicle->set_lane_change_direction(RelativeLane::same);
		platoon_vehicle->reset_lane_change_waiting_time();
		platoon_vehicle->update_time_headway_to_leader();
		platoon_vehicle->reset_origin_lane_velocity_controller();
		platoon_vehicle->set_state(
			std::make_unique<LeaderFirstClosingGapState>());
	}
}

void LeaderFirstLaneChangingState
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

void LeaderFirstClosingGapState
::implement_handle_lane_keeping_intention()
{
	bool change_state = false;

	if (platoon_vehicle->is_platoon_leader())
	{
		if (has_platoon_changed_lanes(
			std::make_unique<LeaderFirstLaneChangingState>()))
		{
			double platoon_desired_vel =
				platoon_vehicle->get_platoon()->get_desired_velocity();
			if (are_other_platoon_gaps_closed(platoon_vehicle->get_id(),
				std::make_unique<LeaderFirstLaneKeepingState>()))
			{
				platoon_vehicle->set_desired_velocity(platoon_desired_vel);
				change_state = true;
			}
			else
			{
				platoon_vehicle->set_desired_velocity(
					waiting_velocity_fraction * platoon_desired_vel);
			}
		}
	}
	else if (platoon_vehicle->has_finished_closing_gap())
	{
			change_state = true;
	}

	if (change_state) 
	{
		platoon_vehicle->set_state(
			std::make_unique<LeaderFirstLaneKeepingState>());
	}
}

void LeaderFirstClosingGapState
::implement_handle_lane_change_intention() 
{
	unexpected_transition_message(this, true);
	implement_handle_lane_keeping_intention();
}