/* Leader First and Invert States ----------------------------------------- */

#include "LeaderFirstAndInvertStates.h"
#include "Platoon.h"
#include "PlatoonVehicle.h"

void LeaderFirstAndInvertLaneKeepingState
::implement_handle_lane_keeping_intention() {}

void LeaderFirstAndInvertLaneKeepingState
::implement_handle_lane_change_intention()
{
	bool change_state = false;

	/* The vehicle only starts the longitudinal adjustment
	once its leader has finished the lane change */
	const VehicleState* leader_state = (platoon_vehicle
		->get_preceding_vehicle_state());
	if (leader_state == nullptr // this is the platoon leader
		|| *leader_state > LeaderFirstAndInvertLaneChangingState())
	{
		platoon_vehicle->update_time_headway_to_leader();
		platoon_vehicle->set_state(
			std::make_unique<
			LeaderFirstAndInvertLookingForSafeGapState>());
	}
}

/* ------------------------------------------------------------------------ */

void LeaderFirstAndInvertLookingForSafeGapState
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
			std::make_unique<LeaderFirstAndInvertLaneKeepingState>());
	}
}

void LeaderFirstAndInvertLookingForSafeGapState
::implement_handle_lane_change_intention()
{
	long preceding_vehicle_id =
		platoon_vehicle->get_preceding_vehicle_id();
	long dest_lane_follower_id = 
		platoon_vehicle->get_destination_lane_follower_id();
	bool has_finished_overtaking = 
		preceding_vehicle_id == dest_lane_follower_id;

	if ((preceding_vehicle_id == 0 // this is the platoon leader
		|| has_finished_overtaking)
		&&
		platoon_vehicle->check_lane_change_gaps())
	{
		platoon_vehicle->set_state(
			std::make_unique<LeaderFirstAndInvertLaneChangingState>());
	}
}

/* ------------------------------------------------------------------------ */

void LeaderFirstAndInvertLaneChangingState
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
			std::make_unique<LeaderFirstAndInvertCreatingGapState>());
	}
}

void LeaderFirstAndInvertLaneChangingState
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

void LeaderFirstAndInvertCreatingGapState::
implement_handle_lane_keeping_intention()
{
	const VehicleState* following_vehicle_state
		= platoon_vehicle->get_following_vehicle_state();
	if (following_vehicle_state == nullptr // last platoon vehicle
		|| *following_vehicle_state > LeaderFirstAndInvertLaneChangingState())
	{
		platoon_vehicle->set_state(
			std::make_unique<LeaderFirstAndInvertClosingGapState>());
	}
}

/* Should never happen, but we include code to avoid getting stuck*/
void LeaderFirstAndInvertCreatingGapState::
implement_handle_lane_change_intention()
{
	unexpected_transition_message(this, true);
	implement_handle_lane_keeping_intention();
}

/* ------------------------------------------------------------------------ */

void LeaderFirstAndInvertClosingGapState::
implement_handle_lane_keeping_intention()
{
	bool change_state = false;

	/* Given the inversion of positions, the vehicle at the last index
	is now the vehicle ahead of the platoon */
	if (platoon_vehicle->is_last_platoon_vehicle())
	{
		double platoon_desired_vel =
			platoon_vehicle->get_platoon()->get_desired_velocity();
		if (are_other_platoon_gaps_closed(platoon_vehicle->get_id(),
			LeaderFirstAndInvertLaneKeepingState().get_state_number()))
		{
			platoon_vehicle->set_desired_velocity(platoon_desired_vel);
			platoon_vehicle->share_platoon()->sort_vehicles_by_distance_traveled();
			change_state = true;
		}
		else
		{
			platoon_vehicle->set_desired_velocity(
				waiting_velocity_fraction * platoon_desired_vel);
		}
	}
	else
	{
		bool has_finished_adjusting_time_headway =
			platoon_vehicle->has_finished_closing_gap();
		bool is_velocity_synchronized = !platoon_vehicle->has_leader()
			|| platoon_vehicle->get_leader()->get_relative_velocity() > -2.0;
		/* The "following" platoon vehicle is the one that just overtook us */
		bool is_following_platoon_vehicle_in_my_lane =
			*platoon_vehicle->get_following_vehicle_state()
			> LeaderFirstAndInvertLaneChangingState();
		if (has_finished_adjusting_time_headway
			&& is_velocity_synchronized
			&& is_following_platoon_vehicle_in_my_lane)
		{
			change_state = true;
		}
	}

	if (change_state)
	{
		platoon_vehicle->set_state(
			std::make_unique<LeaderFirstAndInvertLaneKeepingState>());
	}
}

void LeaderFirstAndInvertClosingGapState::
implement_handle_lane_change_intention()
{
	unexpected_transition_message(this, true);
	implement_handle_lane_keeping_intention();
}