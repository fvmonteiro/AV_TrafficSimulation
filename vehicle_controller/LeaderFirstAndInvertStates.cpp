/* Leader First and Invert States ----------------------------------------- */

#include "LeaderFirstAndInvertStates.h"
#include "Platoon.h"
#include "PlatoonVehicle.h"

void LeaderFirstAndInvertLaneKeepingState
::implement_handle_lane_keeping_intention() {}

void LeaderFirstAndInvertLaneKeepingState
::implement_handle_lane_change_intention()
{
	if (platoon_vehicle->is_platoon_leader())
	{
		platoon_vehicle->update_origin_lane_controller();
		platoon_vehicle->set_state(std::make_unique<
			LeaderFirstAndInvertLongidutinalAdjustmentState>());
	}
	else
	{
		/* Platoon vehicle only starts the longitudinal adjustment
		once its leader has finished the lane change */
		const VehicleState& leader_state = *(platoon_vehicle
			->get_preceding_vehicle_in_platoon()->get_state());
		if (leader_state > LeaderFirstAndInvertLaneChangingState())
		{
			platoon_vehicle->update_origin_lane_controller();
			platoon_vehicle->set_state(
				std::make_unique<
				LeaderFirstAndInvertLongidutinalAdjustmentState>());
		}
	}
}

/* ------------------------------------------------------------------------ */

void LeaderFirstAndInvertLongidutinalAdjustmentState
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
			std::make_unique<LeaderFirstAndInvertLaneKeepingState>());
	}
}

void LeaderFirstAndInvertLongidutinalAdjustmentState
::implement_handle_lane_change_intention()
{
	std::shared_ptr<PlatoonVehicle> preceding_vehicle = 
		platoon_vehicle->get_preceding_vehicle_in_platoon();
	long dest_lane_follower_id = 
		platoon_vehicle->get_dest_lane_follower_id();
	if ((preceding_vehicle == nullptr
		|| preceding_vehicle->get_id() == dest_lane_follower_id)
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
		platoon_vehicle->update_origin_lane_controller();
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
	std::shared_ptr<PlatoonVehicle> following_vehicle
		= platoon_vehicle->get_following_vehicle_in_platoon();
	if (following_vehicle == nullptr) // last platoon vehicle
	{
		platoon_vehicle->set_state(
			std::make_unique<LeaderFirstAndInvertClosingGapState>());
	}
	else if (*following_vehicle->get_state()
			> LeaderFirstAndInvertLaneChangingState())
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
	/* Keep monitoring if vehicle is overtaken by another platoon vehicle?
	In this case, must not go back to lane keeping before all vehicles 
	have changed lanes */

	/* Given the inversion of positions, the vehicle at the last index
	is now the vehicle ahead of the platoon */
	if (platoon_vehicle->is_last_platoon_vehicle())
	{
		double platoon_desired_vel =
			platoon_vehicle->get_platoon()->get_desired_velocity();
		if (are_other_platoon_gaps_closed(platoon_vehicle->get_id(),
			std::make_unique<LeaderFirstAndInvertLaneKeepingState>()))
		{
			platoon_vehicle->set_desired_velocity(platoon_desired_vel);
			platoon_vehicle->get_platoon()->reorder_vehicles();
			platoon_vehicle->set_state(
				std::make_unique<LeaderFirstAndInvertLaneKeepingState>());
		}
		else
		{
			platoon_vehicle->set_desired_velocity(
				waiting_velocity_fraction * platoon_desired_vel);
		}
	}
	else
	{
		double safe_time_headway = platoon_vehicle->get_safe_time_headway();
		bool has_time_headway_transition_ended =
			(std::abs(safe_time_headway
				- platoon_vehicle->get_current_desired_time_headway())
				/ safe_time_headway) < 0.1;
		bool is_gap_closed = platoon_vehicle->get_gap_error()
			< gap_error_margin;
		bool is_velocity_synchronized = !platoon_vehicle->has_leader()
			|| platoon_vehicle->get_leader()->get_relative_velocity() > -2.0;
		/* The "following" platoon vehicle is the one that just overtook us */
		bool is_following_platoon_vehicle_in_my_lane =
			*platoon_vehicle->get_following_vehicle_in_platoon()->get_state()
			> LeaderFirstAndInvertLaneChangingState();
		if (has_time_headway_transition_ended && is_gap_closed
			&& is_velocity_synchronized
			&& is_following_platoon_vehicle_in_my_lane)
		{
			platoon_vehicle->set_state(
				std::make_unique<LeaderFirstAndInvertLaneKeepingState>());
		}
	}
}

void LeaderFirstAndInvertClosingGapState::
implement_handle_lane_change_intention()
{
	unexpected_transition_message(this, true);
	implement_handle_lane_keeping_intention();
}