/* "Leader First" States -------------------------------------------------- */
#include "LeaderFirstStates.h"
#include "Platoon.h"
#include "PlatoonVehicle.h"

void LeaderFirstLaneKeepingState
::implement_handle_lane_keeping_intention() {}

void LeaderFirstLaneKeepingState
::implement_handle_lane_change_intention()
{
	if (platoon_vehicle->is_platoon_leader())
	{
		platoon_vehicle->update_origin_lane_controller();
		platoon_vehicle->set_state(
			std::make_unique<LeaderFirstLongidutinalAdjustmentState>());
	}
	else
	{
		/* Platoon vehicle only starts the longitudinal adjustment
		once its leader has finished the lane change */
		const VehicleState& leader_state = *(platoon_vehicle
			->get_preceding_vehicle_in_platoon()->get_state());
		if (leader_state > LeaderFirstLaneChangingState()
			/*|| leader_state == LeaderFirstLaneKeepingState()*/)
		{
			platoon_vehicle->update_origin_lane_controller();
			platoon_vehicle->set_state(
				std::make_unique<LeaderFirstLongidutinalAdjustmentState>());
		}
	}
}

/* ------------------------------------------------------------------------ */

void LeaderFirstLongidutinalAdjustmentState
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
			std::make_unique<LeaderFirstLaneKeepingState>());
	}
}

void LeaderFirstLongidutinalAdjustmentState
::implement_handle_lane_change_intention()
{
	if (platoon_vehicle->check_lane_change_gaps())
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
		platoon_vehicle->update_origin_lane_controller();
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
	if (platoon_vehicle->is_platoon_leader()
		&& has_platoon_changed_lanes(
			std::make_unique<LeaderFirstLaneChangingState>()))
	{
		double platoon_desired_vel =
			platoon_vehicle->get_platoon()->get_desired_velocity();
		double desired_vel = platoon_desired_vel;
		if (are_other_platoon_gaps_closed(platoon_vehicle->get_id(),
			std::make_unique<LeaderFirstLaneKeepingState>()))
		{
			platoon_vehicle->set_state(
				std::make_unique<LeaderFirstLaneKeepingState>());
		}
		else
		{
			desired_vel = waiting_velocity_fraction * platoon_desired_vel;
		}
		platoon_vehicle->set_desired_velocity(desired_vel);
	}
	else
	{
		double safe_time_headway = platoon_vehicle->get_safe_time_headway();
		bool has_time_headway_transition_ended =
			(std::abs(safe_time_headway
				- platoon_vehicle->get_current_desired_time_headway())
				/ safe_time_headway) < 0.1;
		bool is_gap_safe = platoon_vehicle->get_gap_error() 
			< gap_error_margin;
		if (has_time_headway_transition_ended && is_gap_safe)
		{
			platoon_vehicle->set_state(
				std::make_unique<LeaderFirstLaneKeepingState>());
		}
	}
}

void LeaderFirstClosingGapState
::implement_handle_lane_change_intention() 
{
	unexpected_transition_message(this, true);
	implement_handle_lane_keeping_intention();
}