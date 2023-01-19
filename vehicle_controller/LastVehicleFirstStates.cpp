#include "LastVehicleFirstStates.h"
#include "Platoon.h"
#include "PlatoonVehicle.h"

void LastVehicleFirstLaneKeepingState
::implement_handle_lane_keeping_intention() {}

void LastVehicleFirstLaneKeepingState
::implement_handle_lane_change_intention()
{
	bool change_state = false;
	if (platoon_vehicle->is_last_platoon_vehicle())
	{
		change_state = true;
	}
	else
	{
		/* Platoon vehicle only starts the longitudinal adjustment
		once the vehicle behind it has finished the lane change */
		const VehicleState* follower_state = platoon_vehicle
			->get_following_vehicle_in_platoon()->get_state();
		if (*follower_state > LastVehicleFirstLaneChangingState())
		{
			change_state = true;
		}
	}

	if (change_state)
	{
		platoon_vehicle->update_origin_lane_controller();
		platoon_vehicle->set_state(
			std::make_unique<
			LastVehicleFirstLongidutinalAdjustmentState>());
	}
}

/* ------------------------------------------------------------------------ */

void LastVehicleFirstLongidutinalAdjustmentState
::implement_handle_lane_keeping_intention()
{
	/* This case won't happen in our simulations.
	* For completion, this is coded assuming only the last vehicle can
	abort the maneuver. */
	if (!platoon_vehicle->get_platoon()
		->get_last_vehicle()->has_lane_change_intention())
	{
		platoon_vehicle->reset_lane_change_waiting_time();
		platoon_vehicle->update_origin_lane_controller();
		platoon_vehicle->reset_origin_lane_velocity_controller();
		platoon_vehicle->set_state(
			std::make_unique<LastVehicleFirstLaneKeepingState>());
	}
}

void LastVehicleFirstLongidutinalAdjustmentState
::implement_handle_lane_change_intention()
{
	if (platoon_vehicle->check_lane_change_gaps())
	{
		platoon_vehicle->set_state(
			std::make_unique<LastVehicleFirstLaneChangingState>());
	}
}

/* ------------------------------------------------------------------------ */

void LastVehicleFirstLaneChangingState
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
			std::make_unique<LastVehicleFirstCreatingGapState>());
	}
}

void LastVehicleFirstLaneChangingState
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

void LastVehicleFirstCreatingGapState::
implement_handle_lane_keeping_intention()
{
	const VehicleState* preceding_vehicle_state
		= platoon_vehicle->get_preceding_vehicle_state();
	if (preceding_vehicle_state == nullptr // platoon leader
		|| (*preceding_vehicle_state >= LastVehicleFirstLaneChangingState()))
	{
		platoon_vehicle->set_state(
			std::make_unique<LastVehicleFirstClosingGapState>());
	}
}

/* Should never happen, but we include code to avoid getting stuck*/
void LastVehicleFirstCreatingGapState::
implement_handle_lane_change_intention()
{
	unexpected_transition_message(this, true);
	implement_handle_lane_keeping_intention();
}

/* ------------------------------------------------------------------------ */

void LastVehicleFirstClosingGapState::
implement_handle_lane_keeping_intention()
{
	bool change_state = false;

	if (platoon_vehicle->is_platoon_leader())
	{
		double platoon_desired_vel =
			platoon_vehicle->get_platoon()->get_desired_velocity();
		if (are_other_platoon_gaps_closed(platoon_vehicle->get_id(),
			std::make_unique<LastVehicleFirstLaneKeepingState>()))
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
	else
	{
		double safe_time_headway = platoon_vehicle->get_safe_time_headway();
		bool has_time_headway_transition_ended =
			(std::abs(safe_time_headway
				- platoon_vehicle->get_current_desired_time_headway())
				/ safe_time_headway) < 0.1;
		bool is_gap_closed = platoon_vehicle->get_gap_error() 
			< gap_error_margin;
		bool is_preceding_platoon_vehicle_in_my_lane =
			*platoon_vehicle->get_preceding_vehicle_state()
			> LastVehicleFirstLaneChangingState();
		if (has_time_headway_transition_ended && is_gap_closed
			&& is_preceding_platoon_vehicle_in_my_lane)
		{
			change_state = true;
		}
	}

	if (change_state)
	{
		platoon_vehicle->set_state(
			std::make_unique<LastVehicleFirstLaneKeepingState>());
	}
}

void LastVehicleFirstClosingGapState::
implement_handle_lane_change_intention()
{
	unexpected_transition_message(this, true);
	implement_handle_lane_keeping_intention();
}
