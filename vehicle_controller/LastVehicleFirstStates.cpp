#include "LastVehicleFirstStates.h"
#include "Platoon.h"
#include "PlatoonVehicle.h"

void LastVehicleFirstLaneKeepingState
::implement_handle_lane_keeping_intention() {}

void LastVehicleFirstLaneKeepingState
::implement_handle_lane_change_intention()
{
	if (platoon_vehicle->is_last_platoon_vehicle())
	{
		platoon_vehicle->update_origin_lane_controller();
		platoon_vehicle->set_state(
			std::make_unique<LastVehicleFirstLongidutinalAdjustmentState>());
	}
	else
	{
		/* Platoon vehicle only starts the longitudinal adjustment
		once the vehicle behing it has finished the lane change */
		const VehicleState& leader_state = *(platoon_vehicle
			->get_preceding_vehicle_in_platoon()->get_state());
		if (leader_state == LastVehicleFirstClosingGapState()
			|| leader_state == LastVehicleFirstLaneKeepingState())
		{
			platoon_vehicle->update_origin_lane_controller();
			platoon_vehicle->set_state(
				std::make_unique<LastVehicleFirstLongidutinalAdjustmentState>());
		}
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
{}

void LastVehicleFirstCreatingGapState::
implement_handle_lane_change_intention()
{}

/* ------------------------------------------------------------------------ */

void LastVehicleFirstClosingGapState::
implement_handle_lane_keeping_intention()
{}

void LastVehicleFirstClosingGapState::
implement_handle_lane_change_intention()
{}