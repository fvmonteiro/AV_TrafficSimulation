/* ------------------------------------------------------------------------ */
/* Platoon Vehicle States ------------------------------------------------- */
/* ------------------------------------------------------------------------ */

#include "PlatoonVehicleState.h"
#include "PlatoonVehicle.h"
#include "Platoon.h"

void PlatoonVehicleState::set_specific_type_of_vehicle(
	EgoVehicle* ego_vehicle)
{
	this->platoon_vehicle = dynamic_cast<PlatoonVehicle*>(ego_vehicle);
	if (!this->platoon_vehicle)
	{
		std::clog << "=== ERROR ===\nFailed to cast ego vehicle to "
			"platoon vehicle\n ===============";
	}
}

bool PlatoonVehicleState::are_other_platoon_gaps_closed(long veh_id,
	int lane_keeping_state_number)
{
	for (auto& item :
		platoon_vehicle->get_platoon()->get_vehicles_by_position())
	{
		auto& veh = item.second;
		if (veh->get_id() != veh_id
			&& veh->get_state()->get_state_number() 
			!= lane_keeping_state_number)
		{
			return false;
		}
	}
	return true;
}

bool PlatoonVehicleState::has_platoon_changed_lanes(
	std::unique_ptr<PlatoonVehicleState> lane_changing_state)
{
	for (auto& item 
		: platoon_vehicle->get_platoon()->get_vehicles_by_position())
	{
		if (item.second->has_lane_change_intention())
		{
			return false;
		}
	}
	return true;
}


/* ------------------------------------------------------------------------ */
/* Concrete States -------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

/* [Jan 24] Copying code from Python implementation. For now, let's keep a 
track of the 'original' code and why changes were made. */

void PlatoonVehicleLaneKeepingState
::implement_handle_lane_keeping_intention() {}

void PlatoonVehicleLaneKeepingState
::implement_handle_lane_change_intention()
{
	/* In Python, the preparation for long adjustments included requesting 
	cooperation and setting the lane change end time to inf. 
	The cooperation requests in this code are set at every simulation step
	in the method that updated nearby vehicles. And the lane change end
	times are computed after the simulation is done. */
	platoon_vehicle->prepare_to_start_long_adjustments();
	platoon_vehicle->set_state(
		std::make_unique<PlatoonVehicleLongAdjustmentState>());
}

/* ------------------------------------------------------------------------ */

void PlatoonVehicleLongAdjustmentState
::implement_handle_lane_keeping_intention()
{
	/* This should not happen in our scenarios */
	platoon_vehicle->prepare_to_restart_lane_keeping(false);
	unexpected_transition_message(this, false);
	platoon_vehicle->set_state(
		std::make_unique<PlatoonVehicleLaneKeepingState>());
}

void PlatoonVehicleLongAdjustmentState
::implement_handle_lane_change_intention()
{
	if (platoon_vehicle->can_start_lane_change())
	{
		/* In Python, the preparation to start a lane change included
		setting the start time and setting up a lane change controller.
		We may need to save the start time. */
		platoon_vehicle->share_platoon()->set_lane_change_start_time(
			platoon_vehicle->get_current_time());
		platoon_vehicle->set_state(
			std::make_unique<PlatoonVehicleLaneChangingState>());
	}
}

/* ------------------------------------------------------------------------ */

void PlatoonVehicleLaneChangingState
::implement_handle_lane_keeping_intention()
{
	// Same as single vehicle case except for the next state
	if (!platoon_vehicle->is_lane_changing())
	{
		platoon_vehicle->prepare_to_restart_lane_keeping(true);
		platoon_vehicle->set_state(
			std::make_unique<PlatoonVehicleLaneKeepingState>());
	}
}

void PlatoonVehicleLaneChangingState
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