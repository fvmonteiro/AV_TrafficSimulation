#include "EgoVehicle.h"
#include "Platoon.h"
#include "PlatoonVehicle.h"
#include "VehicleState.h"

/* Base Class ------------------------------------------------------------- */

VehicleState::VehicleState(std::string name) : name(name) {}

void VehicleState::set_ego_vehicle(EgoVehicle* ego_vehicle)
{
	this->ego_vehicle = ego_vehicle;
	set_specific_type_of_vehicle(ego_vehicle);
}

bool VehicleState::is_ego_vehicle_set() const
{
	return ego_vehicle != nullptr;
}

void VehicleState::handle_lane_keeping_intention()
{
	/* Temporaty check to avoid crashes during test phase */
	if (!is_ego_vehicle_set())
	{
		std::clog << "Handle lane keeping. No ego vehicle set\n";
		return;
	}
	implement_handle_lane_keeping_intention();
}

void VehicleState::handle_lane_change_intention()
{
	/* Temporaty check to avoid crashes during test phase */
	if (!is_ego_vehicle_set())
	{
		std::clog << "Handle lane keeping. No ego vehicle set\n";
		return;
	}
	implement_handle_lane_change_intention();
}

std::string VehicleState::unexpected_transition_message(
	VehicleState* vehicle_state)
{
	std::ostringstream oss;
	oss << "WARNING: unexpected state transition at t="
		<< ego_vehicle->get_time() << ", veh " << ego_vehicle->get_id()
		<< " from state: " << vehicle_state;
	return oss.str();
}

/* ------------------------------------------------------------------------ */
/* Single Vehicle States -------------------------------------------------- */
/* ------------------------------------------------------------------------ */

void SingleVehicleLaneKeepingState
::implement_handle_lane_keeping_intention() {}

void SingleVehicleLaneKeepingState
::implement_handle_lane_change_intention()
{
	ego_vehicle->update_origin_lane_controller();
	ego_vehicle->set_state(
		std::make_unique<SingleVehicleLongidutinalAdjustmentState>());
}

/* ------------------------------------------------------------------------ */

void SingleVehicleLongidutinalAdjustmentState
::implement_handle_lane_keeping_intention()
{
	ego_vehicle->reset_lane_change_waiting_time();
	ego_vehicle->update_origin_lane_controller();
	ego_vehicle->reset_origin_lane_velocity_controller();
	ego_vehicle->set_state(
		std::make_unique<SingleVehicleLaneKeepingState>());
}

void SingleVehicleLongidutinalAdjustmentState
::implement_handle_lane_change_intention() 
{
	if (ego_vehicle->check_lane_change_gaps())
	{
		ego_vehicle->set_state(
			std::make_unique<SingleVehicleLaneChangingState>());
	}
	else
	{
		ego_vehicle->update_lane_change_waiting_time();
	}
}

/* ------------------------------------------------------------------------ */

void SingleVehicleLaneChangingState
::implement_handle_lane_keeping_intention()
{
	if (!ego_vehicle->is_lane_changing())
	{
		ego_vehicle->set_lane_change_direction(RelativeLane::same);
		ego_vehicle->reset_lane_change_waiting_time();
		ego_vehicle->update_origin_lane_controller();
		ego_vehicle->reset_origin_lane_velocity_controller();
		ego_vehicle->set_state(
			std::make_unique<SingleVehicleLaneKeepingState>());
	}
}

void SingleVehicleLaneChangingState
::implement_handle_lane_change_intention() 
{
	if (ego_vehicle->is_lane_changing())
	{
		ego_vehicle->set_lane_change_direction(
			ego_vehicle->get_active_lane_change_direction());
	}
	else
	{
		ego_vehicle->set_lane_change_direction(
			ego_vehicle->get_desired_lane_change_direction());
	}
}

/* ------------------------------------------------------------------------ */
/* Platoon Vehicle States ------------------------------------------------- */
/* ------------------------------------------------------------------------ */

void PlatoonVehicleState::set_specific_type_of_vehicle(
	EgoVehicle* ego_vehicle)
{
	this->platoon_vehicle = dynamic_cast<PlatoonVehicle*>(ego_vehicle);
}

void SynchronousLaneKeepingState
::implement_handle_lane_keeping_intention() {}

void SynchronousLaneKeepingState
::implement_handle_lane_change_intention()
{
	if (platoon_vehicle->is_platoon_leader())
	{
		platoon_vehicle->update_origin_lane_controller();
		platoon_vehicle->set_state(
			std::make_unique<SynchronousLongidutinalAdjustmentState>());
	}
	else
	{
		auto& leader = platoon_vehicle->get_platoon()
			->get_preceding_vehicle(platoon_vehicle->get_id());
		bool leader_safe_on_orig_lane =
			leader->get_lane_change_gaps_safety().orig_lane_leader_gap;
		if (leader_safe_on_orig_lane)
		{
			platoon_vehicle->update_origin_lane_controller();
			platoon_vehicle->set_state(
				std::make_unique<SynchronousLongidutinalAdjustmentState>());
		}
	}
}

/* ------------------------------------------------------------------------ */

void SynchronousLongidutinalAdjustmentState
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

void SynchronousLongidutinalAdjustmentState
::implement_handle_lane_change_intention()
{
	/* Check if my gaps are safe and store the info */
	platoon_vehicle->check_lane_change_gaps();
	/* Check if everyone's gaps are safe */
	bool can_start_lane_change = true;
	for (auto& item : platoon_vehicle->get_platoon()->get_vehicles())
	{
		LaneChangeGapsSafety lcgs = 
			item.second->get_lane_change_gaps_safety();
		bool gap1_is_safe = lcgs.dest_lane_follower_gap;
		bool gap2_is_safe = lcgs.dest_lane_leader_gap;
		bool no_conflict = lcgs.no_conflict;
		if (item.second->is_platoon_leader() && !lcgs.is_lane_change_safe())
		{
			can_start_lane_change = false;
			break;
		}
		else if (!(gap1_is_safe && gap2_is_safe && no_conflict))
		{
			can_start_lane_change = false;
			break;
		}
	}

	if (can_start_lane_change)
	{
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
		platoon_vehicle->update_origin_lane_controller();
		platoon_vehicle->reset_origin_lane_velocity_controller();
		platoon_vehicle->set_state(
			std::make_unique<SynchronousLaneKeepingState>());
	}
}

void SynchronousLaneChangingState
::implement_handle_lane_change_intention()
{
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