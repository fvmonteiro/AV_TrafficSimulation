#include "EgoVehicle.h"
#include "VehicleState.h"

/* Base Class ------------------------------------------------------------- */

VehicleState::VehicleState(EgoVehicle* ego_vehicle, std::string name)
	: ego_vehicle(ego_vehicle), name(name) {}

void VehicleState::handle_lane_keeping_intention()
{
	implement_handle_lane_keeping_intention();
}

void VehicleState::handle_lane_change_intention()
{
	implement_handle_lane_change_intention();
}

//void VehicleState::handle_lane_change_gaps_are_safe()
//{
//	implement_handle_lane_change_gaps_are_safe();
//}

//void VehicleState::set_ego_vehicle(EgoVehicle* ego_vehicle) 
//{ 
//	this->ego_vehicle = ego_vehicle;
//}

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

void SingleVehicleLaneKeepingState
::implement_handle_lane_keeping_intention() {}

void SingleVehicleLaneKeepingState
::implement_handle_lane_change_intention()
{
	ego_vehicle->update_origin_lane_controller();
	ego_vehicle->set_state(
		std::make_unique<SingleVehicleLongidutinalAdjustmentState>(
			ego_vehicle));
}

//void SingleVehicleLaneKeepingState
//::implement_handle_lane_change_gaps_are_safe()
//{
//	std::clog << unexpected_transition_message(this) 
//		<< "to lane changing." << std::endl;
//	ego_vehicle->set_state(
//		std::make_unique<SingleVehicleLaneChangingState>(ego_vehicle));
//}

/* ------------------------------------------------------------------------ */

void SingleVehicleLongidutinalAdjustmentState
::implement_handle_lane_keeping_intention()
{
	ego_vehicle->reset_lane_change_waiting_time();
	ego_vehicle->update_origin_lane_controller();
	ego_vehicle->reset_origin_lane_velocity_controller();
	ego_vehicle->set_state(
		std::make_unique<SingleVehicleLaneKeepingState>(
			ego_vehicle));
}

void SingleVehicleLongidutinalAdjustmentState
::implement_handle_lane_change_intention() 
{
	if (ego_vehicle->can_start_lane_change())
	{
		ego_vehicle->set_state(
			std::make_unique<SingleVehicleLaneChangingState>(
				ego_vehicle));
	}
	else
	{
		ego_vehicle->update_lane_change_waiting_time();
	}
}

//void SingleVehicleLongidutinalAdjustmentState
//::implement_handle_lane_change_gaps_are_safe()
//{
//	ego_vehicle->reset_lane_change_waiting_time();
//	ego_vehicle->set_state(
//		std::make_unique<SingleVehicleLaneChangingState>(ego_vehicle));
//}

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
			std::make_unique<SingleVehicleLaneKeepingState>(ego_vehicle));
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

//void SingleVehicleLaneChangingState
//::implement_handle_lane_change_gaps_are_safe() {}