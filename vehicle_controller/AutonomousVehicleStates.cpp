#include "AutonomousVehicleStates.h"
#include "AutonomousVehicle.h"

void AutonomousVehicleState::set_specific_type_of_vehicle()
{
	this->autonomous_vehicle = dynamic_cast<AutonomousVehicle*>(ego_vehicle);
	if (autonomous_vehicle == nullptr)
	{
		std::clog << "[AutonomousVehicleState] Incorrect dynamic cast.\n"
			<< "The ego vehicle passed as parameter is not an "
			<< "autonomous vehicle" << std::endl;
	}
}

/* ------------------------------------------------------------------------ */
/* States for Lane Changing Vehicles -------------------------------------- */
/* ------------------------------------------------------------------------ */

void AutonomousVehicleLaneKeepingState
::implement_handle_lane_keeping_intention() {}

void AutonomousVehicleLaneKeepingState
::implement_handle_lane_change_intention()
{
	autonomous_vehicle->update_origin_lane_controller();
	autonomous_vehicle->set_state(
		std::make_unique<AutonomousVehicleLongidutinalAdjustmentState>());
}

/* ------------------------------------------------------------------------ */

void AutonomousVehicleLongidutinalAdjustmentState
::implement_handle_lane_keeping_intention()
{
	autonomous_vehicle->reset_lane_change_waiting_time();
	autonomous_vehicle->update_origin_lane_controller();
	autonomous_vehicle->reset_origin_lane_velocity_controller();
	autonomous_vehicle->set_state(
		std::make_unique<AutonomousVehicleLaneKeepingState>());
}

void AutonomousVehicleLongidutinalAdjustmentState
::implement_handle_lane_change_intention()
{
	if (autonomous_vehicle->check_lane_change_gaps())
	{
		autonomous_vehicle->set_state(
			std::make_unique<AutonomousVehicleLaneChangingState>());
	}
	else
	{
		autonomous_vehicle->update_lane_change_waiting_time();
	}
}

/* ------------------------------------------------------------------------ */

void AutonomousVehicleLaneChangingState
::implement_handle_lane_keeping_intention()
{
	if (!autonomous_vehicle->is_lane_changing())
	{
		autonomous_vehicle->set_lane_change_direction(RelativeLane::same);
		autonomous_vehicle->reset_lane_change_waiting_time();
		autonomous_vehicle->update_origin_lane_controller();
		autonomous_vehicle->reset_origin_lane_velocity_controller();
		autonomous_vehicle->set_state(
			std::make_unique<AutonomousVehicleLaneKeepingState>());
	}
}

void AutonomousVehicleLaneChangingState
::implement_handle_lane_change_intention()
{
	if (autonomous_vehicle->is_lane_changing())
	{
		autonomous_vehicle->set_lane_change_direction(
			autonomous_vehicle->get_active_lane_change_direction());
	}
	else
	{
		autonomous_vehicle->set_lane_change_direction(
			autonomous_vehicle->get_desired_lane_change_direction());
	}
}