#include "VehicleState.h"
#include "EgoVehicle.h"
#include "PlatoonVehicleState.h"

/* Base Class ------------------------------------------------------------- */

VehicleState::VehicleState(std::string strategy_name, std::string state_name,
	int state_number)
	: strategy_name(strategy_name), state_name(state_name), 
	state_number(state_number) {}

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
		std::clog << "Handle lane change intention. No ego vehicle set\n";
		return;
	}
	implement_handle_lane_change_intention();
}

void VehicleState::unexpected_transition_message(
	VehicleState* vehicle_state, bool has_lane_change_intention)
{
	//std::ostringstream oss;
	std::clog << "[WARNING] unexpected state transition\n\tt="
		<< ego_vehicle->get_time() << ", veh " << ego_vehicle->get_id()
		<< " at state " << vehicle_state << " and has "
		<< (has_lane_change_intention ? "lane change" : "lane keeping")
		<< " intention" << std::endl;
	//return oss.str();
}

bool have_same_strategy(const VehicleState& s1, const VehicleState& s2)
{
	return s1.get_strategy_name() == s2.get_strategy_name();
}

bool operator== (const VehicleState& s1, const VehicleState& s2)
{
	return (have_same_strategy(s1, s2)
		&& s1.get_phase_number() == s2.get_phase_number());
}

bool operator!= (const VehicleState& s1, const VehicleState& s2)
{
	return !(s1 == s2);
}

bool operator> (const VehicleState& s1, const VehicleState& s2)
{
	if (!have_same_strategy(s1, s2))
	{
		std::clog << "Comparing states of different strategies\n";
	}
	return s1.get_phase_number() > s2.get_phase_number();
}

bool operator< (const VehicleState& s1, const VehicleState& s2)
{
	return s2 > s1;
}

bool operator>= (const VehicleState& s1, const VehicleState& s2)
{
	return !(s1 < s2);
}

bool operator<= (const VehicleState& s1, const VehicleState& s2)
{
	return !(s1 > s2);
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
