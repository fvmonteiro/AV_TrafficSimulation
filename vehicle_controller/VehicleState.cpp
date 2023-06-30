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
	set_specific_type_of_vehicle();
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
		&& s1.get_state_number() == s2.get_state_number());
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
	return s1.get_state_number() > s2.get_state_number();
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

