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
}

bool PlatoonVehicleState::are_platoon_gaps_closed(
	std::unique_ptr<PlatoonVehicleState>
	lane_keeping_state)
{
	for (auto& item : platoon_vehicle->get_platoon()->get_vehicles())
	{
		auto& veh = item.second;
		if (!veh->is_platoon_leader()
			&& *veh->get_state() != *lane_keeping_state)
		{
			return false;
		}
	}
	return true;
}

bool PlatoonVehicleState::has_platoon_changed_lanes(
	std::unique_ptr<PlatoonVehicleState>
	lane_changing_state)
{
	for (auto& item : platoon_vehicle->get_platoon()->get_vehicles())
	{
		auto& veh = item.second;
		if (*veh->get_state() < *lane_changing_state)
		{
			return false;
		}
	}
	return true;
}