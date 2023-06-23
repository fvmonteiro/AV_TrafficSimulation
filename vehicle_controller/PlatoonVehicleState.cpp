/* ------------------------------------------------------------------------ */
/* Platoon Vehicle States ------------------------------------------------- */
/* ------------------------------------------------------------------------ */

#include "PlatoonVehicleState.h"
#include "PlatoonVehicle.h"
#include "Platoon.h"

void PlatoonVehicleState::set_specific_type_of_vehicle()
{
	this->platoon_vehicle = dynamic_cast<PlatoonVehicle*>(ego_vehicle);
		//dynamic_cast<PlatoonVehicle*>(ego_vehicle);
	if (platoon_vehicle == nullptr)
	{
		std::clog << "[PlatoonVehicleState] Incorrect dynamic cast.\n"
			<< "The ego vehicle passed as parameter is not a platoon vehicle" 
			<< std::endl;
	}
}

bool PlatoonVehicleState::are_other_platoon_gaps_closed(long veh_id,
	std::unique_ptr<PlatoonVehicleState> lane_keeping_state)
{
	for (auto& item :
		platoon_vehicle->get_platoon()->get_vehicles_by_position())
	{
		auto& veh = item.second;
		if (veh->get_id() != veh_id
			&& *veh->get_state() != *lane_keeping_state)
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