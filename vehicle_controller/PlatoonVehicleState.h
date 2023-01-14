/* ------------------------------------------------------------------------ */
/* Platoon Vehicle States ------------------------------------------------- */
/* ------------------------------------------------------------------------ */

#pragma once

#include "VehicleState.h"

class PlatoonVehicle;

class PlatoonVehicleState : public VehicleState
{
protected:
	PlatoonVehicle* platoon_vehicle{ nullptr };
	double gap_error_margin{ 1.0 };

	PlatoonVehicleState(std::string strategy_name,
		std::string state_name, int number)
		: VehicleState(strategy_name, state_name, number) {}
	bool are_platoon_gaps_closed(std::unique_ptr<PlatoonVehicleState>
		lane_keeping_state);
	bool has_platoon_changed_lanes(std::unique_ptr<PlatoonVehicleState>
		lane_changing_state);

private:
	void set_specific_type_of_vehicle(EgoVehicle* ego_vehicle) override;
};