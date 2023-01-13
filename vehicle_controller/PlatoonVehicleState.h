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

private:
	void set_specific_type_of_vehicle(EgoVehicle* ego_vehicle) override;
};