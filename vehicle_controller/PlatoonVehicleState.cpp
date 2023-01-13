/* ------------------------------------------------------------------------ */
/* Platoon Vehicle States ------------------------------------------------- */
/* ------------------------------------------------------------------------ */

#include "PlatoonVehicleState.h"
#include "PlatoonVehicle.h"

void PlatoonVehicleState::set_specific_type_of_vehicle(
	EgoVehicle* ego_vehicle)
{
	this->platoon_vehicle = dynamic_cast<PlatoonVehicle*>(ego_vehicle);
}