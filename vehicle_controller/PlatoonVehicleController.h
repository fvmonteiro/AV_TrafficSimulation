#pragma once
#include "CAVController.h"
class PlatoonVehicleController : public CAVController
{
public:
	PlatoonVehicleController::PlatoonVehicleController(
		const PlatoonVehicle& platoon_vehicle, bool verbose);

protected:
	PlatoonVehicleController::PlatoonVehicleController(bool verbose)
		: CAVController(verbose) {};
};

