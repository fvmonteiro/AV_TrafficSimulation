#pragma once

#include "VehicleController.h"

class ACCVehicle;

class ACCVehicleController : public VehicleController
{
public:
	ACCVehicleController(const ACCVehicle& acc_vehicle, bool verbose);
};

