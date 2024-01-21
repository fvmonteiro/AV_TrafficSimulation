#pragma once
#include "VehicleController.h"

class AutonomousVehicle;

class AVController : public VehicleController
{
public:
	AVController(const AutonomousVehicle& autonomous_vehicle, bool verbose);

protected:
	AVController(bool verbose) : VehicleController(verbose) {};
};

