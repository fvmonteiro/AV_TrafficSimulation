#pragma once

#include "VehicleController.h"

class ACCVehicle;

class ACCVehicleController : public VehicleController
{
public:
	ACCVehicleController(const ACCVehicle& acc_vehicle, bool verbose);

	/* Active ACC during lane keeping; human (vissim) control if there is
	lane change intention*/
	double get_desired_acceleration(const ACCVehicle& acc_vehicle);

private:
	// TODO not being used yet [Jan 24 2024]
	const ACCVehicle* acc_vehicle{ nullptr };
};

