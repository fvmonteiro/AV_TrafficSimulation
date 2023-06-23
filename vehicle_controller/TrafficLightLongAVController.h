#pragma once
#include "VehicleController.h"

class TrafficLightALCVehicle;

class TrafficLightLongAVController :  public VehicleController
{
public:
	TrafficLightLongAVController(const TrafficLightALCVehicle& tl_av,
		bool verbose);

private:
	const TrafficLightALCVehicle* traffic_light_av{ nullptr };

	double implement_compute_desired_acceleration() override;
};

