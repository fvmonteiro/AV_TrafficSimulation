#pragma once
#include "VehicleController.h"

class AutonomousVehicle;

class AVController : public VehicleController
{
public:
    AVController(const AutonomousVehicle& av, bool verbose);
};

