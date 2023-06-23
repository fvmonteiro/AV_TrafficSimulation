#pragma once
#include "VehicleController.h"

class LongitudinallyAutonomousVehicle;

/* Controller for Longitudinally Autonomous Vehicles */
class LongAVController : public VehicleController
{
public:
    LongAVController(const LongitudinallyAutonomousVehicle& longAV,
        bool verbose);
};

