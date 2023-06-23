#pragma once
#include "AVController.h"

class ConnectedAutonomousVehicle;

class CAVController : public AVController
{
public:
	CAVController(const ConnectedAutonomousVehicle& cav, bool verbose);
};

