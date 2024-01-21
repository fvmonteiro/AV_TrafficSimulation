#pragma once
#include "AVController.h"
class CAVController : public AVController
{
public:
	CAVController(const ConnectedAutonomousVehicle& cav, bool verbose);

protected:
	CAVController(bool verbose) : AVController(verbose) {};
};

