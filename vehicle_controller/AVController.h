#pragma once
#include "LongAVController.h"

class AutonomousVehicle;

class AVController : public LongAVController
{
public:
    AVController(const AutonomousVehicle& av, bool verbose);

protected:
	/* Desired acceleration to adjust to destination lane leader.
	Returns true if the computed acceleration was added to the map */
	bool get_destination_lane_desired_acceleration(
		std::unordered_map<ALCType, double>& possible_accelerations);

private:
    const AutonomousVehicle* autonomous_vehicle{ nullptr };

    double implement_compute_desired_acceleration() override;
};

