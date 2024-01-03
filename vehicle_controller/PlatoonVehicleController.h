#pragma once
#include "CAVController.h"

class PlatoonVehicle;

class PlatoonVehicleController : public CAVController
{
public:
    PlatoonVehicleController() = default;
    PlatoonVehicleController(const PlatoonVehicle& platoon_vehicle,
        bool verbose);

protected:
    bool get_destination_lane_desired_acceleration_when_in_platoon(
        std::unordered_map<ALCType, double>& possible_accelerations);

private:
    const PlatoonVehicle* platoon_vehicle{ nullptr };

    double implement_compute_desired_acceleration() override;
};

