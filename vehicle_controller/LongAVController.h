#pragma once
#include "VehicleController.h"

class LongitudinallyAutonomousVehicle;

/* Controller for Longitudinally Autonomous Vehicles */
class LongAVController : public VehicleController
{
public:
    LongAVController(const LongitudinallyAutonomousVehicle& longitudinal_av,
        bool verbose);

protected:
    /* Desired acceleration relative to the current lane.
    Returns true if the computed acceleration was added to the map */
    bool get_origin_lane_desired_acceleration(
        std::unordered_map<ALCType, double>& possible_accelerations);

    /* Desired acceleration to wait at the end of the lane while
    looking for an appropriate lane change gap. Without this,
    vehicles might miss a desired exit.
    Returns true if the computed acceleration was added to the map */
    bool get_end_of_lane_desired_acceleration(
        std::unordered_map<ALCType, double>& possible_accelerations);

private:
    const LongitudinallyAutonomousVehicle* longitudinal_av{ nullptr };

    double implement_compute_desired_acceleration() override;

    NearbyVehicle create_virtual_stopped_vehicle();
};

