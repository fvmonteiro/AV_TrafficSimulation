#pragma once
#include "LongitudinalController.h"

/* Just passes VISSIM's suggestion acceleration back to the simulation */
class VissimLongitudinalController :
    public LongitudinalController
{
public:
    VissimLongitudinalController() = default;
    VissimLongitudinalController(std::unordered_map<State, color_t>
        state_to_color_map);
private:
    double implement_get_gap_error() const override;
    double implement_compute_desired_acceleration(
        const EgoVehicle& ego_vehicle,
        const std::shared_ptr<NearbyVehicle> leader,
        double velocity_reference) override;
};