/*==========================================================================*/
/* OriginLaneLongitudinalController.h    									*/
/* Origin Lane (usual) Adaptive Cruise controller using the constant time   */
/* headway policy                                                           */
/*                                                                          */
/* Version of xxxx-xx-xx                             Fernando V. Monteiro   */
/*==========================================================================*/

#pragma once
#include "SwitchedLongitudinalController.h"

/* Longitudinal controller to follow "real" leaders, that is, vehicles
on the same lane as the ego vehicle. */
class RealLongitudinalController : public SwitchedLongitudinalController
{
public:

    RealLongitudinalController() = default;
    RealLongitudinalController(const EgoVehicle* ego_vehicle,
        std::unordered_map<State, color_t> state_to_color_map,
        bool verbose);

private:

    double get_max_accepted_brake() const override;
    /* Determines and sets the current state of the longitudinal controller */
    void determine_controller_state(const NearbyVehicle* leader,
        double reference_velocity) override;
    bool implement_is_velocity_reference_outdated() const override;
};
