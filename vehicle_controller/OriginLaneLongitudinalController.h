/*==========================================================================*/
/* OriginLaneLongitudinalController.h    									*/
/* Origin Lane (usual) Adaptive Cruise controller using the constant time   */
/* headway policy                                                           */
/*                                                                          */
/* Version of 2021-xx-xx                             Fernando V. Monteiro   */
/*==========================================================================*/

#pragma once
#include "LongitudinalController.h"
class OriginLaneLongitudinalController :
    public LongitudinalController
{

public:
    OriginLaneLongitudinalController();
    OriginLaneLongitudinalController(const Vehicle& ego_vehicle,
        bool verbose);
    OriginLaneLongitudinalController(const Vehicle& ego_vehicle);

    /* Determines and sets the current state of the longitudinal controller */
    virtual void determine_controller_state(double ego_velocity,
        const NearbyVehicle* leader) override;

};
