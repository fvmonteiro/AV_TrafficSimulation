/*==========================================================================*/
/* OriginLaneLongitudinalController.h    									*/
/* Origin Lane (usual) Adaptive Cruise controller using the constant time   */
/* headway policy                                                           */
/*                                                                          */
/* Version of 2021-xx-xx                             Fernando V. Monteiro   */
/*==========================================================================*/

#pragma once
#include "LongitudinalController.h"

/* Longitudinal controller to follow "real" leaders, that is, vehicles
on the same lane as the ego vehicle. */
class RealLongitudinalController :
    public LongitudinalController
{

public:
    RealLongitudinalController();
    RealLongitudinalController(const VehicleParameters& ego_parameters,
        VelocityControllerGains velocity_controller_gains,
        AutonomousGains autonomous_gains, ConnectedGains connected_gains,
        bool verbose);
    RealLongitudinalController(const VehicleParameters& ego_parameters,
        VelocityControllerGains velocity_controller_gains,
        AutonomousGains autonomous_gains, ConnectedGains connected_gains);
    /* Constructor that allows changing the vehicle following
    gains. Useful to differentiate the actual origin lane 
    controller and the end of lane controller */
    /*OriginLaneLongitudinalController(const EgoVehicle& ego_vehicle,
        double kg, double kv, bool verbose);*/

    /* Determines and sets the current state of the longitudinal controller */
    virtual void determine_controller_state(const EgoVehicle& ego_vehicle,
        const std::shared_ptr<NearbyVehicle> leader,
        double reference_velocity) override;

};
