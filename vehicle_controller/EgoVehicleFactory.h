#pragma once

#include <memory>

#include "ACCVehicle.h"
#include "AutonomousVehicle.h"
#include "ConnectedAutonomousVehicle.h"
#include "NoLaneChangeCAV.h"
#include "PlatoonVehicle.h"
#include "TrafficLightALCVehicle.h"
#include "VirdiVehicle.h"
//#include "Platoon.h"

class EgoVehicleFactory
{
public:

	static std::shared_ptr<EgoVehicle> create_ego_vehicle(long id, int type, 
		double desired_velocity, double simulation_time_step,
		double creation_time, bool verbose)
	{
		std::shared_ptr<EgoVehicle> ego_vehicle;
		switch (VehicleType(type))
		{
		case VehicleType::acc_car:
			ego_vehicle = std::make_shared<ACCVehicle>(id, 
				desired_velocity, simulation_time_step, 
				creation_time, verbose);
			break;
		case VehicleType::autonomous_car:
			ego_vehicle = std::make_shared<AutonomousVehicle>(id,
				desired_velocity, simulation_time_step, 
				creation_time, verbose);
			break;
		case VehicleType::connected_car:
			ego_vehicle = std::make_shared<ConnectedAutonomousVehicle>(id,
				desired_velocity,
				simulation_time_step, creation_time, verbose);
			break;
		case VehicleType::no_lane_change_connected_car:
			ego_vehicle = std::make_shared<NoLaneChangeCAV>(id, 
				desired_velocity, simulation_time_step, 
				creation_time, verbose);
			break;
		case VehicleType::platoon_car:
			ego_vehicle = std::make_shared<PlatoonVehicle>(id, 
				desired_velocity, simulation_time_step, 
				creation_time, verbose);
			break;
		case VehicleType::traffic_light_alc_car:
			ego_vehicle = std::make_shared<TrafficLightALCVehicle>(id,
				desired_velocity, simulation_time_step, 
				creation_time, verbose);
			break;
		case VehicleType::traffic_light_calc_car:
			ego_vehicle = std::make_shared<TrafficLightCALCVehicle>(id,
				desired_velocity, simulation_time_step, 
				creation_time, verbose);
			break;
		case VehicleType::virdi_car:
			ego_vehicle = std::make_shared<VirdiVehicle>(id,
				desired_velocity, simulation_time_step,
				creation_time, verbose);
			break;
		default:
			std::clog << "Trying to create unknown vehicle type\n"
				<< "\ttime=" << creation_time
				<< "\tid=" << id
				<< "\ttype" << type
				<< std::endl;
			return nullptr;
		}

		ego_vehicle->create_controller();
		return ego_vehicle;
	}
};
