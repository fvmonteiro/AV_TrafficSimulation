#pragma once

#include <memory>

#include "ACCVehicle.h"
#include "AutonomousVehicle.h"
#include "ConnectedAutonomousVehicle.h"
#include "PlatoonVehicle.h"
#include "TrafficLightACCVehicle.h"

class EgoVehicleFactory
{
public:

	static std::unique_ptr<EgoVehicle> create_ego_vehicle(long id, int type, 
		double desired_velocity, double simulation_time_step, 
		double creation_time, bool verbose)
	{
		switch (VehicleType(type))
		{
		case VehicleType::acc_car:
			return std::make_unique<ACCVehicle>(id, desired_velocity, 
				simulation_time_step, creation_time, verbose);
		case VehicleType::autonomous_car:
			return std::make_unique<AutonomousVehicle>(id, desired_velocity,
				simulation_time_step, creation_time, verbose);
		case VehicleType::connected_car:
			return std::make_unique<ConnectedAutonomousVehicle>(id, 
				desired_velocity,
				simulation_time_step, creation_time, verbose);
		case VehicleType::platoon_car:
			return std::make_unique<PlatoonVehicle>(id, desired_velocity,
				simulation_time_step, creation_time, verbose);
		case VehicleType::traffic_light_acc_car:
			return std::make_unique<TrafficLightACCVehicle>(id,
				desired_velocity,
				simulation_time_step, creation_time, verbose);
		case VehicleType::traffic_light_cacc_car:
			return std::make_unique<TrafficLightCACCVehicle>(id,
				desired_velocity,
				simulation_time_step, creation_time, verbose);
		default:
			std::clog << "Trying to create unknown vehicle type\n" 
				<< "\ttime=" << creation_time
				<< "\tid=" << id 
				<< "\ttype" << type
				<< std::endl;
			return nullptr;
		}
	}
};

