#pragma once
#include "VehicleState.h"

class AutonomousVehicle;

/* ------------------------------------------------------------------------ */
/* Base class ------------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

class AutonomousVehicleState : public VehicleState
{
protected:
	AutonomousVehicle* autonomous_vehicle{ nullptr };
	double gap_error_margin{ 1.0 };
	double waiting_velocity_fraction{ 0.8 };

	AutonomousVehicleState(std::string strategy_name,
		std::string state_name, int number)
		: VehicleState(strategy_name, state_name, number) {};

private:
	void set_specific_type_of_vehicle() override;
};

/* ------------------------------------------------------------------------ */
/* States for Lane Changing Vehicles -------------------------------------- */
/* ------------------------------------------------------------------------ */

class AutonomousVehicleLaneKeepingState : public AutonomousVehicleState
{
public:
	AutonomousVehicleLaneKeepingState()
		: AutonomousVehicleState("single veh.", "lane keeping", 1) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class AutonomousVehicleLongidutinalAdjustmentState : 
	public AutonomousVehicleState
{
public:
	AutonomousVehicleLongidutinalAdjustmentState()
		: AutonomousVehicleState("single veh.", "intention to lc", 2) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class AutonomousVehicleLaneChangingState : public AutonomousVehicleState
{
public:
	AutonomousVehicleLaneChangingState()
		: AutonomousVehicleState("single veh.", "lane changing", 3) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};
