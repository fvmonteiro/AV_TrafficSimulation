/* ------------------------------------------------------------------------ */
/* Platoon Vehicle States ------------------------------------------------- */
/* ------------------------------------------------------------------------ */

#pragma once

#include "VehicleState.h"

class PlatoonVehicle;

class PlatoonVehicleState : public VehicleState
{

protected:
	const std::string name{ "platoon vehicle" };
	PlatoonVehicle* platoon_vehicle{ nullptr };
	double gap_error_margin{ 1.0 };
	/* Once leader has finished changing lanes, it can travel slower 
	to wait for other platoon vehicles to catch up. Value 1.0 means
	the leader does not wait. */
	double waiting_velocity_fraction{ 1.0 };

	PlatoonVehicleState(std::string strategy_name,
		std::string state_name, int number)
		: VehicleState(strategy_name, state_name, number) {}
	/* Checks if all platoon vehicles, except the one identified in the 
	parameter, are in the lane keeping state. */
	bool are_other_platoon_gaps_closed(long veh_id, 
		int lane_keeping_state_number);
	bool has_platoon_changed_lanes(std::unique_ptr<PlatoonVehicleState>
		lane_changing_state);

private:
	void set_specific_type_of_vehicle(EgoVehicle* ego_vehicle) override;
};


/* ------------------------------------------------------------------------ */
/* Concrete States -------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

class PlatoonVehicleLaneKeepingState : public PlatoonVehicleState
{
public:
	PlatoonVehicleLaneKeepingState()
		: PlatoonVehicleState(name, "lane keeping", 1) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class PlatoonVehicleLongAdjustmentState : public PlatoonVehicleState
{
public:
	PlatoonVehicleLongAdjustmentState()
		: PlatoonVehicleState(name, "long adjustment", 2) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class PlatoonVehicleLaneChangingState : public PlatoonVehicleState
{
public:
	PlatoonVehicleLaneChangingState()
		: PlatoonVehicleState(name, "lane changing", 3) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};