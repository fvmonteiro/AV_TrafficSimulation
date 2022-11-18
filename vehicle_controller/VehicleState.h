#pragma once

#include <sstream>
#include <string>

class EgoVehicle;

class VehicleState
{
public:
	//void set_ego_vehicle(EgoVehicle* ego_vehicle);

	void handle_lane_keeping_intention();
	void handle_lane_change_intention();
	//void handle_lane_change_gaps_are_safe();

	friend std::ostream& operator<< (std::ostream& out,
		const VehicleState& vehicle_state)
	{
		out << vehicle_state.name;
		return out; // return std::ostream so we can chain calls to operator<<
	}

protected:
	VehicleState(EgoVehicle* ego_vehicle, std::string name);
	std::string unexpected_transition_message(VehicleState* vehicle_state);

	EgoVehicle* ego_vehicle;
private:
	virtual void implement_handle_lane_keeping_intention() = 0;
	virtual void implement_handle_lane_change_intention() = 0;
	//virtual void implement_handle_lane_change_gaps_are_safe() = 0;

	std::string name;
};

class SingleVehicleLaneKeepingState : public VehicleState
{
public:
	SingleVehicleLaneKeepingState(EgoVehicle* ego_vehicle)
		: VehicleState(ego_vehicle, "single veh. lane keeping") {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
	//void implement_handle_lane_change_gaps_are_safe() override;
};

class SingleVehicleLongidutinalAdjustmentState : public VehicleState
{
public:
	SingleVehicleLongidutinalAdjustmentState(EgoVehicle* ego_vehicle)
		: VehicleState(ego_vehicle, "single veh. intention to lc") {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
	//void implement_handle_lane_change_gaps_are_safe() override;
};

class SingleVehicleLaneChangingState : public VehicleState
{
public:
	SingleVehicleLaneChangingState(EgoVehicle* ego_vehicle)
		: VehicleState(ego_vehicle, "single veh. lane changing") {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
	//void implement_handle_lane_change_gaps_are_safe() override;
};