#pragma once

#include <sstream>
#include <string>

class EgoVehicle;
class Platoon;
class PlatoonVehicle;

//template<typename V>
//class VState
//{
//public:
//	void set_vehicle(V* vehicle) { this->vehicle = vehicle; };
//	bool is_vehicle_set() const { return vehicle != nullptr };
//
//	void handle_lane_keeping_intention()
//	{
//		implement_handle_lane_keeping_intention();
//	};
//	void handle_lane_change_intention()
//	{
//		implement_handle_lane_change_intention();
//	};
//	//void handle_lane_change_gaps_are_safe();
//
//	friend std::ostream& operator<< (std::ostream& out,
//		const VState& vehicle_state)
//	{
//		out << vehicle_state.name;
//		return out; // return std::ostream so we can chain calls to operator<<
//	};
//
//protected:
//	V* vehicle;
//	VState(std::string name) : name(name) {};
//
//private:
//	std::string name;
//
//	virtual void implement_handle_lane_keeping_intention() = 0;
//	virtual void implement_handle_lane_change_intention() = 0;
//};

/* Base Abstract Class ---------------------------------------------------- */

class VehicleState
{
public:
	void set_ego_vehicle(EgoVehicle* ego_vehicle);
	bool is_ego_vehicle_set() const;

	void handle_lane_keeping_intention();
	void handle_lane_change_intention();
	//void handle_lane_change_gaps_are_safe();

	friend std::ostream& operator<< (std::ostream& out,
		const VehicleState& vehicle_state)
	{
		out << vehicle_state.name;
		return out; // return std::ostream so we can chain calls to operator<<
	};

protected:
	EgoVehicle* ego_vehicle{ nullptr };

	VehicleState(std::string name);
	std::string unexpected_transition_message(VehicleState* vehicle_state);

private:
	std::string name;
	
	/* Derived classes might need to use some concrete implementation of
	EgoVehicle. They can cast it in this function. */
	virtual void set_specific_type_of_vehicle(EgoVehicle* ego_vehicle) {};
	virtual void implement_handle_lane_keeping_intention() = 0;
	virtual void implement_handle_lane_change_intention() = 0;
};

/* ------------------------------------------------------------------------ */
/* Single Vehicle States -------------------------------------------------- */
/* ------------------------------------------------------------------------ */

class SingleVehicleLaneKeepingState : public VehicleState
{
public:
	SingleVehicleLaneKeepingState()
		: VehicleState("single veh. lane keeping") {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class SingleVehicleLongidutinalAdjustmentState : public VehicleState
{
public:
	SingleVehicleLongidutinalAdjustmentState()
		: VehicleState("single veh. intention to lc") {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class SingleVehicleLaneChangingState : public VehicleState
{
public:
	SingleVehicleLaneChangingState()
		: VehicleState("single veh. lane changing") {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

/* ------------------------------------------------------------------------ */
/* Platoon Vehicle States ------------------------------------------------- */
/* ------------------------------------------------------------------------ */

class PlatoonVehicleState : public VehicleState
{
protected:
	PlatoonVehicle* platoon_vehicle{ nullptr };

	PlatoonVehicleState(std::string name) : VehicleState(name) {}

private:
	void set_specific_type_of_vehicle(EgoVehicle* ego_vehicle) override;
};

class SynchronousLaneKeepingState : public PlatoonVehicleState
{
public:
	SynchronousLaneKeepingState()
		: PlatoonVehicleState("synchronous lane keeping") {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class SynchronousLongidutinalAdjustmentState : public PlatoonVehicleState
{
public:
	SynchronousLongidutinalAdjustmentState()
		: PlatoonVehicleState("synchronous intention to lc") {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class SynchronousLaneChangingState : public PlatoonVehicleState
{
public:
	SynchronousLaneChangingState()
		: PlatoonVehicleState("synchronous lane changing") {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};
