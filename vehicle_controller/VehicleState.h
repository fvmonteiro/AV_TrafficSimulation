#pragma once

#include <sstream>
#include <string>

class EgoVehicle;

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
	const static int lane_keeping_state_number{ 1 };

	void set_ego_vehicle(EgoVehicle* ego_vehicle);
	std::string get_strategy_name() const { return strategy_name; };
	std::string get_state_name() const { return state_name; };
	int get_state_number() const { return state_number; };
	bool is_ego_vehicle_set() const;

	void handle_lane_keeping_intention();
	void handle_lane_change_intention();
	//void handle_lane_change_gaps_are_safe();

	friend std::ostream& operator<< (std::ostream& out,
		const VehicleState& vehicle_state)
	{
		out << vehicle_state.strategy_name 
			<< " " << vehicle_state.state_name;
		return out; // return std::ostream so we can chain calls to operator<<
	};

protected:
	EgoVehicle* ego_vehicle{ nullptr };

	VehicleState(std::string strategy_name, std::string state_name,
		int state_number);
	void unexpected_transition_message(VehicleState* vehicle_state,
		bool has_lane_change_intention);

private:
	std::string strategy_name;
	std::string state_name;
	/* Describes the state number withing the entire maneuver */
	int state_number{ 0 };

	/* Derived classes might need to use some concrete implementation of
	EgoVehicle. They can cast it in this function. */
	virtual void set_specific_type_of_vehicle(EgoVehicle* ego_vehicle) {};
	virtual void implement_handle_lane_keeping_intention() = 0;
	virtual void implement_handle_lane_change_intention() = 0;
};

bool have_same_strategy(const VehicleState& s1, const VehicleState& s2);
bool operator== (const VehicleState& s1, const VehicleState& s2);
bool operator!= (const VehicleState& s1, const VehicleState& s2);
bool operator> (const VehicleState& s1, const VehicleState& s2);
bool operator< (const VehicleState& s1, const VehicleState& s2);
bool operator>= (const VehicleState& s1, const VehicleState& s2);
bool operator<= (const VehicleState & s1, const VehicleState & s2);

/* ------------------------------------------------------------------------ */
/* Single Vehicle States -------------------------------------------------- */
/* ------------------------------------------------------------------------ */

class SingleVehicleLaneKeepingState : public VehicleState
{
public:
	SingleVehicleLaneKeepingState()
		: VehicleState("single veh.", "lane keeping", 1) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class SingleVehicleLongidutinalAdjustmentState : public VehicleState
{
public:
	SingleVehicleLongidutinalAdjustmentState()
		: VehicleState("single veh.", "intention to lc", 2) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class SingleVehicleLaneChangingState : public VehicleState
{
public:
	SingleVehicleLaneChangingState()
		: VehicleState("single veh.", "lane changing", 3) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;

	/* Necessary to prevent the vehicle from performing two 
	maneuvers consecutively (and without checking the gaps) */
	bool can_start_maneuver{ true };
};

