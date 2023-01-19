#pragma once

#include <unordered_map>

#include "VehicleState.h"

class PlatoonVehicle;
class Platoon;

class PlatoonLaneChangeStrategy
{
public:

	std::string get_name() const { return name; };
	void set_platoon(Platoon* platoon) { this->platoon = platoon; };
	bool can_vehicle_leave_platoon(
		const PlatoonVehicle& platoon_vehicle) const;
	std::unique_ptr<VehicleState> get_new_lane_keeping_state() const;
	long get_assisted_vehicle_id(const PlatoonVehicle& platoon_vehicle) const;
	bool can_adjust_to_dest_lane_leader(
		const PlatoonVehicle& platoon_vehicle) const;
	void set_state_of_all_vehicles() const;
	long create_lane_change_request_for_vehicle(long veh_id) const;

	friend std::ostream& operator<< (std::ostream& out,
		const PlatoonLaneChangeStrategy& platoon_lc_strategy)
	{
		out << platoon_lc_strategy.name;
		return out; // return std::ostream so we can chain calls to operator<<
	};

protected:
	/* Check if we should make this a shared pointer */
	Platoon* platoon{ nullptr };

	PlatoonLaneChangeStrategy(std::string name) : name(name) {};
private:
	std::string name;

	virtual bool implement_can_vehicle_leave_platoon(
		const PlatoonVehicle& platoon_vehicle) const = 0;
	virtual std::unique_ptr<VehicleState> 
		implement_get_new_lane_keeping_state() const = 0;
	virtual long implement_get_assisted_vehicle_id(
		const PlatoonVehicle& platoon_vehicle) const = 0;
	virtual bool implement_can_adjust_to_dest_lane_leader(
		const PlatoonVehicle& platoon_vehicle) const = 0;
};

/* ------------------------------------------------------------------------ */

class NoStrategy : public PlatoonLaneChangeStrategy
{
public:
	NoStrategy() : PlatoonLaneChangeStrategy("no strategy") {};
private:
	bool implement_can_vehicle_leave_platoon(
		const PlatoonVehicle& platoon_vehicle) const override;
	std::unique_ptr<VehicleState> 
		implement_get_new_lane_keeping_state() const override;
	long implement_get_assisted_vehicle_id(
		const PlatoonVehicle& platoon_vehicle) const override {
		return 0;
	};
	bool implement_can_adjust_to_dest_lane_leader(
		const PlatoonVehicle& platoon_vehicle) const override {
		return true;
	};
};

/* ------------------------------------------------------------------------ */

class SynchronousStrategy: public PlatoonLaneChangeStrategy
{
public:
	SynchronousStrategy() : PlatoonLaneChangeStrategy("synchronous") {};
private:
	bool implement_can_vehicle_leave_platoon(
		const PlatoonVehicle& platoon_vehicle) const override;
	std::unique_ptr<VehicleState>
		implement_get_new_lane_keeping_state() const override;
	long implement_get_assisted_vehicle_id(
		const PlatoonVehicle& platoon_vehicle) const override {
		return 0;
	};
	bool implement_can_adjust_to_dest_lane_leader(
		const PlatoonVehicle& platoon_vehicle) const override;
};

/* ------------------------------------------------------------------------ */

class LeaderFirstStrategy: public PlatoonLaneChangeStrategy
{
public:
	LeaderFirstStrategy() : PlatoonLaneChangeStrategy("leader first") {};
private:
	bool implement_can_vehicle_leave_platoon(
		const PlatoonVehicle& platoon_vehicle) const override;
	std::unique_ptr<VehicleState>
		implement_get_new_lane_keeping_state() const override;
	long implement_get_assisted_vehicle_id(
		const PlatoonVehicle& platoon_vehicle) const override {
		return 0; 
	};
	bool implement_can_adjust_to_dest_lane_leader(
		const PlatoonVehicle& platoon_vehicle) const override;
};

/* ------------------------------------------------------------------------ */

class LastVehicleFirstStrategy : public PlatoonLaneChangeStrategy
{
public:
	LastVehicleFirstStrategy() 
		: PlatoonLaneChangeStrategy("last vehicle first") {};
private:
	bool implement_can_vehicle_leave_platoon(
		const PlatoonVehicle& platoon_vehicle) const override;
	std::unique_ptr<VehicleState>
		implement_get_new_lane_keeping_state() const override;
	long implement_get_assisted_vehicle_id(
			const PlatoonVehicle& platoon_vehicle) const override;
	bool implement_can_adjust_to_dest_lane_leader(
		const PlatoonVehicle& platoon_vehicle) const override {
		return true;
	};
};

/* ------------------------------------------------------------------------ */

class LeaderFirstAndInvertStrategy : public PlatoonLaneChangeStrategy
{
private:
public:
	LeaderFirstAndInvertStrategy()
		: PlatoonLaneChangeStrategy("leader first and invert") {};
private:
	bool implement_can_vehicle_leave_platoon(
		const PlatoonVehicle& platoon_vehicle) const override;
	std::unique_ptr<VehicleState>
		implement_get_new_lane_keeping_state() const override;
	long implement_get_assisted_vehicle_id(
		const PlatoonVehicle& platoon_vehicle) const override;
	bool implement_can_adjust_to_dest_lane_leader(
		const PlatoonVehicle& platoon_vehicle) const override;
};