/* ============================ DEPRACATED ================================ */

/* Leader First and Invert States ----------------------------------------- */

#pragma once

#include "PlatoonVehicleState.h"

class LeaderFirstAndInvertLaneKeepingState : public PlatoonVehicleState
{
public:
	const static int number{ 1 };
	LeaderFirstAndInvertLaneKeepingState()
		: PlatoonVehicleState("leader first and invert", 
			"lane keeping", 1) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class LeaderFirstAndInvertLookingForSafeGapState 
	: public PlatoonVehicleState
{
public:
	LeaderFirstAndInvertLookingForSafeGapState()
		: PlatoonVehicleState("leader first and invert", 
			"looking for gap", 2) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class LeaderFirstAndInvertLaneChangingState : public PlatoonVehicleState
{
public:
	LeaderFirstAndInvertLaneChangingState()
		: PlatoonVehicleState("leader first and invert", 
			"lane changing", 3) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class LeaderFirstAndInvertCreatingGapState : public PlatoonVehicleState
{
public:
	LeaderFirstAndInvertCreatingGapState()
		: PlatoonVehicleState("leader first and invert", 
			"creating gap", 4) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class LeaderFirstAndInvertClosingGapState : public PlatoonVehicleState
{
public:
	LeaderFirstAndInvertClosingGapState()
		: PlatoonVehicleState("leader first and invert", 
			"closing gap", 5) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};