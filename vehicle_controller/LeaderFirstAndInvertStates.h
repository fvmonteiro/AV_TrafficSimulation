/* Leader First and Invert States ----------------------------------------- */

#pragma once

#include "PlatoonVehicleState.h"

class LeaderFirstAndInvertLaneKeepingState : public PlatoonVehicleState
{
public:
	LeaderFirstAndInvertLaneKeepingState()
		: PlatoonVehicleState("leader first and invert", 
			"lane keeping", 1) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class LeaderFirstAndInvertLongidutinalAdjustmentState 
	: public PlatoonVehicleState
{
public:
	LeaderFirstAndInvertLongidutinalAdjustmentState()
		: PlatoonVehicleState("leader first and invert", 
			"long adjustments", 2) {}
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