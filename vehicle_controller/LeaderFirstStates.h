/* Leader First States ---------------------------------------------------- */

#pragma once

#include "PlatoonVehicleState.h"

class LeaderFirstLaneKeepingState : public PlatoonVehicleState
{
public:
	LeaderFirstLaneKeepingState()
		: PlatoonVehicleState("leader first", "lane keeping", 1) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class LeaderFirstLongidutinalAdjustmentState : public PlatoonVehicleState
{
public:
	LeaderFirstLongidutinalAdjustmentState()
		: PlatoonVehicleState("leader first", "long adjustments", 2) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class LeaderFirstLaneChangingState : public PlatoonVehicleState
{
public:
	LeaderFirstLaneChangingState()
		: PlatoonVehicleState("leader first", "lane changing", 3) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class LeaderFirstClosingGapState : public PlatoonVehicleState
{
public:
	LeaderFirstClosingGapState()
		: PlatoonVehicleState("leader first", "closing gap", 4) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};