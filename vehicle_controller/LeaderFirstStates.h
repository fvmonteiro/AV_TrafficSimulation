/* ============================ DEPRACATED ================================ */

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

class LeaderFirstIncreasingGapState : public PlatoonVehicleState
{
public:
	LeaderFirstIncreasingGapState()
		: PlatoonVehicleState("leader first", "increasing gap", 2) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class LeaderFirstLookingForSafeGapState : public PlatoonVehicleState
{
public:
	LeaderFirstLookingForSafeGapState()
		: PlatoonVehicleState("leader first", "looking for gap", 3) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class LeaderFirstLaneChangingState : public PlatoonVehicleState
{
public:
	LeaderFirstLaneChangingState()
		: PlatoonVehicleState("leader first", "lane changing", 4) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class LeaderFirstClosingGapState : public PlatoonVehicleState
{
public:
	LeaderFirstClosingGapState()
		: PlatoonVehicleState("leader first", "closing gap", 5) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};