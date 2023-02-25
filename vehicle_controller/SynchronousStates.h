/* Synchronous States ----------------------------------------------------- */

#pragma once

#include "PlatoonVehicleState.h"

class SynchronousLaneKeepingState : public PlatoonVehicleState
{
public:
	SynchronousLaneKeepingState()
		: PlatoonVehicleState("synchronous", "lane keeping", 1) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class SynchronousIncreasingGapState : public PlatoonVehicleState
{
public:
	SynchronousIncreasingGapState()
		: PlatoonVehicleState("synchronous", "increasing gap", 2) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class SynchronousLookingForSafeGapState : public PlatoonVehicleState
{
public:
	SynchronousLookingForSafeGapState()
		: PlatoonVehicleState("synchronous", "looking for gap", 3) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class SynchronousLaneChangingState : public PlatoonVehicleState
{
public:
	SynchronousLaneChangingState()
		: PlatoonVehicleState("synchronous", "lane changing", 4) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

// Waits for followers before closing its own gap
class SynchronousWaitingOthersState : public PlatoonVehicleState
{
public:
	SynchronousWaitingOthersState()
		: PlatoonVehicleState("synchronous", "waiting others", 5) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class SynchronousClosingGapState : public PlatoonVehicleState
{
public:
	SynchronousClosingGapState()
		: PlatoonVehicleState("synchronous", "closing gap", 6) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};