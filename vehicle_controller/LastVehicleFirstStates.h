/* Last Vehicle First States ---------------------------------------------- */

#pragma once

#include "PlatoonVehicleState.h"

class LastVehicleFirstLaneKeepingState : public PlatoonVehicleState
{
public:
	LastVehicleFirstLaneKeepingState()
		: PlatoonVehicleState("last vehicle first", "lane keeping", 1) {}
private:
	double waiting_velocity_percent{ 0.8 };

	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class LastVehicleFirstLongidutinalAdjustmentState : public PlatoonVehicleState
{
public:
	LastVehicleFirstLongidutinalAdjustmentState()
		: PlatoonVehicleState("last vehicle first", "intention to lc", 2) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class LastVehicleFirstLaneChangingState : public PlatoonVehicleState
{
public:
	LastVehicleFirstLaneChangingState()
		: PlatoonVehicleState("last vehicle first", "lane changing", 3) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class LastVehicleFirstCreatingGapState : public PlatoonVehicleState
{
public:
	LastVehicleFirstCreatingGapState()
		: PlatoonVehicleState("last vehicle first", "creating gap", 4) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class LastVehicleFirstClosingGapState : public PlatoonVehicleState
{
public:
	LastVehicleFirstClosingGapState()
		: PlatoonVehicleState("last vehicle first", "closing gap", 5) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

