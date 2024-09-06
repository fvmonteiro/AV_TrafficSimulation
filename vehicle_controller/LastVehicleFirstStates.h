/* ============================ DEPRACATED ================================ */

/* Last Vehicle First States ---------------------------------------------- */

#pragma once

#include "PlatoonVehicleState.h"

class LastVehicleFirstLaneKeepingState : public PlatoonVehicleState
{
public:
	LastVehicleFirstLaneKeepingState()
		: PlatoonVehicleState("last vehicle first", "lane keeping", 1) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class LastVehicleFirstIncreasingGapState : public PlatoonVehicleState
{
public:
	LastVehicleFirstIncreasingGapState()
		: PlatoonVehicleState("last vehicle first", "increasing gap", 2) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class LastVehicleFirstLookingForSafeGapState : public PlatoonVehicleState
{
public:
	LastVehicleFirstLookingForSafeGapState()
		: PlatoonVehicleState("last vehicle first", "looking for gap", 3) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class LastVehicleFirstLaneChangingState : public PlatoonVehicleState
{
public:
	LastVehicleFirstLaneChangingState()
		: PlatoonVehicleState("last vehicle first", "lane changing", 4) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

/* In terms of functionality, we can probably do without this state,
but it is useful to keep track of the maneuver. */
class LastVehicleFirstCreatingGapState : public PlatoonVehicleState
{
public:
	LastVehicleFirstCreatingGapState()
		: PlatoonVehicleState("last vehicle first", "creating gap", 5) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

class LastVehicleFirstClosingGapState : public PlatoonVehicleState
{
public:
	LastVehicleFirstClosingGapState()
		: PlatoonVehicleState("last vehicle first", "closing gap", 6) {}
private:
	void implement_handle_lane_keeping_intention() override;
	void implement_handle_lane_change_intention() override;
};

