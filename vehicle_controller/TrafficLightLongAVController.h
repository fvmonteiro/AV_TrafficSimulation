#pragma once
#include "VehicleController.h"

class TrafficLightALCVehicle;

class TrafficLightLongAVController :  public VehicleController
{
public:
	TrafficLightLongAVController() = default;
	TrafficLightLongAVController(const TrafficLightALCVehicle& tl_av,
		bool verbose);

private:
	const TrafficLightALCVehicle* traffic_light_av{ nullptr };
	std::unordered_map<LongitudinalController::State, color_t>
		tl_alc_colors =
	{
		{ LongitudinalController::State::comf_accel, BLUE_GREEN },
		{ LongitudinalController::State::velocity_control, GREEN },
		{ LongitudinalController::State::vehicle_following, DARK_GREEN },
		{ LongitudinalController::State::traffic_light, YELLOW},
		{ LongitudinalController::State::too_close, RED },
	};

	double implement_compute_desired_acceleration() override;
	void add_traffic_lights_controller();
};

