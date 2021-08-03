/*==========================================================================*/
/*  NearbyVehicle.h	    												    */
/*  Class to help manage neighboring vehicles								*/
/*                                                                          */
/*  Version of 2021-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#pragma once

#include <iostream>

#include "Vehicle.h"

class NearbyVehicle : public Vehicle{
public:
	using Vehicle::set_category;

	NearbyVehicle() = default;
	NearbyVehicle(long id, RelativeLane relative_lane, long relative_position);
	NearbyVehicle(long id, long relative_lane, long relative_position);

	/* Getters and setters */

	RelativeLane get_relative_lane() const { return relative_lane; };
	long get_relative_position() const { return relative_position; };
	double get_distance() const { return distance; };
	/* Relative velocity is: ego speed - other speed [m/s] */
	double get_relative_velocity() const { 
		return relative_velocity; 
	};
	double get_acceleration() const { return acceleration; };
	RelativeLane get_lane_change_direction() const { 
		return lane_change_direction; 
	};

	void set_distance(double distance) {
		this->distance = distance;
	};
	void set_relative_velocity(double relative_velocity) {
		this->relative_velocity = relative_velocity;
	};
	void set_acceleration(double acceleration) {
		this->acceleration = acceleration;
	};
	void set_lane_change_direction(long lane_change_direction) {
		this->lane_change_direction = RelativeLane(lane_change_direction);
	};

	/* Special getters and setters */

	double get_lambda_0() const { return lambda_0; };
	double get_lambda_1() const { return lambda_1; };
	/* set_category also sets the estimated maximum braking of the
	nearby vehicle and computes lambda_0 and lambda_1. */
	void set_category(VehicleCategory category) override;


	double compute_velocity(double ego_velocity) const;
	bool is_on_same_lane() const;
	bool is_ahead() const;
	/*void fill_with_dummy_values();
	void copy_current_states(NearbyVehicle& nearby_vehicle);*/
	void compute_safe_gap_parameters();
	friend std::ostream& operator<< (std::ostream& out, const NearbyVehicle& vehicle);

private:
	RelativeLane relative_lane{ RelativeLane::same };
	/* Relative position:
	positive = downstream (+1 next, +2 second next)
	negative = upstream (-1 next, -2 second next)
	It's possible to get more vehicles if DRIVER_DATA_WANTS_ALL_NVEHS
	in DriverModel.cpp  is set to 1 */
	long relative_position{ 0 };
	double distance{ 0.0 }; // front end to front end [m]
	double relative_velocity{ 0.0 }; // ego speed - other speed [m/s]
	double acceleration{ 0.0 }; // [m/s^2]
	RelativeLane lane_change_direction{ RelativeLane::same };
};
