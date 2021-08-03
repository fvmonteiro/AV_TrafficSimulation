/*==========================================================================*/
/*  ColorConstants.h													    */
/*  Version of 2021-06-08                             Fernando V. Monteiro  */
/*==========================================================================*/

#pragma once

/* TODO: move all this code to a an (abstract) class vehicle that will 
be derived for EgoVehicle and NearbyVehicle */

const double MAX_DISTANCE = 300.0; // [m]
const double CAR_MAX_BRAKE{ 7.5 }; // [m/s^2]
const double TRUCK_MAX_BRAKE{ 5.5 }; // [m/s^2]
const double ACTUATOR_CONSTANT{ 0.5 }; // [s].
const double COMFORTABLE_ACCELERATION{ 2.0 }; // [m/s^2] TODO: vary with speed?
const double MAX_JERK{ 50.0 }; // [m/s^3]
const double CONNECTED_BRAKE_DELAY{ 0.1 }; // [s]
const double AUTONOMOUS_BRAKE_DELAY{ 0.2 }; // [s]

enum class RelativeLane {
	right_right = -2, // second to the right
	right, // next to the right
	same,
	left, // next to the left
	left_left, // second to the left
};

/* Categories set by VISSIM */
enum class VehicleCategory {
	undefined,
	car = 1,
	truck,
	bus,
	tram,
	pedestrian,
	bike,
};

/* User defined vehicle type. To be consistent with existing type,
follow these rules when defining a new type:
- Types should contain 3 digits
- The first digit of the type should match the vehicle category
of the type */
enum class VehicleType {
	undefined,
	human_driven_car = 100,
	autonomous_car = 110,
	truck = 200,
	bus = 300
};