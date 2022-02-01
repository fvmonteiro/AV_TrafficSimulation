/*==========================================================================*/
/*  ColorConstants.h													    */
/*  Version of 2021-06-08                             Fernando V. Monteiro  */
/*==========================================================================*/

#pragma once

typedef unsigned long color_t;
/* Based on the _WINGDI_ RGB definition and some tests on VISSIM */
//#define ARGB(a,r,g,b)   ((COLORREF)((((BYTE)(b) | ((WORD)((BYTE)(g)) << 8)) | (((DWORD)(BYTE)(r)) << 16))|((BYTE)(a) << 24)))
#define ARGB(a, r, g, b) ((color_t)((long)(b) + (long)(g)*256 + (long)(r)*256*256 + (unsigned long)(a)*256*256*256))

const color_t BLACK = ARGB(255, 0, 0, 0);
const color_t WHITE = ARGB(255, 255, 255, 255);
const color_t RED = ARGB(255, 255, 0, 0);
const color_t GREEN = ARGB(255, 0, 255, 0);
const color_t BLUE = ARGB(255, 0, 0, 255);
const color_t YELLOW = ARGB(255, 255, 255, 0);
const color_t MAGENTA = ARGB(255, 255, 0, 255);
const color_t CYAN = ARGB(255, 0, 255, 255);

const color_t DARK_RED = ARGB(255, 128, 0, 0);
const color_t DARK_GREEN = ARGB(255, 0, 128, 0);
const color_t DARK_BLUE = ARGB(255, 0, 0, 128);
const color_t DARK_YELLOW = ARGB(255, 196, 196, 0);
const color_t DARK_MAGENTA = ARGB(255, 128, 0, 128); // PURPLE
const color_t DARK_CYAN = ARGB(255, 0, 128, 128);

const color_t LIGHT_BLUE = ARGB(255, 0, 196, 255);
const color_t BLUE_GREEN = ARGB(255, 0, 128, 128);

const double MAX_DISTANCE = 300.0; // [m]
const double MAX_VELOCITY = 130.0; /* [m/s] depends on which VISSIM desired
								   speed distribution we use */
const double CAR_MAX_BRAKE{ 6.0 }; // [m/s^2]
const double TRUCK_MAX_BRAKE{ 5.5 }; // [m/s^2]
const double ACTUATOR_CONSTANT{ 0.5 }; // [s].
const double COMFORTABLE_ACCELERATION{ 2.0 }; // [m/s^2]
const double COMFORTABLE_BRAKE{ 2.5 }; // [m/s^2]
const double CAR_MAX_JERK{ 50.0 }; // [m/s^3]
const double TRUCK_MAX_JERK{ 30.0 }; // [m/s^3]
const double CONNECTED_BRAKE_DELAY{ 0.1 }; // [s]
const double AUTONOMOUS_BRAKE_DELAY{ 0.2 }; // [s]
const double HUMAN_BRAKE_DELAY{ 0.75 }; // [s]

//enum class RelativeLane {
//	right_right = -2, // second to the right
//	right, // next to the right
//	same,
//	left, // next to the left
//	left_left, // second to the left
//};

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
	ACC_car = 105,
	autonomous_car = 110,
	connected_car = 120,
	truck = 200,
	bus = 300
};