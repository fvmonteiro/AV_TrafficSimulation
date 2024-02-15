/*==========================================================================*/
/*  ColorConstants.h													    */
/*						                              Fernando V. Monteiro  */
/*==========================================================================*/

#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
#include <string>
#include <vector>

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
const color_t GRAY = ARGB(255, 128, 128, 128);  // SAME COLOR AS THE ROAD

const color_t DARK_RED = ARGB(255, 128, 0, 0);
const color_t DARK_GREEN = ARGB(255, 0, 128, 0);
const color_t DARK_BLUE = ARGB(255, 0, 0, 128);
const color_t DARK_YELLOW = ARGB(255, 196, 196, 0);
const color_t DARK_MAGENTA = ARGB(255, 128, 0, 128); // PURPLE
const color_t DARK_CYAN = ARGB(255, 0, 128, 128);
const color_t DARK_GRAY = ARGB(255, 64, 64, 64);

const color_t LIGHT_BLUE = ARGB(255, 0, 196, 255);
const color_t LIGHT_GRAY = ARGB(255, 192, 192, 192);
const color_t BLUE_GREEN = ARGB(255, 0, 128, 128);

const double MAX_DISTANCE{ 300.0 }; // [m]
const double MAX_VELOCITY{ 130.0 / 3.6 }; /* [m/s] depends on which VISSIM
	desired speed distribution we use */
const double CAR_MAX_BRAKE{ 6.0 }; // absolute value [m/s^2]
const double TRUCK_MAX_BRAKE{ 5.5 }; // absolute value  [m/s^2]
const double ACTUATOR_CONSTANT{ 0.5 }; // [s].
const double COMFORTABLE_ACCELERATION{ 2.0 }; // [m/s^2]
const double COMFORTABLE_BRAKE{ 2.0 }; // absolute value [m/s^2]
const double CAR_MAX_JERK{ 50.0 }; // [m/s^3]
const double TRUCK_MAX_JERK{ 30.0 }; // [m/s^3]
const double CONNECTED_BRAKE_DELAY{ 0.1 }; // [s]
const double AUTONOMOUS_BRAKE_DELAY{ 0.2 }; // [s]
const double HUMAN_BRAKE_DELAY{ 0.75 }; // [s]
const double LANE_WIDTH{ 3.6 }; // [m]

const double PI{ M_PI };  // because I never remember how to find PI

/* Parameters used in discretionary platoon lane change scenarios */

const double SAFE_TIME_HEADWAY{ 2.0 }; // [s]
const double CONNECTED_SAFE_TIME_HEADWAY{ 1.0 }; // [s]
const long MAIN_LINK_NUMBER{ 3 };


const std::string STRATEGY_MAPS_FOLDER{ "C:\\Users\\fvall\\Documents"
	"\\Research\\data\\strategy_maps\\" };
const std::string QUANTIZATION_PARAMS_FOLDER{ "C:\\Users\\fvall\\Documents"
	"\\Research\\data\\quantization_parameters\\" };

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
	acc_car = 105,
	autonomous_car = 110,
	connected_car = 120,
	no_lane_change_connected_car = 121,
	traffic_light_alc_car = 130,
	traffic_light_calc_car = 135,
	platoon_car = 140,
	virdi_car = 150,
	truck = 200,
	bus = 300
};

template <typename T>
std::string vector_to_string(std::vector<T> v)
{
	std::string ret_str = "[";
	for (T i : v)
	{
		ret_str += std::to_string(i) + ", ";
	}
	if (ret_str.size() > 1) ret_str.erase(ret_str.size() - 2);
	ret_str += "], ";
	return ret_str;
};