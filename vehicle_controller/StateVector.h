#pragma once

struct StateVector
{
	double x{ 0.0 };  // [m]
	double y{ 0.0 };  // [m]
	double theta{ 0.0 };  // [rad]
	double v{ 0.0 };  // [m/s]
	bool is_empty{ true };

	StateVector() = default;
	StateVector(double x, double y, double theta, double v)
		: x{ x }, y{ y }, theta{ theta }, v{ v }, is_empty{ false } {};

	void offset(double off_x, double off_y)
	{
		x -= off_x;
		y -= off_y;
	};
};
