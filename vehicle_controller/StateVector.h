#pragma once
#include <vector>

template <typename T>
class StateVector
{
public:
	StateVector() = default;
	StateVector(T x, T y, T theta, T vel)
		: vector{ {x, y, theta, vel} }, is_empty(false) {};

	std::vector<T> get() const { return vector; };
	T get_x() const { return vector[x_idx]; };
	T get_y() const { return vector[y_idx]; };
	T get_theta() const { return vector[theta_idx]; };
	T get_vel() const { return vector[vel_idx]; };
	bool get_is_empty() const { return is_empty; };

	void add_to_x(T value)
	{
		vector[x_idx] += value;
	};
	void add_to_y(T value)
	{
		vector[y_idx] += value;
	};

	void offset(T off_x, T off_y)
	{
		add_to_x(-off_x);
		add_to_y(-off_y);
	};

private:
	std::vector<T> vector;
	static const int x_idx{ 0 };
	static const int y_idx{ 1 };
	static const int theta_idx{ 2 };
	static const int vel_idx{ 3 };
	bool is_empty{ true };
};

using ContinuousStateVector = StateVector<double>;
using QuantizedStateVector = StateVector<int>;