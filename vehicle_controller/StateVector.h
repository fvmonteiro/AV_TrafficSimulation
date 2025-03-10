#pragma once
#include <string>
#include <vector>

template <typename T>
class StateVector
{
public:
	StateVector() = default;
	StateVector(T x, T y, T theta, T vel)
		: internal_vector{ {x, y, theta, vel} }, is_empty(false) {};

	//static int get_size() { return size; };

	std::vector<T> get() const { return internal_vector; };
	T get_x() const { return internal_vector[x_idx]; };
	T get_y() const { return internal_vector[y_idx]; };
	T get_theta() const { return internal_vector[theta_idx]; };
	T get_vel() const { return internal_vector[vel_idx]; };
	bool get_is_empty() const { return is_empty; };

	void add_to_x(T value)
	{
		internal_vector[x_idx] += value;
	};
	void add_to_y(T value)
	{
		internal_vector[y_idx] += value;
	};
	void offset(T off_x, T off_y)
	{
		add_to_x(-off_x);
		add_to_y(-off_y);
	};
	std::string to_string() const
	{
		return vector_to_string(internal_vector);
	};

private:
	std::vector<T> internal_vector;
	static const int x_idx{ 0 };
	static const int y_idx{ 1 };
	static const int theta_idx{ 2 };
	static const int vel_idx{ 3 };
	//static const int size{ 4 };
	bool is_empty{ true };
};

using ContinuousStateVector = StateVector<double>;
using QuantizedStateVector = StateVector<int>;

/* Transforms a vector of vectors into a single long vector
	TODO: make static? or move outside class*/
template<typename T>
inline std::vector<T> flatten_state_matrix(
	std::vector<StateVector<T>>& state_matrix)
{
	std::vector<T> system_state_vector;
	//int n_vehs = static_cast<int>(state_matrix.size());
	//int n_states = state_matrix.front().get().size();
	//system_state_vector.reserve(n_states * n_vehs);
	for (StateVector<T>& v : state_matrix)
	{
		for (auto i : v.get()) system_state_vector.push_back(i);
	}
	return system_state_vector;
}
