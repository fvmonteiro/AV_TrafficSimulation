#include <algorithm>

#include "StateQuantizer.h"

QuantizedStateVector StateQuantizer::quantize_state(
	ContinuousStateVector state)
{
	double x = std::max(std::min(state.get_x(), max_x), min_x);
	double vel = std::max(std::min(state.get_vel(), max_v), min_v);
	int qx = static_cast<int>(x / dx);
	int qy = static_cast<int>(state.get_y() / dy);
	int qtheta = 0;
	int qv = static_cast<int>(vel / dv);
	
	return QuantizedStateVector(qx, qy, qtheta, qv);
}

std::vector <QuantizedStateVector> StateQuantizer::quantize_states(
	std::vector<ContinuousStateVector> system_state)
{
	std::vector <QuantizedStateVector> full_quantized_state;
	for (const ContinuousStateVector& state : system_state)
	{
		full_quantized_state.push_back(quantize_state(state));
	}
	return full_quantized_state;
}