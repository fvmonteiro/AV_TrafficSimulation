#include <algorithm>

#include "StateQuantizer.h"

//StateQuantizer::StateQuantizer(
//	std::unordered_map<std::string, double> parameters)
//{
//	dx = parameters["dx"];
//	dy = parameters["dy"];
//	dv = parameters["dv"];
//	max_x_lo = parameters["max_x_lo"];
//	min_x_lo = parameters["min_x_lo"];
//	max_v_lo = parameters["max_v_lo"];
//	min_v_lo = parameters["min_v_lo"];
//	max_x_ld = parameters["max_x_ld"];
//	min_x_ld = parameters["min_x_ld"];
//	max_v_ld = parameters["max_v_ld"];
//	min_v_ld = parameters["min_v_ld"];
//}

QuantizedStateVector StateQuantizer::quantize_state(
	ContinuousStateVector state)
{
	int qx = static_cast<int>(state.get_x() / dx);
	int qy = static_cast<int>(state.get_y() / dy);
	int qtheta = static_cast<int>(state.get_theta() / dtheta);
	int qv = static_cast<int>(state.get_vel() / dv);
	
	return QuantizedStateVector(qx, qy, qtheta, qv);
}

std::vector <QuantizedStateVector> StateQuantizer::quantize_states(
	std::vector<ContinuousStateVector> system_state)
{
	std::vector <QuantizedStateVector> quantized_state_matrix;
	for (int i = 0; i < system_state.size(); i++)
	{
		quantized_state_matrix.push_back(quantize_state(system_state[i]));
	}
	return quantized_state_matrix;
}

