#pragma once

#include <string>
#include <unordered_map>
#include <vector>
#include "StateVector.h"

/* TODO: merge with StateVector? */
class StateQuantizer
{
public:
	StateQuantizer() = default;
	StateQuantizer(std::unordered_map<std::string, double> parameters);

	QuantizedStateVector quantize_state(ContinuousStateVector state);
	std::vector<QuantizedStateVector> quantize_states(
		std::vector<ContinuousStateVector> system_state);

private:
	/* Quantization intervals */
	double dx{ 9.0 };
	double dy{ 4.0 };
	double dv{ 2.0 };

	/* Max and min values for variables that can 
	be outside initially tested ranges */
	double max_x_lo{ INFINITY };
	double min_x_lo{ -INFINITY };
	double max_v_lo{ INFINITY };
	double min_v_lo{ -INFINITY };
	double max_x_ld{ INFINITY };
	double min_x_ld{ -INFINITY };
	double max_v_ld{ INFINITY };
	double min_v_ld{ -INFINITY };

	QuantizedStateVector apply_bounds(QuantizedStateVector qs);
};

