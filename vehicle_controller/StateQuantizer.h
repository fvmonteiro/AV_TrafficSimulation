#pragma once

#include <vector>
#include "StateVector.h"

/* TODO: merge with StateVector? */
class StateQuantizer
{
public:
	StateQuantizer() = default;

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
	double max_x{ INFINITY };
	double min_x{ -INFINITY };
	double max_v{ INFINITY };
	double min_v{ -INFINITY };
};

