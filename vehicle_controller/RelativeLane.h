/* Class to deal with relative lanes in a more intuitive way.
Code inspired by the answer at:
https://stackoverflow.com/questions/21295935/can-a-c-enum-class-have-methods*/

#pragma once

#include <iostream>
#include <string>

class RelativeLane
{
public:
	enum Value
	{
		right_right = -2, // second to the right
		right, // next to the right
		same,
		left, // next to the left
		left_left, // second to the left
	};

	RelativeLane() = default;
	constexpr RelativeLane(Value relative_lane) : value(relative_lane) {}
	static RelativeLane from_long(long relative_lane) {
		return RelativeLane(Value(relative_lane)); 
	}
	//constexpr RelativeLane(long relative_lane): value(Value(relative_lane)) {}

	explicit operator bool() = delete; // Prevent usage: if(relative_lane)
	constexpr bool operator==(RelativeLane a) const
	{
		return this->value == a.value;
	}
	constexpr bool operator!=(RelativeLane a) const
	{
		return !(this->value == a.value);
	}

	int to_int() const
	{
		return static_cast<int>(this->value);
	}

	RelativeLane get_opposite() const
	{
		return RelativeLane(Value(-this->value));
	}

	bool on_same_side(const RelativeLane& other) const
	{
		if (this->value == same) return other == same;
		else return this->value * other.value > 0;
	}

	bool is_to_the_left() const
	{
		return on_same_side(left);
		/* less elegant, but maybe faster:
		return this->value > 0 */
	}

	bool is_to_the_right() const
	{
		return on_same_side(right);
		/* less elegant, but maybe faster:
		return this->value < 0 */
	}

	friend std::ostream& operator<< (std::ostream& out,
		const RelativeLane& relative_lane)
	{
		switch (relative_lane.value)
		{
		case Value::right_right:
			out << "right of right";
			break;
		case Value::right:
			out << "right";
			break;
		case Value::same:
			out << "same";
			break;
		case Value::left:
			out << "left";
			break;
		case Value::left_left:
			out << "left of left";
			break;
		default:
			out << "unknown relative lane";
			break;
		}
		return out;
	}

private:
	Value value;
};
