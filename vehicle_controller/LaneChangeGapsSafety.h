#pragma once

#include <sstream>

struct LaneChangeGapsSafety
{
	bool orig_lane_leader_gap{ false };
	bool dest_lane_leader_gap{ false };
	bool dest_lane_follower_gap{ false };
	bool no_conflict{ false };

	bool is_lane_change_safe() const
	{
		return orig_lane_leader_gap && dest_lane_leader_gap
			&& dest_lane_follower_gap && no_conflict;
	}

	friend std::ostream& operator<< (
		std::ostream& out, const LaneChangeGapsSafety& lc_gaps_safety)
	{
		out << "[orig lane] gap ahead is safe? "
			<< lc_gaps_safety.orig_lane_leader_gap
			<< ", [dest lane] gap ahead is safe? "
			<< lc_gaps_safety.dest_lane_leader_gap
			<< ", [dest_lane] gap behind is safe? "
			<< lc_gaps_safety.dest_lane_follower_gap
			<< ", no conflict? " << lc_gaps_safety.no_conflict;
		return out;
	}
};

