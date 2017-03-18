#pragma once
#include "hlstrafe.hpp"

namespace HLStrafe
{

namespace VCT
{
	struct Entry {
		/// Difference between the angle given by the [F, S] vector and its anglemod.
		double r;

		/// The corresponding forwardmove and sidemove.
		/// F and S satisfy:
		/// - F >= 0, S >= 0,
		/// - F <= 2047, S <= 2047,
		/// - Length of the [F, S] vector is maximal under given constraints.
		int16_t F;
		int16_t S;

		friend inline bool operator<(const Entry& l, const Entry& r) {
			return l.r < r.r;
		}
	};
} // VCT

} // HLStrafe