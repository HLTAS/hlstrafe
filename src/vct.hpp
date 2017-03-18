#pragma once
#include "hlstrafe.hpp"

namespace HLStrafe
{

namespace VCT
{
	struct Entry {
		/// Difference between atan2(S, F) and anglemod(atan2(S, F)).
		double r;

		/// The corresponding forwardmove and sidemove.
		/// F and S satisfy:
		/// - F and S are coprime,
		/// - F >= 0, S >= 0,
		/// - F <= 2047, S <= 2047.
		int16_t F;
		int16_t S;

		friend inline bool operator<(const Entry& l, const Entry& r) {
			return l.r < r.r;
		}
	};
} // VCT

} // HLStrafe