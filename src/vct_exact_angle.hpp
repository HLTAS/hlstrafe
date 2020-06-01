#pragma once
#include "hlstrafe.hpp"
#include "util.hpp"

namespace HLStrafe
{

namespace VCTExactAngle
{
	struct Entry {
		/// The corresponding forwardmove and sidemove.
		/// F and S satisfy:
		/// - F >= -2047, S >= -2047,
		/// - F <= 2047, S <= 2047,
		/// - Length of the [F, S] vector is maximal under given constraints.
		int16_t F;
		int16_t S;

		/// Angle of the [F, S] vector.
		double angle;

		Entry(int16_t F, int16_t S)
			: F(F)
			, S(S)
			, angle(Atan2(-S, F)) {
		}

		friend inline bool operator<(const Entry& l, const Entry& r) {
			return l.angle < r.angle;
		}
	};

	/**
	 * Find and return the best VCT entry for the given target accel angle.
	 *
	 * @param vars Movement variables, only Maxspeed is used
	 * @param accel_angle Desired acceleration vector angle, relative to the player angle
	 * @return The optimal VCT entry.
	 */
	const Entry& GetBestVector(const MovementVars& vars, double accel_angle);
} // VCTExactAngle

} // HLStrafe
