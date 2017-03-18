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

	/**
	 * Find and return the best VCT entry for the given target accel angle.
	 *
	 * @param vars Movement variables, only Maxspeed is used
	 * @param target_angle Desired acceleration vector angle
	 * @param yaw_constraints The resulting player yaw should be >= the first angle
	 *                        and <= the second angle. The angles are given in M_U multiples
	 * @return The optimal VCT entry.
	 */
	const Entry& GetBestVector(const MovementVars& vars,
	                           double target_angle,
	                           std::pair<uint16_t, uint16_t> yaw_constraints);
} // VCT

} // HLStrafe
