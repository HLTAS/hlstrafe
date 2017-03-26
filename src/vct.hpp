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
	 * Class representing angle constraints.
	 *
	 * The values are given in M_U multiples mod 65536. So, for example,
	 * if the lowest constraint is set to 65534 and the highest set to 1,
	 * the satisfying angles would be 65534, 65535, 0 and 1.
	 */
	class AngleConstraints {
	public:
		AngleConstraints(uint16_t lowest, uint16_t highest)
			: lowest(lowest)
			, highest(highest)
		{
		}

		/// Lowest angle satisfying the constraints.
		const uint16_t lowest;

		/// Highest angle satisfying the constraints.
		const uint16_t highest;
	};

	/**
	 * Find and return the best VCT entry for the given target accel angle.
	 *
	 * @param vars Movement variables, only Maxspeed is used
	 * @param target_angle Desired acceleration vector angle
	 * @param yaw_constraints Constraints for the player yaw angle
	 * @return The optimal VCT entry.
	 */
	const Entry& GetBestVector(const MovementVars& vars,
	                           double target_angle,
	                           const AngleConstraints& yaw_constraints);
} // VCT

} // HLStrafe
