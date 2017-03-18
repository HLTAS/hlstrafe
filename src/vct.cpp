#include <algorithm>
#include <cmath>
// Temporary, for debug printfs
#include <cstdio>
#include <vector>

#include "hlstrafe.hpp"
#include "util.hpp"
#include "vct.hpp"

namespace HLStrafe
{

namespace VCT
{
	namespace
	{
		/// The vectorial compensation table itself.
		/// This vector is sorted by entry.r.
		std::vector<Entry> table;

		/// Maxspeed used during the table generation.
		float maxspeed;

		/// The VCT is exactly the same for any maxspeed less than or equal to this value.
		constexpr float MAXSPEED_VCT_CAP = 1023.0f;

		/// Generate the VCT. Only Maxspeed is needed from vars.
		void ComputeVCT(const MovementVars& vars) {
			/// Maximal value for forwardmove and sidemove.
			constexpr int16_t MAX_MOVE = 2047;

			table.clear();
			maxspeed = vars.Maxspeed;

			int16_t F = 0;
			int16_t S = 1;
			int16_t p = 1;
			int16_t q = MAX_MOVE;

			while (p != 1 || q != 1)
			{
				int16_t k = (MAX_MOVE + S) / q;
				int16_t tmpF = F;
				int16_t tmpS = S;
				F = p;
				S = q;
				p = k * p - tmpF;
				q = k * q - tmpS;

				int16_t fac = 2047 / S;

				Entry entry;
				entry.F = F * fac;
				entry.S = S * fac;

				// Skip pairs giving speed below maxspeed.
				if (maxspeed > MAXSPEED_VCT_CAP
					&& maxspeed * maxspeed > entry.F * entry.F + entry.S * entry.S)
					continue;

				double phi = 2 * M_PI - std::atan(static_cast<double>(S) / F);
				entry.r = phi - AngleModRad(phi);
				table.push_back(entry);

				entry.r = (entry.r == 0.0 ? 0.0 : M_U_RAD - entry.r);
				std::swap(entry.F, entry.S);
				table.push_back(entry);
			}

			// Add 0 and PI / 4 angles omitted in the loop above.
			table.push_back(Entry { 0.0, 0, 2047 });
			table.push_back(Entry { 0.0, 2047, 2047 });

			std::sort(table.begin(), table.end());

			std::printf("VCT size: %zu\n", table.size());
		}
	}

	const Entry& GetBestVector(const MovementVars& vars,
	                           double target_angle,
	                           std::pair<uint16_t, uint16_t> yaw_constraints) {
		// Regenerate the VCT if needed,
		if (table.empty() // so either the table is empty,
			|| (maxspeed != vars.Maxspeed // or the maxspeed is different and above the cap
				&& (maxspeed > MAXSPEED_VCT_CAP || vars.Maxspeed > MAXSPEED_VCT_CAP)))
			ComputeVCT(vars);

		// Stub.
		return table[0];
	}
} // VCT

} // HLStrafe
