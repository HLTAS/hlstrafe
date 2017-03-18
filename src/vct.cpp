#include <algorithm>
#include <cmath>
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

				int fac = std::ceil(maxspeed / std::sqrt(F * F + S * S));
				if (S * fac > MAX_MOVE)
					continue;

				Entry entry;
				entry.F = F;
				entry.S = S;

				double phi = 2 * M_PI - std::atan(static_cast<double>(S) / F);
				entry.r = phi - AngleModRad(phi);
				table.push_back(entry);

				entry.r = (entry.r == 0.0 ? 0.0 : M_U_RAD - entry.r);
				tmpS = entry.S;
				entry.S = entry.F;
				entry.F = tmpS;
				table.push_back(entry);
			}

			// Add 0 and PI / 4 angles omitted in the loop above.
			table.push_back(Entry { 0.0, 0, 1 });
			table.push_back(Entry { 0.0, 1, 1 });

			std::sort(table.begin(), table.end());
		}
	}
} // VCT

} // HLStrafe