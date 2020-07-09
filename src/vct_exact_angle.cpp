#include <algorithm>
#include <array>
#include <cmath>
// Temporary, for debug printfs
#include <cstdio>
#include <mutex>

#include "hlstrafe.hpp"
#include "util.hpp"
#include "vct_exact_angle.hpp"

namespace HLStrafe
{

namespace VCTExactAngle
{
	namespace
	{
		// This used to be a vector, but filling such a large vector of values causes heap
		// corruption on WON DLLs seemingly caused by some bug in Half-Life itself. Using an array
		// of known largest size avoids the heap allocations and hence triggering the bug.

		/// The vectorial compensation table itself.
		/// This array is sorted by entry.angle.
		std::array<Entry, 10196504> table;
		size_t table_size = 0;

		/// Mutex for accessing the table.
		std::mutex table_mutex;

		/// Maxspeed used during the table generation.
		float maxspeed;

		/// The VCT is exactly the same for any maxspeed less than or equal to this value.
		constexpr float MAXSPEED_VCT_CAP = 1023.0f;

		/// Add all 8 combinations of F and S into the table.
		void AddCombinations(int16_t F, int16_t S) {
			table[table_size++] = Entry( F,  S);
			table[table_size++] = Entry( F, -S);
			table[table_size++] = Entry(-F,  S);
			table[table_size++] = Entry(-F, -S);

			table[table_size++] = Entry( S,  F);
			table[table_size++] = Entry( S, -F);
			table[table_size++] = Entry(-S,  F);
			table[table_size++] = Entry(-S, -F);
		}

		/// Generate the VCT. Only Maxspeed is needed from vars.
		void ComputeVCT(const MovementVars& vars) {
			/// Maximal value for forwardmove and sidemove.
			constexpr int16_t MAX_MOVE = 2047;

			std::printf("Computing the vectorial compensation table...\n");

			table_size = 0;
			maxspeed = vars.Maxspeed;

			// Compute the Farey sequence in ascending order, starting from 0 / 1 and 1 / MAX_MOVE.
			// This produces all co-prime F and S in the first octant (angles from -90 to -45 degrees).
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

				// Scale F and S to be as large as possible.
				int16_t fac = 2047 / S;
				int16_t scaledF = F * fac;
				int16_t scaledS = S * fac;

				// Skip pairs giving speed below maxspeed.
				if (maxspeed > MAXSPEED_VCT_CAP
					&& maxspeed * maxspeed > scaledF * scaledF + scaledS * scaledS)
					continue;

				AddCombinations(scaledF, scaledS);
			}

			// Add 0 and PI / 4 angles omitted in the loop above.
			AddCombinations(0, 2047);
			AddCombinations(2047, 2047);

			std::sort(table.begin(), (table.begin() + table_size));

			std::printf("Vectorial compensation table size: %zu\n", table_size);

			if (table_size > table.size()) {
				std::printf("ERROR: got bigger table size than expected!\n");
				std::abort();
			}
		}
	}

	const Entry& GetBestVector(const MovementVars& vars, double accel_angle, unsigned version) {
		std::lock_guard<std::mutex> table_guard(table_mutex);

		// Regenerate the VCT if needed,
		if (table_size == 0 // so either the table is empty,
			|| (maxspeed != vars.Maxspeed // or the maxspeed is different and above the cap
				&& (maxspeed > MAXSPEED_VCT_CAP || vars.Maxspeed > MAXSPEED_VCT_CAP)))
			ComputeVCT(vars);

		// std::printf("Target yaw: %.8f", accel_angle);

		accel_angle = NormalizeRad(accel_angle);

		const auto it = std::lower_bound(table.cbegin(),
		                                 (table.cbegin() + table_size),
		                                 accel_angle,
		                                 [](const Entry& a, const double& b) {
		                                    return a.angle < b;
		                                 });

		if (it == table.cbegin()) {
			if (version < 3) {
				// Previous HLStrafe versions had a bug where they returned a reference to the item
				// before the start of the table, which happened to equal to zeros.
				static Entry zero(0, 0);
				return zero;
			}

			return *it;
		}

		const auto prev = it - 1;

		if (it == (table.cbegin() + table_size)) {
			return *prev;
		} else {
			const auto dif1 = it->angle - accel_angle;
			const auto dif2 = accel_angle - prev->angle;

			if (dif1 <= dif2)
				return *it;
			else
				return *prev;
		}
	}
} // VCTExactAngle

} // HLStrafe
