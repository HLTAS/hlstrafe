#include <algorithm>
#include <cmath>
// Temporary, for debug printfs
#include <cstdio>
#include <mutex>
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

		/// Mutex for accessing the table.
		std::mutex table_mutex;

		/// Maxspeed used during the table generation.
		float maxspeed;

		/// The VCT is exactly the same for any maxspeed less than or equal to this value.
		constexpr float MAXSPEED_VCT_CAP = 1023.0f;

		/// Add rotations of the given entry to the VCT.
		void AddRotations(Entry& entry) {
			const auto F = entry.F;
			const auto S = entry.S;

			entry.F = -S;
			entry.S = F;
			table.push_back(entry);

			entry.F = -F;
			entry.S = -S;
			table.push_back(entry);

			entry.F = S;
			entry.S = -F;
			table.push_back(entry);
		}

		/// Generate the VCT. Only Maxspeed is needed from vars.
		void ComputeVCT(const MovementVars& vars) {
			/// Maximal value for forwardmove and sidemove.
			constexpr int16_t MAX_MOVE = 2047;

			std::printf("Computing the vectorial compensation table...\n");

			table.clear();
			maxspeed = vars.Maxspeed;

			int16_t F = 0;
			int16_t S = 1;
			int16_t p = 1;
			int16_t q = MAX_MOVE;

			Entry entry;

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

				entry.F = F * fac;
				entry.S = S * fac;

				// Skip pairs giving speed below maxspeed.
				if (maxspeed > MAXSPEED_VCT_CAP
					&& maxspeed * maxspeed > entry.F * entry.F + entry.S * entry.S)
					continue;

				double phi = 2 * M_PI - std::atan(static_cast<double>(S) / F);
				entry.r = phi - AngleModRad(phi);
				table.push_back(entry);
				AddRotations(entry);

				entry.r = (entry.r == 0.0 ? 0.0 : M_U_RAD - entry.r);
				std::swap(entry.F, entry.S);
				table.push_back(entry);
				AddRotations(entry);
			}

			// Add 0 and PI / 4 angles omitted in the loop above.
			entry = Entry { 0.0, 0, 2047 };
			table.push_back(entry);
			AddRotations(entry);

			entry.F = 2047;
			table.push_back(entry);
			AddRotations(entry);

			std::sort(table.begin(), table.end());

			std::printf("Vectorial compensation table size: %zu\n", table.size());
		}

		class BestMatchIterator {
			using It = decltype(table)::const_iterator;

		public:
			BestMatchIterator(It start, double difference)
				: low(start)
				, high(start)
				, difference(difference) {
				if (high != table.cend())
					++high;

				if (low != table.cbegin())
					--low;
				else
					low = table.cend();
			}

			bool HasNext() const {
				return low != table.cend() || high != table.cend();
			}

			It Next() {
				if (high == table.cend()) {
					const auto rv = low;

					if (low == table.cbegin())
						low = table.cend();
					else
						--low;

					return rv;
				} else if (low == table.cend()) {
					return high++;
				} else {
					// Both high and low are valid.
					// Pick the one closest by r.
					const auto low_dif = difference - low->r;
					const auto high_dif = high->r - difference;

					if (high_dif <= low_dif) {
						return high++;
					} else {
						const auto rv = low;

						if (low == table.cbegin())
							low = table.cend();
						else
							--low;

						return rv;
					}
				}
			}

		private:
			It low;
			It high;
			const double difference;
		};
	}

	const Entry& GetBestVector(const MovementVars& vars,
	                           double target_angle,
	                           const AngleConstraints& yaw_constraints) {
		std::lock_guard<std::mutex> table_guard(table_mutex);

		// Regenerate the VCT if needed,
		if (table.empty() // so either the table is empty,
			|| (maxspeed != vars.Maxspeed // or the maxspeed is different and above the cap
				&& (maxspeed > MAXSPEED_VCT_CAP || vars.Maxspeed > MAXSPEED_VCT_CAP)))
			ComputeVCT(vars);

		// std::printf("Target yaw: %.8f", target_angle);

		target_angle = NormalizeRad(target_angle);

		// We need angle in [0; 2Pi) to correctly compare with anglemod.
		if (target_angle < 0)
			target_angle += 2 * M_PI;

		const auto target_angle_am = AngleModRad(target_angle);
		const auto difference = target_angle - target_angle_am; // difference >= 0.

		auto best_match_it = table.cbegin(); // This works for difference = 0.

		// Find best matching entry, unconstrained.
		if (difference > 0) {
			const auto it = std::lower_bound(table.cbegin(),
			                                 table.cend(),
			                                 difference,
			                                 [](const Entry& a, const double& b) {
			                                 	return a.r < b;
			                                 });

			const auto prev = it - 1;

			if (it == table.cend()) {
				best_match_it = prev;
			} else if (difference == it->r) {
				best_match_it = it;
			} else {
				const auto dif1 = it->r - difference;
				const auto dif2 = difference - prev->r;

				if (dif1 <= dif2)
					best_match_it = it;
				else
					best_match_it = prev;
			}
		}

		// Check the constraints.
		auto fs_angle = NormalizeRad(Atan2(-best_match_it->S, best_match_it->F));
		if (fs_angle < 0)
			fs_angle += 2 * M_PI;

		auto yaw = static_cast<uint16_t>(std::floor(target_angle * M_INVU_RAD) - std::floor(fs_angle * M_INVU_RAD));
		if (!yaw_constraints.Contain(yaw)) {
			// Find an entry that does satisfy the constraints.
			auto best_distance = yaw_constraints.DistanceTo(yaw);
			
			size_t iterations = 0;

			BestMatchIterator it(best_match_it, difference);
			while (best_distance > 0 && it.HasNext()) {
				++iterations;
				const auto check_it = it.Next();

				fs_angle = NormalizeRad(Atan2(-check_it->S, check_it->F));
				if (fs_angle < 0)
					fs_angle += 2 * M_PI;

				yaw = static_cast<uint16_t>(std::floor(target_angle * M_INVU_RAD) - std::floor(fs_angle * M_INVU_RAD));
				const auto distance = yaw_constraints.DistanceTo(yaw);

				if (distance < best_distance) {
					best_distance = distance;
					best_match_it = check_it;
				}
			}

			// std::printf("; found a good entry in %zu iterations", iterations);
		}

		// std::printf("; best entry offset: %.16f, atan2: %.8f, fs: %hd, %hd\n", std::fabs(best_match_it->r - difference), std::atan2(-best_match_it->S, best_match_it->F), best_match_it->F, best_match_it->S);

		return *best_match_it;
	}
} // VCT

} // HLStrafe
