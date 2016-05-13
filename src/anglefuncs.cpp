#include "hlstrafe.hpp"
#include "util.hpp"

namespace HLStrafe
{

namespace anglefuncs
{
	double MaxAccelTheta(const PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed)
	{
		assert(postype != PositionType::WATER);

		bool onground = (postype == PositionType::GROUND);
		double accel = onground ? vars.Accelerate : vars.Airaccelerate;

		double accelspeed = accel * wishspeed * vars.EntFriction * vars.Frametime;
		if (accelspeed <= 0.0)
			return M_PI;

		if (player.Velocity.as_2d().is_zero())
			return 0.0;

		double wishspeed_capped = onground ? wishspeed : 30;
		double tmp = wishspeed_capped - accelspeed;
		if (tmp <= 0.0)
			return M_PI / 2;

		double speed = player.Velocity.as_2d().len();
		if (tmp < speed)
			return std::acos(tmp / speed);

		return 0.0;
	}

	double ConstSpeedTheta(const PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed)
	{
		assert(postype != PositionType::WATER);

		double gamma1 = vars.EntFriction * vars.Frametime * wishspeed;
		double speedsqr = player.Velocity.as_2d().len_sq();
		double numer, denom;

		if (postype == PositionType::AIR) {
			gamma1 *= vars.Airaccelerate;
			if (gamma1 <= 60) {
				numer = -gamma1;
				denom = 2 * std::sqrt(speedsqr);
			} else {
				numer = -30;
				denom = std::sqrt(speedsqr);
			}
		} else {
			gamma1 *= vars.Accelerate;
			double sqrdiff = player.SpeedBeforeFriction * player.SpeedBeforeFriction - speedsqr;
			double tmp = sqrdiff / gamma1;
			if (tmp + gamma1 <= 2 * wishspeed) {
				numer = tmp - gamma1;
				denom = 2 * std::sqrt(speedsqr);
			} else if (gamma1 > wishspeed && wishspeed * wishspeed >= sqrdiff) {
				numer = -std::sqrt(wishspeed * wishspeed - sqrdiff);
				denom = std::sqrt(speedsqr);
			} else {
				return MaxAccelTheta(player, vars, postype, wishspeed);
			}
		}

		if (denom < std::fabs(numer))
			return MaxAccelTheta(player, vars, postype, wishspeed);

		return std::acos(numer / denom);
	}

	double MaxAccelIntoYawTheta(
		const PlayerData& player,
		const MovementVars& vars,
		PositionType postype,
		double wishspeed,
		double vel_yaw,
		double yaw)
	{
		assert(postype != PositionType::WATER);

		if (!player.Velocity.as_2d().is_zero())
			vel_yaw = Atan2(player.Velocity.y, player.Velocity.x);

		double theta = MaxAccelTheta(player, vars, postype, wishspeed);
		if (theta == 0.0 || theta == M_PI)
			return NormalizeRad(yaw - vel_yaw + theta);
		return std::copysign(theta, NormalizeRad(yaw - vel_yaw));
	}

	double MaxAngleTheta(
		const PlayerData& player,
		const MovementVars& vars,
		PositionType postype,
		double wishspeed,
		bool& safeguard_yaw)
	{
		assert(postype != PositionType::WATER);

		safeguard_yaw = false;
		bool onground = (postype == PositionType::GROUND);
		double speed = player.Velocity.as_2d().len();
		double accel = onground ? vars.Accelerate : vars.Airaccelerate;
		double accelspeed = accel * wishspeed * vars.EntFriction * vars.Frametime;

		if (accelspeed <= 0.0) {
			double wishspeed_capped = onground ? wishspeed : 30;
			accelspeed *= -1;

			if (accelspeed >= speed) {
				if (wishspeed_capped >= speed)
					return 0.0;
				else {
					safeguard_yaw = true;
					return std::acos(wishspeed_capped / speed); // The actual angle needs to be _less_ than this.
				}
			} else {
				if (wishspeed_capped >= speed)
					return std::acos(accelspeed / speed);
				else {
					safeguard_yaw = (wishspeed_capped <= accelspeed);
					return std::acos(std::min(accelspeed, wishspeed_capped) / speed); // The actual angle needs to be _less_ than this if wishspeed_capped <= accelspeed.
				}
			}
		} else {
			if (accelspeed >= speed)
				return M_PI;
			else
				return std::acos(-1 * accelspeed / speed);
		}
	}

	double MaxDeccelTheta(
		const PlayerData& player,
		const MovementVars& vars,
		PositionType postype,
		double wishspeed,
		bool& safeguard_yaw)
	{
		assert(postype != PositionType::WATER);

		safeguard_yaw = false;
		bool onground = (postype == PositionType::GROUND);
		double speed = player.Velocity.as_2d().len();
		double accel = onground ? vars.Accelerate : vars.Airaccelerate;
		double accelspeed = accel * wishspeed * vars.EntFriction * vars.Frametime;

		// Check for these conditions in the function above where we can actually set strafed = false.
		assert(speed != 0.0);
		assert(accelspeed != 0.0);

		if (accelspeed < 0.0) {
			double wishspeed_capped = onground ? wishspeed : 30;

			// And for this one as well.
			assert(accelspeed > -wishspeed_capped * 2);

			safeguard_yaw = true;
			return std::acos(wishspeed_capped / speed); // The actual angle needs to be _less_ than this.
		} else {
			return M_PI;
		}
	}
}

} // namespace HLStrafe
