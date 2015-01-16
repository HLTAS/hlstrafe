#include <cassert>
#include <cmath>

#include "hlstrafe.hpp"
#include "util.hpp"

namespace HLStrafe
{
	double MaxAccelTheta(const PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed)
	{
		assert(postype != PositionType::WATER);

		bool onground = (postype == PositionType::GROUND);
		double accel = onground ? vars.Accelerate : vars.Airaccelerate;
		double accelspeed = accel * wishspeed * vars.EntFriction * vars.Frametime;
		if (accelspeed <= 0.0)
			return M_PI;

		double wishspeed_capped = onground ? wishspeed : 30;
		double tmp = wishspeed_capped - accelspeed;
		if (tmp <= 0.0)
			return M_PI / 2;

		double speed = Length<float, 2>(player.Velocity);
		if (tmp < speed)
			return std::acos(tmp / speed);

		return 0.0;
	}

	void VectorFME(PlayerData& player, const MovementVars& vars, const double a[2], PositionType postype, double wishspeed)
	{
		assert(postype != PositionType::WATER);

		double tmp = (postype == PositionType::GROUND ? wishspeed : 30) - player.Velocity[0] * a[0] - player.Velocity[1] * a[1];
		if (tmp <= 0.0)
			return;

		double accel = postype == PositionType::GROUND ? vars.Accelerate : vars.Airaccelerate;
		double accelspeed = accel * wishspeed * vars.EntFriction * vars.Frametime;
		if (accelspeed <= tmp)
			tmp = accelspeed;

		player.Velocity[0] += a[0] * tmp;
		player.Velocity[1] += a[1] * tmp;
	}
}
