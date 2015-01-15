#include <cassert>
#include <cmath>

#include "hlstrafe.hpp"
#include "util.hpp"

namespace HLStrafe
{
	double GetMaxAccelAngle(const PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed)
	{
		assert(!IsZero(player.Velocity));
		assert(postype != PositionType::WATER);

		bool onground = (postype == PositionType::GROUND);
		double accel = onground ? vars.Accelerate : vars.Airaccelerate;
		double accelspeed = accel * wishspeed * vars.EntFriction * vars.Frametime;
		if (accelspeed <= 0.0)
			return 180.0;

		double wishspeed_capped = onground ? wishspeed : 30;
		double anglecos = (wishspeed_capped - accelspeed) / Length(player.Velocity);
		if (anglecos >= 1)
			return 0.0;
		else if (anglecos <= -1)
			return 180.0;
		return (std::acos(anglecos) * M_RAD2DEG);
	}
}
