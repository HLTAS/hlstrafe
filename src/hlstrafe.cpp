#include <algorithm>
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

	double MaxAngleTheta(const PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed)
	{
		assert(postype != PositionType::WATER);

		bool onground = (postype == PositionType::GROUND);
		double speed = Length<float, 2>(player.Velocity);
		double accel = onground ? vars.Accelerate : vars.Airaccelerate;
		double accelspeed = accel * wishspeed * vars.EntFriction * vars.Frametime;
		if (accelspeed <= 0.0) {
			double wishspeed_capped = onground ? wishspeed : 30;
			accelspeed *= -1;
			if (accelspeed >= speed) {
				if (wishspeed_capped >= speed)
					return 0.0;
				else
					return std::acos(wishspeed_capped / speed); // The actual angle needs to be _less_ than this.
			} else {
				if (wishspeed_capped >= speed)
					return std::acos(accelspeed / speed);
				else
					return std::acos(std::min(accelspeed, wishspeed_capped) / speed); // The actual angle needs to be _less_ than this if wishspeed_capped <= accelspeed.
			}
		} else {
			if (accelspeed >= speed)
				return M_PI;
			else
				return std::acos(-1 * accelspeed / speed);
		}
	}

	void VectorFME(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const double a[2])
	{
		assert(postype != PositionType::WATER);

		bool onground = (postype == PositionType::GROUND);
		double wishspeed_capped = onground ? wishspeed : 30;
		double tmp = wishspeed_capped - DotProduct<float, double, 2>(player.Velocity, a);
		if (tmp <= 0.0)
			return;

		double accel = onground ? vars.Accelerate : vars.Airaccelerate;
		double accelspeed = accel * wishspeed * vars.EntFriction * vars.Frametime;
		if (accelspeed <= tmp)
			tmp = accelspeed;

		player.Velocity[0] += a[0] * tmp;
		player.Velocity[1] += a[1] * tmp;
	}

	inline double ButtonsPhi(HLTAS::Button button)
	{
		switch (button) {
		case HLTAS::Button::      FORWARD: return 0;
		case HLTAS::Button:: FORWARD_LEFT: return -M_PI / 4;
		case HLTAS::Button::         LEFT: return -M_PI / 2;
		case HLTAS::Button::    BACK_LEFT: return -3 * M_PI / 2;
		case HLTAS::Button::         BACK: return M_PI;
		case HLTAS::Button::   BACK_RIGHT: return 3 * M_PI / 2;
		case HLTAS::Button::        RIGHT: return M_PI / 2;
		case HLTAS::Button::FORWARD_RIGHT: return M_PI / 4;
		default: return 0;
		}
	}

	double SideStrafe(PlayerData& player, const MovementVars& vars,
		PositionType postype, double wishspeed, HLTAS::Button buttons, double yaw, double theta, bool right)
	{
		assert(postype != PositionType::WATER);

		double phi = ButtonsPhi(buttons);
		phi = right ? phi : -phi;
		theta = right ? theta : -theta;

		if (!IsZero<float, 2>(player.Velocity))
			yaw = std::atan2(player.Velocity[1], player.Velocity[0]);
		yaw += phi - theta;

		double trial_yaws[2] = {
			AngleModRad(yaw),
			AngleModRad(yaw + (right ? M_U_RAD : -M_U_RAD))
		};
		// Very rare case.
		if (yaw == trial_yaws[0])
			trial_yaws[0] -= (right ? M_U_RAD : -M_U_RAD);

		double avec[2] = {
			std::cos(trial_yaws[0] - phi),
			std::sin(trial_yaws[0] - phi)
		};
		float orig_vel[2] = { player.Velocity[0], player.Velocity[1] };
		VectorFME(player, vars, postype, wishspeed, avec);
		float trial_vel1[2] = { player.Velocity[0], player.Velocity[1] };
		player.Velocity[0] = orig_vel[0];
		player.Velocity[1] = orig_vel[1];

		avec[0] = std::cos(trial_yaws[1] - phi);
		avec[1] = std::sin(trial_yaws[1] - phi);
		VectorFME(player, vars, postype, wishspeed, avec);

		double trial_speedsqrs[2] = {
			DotProduct<float, float, 2>(trial_vel1, trial_vel1),
			DotProduct<float, float, 2>(player.Velocity, player.Velocity)
		};

		if (trial_speedsqrs[0] > trial_speedsqrs[1]) {
			player.Velocity[0] = trial_vel1[0];
			player.Velocity[1] = trial_vel1[1];
			return trial_yaws[1];
		} else
			return trial_yaws[0];
	}
}
