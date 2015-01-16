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

	static void SideStrafeGeneral(PlayerData& player, float trial_vel[2], const MovementVars& vars, PositionType postype, double wishspeed,
		HLTAS::Button buttons, double& yaw, double& trial_yaw, double theta, bool right)
	{
		assert(postype != PositionType::WATER);

		double phi = ButtonsPhi(buttons);
		phi = right ? phi : -phi;
		theta = right ? theta : -theta;

		if (!IsZero<float, 2>(player.Velocity))
			yaw = std::atan2(player.Velocity[1], player.Velocity[0]);
		yaw += phi - theta;

		trial_yaw = AngleModRad(yaw + std::copysign(yaw, M_U_RAD));
		yaw = AngleModRad(yaw);

		double avec[2] = { std::cos(trial_yaw - phi), std::sin(trial_yaw - phi) };
		float orig_vel[2] = { player.Velocity[0], player.Velocity[1] };
		VectorFME(player, vars, postype, wishspeed, avec);
		trial_vel[0] = player.Velocity[0];
		trial_vel[1] = player.Velocity[1];
		player.Velocity[0] = orig_vel[0];
		player.Velocity[1] = orig_vel[1];

		avec[0] = std::cos(yaw - phi);
		avec[1] = std::sin(yaw - phi);
		VectorFME(player, vars, postype, wishspeed, avec);
	}

	void SideStrafeMaxAccel(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, HLTAS::Button buttons,
		double& yaw, bool right)
	{
		assert(postype != PositionType::WATER);

		float trial_vel[2];
		double trial_yaw = yaw;
		double theta = MaxAccelTheta(player, vars, postype, wishspeed);
		SideStrafeGeneral(player, trial_vel, vars, postype, wishspeed, buttons, yaw, trial_yaw, theta, right);

		double trial_speedsqrs[2] = {
			DotProduct<float, float, 2>(player.Velocity, player.Velocity),
			DotProduct<float, float, 2>(trial_vel, trial_vel)
		};

		if (trial_speedsqrs[1] > trial_speedsqrs[0]) {
			player.Velocity[0] = trial_vel[0];
			player.Velocity[1] = trial_vel[1];
			yaw = trial_yaw;
		}
	}
}
