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

	double MaxAccelIntoYawTheta(const PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, double vel_yaw, double yaw)
	{
		assert(postype != PositionType::WATER);

		if (!IsZero<float, 2>(player.Velocity))
			vel_yaw = std::atan2(player.Velocity[1], player.Velocity[0]);

		double theta = MaxAccelTheta(player, vars, postype, wishspeed);
		if (theta == 0.0 || theta == M_PI)
			return NormalizeRad(yaw - vel_yaw + theta);
		return std::copysign(theta, NormalizeRad(yaw - vel_yaw));
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

	static void SideStrafeGeneral(const PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed,
		HLTAS::Button buttons, double vel_yaw, double theta, bool right, float velocities[2][2], double yaws[2])
	{
		assert(postype != PositionType::WATER);

		double phi = ButtonsPhi(buttons);
		phi = right ? phi : -phi;
		theta = right ? theta : -theta;

		if (!IsZero<float, 2>(player.Velocity))
			vel_yaw = std::atan2(player.Velocity[1], player.Velocity[0]);

		double yaw = vel_yaw + phi - theta;
		yaws[0] = AngleModRad(yaw);
		yaws[1] = AngleModRad(yaw + std::copysign(M_U_RAD, yaw));

		double avec[2] = { std::cos(yaws[0] - phi), std::sin(yaws[0] - phi) };
		PlayerData pl = player;
		VectorFME(pl, vars, postype, wishspeed, avec);
		VecCopy<float, 2>(pl.Velocity, velocities[0]);

		avec[0] = std::cos(yaws[1] - phi);
		avec[1] = std::sin(yaws[1] - phi);
		VecCopy<float, 2>(player.Velocity, pl.Velocity);
		VectorFME(pl, vars, postype, wishspeed, avec);
		VecCopy<float, 2>(pl.Velocity, velocities[1]);
	}

	double SideStrafeMaxAccel(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, HLTAS::Button buttons,
		double vel_yaw, bool right)
	{
		assert(postype != PositionType::WATER);

		double theta = MaxAccelTheta(player, vars, postype, wishspeed);
		float velocities[2][2];
		double yaws[2];
		SideStrafeGeneral(player, vars, postype, wishspeed, buttons, vel_yaw, theta, right, velocities, yaws);

		double speedsqrs[2] = {
			DotProduct<float, float, 2>(velocities[0], velocities[0]),
			DotProduct<float, float, 2>(velocities[1], velocities[1])
		};

		if (speedsqrs[0] > speedsqrs[1]) {
			VecCopy<float, 2>(velocities[0], player.Velocity);
			return yaws[0];
		} else {
			VecCopy<float, 2>(velocities[1], player.Velocity);
			return yaws[1];
		}
	}
}
