#include "hlstrafe.hpp"
#include "util.hpp"
#include "vec.hpp"

namespace HLStrafe
{

using namespace anglefuncs;
using namespace prediction;

namespace strafefuncs
{
	static void SideStrafeGeneral(
		const PlayerData& player,
		const MovementVars& vars,
		PositionType postype,
		double wishspeed,
		const HLTAS::StrafeButtons& strafeButtons,
		bool useGivenButtons,
		HLTAS::Button& usedButton,
		double vel_yaw,
		double theta,
		bool right,
		bool safeguard_yaw,
		vec2d velocities[2],
		double yaws[2])
	{
		assert(postype != PositionType::WATER);

		if (useGivenButtons) {
			if (postype == PositionType::AIR) {
				if (right)
					usedButton = strafeButtons.AirRight;
				else
					usedButton = strafeButtons.AirLeft;
			} else {
				if (right)
					usedButton = strafeButtons.GroundRight;
				else
					usedButton = strafeButtons.GroundLeft;
			}
		} else {
			usedButton = GetBestButtons(theta, right);
		}

		double phi = ButtonsPhi(usedButton);
		theta = right ? -theta : theta;

		if (!player.Velocity.as_2d().is_zero())
			vel_yaw = Atan2(player.Velocity.y, player.Velocity.x);

		double yaw = vel_yaw - phi + theta;
		yaws[0] = AngleModRad(yaw);

		// Very rare case of yaw == anglemod(yaw).
		if (yaws[0] == yaw) {
			// Multiply by 1.5 because the fp precision might make the yaw a value not enough
			// to reach the next anglemod.
			// Or divide by 2 because it might throw us a value too far back.
			yaws[1] = AngleModRad(yaw + std::copysign(M_U_RAD * 1.5, yaw));

			// We need to handle this when we may have yaw equal to the speed change boundary.
			if (safeguard_yaw)
				yaws[0] = AngleModRad(yaw - std::copysign(M_U_RAD / 2, yaw));
		} else
			yaws[1] = AngleModRad(yaw + std::copysign(M_U_RAD, yaw));

		vec2d wishdir(std::cos(yaws[0] + phi), std::sin(yaws[0] + phi));
		PlayerData pl = player;

		VectorFME(pl, vars, postype, wishspeed, wishdir);
		velocities[0] = pl.Velocity.as_2d();

		wishdir = vec2d(std::cos(yaws[0] + phi), std::sin(yaws[0] + phi));
		pl.Velocity = player.Velocity;

		VectorFME(pl, vars, postype, wishspeed, wishdir);
		velocities[1] = pl.Velocity.as_2d();
	}

	double SideStrafeMaxAccel(
		PlayerData& player,
		const MovementVars& vars,
		PositionType postype,
		double wishspeed,
		const HLTAS::StrafeButtons& strafeButtons,
		bool useGivenButtons,
		HLTAS::Button& usedButton,
		double vel_yaw,
		bool right)
	{
		assert(postype != PositionType::WATER);

		double theta = MaxAccelTheta(player, vars, postype, wishspeed);

		vec2d velocities[2];
		double yaws[2];
		SideStrafeGeneral(
			player,
			vars,
			postype,
			wishspeed,
			strafeButtons,
			useGivenButtons,
			usedButton,
			vel_yaw,
			theta,
			right,
			false,
			velocities,
			yaws);

		double speedsqrs[2] = {
			velocities[0].len_sq(),
			velocities[1].len_sq()
		};

		if (speedsqrs[0] > speedsqrs[1]) {
			player.Velocity.as_2d() = velocities[0];
			return yaws[0];
		} else {
			player.Velocity.as_2d() = velocities[1];
			return yaws[1];
		}
	}

	double SideStrafeMaxAngle(
		PlayerData& player,
		const MovementVars& vars,
		PositionType postype,
		double wishspeed,
		const HLTAS::StrafeButtons& strafeButtons,
		bool useGivenButtons,
		HLTAS::Button& usedButton,
		double vel_yaw,
		bool right)
	{
		assert(postype != PositionType::WATER);

		bool safeguard_yaw;
		double theta = MaxAngleTheta(player, vars, postype, wishspeed, safeguard_yaw);

		vec2d velocities[2];
		double yaws[2];
		SideStrafeGeneral(
			player,
			vars,
			postype,
			wishspeed,
			strafeButtons,
			useGivenButtons,
			usedButton,
			vel_yaw,
			theta,
			right,
			safeguard_yaw,
			velocities,
			yaws);

		double old_speed = player.Velocity.as_2d().len();
		double speeds[2] = { velocities[0].len(), velocities[1].len() };
		double cosangles[2] = {
			velocities[0].dot(player.Velocity.as_2d()) / (old_speed * speeds[0]),
			velocities[1].dot(player.Velocity.as_2d()) / (old_speed * speeds[1])
		};

		if (cosangles[0] < cosangles[1]) {
			player.Velocity.as_2d() = velocities[0];
			return yaws[0];
		} else {
			player.Velocity.as_2d() = velocities[1];
			return yaws[1];
		}
	}

	double SideStrafeMaxDeccel(
		PlayerData& player,
		const MovementVars& vars,
		PositionType postype,
		double wishspeed,
		const HLTAS::StrafeButtons& strafeButtons,
		bool useGivenButtons,
		HLTAS::Button& usedButton,
		double vel_yaw,
		bool right,
		bool& strafed)
	{
		assert(postype != PositionType::WATER);

		// Check a bunch of stuff.
		bool onground = (postype == PositionType::GROUND);
		double speed = player.Velocity.as_2d().len();
		double accel = onground ? vars.Accelerate : vars.Airaccelerate;
		double accelspeed = accel * wishspeed * vars.EntFriction * vars.Frametime;
		double wishspeed_capped = onground ? wishspeed : 30;
		if (speed == 0.0 || accelspeed == 0.0 || (accelspeed < 0.0 && accelspeed <= -wishspeed_capped * 2)) {
			strafed = false;
			return 0.0;
		}
		strafed = true;

		bool safeguard_yaw;
		double theta = MaxDeccelTheta(player, vars, postype, wishspeed, safeguard_yaw);

		vec2d velocities[2];
		double yaws[2];
		SideStrafeGeneral(
			player,
			vars,
			postype,
			wishspeed,
			strafeButtons,
			useGivenButtons,
			usedButton,
			vel_yaw,
			theta,
			right,
			safeguard_yaw,
			velocities,
			yaws);

		double speedsqrs[2] = {
			velocities[0].len_sq(),
			velocities[1].len_sq()
		};

		if (speedsqrs[0] < speedsqrs[1]) {
			player.Velocity.as_2d() = velocities[0];
			return yaws[0];
		} else {
			player.Velocity.as_2d() = velocities[1];
			return yaws[1];
		}
	}

	double SideStrafeConstSpeed(
		PlayerData& player,
		const MovementVars& vars,
		PositionType postype,
		double wishspeed,
		const HLTAS::StrafeButtons& strafeButtons,
		bool useGivenButtons,
		HLTAS::Button& usedButton,
		double vel_yaw,
		bool right)
	{
		assert(postype != PositionType::WATER);

		double theta = ConstSpeedTheta(player, vars, postype, wishspeed);

		vec2d velocities[2];
		double yaws[2];
		SideStrafeGeneral(
			player,
			vars,
			postype,
			wishspeed,
			strafeButtons,
			useGivenButtons,
			usedButton,
			vel_yaw,
			theta,
			right,
			false,
			velocities,
			yaws);

		double speedsqrs[2] = {
			velocities[0].len_sq(),
			velocities[1].len_sq()
		};
		double oldspeedsqr = player.SpeedBeforeFriction * player.SpeedBeforeFriction;

		if (std::fabs(speedsqrs[0] - oldspeedsqr) <= std::fabs(speedsqrs[1] - oldspeedsqr)) {
			player.Velocity.as_2d() = velocities[0];
			return yaws[0];
		} else {
			player.Velocity.as_2d() = velocities[1];
			return yaws[1];
		}
	}

	double BestStrafeMaxAccel(
		PlayerData& player,
		const MovementVars& vars,
		PositionType postype,
		double wishspeed,
		const HLTAS::StrafeButtons& strafeButtons,
		bool useGivenButtons,
		HLTAS::Button& usedButton,
		double vel_yaw)
	{
		assert(postype != PositionType::WATER);

		double yaws[2];
		HLTAS::Button buttons[2];
		vec orig_vel = player.Velocity;
		yaws[0] = SideStrafeMaxAccel(
			player,
			vars,
			postype,
			wishspeed,
			strafeButtons,
			useGivenButtons,
			buttons[0],
			vel_yaw,
			false);

		vec temp_vel = player.Velocity;
		player.Velocity = orig_vel;
		yaws[1] = SideStrafeMaxAccel(
			player,
			vars,
			postype,
			wishspeed,
			strafeButtons,
			useGivenButtons,
			buttons[1],
			vel_yaw,
			true);

		double speedsqrs[2] = {
			temp_vel.as_2d().len_sq(),
			player.Velocity.as_2d().len_sq()
		};

		if (speedsqrs[0] > speedsqrs[1]) {
			player.Velocity = temp_vel;
			usedButton = buttons[0];
			return yaws[0];
		} else {
			usedButton = buttons[1];
			return yaws[1];
		}
	}

	double BestStrafeMaxAngle(
		PlayerData& player,
		const MovementVars& vars,
		PositionType postype,
		double wishspeed,
		const HLTAS::StrafeButtons& strafeButtons,
		bool useGivenButtons,
		HLTAS::Button& usedButton,
		double vel_yaw)
	{
		assert(postype != PositionType::WATER);

		double yaws[2];
		HLTAS::Button buttons[2];
		vec orig_vel = player.Velocity;
		yaws[0] = SideStrafeMaxAngle(
			player,
			vars,
			postype,
			wishspeed,
			strafeButtons,
			useGivenButtons,
			buttons[0],
			vel_yaw,
			false);

		vec temp_vel = player.Velocity;
		player.Velocity = orig_vel;
		yaws[1] = SideStrafeMaxAngle(
			player,
			vars,
			postype,
			wishspeed,
			strafeButtons,
			useGivenButtons,
			buttons[1],
			vel_yaw,
			true);

		double old_speed = orig_vel.as_2d().len();
		double speeds[2] = { temp_vel.as_2d().len(), player.Velocity.as_2d().len() };
		double cosangles[2] = {
			temp_vel.as_2d().dot(orig_vel.as_2d()) / (old_speed * speeds[0]),
			player.Velocity.as_2d().dot(orig_vel.as_2d()) / (old_speed * speeds[1])
		};

		if (cosangles[0] < cosangles[1]) {
			player.Velocity = temp_vel;
			usedButton = buttons[0];
			return yaws[0];
		} else {
			usedButton = buttons[1];
			return yaws[1];
		}
	}

	double BestStrafeMaxDeccel(
		PlayerData& player,
		const MovementVars& vars,
		PositionType postype,
		double wishspeed,
		const HLTAS::StrafeButtons& strafeButtons,
		bool useGivenButtons,
		HLTAS::Button& usedButton,
		double vel_yaw,
		bool& strafed)
	{
		assert(postype != PositionType::WATER);

		double yaws[2];
		HLTAS::Button buttons[2];
		vec orig_vel = player.Velocity;
		yaws[0] = SideStrafeMaxDeccel(
			player,
			vars,
			postype,
			wishspeed,
			strafeButtons,
			useGivenButtons,
			buttons[0],
			vel_yaw,
			false,
			strafed);

		vec temp_vel = player.Velocity;
		player.Velocity = orig_vel;
		yaws[1] = SideStrafeMaxDeccel(
			player,
			vars,
			postype,
			wishspeed,
			strafeButtons,
			useGivenButtons,
			buttons[1],
			vel_yaw,
			true,
			strafed);

		// The condition for strafed does not depend on the strafing direction so
		// either both functions returned true, or both returned false.
		if (!strafed)
			return 0.0;

		double speedsqrs[2] = {
			temp_vel.as_2d().len_sq(),
			player.Velocity.as_2d().len_sq()
		};

		if (speedsqrs[0] < speedsqrs[1]) {
			player.Velocity = temp_vel;
			usedButton = buttons[0];
			return yaws[0];
		} else {
			usedButton = buttons[1];
			return yaws[1];
		}
	}

	double BestStrafeConstSpeed(
		PlayerData& player,
		const MovementVars& vars,
		PositionType postype,
		double wishspeed,
		const HLTAS::StrafeButtons& strafeButtons,
		bool useGivenButtons,
		HLTAS::Button& usedButton,
		double vel_yaw)
	{
		assert(postype != PositionType::WATER);

		double yaws[2];
		HLTAS::Button buttons[2];
		vec orig_vel = player.Velocity;
		yaws[0] = SideStrafeConstSpeed(
			player,
			vars,
			postype,
			wishspeed,
			strafeButtons,
			useGivenButtons,
			buttons[0],
			vel_yaw,
			false);

		vec temp_vel = player.Velocity;
		player.Velocity = orig_vel;
		yaws[1] = SideStrafeConstSpeed(
			player,
			vars,
			postype,
			wishspeed,
			strafeButtons,
			useGivenButtons,
			buttons[1],
			vel_yaw,
			true);

		double speedsqrs[2] = {
			temp_vel.as_2d().len_sq(),
			player.Velocity.as_2d().len_sq()
		};
		double oldspeedsqr = orig_vel.as_2d().len_sq();

		if (std::fabs(oldspeedsqr - speedsqrs[0]) <= std::fabs(oldspeedsqr - speedsqrs[1])) {
			player.Velocity = temp_vel;
			usedButton = buttons[0];
			return yaws[0];
		} else {
			usedButton = buttons[1];
			return yaws[1];
		}
	}

	double YawStrafeMaxAccel(
		PlayerData& player,
		const MovementVars& vars,
		PositionType postype,
		double wishspeed,
		const HLTAS::StrafeButtons& strafeButtons,
		bool useGivenButtons,
		HLTAS::Button& usedButton,
		double vel_yaw,
		double yaw)
	{
		assert(postype != PositionType::WATER);

		double theta = MaxAccelIntoYawTheta(player, vars, postype, wishspeed, vel_yaw, yaw);

		vec2d velocities[2];
		double yaws[2];
		SideStrafeGeneral(
			player,
			vars,
			postype,
			wishspeed,
			strafeButtons,
			useGivenButtons,
			usedButton,
			vel_yaw,
			std::fabs(theta),
			(theta < 0),
			false,
			velocities,
			yaws);

		double speedsqrs[2] = {
			velocities[0].len_sq(),
			velocities[1].len_sq()
		};

		if (speedsqrs[0] > speedsqrs[1]) {
			player.Velocity.as_2d() = velocities[0];
			return yaws[0];
		} else {
			player.Velocity.as_2d() = velocities[1];
			return yaws[1];
		}
	}

	double YawStrafeMaxAngle(
		PlayerData& player,
		const MovementVars& vars,
		PositionType postype,
		double wishspeed,
		const HLTAS::StrafeButtons& strafeButtons,
		bool useGivenButtons,
		HLTAS::Button& usedButton,
		double vel_yaw,
		double yaw)
	{
		assert(postype != PositionType::WATER);

		bool safeguard_yaw;
		double theta = MaxAngleTheta(player, vars, postype, wishspeed, safeguard_yaw);

		vec2d velocities[2];
		double yaws[2];
		if (!player.Velocity.as_2d().is_zero())
			vel_yaw = Atan2(player.Velocity.y, player.Velocity.x);

		SideStrafeGeneral(
			player,
			vars,
			postype,
			wishspeed,
			strafeButtons,
			useGivenButtons,
			usedButton,
			vel_yaw,
			theta,
			(NormalizeRad(yaw - vel_yaw) < 0),
			safeguard_yaw,
			velocities,
			yaws);

		double old_speed = player.Velocity.as_2d().len();
		double speeds[2] = { velocities[0].len(), velocities[1].len() };
		double cosangles[2] = {
			velocities[0].dot(player.Velocity.as_2d()) / (old_speed * speeds[0]),
			velocities[1].dot(player.Velocity.as_2d()) / (old_speed * speeds[1])
		};

		if (cosangles[0] < cosangles[1]) {
			player.Velocity.as_2d() = velocities[0];
			return yaws[0];
		} else {
			player.Velocity.as_2d() = velocities[1];
			return yaws[1];
		}
	}

	double YawStrafeMaxDeccel(
		PlayerData& player,
		const MovementVars& vars,
		PositionType postype,
		double wishspeed,
		const HLTAS::StrafeButtons& strafeButtons,
		bool useGivenButtons,
		HLTAS::Button& usedButton,
		double vel_yaw,
		double yaw,
		bool& strafed)
	{
		assert(postype != PositionType::WATER);

		// Check a bunch of stuff.
		bool onground = (postype == PositionType::GROUND);
		double speed = player.Velocity.as_2d().len();
		double accel = onground ? vars.Accelerate : vars.Airaccelerate;
		double accelspeed = accel * wishspeed * vars.EntFriction * vars.Frametime;
		double wishspeed_capped = onground ? wishspeed : 30;
		if (speed == 0.0 || accelspeed == 0.0 || (accelspeed < 0.0 && accelspeed <= -wishspeed_capped * 2)) {
			strafed = false;
			return 0.0;
		}
		strafed = true;

		bool safeguard_yaw;
		double theta = MaxDeccelTheta(player, vars, postype, wishspeed, safeguard_yaw);

		vec2d velocities[2];
		double yaws[2];
		vel_yaw = Atan2(player.Velocity.y, player.Velocity.x);
		SideStrafeGeneral(
			player,
			vars,
			postype,
			wishspeed,
			strafeButtons,
			useGivenButtons,
			usedButton,
			vel_yaw,
			theta,
			(NormalizeRad(yaw - vel_yaw) < 0),
			safeguard_yaw,
			velocities,
			yaws);

		double speedsqrs[2] = {
			velocities[0].len_sq(),
			velocities[1].len_sq()
		};

		if (speedsqrs[0] < speedsqrs[1]) {
			player.Velocity.as_2d() = velocities[0];
			return yaws[0];
		} else {
			player.Velocity.as_2d() = velocities[1];
			return yaws[1];
		}
	}

	double YawStrafeConstSpeed(
		PlayerData& player,
		const MovementVars& vars,
		PositionType postype,
		double wishspeed,
		const HLTAS::StrafeButtons& strafeButtons,
		bool useGivenButtons,
		HLTAS::Button& usedButton,
		double vel_yaw,
		double yaw)
	{
		assert(postype != PositionType::WATER);

		double oldspeedsqr = player.Velocity.as_2d().len_sq();
		double theta = ConstSpeedTheta(player, vars, postype, wishspeed);

		vec2d velocities[2];
		double yaws[2];
		SideStrafeGeneral(
			player,
			vars,
			postype,
			wishspeed,
			strafeButtons,
			useGivenButtons,
			usedButton,
			vel_yaw,
			theta,
			(NormalizeRad(yaw - vel_yaw) < 0),
			false,
			velocities,
			yaws);

		double speedsqrs[2] = {
			velocities[0].len_sq(),
			velocities[1].len_sq()
		};

		if (std::fabs(speedsqrs[0] - oldspeedsqr) <= std::fabs(speedsqrs[1] - oldspeedsqr)) {
			player.Velocity.as_2d() = velocities[0];
			return yaws[0];
		} else {
			player.Velocity.as_2d() = velocities[1];
			return yaws[1];
		}
	}

	double PointStrafe(
		PlayerData& player,
		const MovementVars& vars,
		PositionType postype,
		double wishspeed,
		const HLTAS::StrafeButtons& strafeButtons,
		bool useGivenButtons,
		HLTAS::Button& usedButton,
		double vel_yaw,
		HLTAS::StrafeType type,
		const vec2d& point,
		bool& strafed)
	{
		assert(postype != PositionType::WATER);

		vec2d difference = point - player.Origin.as_2d();

		// Covers the case where both vectors are zero.
		if (difference.len_sq() <= 4.0) {
			strafed = false;
			return 0.0;
		}
		strafed = true;

		double yaw = Atan2(difference.y, difference.x);

		switch (type) {
		default:
		case HLTAS::StrafeType::MAXACCEL: return YawStrafeMaxAccel(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, usedButton, vel_yaw, yaw);
		case HLTAS::StrafeType::MAXANGLE: return YawStrafeMaxAngle(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, usedButton, vel_yaw, yaw);
		case HLTAS::StrafeType::MAXDECCEL: return YawStrafeMaxDeccel(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, usedButton, vel_yaw, yaw, strafed);
		case HLTAS::StrafeType::CONSTSPEED: return YawStrafeConstSpeed(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, usedButton, vel_yaw, yaw);
		}
	}
}

} // namespace HLStrafe