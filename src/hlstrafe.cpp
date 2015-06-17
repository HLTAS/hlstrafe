#include <algorithm>
#include <cassert>
#include <cmath>

#include "hlstrafe.hpp"
#include "util.hpp"

// #include "../../SPTLib/sptlib.hpp"

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

	double ConstSpeedTheta(const PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed)
	{
		assert(postype != PositionType::WATER);

		double gamma1 = vars.EntFriction * vars.Frametime * wishspeed;
		double speedsqr = DotProduct<float, float, 2>(player.Velocity, player.Velocity);
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
			} else
				return MaxAccelTheta(player, vars, postype, wishspeed);
		}

		if (denom < std::fabs(numer))
			return MaxAccelTheta(player, vars, postype, wishspeed);

		return std::acos(numer / denom);
	}

	double MaxAccelIntoYawTheta(const PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, double vel_yaw, double yaw)
	{
		assert(postype != PositionType::WATER);

		if (!IsZero<float, 2>(player.Velocity))
			vel_yaw = Atan2(player.Velocity[1], player.Velocity[0]);

		double theta = MaxAccelTheta(player, vars, postype, wishspeed);
		if (theta == 0.0 || theta == M_PI)
			return NormalizeRad(yaw - vel_yaw + theta);
		return std::copysign(theta, NormalizeRad(yaw - vel_yaw));
	}

	double MaxAngleTheta(const PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, bool& safeguard_yaw)
	{
		assert(postype != PositionType::WATER);

		safeguard_yaw = false;
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

	double MaxDeccelTheta(const PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, bool& safeguard_yaw)
	{
		assert(postype != PositionType::WATER);

		safeguard_yaw = false;
		bool onground = (postype == PositionType::GROUND);
		double speed = Length<float, 2>(player.Velocity);
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

		player.Velocity[0] += static_cast<float>(a[0] * tmp);
		player.Velocity[1] += static_cast<float>(a[1] * tmp);
	}

	PositionType Move(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, TraceFunc traceFunc, bool calcVelocity, const double a[2], float fractions[4], float normalzs[4])
	{
		assert(postype != PositionType::WATER);

		bool onground = (postype == PositionType::GROUND);
		CheckVelocity(player, vars);

		// AddCorrectGravity
		float entGravity = vars.EntGravity;
		if (entGravity == 0.0f)
			entGravity = 1.0f;
		player.Velocity[2] -= static_cast<float>(entGravity * vars.Gravity * 0.5 * vars.Frametime);
		player.Velocity[2] += player.Basevelocity[2] * vars.Frametime;
		player.Basevelocity[2] = 0;
		CheckVelocity(player, vars);

		// Move
		wishspeed = std::min(wishspeed, static_cast<double>(vars.Maxspeed));
		if (onground)
			player.Velocity[2] = 0;

		// Accelerate
		if (calcVelocity && a)
			VectorFME(player, vars, postype, wishspeed, a);

		// Move
		VecAdd<float, float, 3>(player.Velocity, player.Basevelocity, player.Velocity);
		if (onground) {
			// WalkMove
			auto spd = Length<float, 3>(player.Velocity);
			if (spd < 1) {
				VecScale<float, 3>(player.Velocity, 0, player.Velocity); // Clear velocity.
			} else {
				float dest[3];
				VecCopy<float, 3>(player.Origin, dest);
				dest[0] += player.Velocity[0] * vars.Frametime;
				dest[1] += player.Velocity[1] * vars.Frametime;

				auto tr = traceFunc(player.Origin, dest, (player.Ducking) ? HullType::DUCKED : HullType::NORMAL);
				if (tr.Fraction == 1.0f) {
					VecCopy<float, 3>(tr.EndPos, player.Origin);
				} else {
					// Figure out the end position when trying to walk up a step.
					auto playerUp = PlayerData(player);
					dest[2] += vars.Stepsize;
					tr = traceFunc(playerUp.Origin, dest, (player.Ducking) ? HullType::DUCKED : HullType::NORMAL);
					if (!tr.StartSolid && !tr.AllSolid)
						VecCopy<float, 3>(tr.EndPos, playerUp.Origin);

					FlyMove(playerUp, vars, postype, traceFunc, fractions, normalzs);
					VecCopy<float, 3>(playerUp.Origin, dest);
					dest[2] -= vars.Stepsize;

					tr = traceFunc(playerUp.Origin, dest, (player.Ducking) ? HullType::DUCKED : HullType::NORMAL);
					if (!tr.StartSolid && !tr.AllSolid)
						VecCopy<float, 3>(tr.EndPos, playerUp.Origin);

					// Figure out the end position when _not_ trying to walk up a step.
					auto playerDown = PlayerData(player);
					FlyMove(playerDown, vars, postype, traceFunc);

					// Take whichever move was the furthest.
					auto downdist = (playerDown.Origin[0] - player.Origin[0]) * (playerDown.Origin[0] - player.Origin[0])
						+ (playerDown.Origin[1] - player.Origin[1]) * (playerDown.Origin[1] - player.Origin[1]);
					auto updist = (playerUp.Origin[0] - player.Origin[0]) * (playerUp.Origin[0] - player.Origin[0])
						+ (playerUp.Origin[1] - player.Origin[1]) * (playerUp.Origin[1] - player.Origin[1]);

					if ((tr.PlaneNormal[2] < 0.7) || (downdist > updist)) {
						VecCopy<float, 3>(playerDown.Origin, player.Origin);
						VecCopy<float, 3>(playerDown.Velocity, player.Velocity);
					} else {
						VecCopy<float, 3>(playerUp.Origin, player.Origin);
						VecCopy<float, 2>(playerUp.Velocity, player.Velocity);
						player.Velocity[2] = playerDown.Velocity[2];
					}
				}
			}
		} else {
			// AirMove
			FlyMove(player, vars, postype, traceFunc, fractions, normalzs);
		}

		postype = GetPositionType(player, traceFunc);
		VecSubtract<float, float, 3>(player.Velocity, player.Basevelocity, player.Velocity);
		CheckVelocity(player, vars);
		if (postype != PositionType::GROUND && postype != PositionType::WATER) {
			// FixupGravityVelocity
			player.Velocity[2] -= static_cast<float>(entGravity * vars.Gravity * 0.5 * vars.Frametime);
			CheckVelocity(player, vars);
		}

		return postype;
	}

	void FlyMove(PlayerData& player, const MovementVars& vars, PositionType postype, TraceFunc traceFunc, float fractions[4], float normalzs[4])
	{
		const auto MAX_BUMPS = 4;
		const auto MAX_CLIP_PLANES = 5;

		float originalVelocity[3], savedVelocity[3];
		VecCopy<float, 3>(player.Velocity, originalVelocity);
		VecCopy<float, 3>(player.Velocity, savedVelocity);

		auto timeLeft = vars.Frametime;
		auto allFraction = 0.0f;
		auto numPlanes = 0;
		auto blockedState = 0;
		float planes[MAX_CLIP_PLANES][3];

		for (auto bumpCount = 0; bumpCount < MAX_BUMPS; ++bumpCount) {
			if (IsZero<float, 3>(player.Velocity))
				break;

			float end[3];
			for (size_t i = 0; i < 3; ++i)
				end[i] = player.Origin[i] + timeLeft * player.Velocity[i];

			auto tr = traceFunc(player.Origin, end, (player.Ducking) ? HullType::DUCKED : HullType::NORMAL);
			if (fractions)
				fractions[bumpCount] = tr.Fraction;
			if (normalzs)
				normalzs[bumpCount] = tr.PlaneNormal[2];

			allFraction += tr.Fraction;
			if (tr.AllSolid) {
				VecScale<float, 3>(player.Velocity, 0, player.Velocity);
				blockedState = 4;
				break;
			}
			if (tr.Fraction > 0) {
				VecCopy<float, 3>(tr.EndPos, player.Origin);
				VecCopy<float, 3>(player.Velocity, savedVelocity);
				numPlanes = 0;
			}
			if (tr.Fraction == 1)
				break;

			if (tr.PlaneNormal[2] > 0.7)
				blockedState |= 1;
			else if (tr.PlaneNormal[2] == 0)
				blockedState |= 2;

			timeLeft -= timeLeft * tr.Fraction;

			if (numPlanes >= MAX_CLIP_PLANES) {
				VecScale<float, 3>(player.Velocity, 0, player.Velocity);
				break;
			}

			VecCopy<float, 3>(tr.PlaneNormal, planes[numPlanes]);
			numPlanes++;

			if (postype != PositionType::GROUND || vars.EntFriction != 1) {
				for (auto i = 0; i < numPlanes; ++i)
					if (planes[i][2] > 0.7)
						ClipVelocity(savedVelocity, planes[i], 1);
					else
						ClipVelocity(savedVelocity, planes[i], static_cast<float>(1.0 + vars.Bounce * (1 - vars.EntFriction)));

				VecCopy<float, 3>(savedVelocity, player.Velocity);
			} else {
				int i = 0;
				for (i = 0; i < numPlanes; ++i) {
					VecCopy<float, 3>(savedVelocity, player.Velocity);
					ClipVelocity(player.Velocity, planes[i], 1);

					int j;
					for (j = 0; j < numPlanes; ++j)
						if (j != i)
							if (DotProduct<float, float, 3>(player.Velocity, planes[j]) < 0)
								break;

					if (j == numPlanes)
						break;
				}

				if (i == numPlanes) {
					if (numPlanes != 2) {
						VecScale<float, 3>(player.Velocity, 0, player.Velocity);
						break;
					}

					float dir[3];
					CrossProduct<float, float>(planes[0], planes[1], dir);
					auto d = static_cast<float>(DotProduct<float, float, 3>(dir, player.Velocity));
					VecScale<float, 3>(dir, d, player.Velocity);
				}

				if (DotProduct<float, float, 3>(player.Velocity, originalVelocity) <= 0) {
					VecScale<float, 3>(player.Velocity, 0, player.Velocity);
					break;
				}
			}
		}

		if (allFraction == 0)
			VecScale<float, 3>(player.Velocity, 0, player.Velocity);
	}

	int ClipVelocity(float velocty[3], const float normal[3], float overbounce)
	{
		const auto STOP_EPSILON = 0.1;

		auto backoff = static_cast<float>(DotProduct<float, float, 3>(velocty, normal) * overbounce);

		for (size_t i = 0; i < 3; ++i) {
			auto change = normal[i] * backoff;
			velocty[i] -= change;

			if (velocty[i] > -STOP_EPSILON && velocty[i] < STOP_EPSILON)
				velocty[i] = 0;
		}

		if (normal[2] > 0)
			return 1;
		else if (normal[2] == 0)
			return 2;
		else
			return 0;
	}

	static inline double ButtonsPhi(HLTAS::Button button)
	{
		switch (button) {
		case HLTAS::Button::      FORWARD: return 0;
		case HLTAS::Button:: FORWARD_LEFT: return M_PI / 4;
		case HLTAS::Button::         LEFT: return M_PI / 2;
		case HLTAS::Button::    BACK_LEFT: return 3 * M_PI / 4;
		case HLTAS::Button::         BACK: return -M_PI;
		case HLTAS::Button::   BACK_RIGHT: return -3 * M_PI / 4;
		case HLTAS::Button::        RIGHT: return -M_PI / 2;
		case HLTAS::Button::FORWARD_RIGHT: return -M_PI / 4;
		default: return 0;
		}
	}

	static inline HLTAS::Button GetBestButtons(double theta, bool right)
	{
		if (theta < M_PI / 8)
			return HLTAS::Button::FORWARD;
		else if (theta < 3 * M_PI / 8)
			return right ? HLTAS::Button::FORWARD_RIGHT : HLTAS::Button::FORWARD_LEFT;
		else if (theta < 5 * M_PI / 8)
			return right ? HLTAS::Button::RIGHT : HLTAS::Button::LEFT;
		else if (theta < 7 * M_PI / 8)
			return right ? HLTAS::Button::BACK_RIGHT : HLTAS::Button::BACK_LEFT;
		else
			return HLTAS::Button::BACK;
	}

	static void SideStrafeGeneral(const PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed,
		const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, HLTAS::Button& usedButton, double vel_yaw, double theta, bool right, bool safeguard_yaw, float velocities[2][2], double yaws[2])
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

		if (!IsZero<float, 2>(player.Velocity))
			vel_yaw = Atan2(player.Velocity[1], player.Velocity[0]);

		double yaw = vel_yaw - phi + theta;
		yaws[0] = AngleModRad(yaw);
		// Very rare case of yaw == anglemod(yaw).
		if (yaws[0] == yaw) {
			// Multiply by 1.5 because the fp precision might make the yaw a value not enough to reach the next anglemod.
			// Or divide by 2 because it might throw us a value too far back.
			yaws[1] = AngleModRad(yaw + std::copysign(M_U_RAD * 1.5, yaw));

			// We need to handle this when we may have yaw equal to the speed change boundary.
			if (safeguard_yaw)
				yaws[0] = AngleModRad(yaw - std::copysign(M_U_RAD / 2, yaw));
		} else
			yaws[1] = AngleModRad(yaw + std::copysign(M_U_RAD, yaw));

		double avec[2] = { std::cos(yaws[0] + phi), std::sin(yaws[0] + phi) };
		PlayerData pl = player;
		VectorFME(pl, vars, postype, wishspeed, avec);
		VecCopy<float, 2>(pl.Velocity, velocities[0]);

		avec[0] = std::cos(yaws[1] + phi);
		avec[1] = std::sin(yaws[1] + phi);
		VecCopy<float, 2>(player.Velocity, pl.Velocity);
		VectorFME(pl, vars, postype, wishspeed, avec);
		VecCopy<float, 2>(pl.Velocity, velocities[1]);
	}

	double SideStrafeMaxAccel(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, HLTAS::Button& usedButton,
		double vel_yaw, bool right)
	{
		assert(postype != PositionType::WATER);

		double theta = MaxAccelTheta(player, vars, postype, wishspeed);
		float velocities[2][2];
		double yaws[2];
		SideStrafeGeneral(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, usedButton, vel_yaw, theta, right, false, velocities, yaws);

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

	double SideStrafeConstSpeed(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, HLTAS::Button& usedButton,
		double vel_yaw, bool right)
	{
		assert(postype != PositionType::WATER);

		double theta = ConstSpeedTheta(player, vars, postype, wishspeed);
		float velocities[2][2];
		double yaws[2];
		SideStrafeGeneral(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, usedButton, vel_yaw, theta, right, false, velocities, yaws);

		double speedsqrs[2] = {
			DotProduct<float, float, 2>(velocities[0], velocities[0]),
			DotProduct<float, float, 2>(velocities[1], velocities[1])
		};
		double oldspeedsqr = player.SpeedBeforeFriction * player.SpeedBeforeFriction;

		if (std::fabs(speedsqrs[0] - oldspeedsqr) <= std::fabs(speedsqrs[1] - oldspeedsqr)) {
			VecCopy<float, 2>(velocities[0], player.Velocity);
			return yaws[0];
		} else {
			VecCopy<float, 2>(velocities[1], player.Velocity);
			return yaws[1];
		}
	}

	double SideStrafeMaxAngle(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, HLTAS::Button& usedButton,
		double vel_yaw, bool right)
	{
		assert(postype != PositionType::WATER);

		bool safeguard_yaw;
		double theta = MaxAngleTheta(player, vars, postype, wishspeed, safeguard_yaw);
		float velocities[2][2];
		double yaws[2];
		SideStrafeGeneral(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, usedButton, vel_yaw, theta, right, safeguard_yaw, velocities, yaws);

		double old_speed = Length<float, 2>(player.Velocity);
		double speeds[2] = { Length<float, 2>(velocities[0]), Length<float, 2>(velocities[1]) };
		double cosangles[2] = {
			DotProduct<float, float, 2>(velocities[0], player.Velocity) / (old_speed * speeds[0]),
			DotProduct<float, float, 2>(velocities[1], player.Velocity) / (old_speed * speeds[1])
		};

		if (cosangles[0] < cosangles[1]) {
			VecCopy<float, 2>(velocities[0], player.Velocity);
			return yaws[0];
		} else {
			VecCopy<float, 2>(velocities[1], player.Velocity);
			return yaws[1];
		}
	}

	double SideStrafeMaxDeccel(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, HLTAS::Button& usedButton,
		double vel_yaw, bool right, bool& strafed)
	{
		assert(postype != PositionType::WATER);

		// Check a bunch of stuff.
		bool onground = (postype == PositionType::GROUND);
		double speed = Length<float, 2>(player.Velocity);
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
		float velocities[2][2];
		double yaws[2];
		SideStrafeGeneral(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, usedButton, vel_yaw, theta, right, safeguard_yaw, velocities, yaws);

		double speedsqrs[2] = {
			DotProduct<float, float, 2>(velocities[0], velocities[0]),
			DotProduct<float, float, 2>(velocities[1], velocities[1])
		};

		if (speedsqrs[0] < speedsqrs[1]) {
			VecCopy<float, 2>(velocities[0], player.Velocity);
			return yaws[0];
		} else {
			VecCopy<float, 2>(velocities[1], player.Velocity);
			return yaws[1];
		}
	}

	double BestStrafeMaxAccel(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, HLTAS::Button& usedButton,
		double vel_yaw)
	{
		assert(postype != PositionType::WATER);

		float temp_vel[2], orig_vel[2];
		double yaws[2];
		HLTAS::Button buttons[2];
		VecCopy<float, 2>(player.Velocity, orig_vel);
		yaws[0] = SideStrafeMaxAccel(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, buttons[0], vel_yaw, false);
		VecCopy<float, 2>(player.Velocity, temp_vel);
		VecCopy<float, 2>(orig_vel, player.Velocity);
		yaws[1] = SideStrafeMaxAccel(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, buttons[1], vel_yaw, true);

		double speedsqrs[2] = {
			DotProduct<float, float, 2>(temp_vel, temp_vel),
			DotProduct<float, float, 2>(player.Velocity, player.Velocity)
		};

		if (speedsqrs[0] > speedsqrs[1]) {
			VecCopy<float, 2>(temp_vel, player.Velocity);
			usedButton = buttons[0];
			return yaws[0];
		} else {
			usedButton = buttons[1];
			return yaws[1];
		}
	}

	double BestStrafeMaxAngle(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, HLTAS::Button& usedButton,
		double vel_yaw)
	{
		assert(postype != PositionType::WATER);

		float temp_vel[2], orig_vel[2];
		double yaws[2];
		HLTAS::Button buttons[2];
		VecCopy<float, 2>(player.Velocity, orig_vel);
		yaws[0] = SideStrafeMaxAngle(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, buttons[0], vel_yaw, false);
		VecCopy<float, 2>(player.Velocity, temp_vel);
		VecCopy<float, 2>(orig_vel, player.Velocity);
		yaws[1] = SideStrafeMaxAngle(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, buttons[1], vel_yaw, true);

		double old_speed = Length<float, 2>(orig_vel);
		double speeds[2] = { Length<float, 2>(temp_vel), Length<float, 2>(player.Velocity) };
		double cosangles[2] = {
			DotProduct<float, float, 2>(temp_vel, orig_vel) / (old_speed * speeds[0]),
			DotProduct<float, float, 2>(player.Velocity, orig_vel) / (old_speed * speeds[1])
		};

		if (cosangles[0] < cosangles[1]) {
			VecCopy<float, 2>(temp_vel, player.Velocity);
			usedButton = buttons[0];
			return yaws[0];
		} else {
			usedButton = buttons[1];
			return yaws[1];
		}
	}

	double BestStrafeMaxDeccel(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, HLTAS::Button& usedButton,
		double vel_yaw, bool& strafed)
	{
		assert(postype != PositionType::WATER);

		float temp_vel[2], orig_vel[2];
		double yaws[2];
		HLTAS::Button buttons[2];
		VecCopy<float, 2>(player.Velocity, orig_vel);
		yaws[0] = SideStrafeMaxDeccel(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, buttons[0], vel_yaw, false, strafed);
		VecCopy<float, 2>(player.Velocity, temp_vel);
		VecCopy<float, 2>(orig_vel, player.Velocity);
		yaws[1] = SideStrafeMaxDeccel(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, buttons[1], vel_yaw, true, strafed);

		// The condition for strafed does not depend on the strafing direction so
		// either both functions returned true, or both returned false.
		if (!strafed)
			return 0.0;

		double speedsqrs[2] = {
			DotProduct<float, float, 2>(temp_vel, temp_vel),
			DotProduct<float, float, 2>(player.Velocity, player.Velocity)
		};

		if (speedsqrs[0] < speedsqrs[1]) {
			VecCopy<float, 2>(temp_vel, player.Velocity);
			usedButton = buttons[0];
			return yaws[0];
		} else {
			usedButton = buttons[1];
			return yaws[1];
		}
	}

	double YawStrafeMaxAccel(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, HLTAS::Button& usedButton,
		double vel_yaw, double yaw)
	{
		assert(postype != PositionType::WATER);

		double theta = MaxAccelIntoYawTheta(player, vars, postype, wishspeed, vel_yaw, yaw);
		float velocities[2][2];
		double yaws[2];
		SideStrafeGeneral(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, usedButton, vel_yaw, std::fabs(theta), (theta < 0), false, velocities, yaws);

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

	double YawStrafeMaxAngle(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, HLTAS::Button& usedButton,
		double vel_yaw, double yaw)
	{
		assert(postype != PositionType::WATER);

		bool safeguard_yaw;
		double theta = MaxAngleTheta(player, vars, postype, wishspeed, safeguard_yaw);
		float velocities[2][2];
		double yaws[2];
		if (!IsZero<float, 2>(player.Velocity))
			vel_yaw = Atan2(player.Velocity[1], player.Velocity[0]);
		SideStrafeGeneral(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, usedButton, vel_yaw, theta, (NormalizeRad(yaw - vel_yaw) < 0), safeguard_yaw, velocities, yaws);

		double old_speed = Length<float, 2>(player.Velocity);
		double speeds[2] = { Length<float, 2>(velocities[0]), Length<float, 2>(velocities[1]) };
		double cosangles[2] = {
			DotProduct<float, float, 2>(velocities[0], player.Velocity) / (old_speed * speeds[0]),
			DotProduct<float, float, 2>(velocities[1], player.Velocity) / (old_speed * speeds[1])
		};

		if (cosangles[0] < cosangles[1]) {
			VecCopy<float, 2>(velocities[0], player.Velocity);
			return yaws[0];
		} else {
			VecCopy<float, 2>(velocities[1], player.Velocity);
			return yaws[1];
		}
	}

	double YawStrafeMaxDeccel(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, HLTAS::Button& usedButton,
		double vel_yaw, double yaw, bool& strafed)
	{
		assert(postype != PositionType::WATER);

		// Check a bunch of stuff.
		bool onground = (postype == PositionType::GROUND);
		double speed = Length<float, 2>(player.Velocity);
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
		float velocities[2][2];
		double yaws[2];
		vel_yaw = Atan2(player.Velocity[1], player.Velocity[0]);
		SideStrafeGeneral(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, usedButton, vel_yaw, theta, (NormalizeRad(yaw - vel_yaw) < 0), safeguard_yaw, velocities, yaws);

		double speedsqrs[2] = {
			DotProduct<float, float, 2>(velocities[0], velocities[0]),
			DotProduct<float, float, 2>(velocities[1], velocities[1])
		};

		if (speedsqrs[0] < speedsqrs[1]) {
			VecCopy<float, 2>(velocities[0], player.Velocity);
			return yaws[0];
		} else {
			VecCopy<float, 2>(velocities[1], player.Velocity);
			return yaws[1];
		}
	}

	double PointStrafe(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, HLTAS::Button& usedButton,
		double vel_yaw, HLTAS::StrafeType type, double point[2], bool& strafed)
	{
		assert(postype != PositionType::WATER);

		// Covers the case where both vectors are zero.
		if (Distance<float, double, 2>(player.Origin, point) <= 2.0) {
			strafed = false;
			return 0.0;
		}
		strafed = true;

		double difference[2];
		VecSubtract<double, float, 2>(point, player.Origin, difference);
		double yaw = Atan2(difference[1], difference[0]);

		switch (type) {
		default:
		case HLTAS::StrafeType::MAXACCEL: return YawStrafeMaxAccel(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, usedButton, vel_yaw, yaw);
		case HLTAS::StrafeType::MAXANGLE: return YawStrafeMaxAngle(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, usedButton, vel_yaw, yaw);
		case HLTAS::StrafeType::MAXDECCEL: return YawStrafeMaxDeccel(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, usedButton, vel_yaw, yaw, strafed);
		// TODO add the rest of the calls when the functions are done.
		}
	}

	PositionType GetPositionType(PlayerData& player, TraceFunc traceFunc)
	{
		// TODO: Check water. If we're under water, return here.

		// Check ground.
		if (player.Velocity[2] > 180)
			return PositionType::AIR;

		float point[3];
		VecCopy<float, 3>(player.Origin, point);
		point[2] -= 2;

		auto tr = traceFunc(player.Origin, point, (player.Ducking) ? HullType::DUCKED : HullType::NORMAL);
		if (tr.PlaneNormal[2] < 0.7 || tr.Entity == -1)
			return PositionType::AIR;

		if (!tr.StartSolid && !tr.AllSolid)
			VecCopy<float, 3>(tr.EndPos, player.Origin);
		return PositionType::GROUND;
	}

	void CheckVelocity(PlayerData& player, const MovementVars& vars)
	{
		for (std::size_t i = 0; i < 3; ++i) {
			if (player.Velocity[i] > vars.Maxvelocity)
				player.Velocity[i] = vars.Maxvelocity;
			if (player.Velocity[i] < -vars.Maxvelocity)
				player.Velocity[i] = -vars.Maxvelocity;
		}
	}

	PositionType PredictDuck(PlayerData& player, PositionType postype, const MovementVars& vars, const CurrentState& curState, const ProcessedFrame& out, TraceFunc traceFunc)
	{
		if (!out.Duck
			&& !player.InDuckAnimation
			&& !player.Ducking)
			return postype;

		if (out.Duck) {
			if (!curState.Duck && !player.Ducking) {
				player.DuckTime = 1000;
				player.InDuckAnimation = true;
			}

			if (player.InDuckAnimation
				&& (player.DuckTime / 1000.0 <= (1.0 - 0.4)
					|| postype != PositionType::GROUND)) {
				player.Ducking = true;
				player.InDuckAnimation = false;
				if (postype == PositionType::GROUND) {
					for (std::size_t i = 0; i < 3; ++i)
						player.Origin[i] -= (VEC_DUCK_HULL_MIN[i] - VEC_HULL_MIN[i]);
					// Is PM_FixPlayerCrouchStuck() prediction needed here?
					return GetPositionType(player, traceFunc);
				}
			}
		} else {
			// Unduck.
			float newOrigin[3];
			VecCopy<float, 3>(player.Origin, newOrigin);
			if (postype == PositionType::GROUND)
				for (std::size_t i = 0; i < 3; ++i)
					newOrigin[i] += (VEC_DUCK_HULL_MIN[i] - VEC_HULL_MIN[i]);

			auto tr = traceFunc(newOrigin, newOrigin, (player.Ducking) ? HullType::DUCKED : HullType::NORMAL);
			if (!tr.StartSolid) {
				tr = traceFunc(newOrigin, newOrigin, HullType::NORMAL);
				if (!tr.StartSolid) {
					player.Ducking = false;
					player.InDuckAnimation = false;
					player.DuckTime = 0;
					VecCopy<float, 3>(newOrigin, player.Origin);
					return GetPositionType(player, traceFunc);
				}
			}
		}

		return postype;
	}

	PositionType PredictJump(PlayerData& player, PositionType postype, const MovementVars& vars, const CurrentState& curState, ProcessedFrame& out)
	{
		assert(postype != PositionType::WATER);

		if (!out.Jump // Not pressing Jump.
			|| curState.Jump // Jump was already down.
			|| postype != PositionType::GROUND) // Not on ground.
			return postype;

		if (vars.Bhopcap) {
			auto maxscaledspeed = 1.2f * vars.Maxspeed;
			if (maxscaledspeed > 0) {
				auto speed = Length<float, 3>(player.Velocity);
				if (speed > maxscaledspeed)
					VecScale<float, 3>(player.Velocity, (maxscaledspeed / speed) * 0.8, player.Velocity);
			}
		}

		uint32_t temp = 1151629635;
		player.StaminaTime = *reinterpret_cast<float*>(&temp);

		// TODO: duck-when autofuncs.
		// We don't care about the vertical velocity after the jump prediction.
		/*if (player.InDuckAnimation || player.Ducking) {
			if (player.HasLJModule && out.Duck && player.DuckTime > 0 && Length<float, 3>(player.Velocity) > 50) {
				player.Velocity[0] = static_cast<float>(std::cos(player.Viewangles[1] * M_DEG2RAD) * std::cos(player.Viewangles[0] * M_DEG2RAD) * 350 * 1.6);
				player.Velocity[1] = static_cast<float>(std::sin(player.Viewangles[1] * M_DEG2RAD) * std::cos(player.Viewangles[0] * M_DEG2RAD) * 350 * 1.6);
			}
		}*/
		CheckVelocity(player, vars);

		return PositionType::AIR;
	}

	void Friction(PlayerData& player, PositionType postype, const MovementVars& vars, TraceFunc traceFunc)
	{
		player.SpeedBeforeFriction = Length<float, 2>(player.Velocity);

		if (postype != PositionType::GROUND)
			return;

		// Doing all this in floats, mismatch is too real otherwise.
		auto speed = static_cast<float>( std::sqrt(static_cast<double>(player.Velocity[0] * player.Velocity[0] + player.Velocity[1] * player.Velocity[1] + player.Velocity[2] * player.Velocity[2])) );
		if (speed < 0.1)
			return;

		auto friction = float{ vars.Friction * vars.EntFriction };

		float start[3], stop[3];
		start[0] = static_cast<float>(player.Origin[0] + player.Velocity[0] / speed * 16);
		start[1] = static_cast<float>(player.Origin[1] + player.Velocity[1] / speed * 16);
		start[2] = player.Origin[2] + ((player.Ducking) ? VEC_DUCK_HULL_MIN[2] : VEC_HULL_MIN[2]);
		VecCopy<float, 3>(start, stop);
		stop[2] -= 34;

		auto tr = traceFunc(start, stop, (player.Ducking) ? HullType::DUCKED : HullType::NORMAL);
		if (tr.Fraction == 1.0)
			friction *= vars.Edgefriction;

		auto control = (speed < vars.Stopspeed) ? vars.Stopspeed : speed;
		auto drop = control * friction * vars.Frametime;
		auto newspeed = std::max(speed - drop, 0.f);
		VecScale<float, 3>(player.Velocity, newspeed / speed, player.Velocity);

		// WalkMove
		VecScale<float, 2>(player.Velocity, (100 - player.StaminaTime / 1000 * 19) / 100, player.Velocity);
		// Reduce this here since if we're calling Friction then we're predicting something
		// and most likely will call Friction again and this needs to be correct.
		player.StaminaTime = std::max(player.StaminaTime - static_cast<int>(vars.Frametime * 1000), 0.f);
	}

	void Ducktap(const PlayerData& player, PositionType postype, const HLTAS::Frame& frame, CurrentState& curState, ProcessedFrame& out, TraceFunc traceFunc)
	{
		assert(postype != PositionType::WATER);

		if ((frame.Lgagst || curState.LgagstsLeft) || (!frame.Ducktap && !curState.DucktapsLeft))
			return;
		if (postype != PositionType::GROUND)
			return;

		// Allow ducktapping with pressed duck - release duck if we're on ground and can unduck.
		// Otherwise we don't want to handle situations where the player is ducking.
		if (player.Ducking) {
			float newOrigin[3];
			VecCopy<float, 3>(player.Origin, newOrigin);
			for (std::size_t i = 0; i < 3; ++i)
				newOrigin[i] += (VEC_DUCK_HULL_MIN[i] - VEC_HULL_MIN[i]);

			auto tr = traceFunc(newOrigin, newOrigin, (player.Ducking) ? HullType::DUCKED : HullType::NORMAL);
			if (!tr.StartSolid) {
				tr = traceFunc(newOrigin, newOrigin, HullType::NORMAL);
				if (!tr.StartSolid)
					out.Duck = false;
			}
			
			return;
		}

		if (player.InDuckAnimation) {
			out.Duck = false;
			if (curState.DucktapsLeft)
				curState.DucktapsLeft--;
		} else {
			// TODO: this should check against the next frame's origin, so this needs a call to Strafe() and stuff.
			float newOrigin[3];
			VecCopy<float, 3>(player.Origin, newOrigin);
			for (std::size_t i = 0; i < 3; ++i)
				newOrigin[i] += (VEC_DUCK_HULL_MIN[i] - VEC_HULL_MIN[i]);

			auto tr = traceFunc(newOrigin, newOrigin, HullType::NORMAL);
			if (!tr.StartSolid)
				out.Duck = true;
		}
	}

	void Jumpbug(const PlayerData& player, const MovementVars& vars, PositionType postype, const HLTAS::Frame& frame, ProcessedFrame& out, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, CurrentState& curState, TraceFunc traceFunc)
	{
		assert(postype != PositionType::WATER);

		if ((!frame.Jumpbug && !curState.JumpbugsLeft) || postype == PositionType::GROUND)
			return;

		auto playerCopy = PlayerData(player);
		auto outCopy = ProcessedFrame(out);
		if (player.Ducking) {
			outCopy.Duck = false;
			postype = PredictDuck(playerCopy, postype, vars, curState, outCopy, traceFunc);

			// If we're still ducking, then it is too late to jumpbug. Prepare for the inevitable.
			if (playerCopy.Ducking)
				return;

			// If after unducking we end up on the ground, jumpbug now!
			if (postype == PositionType::GROUND) {
				if ((!out.Jump && !curState.Jump) || (out.Duck && curState.Duck)) {
					out.Jump = true;
					out.Duck = false;
					if (curState.JumpbugsLeft)
						curState.JumpbugsLeft--;
				}
				return;
			}
		}

		postype = Strafe(playerCopy, vars, postype, frame, outCopy, false, strafeButtons, useGivenButtons, true, traceFunc);
		if (postype == PositionType::GROUND) {
			// Duck now so that in the next frame we can do the jumpbug.
			out.Duck = true;
			out.Jump = false;
		}
	}

	void Autojump(PositionType postype, const HLTAS::Frame& frame, CurrentState& curState, ProcessedFrame& out)
	{
		assert(postype != PositionType::WATER);

		if ((frame.Lgagst || curState.LgagstsLeft) || (!frame.Autojump && !curState.AutojumpsLeft))
			return;

		if (postype == PositionType::GROUND && !curState.Jump && !out.Jump) {
			out.Jump = true;
			if (curState.AutojumpsLeft)
				curState.AutojumpsLeft--;
		}
	}

	void Dbc(const PlayerData& player, const MovementVars& vars, PositionType postype, const HLTAS::Frame& frame, ProcessedFrame& out, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, CurrentState& curState, TraceFunc traceFunc)
	{
		assert(postype != PositionType::WATER);

		if ((!frame.Dbc && !curState.DbcsLeft)
			|| postype == PositionType::GROUND
			|| out.Duck)
			return;

		// Default to not ceiling.
		float normalzsUnducked[4] = { 0, 0, 0, 0 };
		// tr.Fraction can never be this high.
		float fractionsUnducked[4] = { 2, 2, 2, 2 };
		float fractionsDucked[4] =   { 2, 2, 2, 2 };

		auto playerCopy = PlayerData(player);
		auto out_temp = ProcessedFrame(out);
		postype = PredictDuck(playerCopy, postype, vars, curState, out, traceFunc);
		// If we cannot unduck, there's nothing we can change.
		if (playerCopy.Ducking)
			return;

		// If we unducked and ended up on ground, then we don't care about this situation.
		if (postype == PositionType::GROUND)
			return;

		Strafe(playerCopy, vars, postype, frame, out_temp, false, strafeButtons, useGivenButtons, true, traceFunc, fractionsUnducked, normalzsUnducked);

		playerCopy = PlayerData(player);
		out_temp = ProcessedFrame(out);
		out_temp.Duck = true;
		postype = PredictDuck(playerCopy, postype, vars, curState, out_temp, traceFunc);
		Strafe(playerCopy, vars, postype, frame, out_temp, false, strafeButtons, useGivenButtons, true, traceFunc, fractionsDucked);

		for (int i = 0; i < 4; ++i) {
			// Don't dbc if we encountered a ground plane first. Let dbg handle it.
			if (normalzsUnducked[i] >= 0.7f)
				break;

			if (fractionsDucked[i] > fractionsUnducked[i]) {
				if (normalzsUnducked[i] == -1.0f && !curState.DbcCeilings) {
					// EngineMsg("Not Dbcing because ceiling not set.\n");
					break;
				}
				// EngineMsg("Dbc-ing\n");
				out.Duck = true;
				if (curState.DbcsLeft)
					curState.DbcsLeft--;
				break;
			} else if (fractionsDucked[i] < fractionsUnducked[i]) {
				break; // Don't dbc.
			}
		}
	}

	void Dbg(const PlayerData& player, const MovementVars& vars, PositionType postype, const HLTAS::Frame& frame, ProcessedFrame& out, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, CurrentState& curState, TraceFunc traceFunc)
	{
		assert(postype != PositionType::WATER);

		if ((!frame.Dbg && !curState.DbgsLeft)
			|| postype == PositionType::GROUND
			|| out.Duck)
			return;

		// Default to not ground.
		float normalzsUnducked[4] = { 0, 0, 0, 0 };

		auto playerCopy = PlayerData(player);
		auto out_temp = ProcessedFrame(out);
		postype = PredictDuck(playerCopy, postype, vars, curState, out_temp, traceFunc);
		// If we cannot unduck, there's nothing we can change.
		if (playerCopy.Ducking)
			return;

		// If we unducked and ended up on ground, keep ducking.
		if (postype == PositionType::GROUND) {
			// EngineMsg("Dbg [unduck]!\n");
			out.Duck = true;
			if (curState.DbgsLeft)
				curState.DbgsLeft--;
			return;
		}

		postype = Strafe(playerCopy, vars, postype, frame, out_temp, false, strafeButtons, useGivenButtons, true, traceFunc, nullptr, normalzsUnducked);
		// If we moved and ended up on ground, dbg.
		if (postype == PositionType::GROUND) {
			// EngineMsg("Dbg!\n");
			out.Duck = true;
			if (curState.DbgsLeft)
				curState.DbgsLeft--;
			return;
		}

		// Otherwise, check if we hit a ground plane along the way.
		// If so, dbg.
		for (int i = 0; i < 4; ++i) {
			if (normalzsUnducked[i] >= 0.7f) {
				// EngineMsg("Dbg [normal]!\n");
				out.Duck = true;
				if (curState.DbgsLeft)
					curState.DbgsLeft--;
				return;
			}
		}
	}

	void LgagstDucktap(const PlayerData& player, const MovementVars& vars, PositionType postype, const HLTAS::Frame& frame, ProcessedFrame& out, bool reduceWishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, CurrentState& curState, TraceFunc traceFunc)
	{
		assert(postype != PositionType::WATER);

		if ((!frame.Lgagst && !curState.LgagstsLeft) || !curState.LgagstType // Lgagst not enabled or LgagstAutojump.
			|| !frame.Strafe || frame.GetType() != HLTAS::StrafeType::MAXACCEL // Not maxaccel strafing.
			|| postype != PositionType::GROUND || player.Ducking || player.InDuckAnimation // Not on ground, ducking or in duck animation.
			|| out.Duck
			|| Length<float, 2>(player.Velocity) < curState.LgagstMinSpeed)
			return;

		// Predict the next frame's origin.
		auto playerCopy = PlayerData(player);
		Friction(playerCopy, postype, vars, traceFunc);
		auto out_temp = ProcessedFrame(out);
		Strafe(playerCopy, vars, postype, frame, out_temp, reduceWishspeed, strafeButtons, useGivenButtons, true, traceFunc);

		// Check if we can ducktap there.
		float newOrigin[3];
		VecCopy<float, 3>(playerCopy.Origin, newOrigin);
		for (std::size_t i = 0; i < 3; ++i)
			newOrigin[i] += (VEC_DUCK_HULL_MIN[i] - VEC_HULL_MIN[i]);
		auto tr = traceFunc(newOrigin, newOrigin, HullType::NORMAL);
		if (tr.StartSolid)
			return;

		// Do the actual lgagst check.
		auto ground = PlayerData(playerCopy);
		Friction(ground, postype, vars, traceFunc);
		CheckVelocity(ground, vars);
		out_temp = ProcessedFrame(out);
		Strafe(ground, vars, postype, frame, out_temp, false, strafeButtons, useGivenButtons, false, traceFunc);

		auto air = PlayerData(playerCopy);
		out_temp = ProcessedFrame(out);
		air.InDuckAnimation = true;
		postype = PredictDuck(air, postype, vars, curState, out_temp, traceFunc);
		Strafe(air, vars, postype, frame, out_temp, false, strafeButtons, useGivenButtons, false, traceFunc);

		auto l_gr = Length<float, 2>(ground.Velocity);
		auto l_air = Length<float, 2>(air.Velocity);
		if (l_air > l_gr) {
			out.Duck = true;
			if (curState.LgagstsLeft)
				curState.LgagstsLeft--;
		}
	}

	void LgagstJump(const PlayerData& player, const MovementVars& vars, PositionType postype, const HLTAS::Frame& frame, ProcessedFrame& out, bool reduceWishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, CurrentState& curState, TraceFunc traceFunc)
	{
		assert(postype != PositionType::WATER);

		if ((!frame.Lgagst && !curState.LgagstsLeft) || curState.LgagstType // Lgagst not enabled or LgagstDucktap.
			|| postype != PositionType::GROUND || curState.Jump || out.Jump // Cannot jump / already jumped.
			|| !frame.Strafe || frame.GetType() != HLTAS::StrafeType::MAXACCEL // Not maxaccel strafing.
			|| Length<float, 2>(player.Velocity) < curState.LgagstMinSpeed)
			return;

		auto ground = PlayerData(player);
		Friction(ground, postype, vars, traceFunc);
		CheckVelocity(ground, vars);
		auto out_temp = ProcessedFrame(out);
		Strafe(ground, vars, postype, frame, out_temp, reduceWishspeed && !curState.LgagstFullMaxspeed, strafeButtons, useGivenButtons, false, traceFunc);

		auto air = PlayerData(player);
		out_temp = ProcessedFrame(out);
		out_temp.Jump = true;
		postype = PredictJump(air, postype, vars, curState, out_temp);
		Strafe(air, vars, postype, frame, out_temp, reduceWishspeed && !curState.LgagstFullMaxspeed, strafeButtons, useGivenButtons, false, traceFunc);

		auto l_gr = Length<float, 2>(ground.Velocity);
		auto l_air = Length<float, 2>(air.Velocity);
		if (l_air > l_gr) {
			out.Jump = true;
			if (curState.LgagstsLeft)
				curState.LgagstsLeft--;
		}
	}

	PositionType Strafe(PlayerData& player, const MovementVars& vars, PositionType postype, const HLTAS::Frame& frame, ProcessedFrame& out, bool reduceWishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, bool predictOrigin, TraceFunc traceFunc, float fractions[4], float normalzs[4])
	{
		double wishspeed = vars.Maxspeed;
		if (reduceWishspeed)
			wishspeed *= 0.333;

		bool strafed = false;
		HLTAS::Button usedButton;
		if (frame.Strafe) {
			strafed = true;

			switch (frame.GetDir()) {
			case HLTAS::StrafeDir::LEFT:
				if (frame.GetType() == HLTAS::StrafeType::MAXACCEL)
					out.Yaw = static_cast<float>(SideStrafeMaxAccel(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, usedButton, out.Yaw * M_DEG2RAD, false) * M_RAD2DEG);
				else if (frame.GetType() == HLTAS::StrafeType::MAXANGLE)
					out.Yaw = static_cast<float>(SideStrafeMaxAngle(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, usedButton, out.Yaw * M_DEG2RAD, false) * M_RAD2DEG);
				else if (frame.GetType() == HLTAS::StrafeType::CONSTSPEED)
					out.Yaw = static_cast<float>(SideStrafeConstSpeed(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, usedButton, out.Yaw * M_DEG2RAD, false) * M_RAD2DEG);
				else if (frame.GetType() == HLTAS::StrafeType::MAXDECCEL) {
					auto yaw = static_cast<float>(SideStrafeMaxDeccel(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, usedButton, out.Yaw * M_DEG2RAD, false, strafed) * M_RAD2DEG);
					if (strafed)
						out.Yaw = yaw;
				}
				break;

			case HLTAS::StrafeDir::RIGHT:
				if (frame.GetType() == HLTAS::StrafeType::MAXACCEL)
					out.Yaw = static_cast<float>(SideStrafeMaxAccel(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, usedButton, out.Yaw * M_DEG2RAD, true) * M_RAD2DEG);
				else if (frame.GetType() == HLTAS::StrafeType::MAXANGLE)
					out.Yaw = static_cast<float>(SideStrafeMaxAngle(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, usedButton, out.Yaw * M_DEG2RAD, true) * M_RAD2DEG);
				else if (frame.GetType() == HLTAS::StrafeType::CONSTSPEED)
					out.Yaw = static_cast<float>(SideStrafeConstSpeed(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, usedButton, out.Yaw * M_DEG2RAD, true) * M_RAD2DEG);
				else if (frame.GetType() == HLTAS::StrafeType::MAXDECCEL) {
					auto yaw = static_cast<float>(SideStrafeMaxDeccel(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, usedButton, out.Yaw * M_DEG2RAD, true, strafed) * M_RAD2DEG);
					if (strafed)
						out.Yaw = yaw;
				}
				break;

			case HLTAS::StrafeDir::BEST:
				if (frame.GetType() == HLTAS::StrafeType::MAXACCEL)
					out.Yaw = static_cast<float>(BestStrafeMaxAccel(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, usedButton, out.Yaw * M_DEG2RAD) * M_RAD2DEG);
				else if (frame.GetType() == HLTAS::StrafeType::MAXANGLE)
					out.Yaw = static_cast<float>(BestStrafeMaxAngle(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, usedButton, out.Yaw * M_DEG2RAD) * M_RAD2DEG);
				else if (frame.GetType() == HLTAS::StrafeType::MAXDECCEL) {
					auto yaw = static_cast<float>(BestStrafeMaxDeccel(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, usedButton, out.Yaw * M_DEG2RAD, strafed) * M_RAD2DEG);
					if (strafed)
						out.Yaw = yaw;
				}
				break;

			case HLTAS::StrafeDir::YAW:
				if (frame.GetType() == HLTAS::StrafeType::MAXACCEL)
					out.Yaw = static_cast<float>(YawStrafeMaxAccel(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, usedButton, out.Yaw * M_DEG2RAD, frame.GetYaw() * M_DEG2RAD) * M_RAD2DEG);
				else if (frame.GetType() == HLTAS::StrafeType::MAXANGLE)
					out.Yaw = static_cast<float>(YawStrafeMaxAngle(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, usedButton, out.Yaw * M_DEG2RAD, frame.GetYaw() * M_DEG2RAD) * M_RAD2DEG);
				else if (frame.GetType() == HLTAS::StrafeType::MAXDECCEL) {
					auto yaw = static_cast<float>(YawStrafeMaxDeccel(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, usedButton, out.Yaw * M_DEG2RAD, frame.GetYaw() * M_DEG2RAD, strafed) * M_RAD2DEG);
					if (strafed)
						out.Yaw = yaw;
				}
				break;

			case HLTAS::StrafeDir::POINT:
			{
				double point[] = { frame.GetX(), frame.GetY() };
				auto yaw = static_cast<float>(PointStrafe(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, usedButton, out.Yaw * M_DEG2RAD, frame.GetType(), point, strafed) * M_RAD2DEG);
				if (strafed)
					out.Yaw = yaw;
			}
				break;

			default:
				strafed = false;
				break;
			}
		}

		double a[2];
		if (strafed) {
			out.Forward = (usedButton == HLTAS::Button::FORWARD || usedButton == HLTAS::Button::FORWARD_LEFT  || usedButton == HLTAS::Button::FORWARD_RIGHT);
			out.Back =    (usedButton == HLTAS::Button::BACK    || usedButton == HLTAS::Button::BACK_LEFT     || usedButton == HLTAS::Button::BACK_RIGHT);
			out.Right =   (usedButton == HLTAS::Button::RIGHT   || usedButton == HLTAS::Button::FORWARD_RIGHT || usedButton == HLTAS::Button::BACK_RIGHT);
			out.Left =    (usedButton == HLTAS::Button::LEFT    || usedButton == HLTAS::Button::FORWARD_LEFT  || usedButton == HLTAS::Button::BACK_LEFT);
		} else {
			auto forwardmove = out.Forward * out.Forwardspeed - out.Back * out.Backspeed;
			auto sidemove = out.Right * out.Sidespeed - out.Left * out.Sidespeed;
			auto cy = std::cos(out.Yaw * M_DEG2RAD);
			auto sy = std::sin(out.Yaw * M_DEG2RAD);
			double wishvel[] = { cy * forwardmove + sy * sidemove, sy * forwardmove - cy * sidemove }; // TODO: consider pitch & roll.
			if (IsZero<double, 2>(wishvel)) {
				a[0] = 0;
				a[1] = 0;
			} else {
				Normalize<double, 2>(wishvel, a);
			}

			if (!predictOrigin)
				VectorFME(player, vars, postype, wishspeed, a);
		}

		if (predictOrigin)
			postype = Move(player, vars, postype, wishspeed, traceFunc, !strafed, a, fractions, normalzs);

		return postype;
	}

	ProcessedFrame MainFunc(const PlayerData& player, const MovementVars& vars, const HLTAS::Frame& frame, CurrentState& curState, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, TraceFunc traceFunc)
	{
		auto out = ProcessedFrame();

		out.Pitch = player.Viewangles[0];
		out.Yaw = player.Viewangles[1];

		out.Forward = frame.Forward;
		out.Left = frame.Left;
		out.Right = frame.Right;
		out.Back = frame.Back;
		out.Up = frame.Up;
		out.Down = frame.Down;

		out.Jump = frame.Jump;
		out.Duck = frame.Duck;
		out.Use = frame.Use;
		out.Attack1 = frame.Attack1;
		out.Attack2 = frame.Attack2;
		out.Reload = frame.Reload;

		out.Forwardspeed = vars.Maxspeed;
		out.Sidespeed = vars.Maxspeed;
		out.Backspeed = vars.Maxspeed;
		out.Upspeed = vars.Maxspeed;

		if (frame.PitchPresent)
			out.Pitch = static_cast<float>(frame.GetPitch());
		if (!frame.Strafe && frame.GetYawPresent())
			out.Yaw = static_cast<float>(AngleModDeg(frame.GetYaw()));

		if (frame.Autojump)
			curState.AutojumpsLeft = frame.GetAutojumpTimes();
		if (frame.Ducktap)
			curState.DucktapsLeft = frame.GetDucktapTimes();
		if (frame.Dbc) {
			curState.DbcCeilings = frame.GetDbcCeilings();
			curState.DbcsLeft = frame.GetDbcTimes();
		}
		if (frame.Dbg)
			curState.DbgsLeft = frame.GetDbgTimes();
		if (frame.Lgagst) {
			curState.LgagstFullMaxspeed = frame.GetLgagstFullMaxspeed();
			curState.LgagstType = frame.Ducktap;
			curState.LgagstsLeft = frame.GetLgagstTimes();
		}

		//EngineMsg("p pr %f\t%f\t%f\t%f\t%f\t%f\n", player.Origin[0], player.Origin[1], player.Origin[2], player.Velocity[0], player.Velocity[1], player.Velocity[2]);
		auto playerCopy = PlayerData(player); // Our copy that we will mess with.
		auto postype = GetPositionType(playerCopy, traceFunc);
		if (postype == PositionType::WATER)
			return out;

		bool reduceWishspeed = playerCopy.Ducking || playerCopy.InDuckAnimation;
		// Same as in ReduceTimers().
		playerCopy.DuckTime = std::max(playerCopy.DuckTime - static_cast<int>(vars.Frametime * 1000), 0.f);
		playerCopy.StaminaTime = std::max(playerCopy.StaminaTime - static_cast<int>(vars.Frametime * 1000), 0.f);

		// This order may change.
		Jumpbug(playerCopy, vars, postype, frame, out, strafeButtons, useGivenButtons, curState, traceFunc);
		Dbc(playerCopy, vars, postype, frame, out, strafeButtons, useGivenButtons, curState, traceFunc);
		Dbg(playerCopy, vars, postype, frame, out, strafeButtons, useGivenButtons, curState, traceFunc);
		LgagstDucktap(playerCopy, vars, postype, frame, out, reduceWishspeed, strafeButtons, useGivenButtons, curState, traceFunc);
		Ducktap(playerCopy, postype, frame, curState, out, traceFunc);
		postype = PredictDuck(playerCopy, postype, vars, curState, out, traceFunc);

		// This has to be after PredictDuck() since we may have ducktapped,
		// if this messes with Lgagst-ducktap() then it's the user's problem.
		// (Though it shouldn't, generally).
		if (out.Use && postype == PositionType::GROUND)
			VecScale<float, 3>(playerCopy.Velocity, 0.3, playerCopy.Velocity);

		LgagstJump(playerCopy, vars, postype, frame, out, reduceWishspeed, strafeButtons, useGivenButtons, curState, traceFunc);
		Autojump(postype, frame, curState, out);
		postype = PredictJump(playerCopy, postype, vars, curState, out);
		Friction(playerCopy, postype, vars, traceFunc);
		CheckVelocity(playerCopy, vars);
		postype = Strafe(playerCopy, vars, postype, frame, out, reduceWishspeed, strafeButtons, useGivenButtons, true, traceFunc);

		//EngineMsg("p po %f\t%f\t%f\t%f\t%f\t%f\n", playerCopy.Origin[0], playerCopy.Origin[1], playerCopy.Origin[2], playerCopy.Velocity[0], playerCopy.Velocity[1], playerCopy.Velocity[2]);
		curState.Jump = out.Jump;
		curState.Duck = out.Duck;
		return out;
	}

	double GetAngleDifference(float oldang, float newang)
	{
		return NormalizeDeg(static_cast<double>(newang) - oldang);
	}

	std::string GetAngleSpeedString(float oldpitch, float oldyaw, float newpitch, float newyaw, double pitchStateMultiplier, double yawStateMultiplier, float frametime)
	{
		std::ostringstream ss;
		ss.setf(std::ios::fixed, std::ios::floatfield);
		ss.precision(std::numeric_limits<double>::digits10);

		if (newpitch != oldpitch) {
			double pitchDifference = std::abs(GetAngleDifference(oldpitch, newpitch));
			double pitchspeed = (pitchDifference / frametime) / pitchStateMultiplier;
			ss << "cl_pitchspeed " << pitchspeed << '\n';
		}

		if (newyaw != oldyaw) {
			auto angleDifference = GetAngleDifference(oldyaw, newyaw);

			auto newyawIsNegative = (oldyaw + angleDifference < 0.0);
			auto difIsNegative = (angleDifference < 0.0);
			auto add = (difIsNegative && newyawIsNegative) || (!difIsNegative && !newyawIsNegative);

			auto yawDifference = std::abs(angleDifference) + (add ? M_U_DEG_HALF : -M_U_DEG_HALF);
			auto yawspeed = (yawDifference / frametime) / yawStateMultiplier;
			ss << "cl_yawspeed " << yawspeed << '\n';
		}

		return ss.str();
	}
}
