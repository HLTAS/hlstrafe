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

	inline double ButtonsPhi(HLTAS::Button button)
	{
		switch (button) {
		case HLTAS::Button::      FORWARD: return 0;
		case HLTAS::Button:: FORWARD_LEFT: return M_PI / 4;
		case HLTAS::Button::         LEFT: return M_PI / 2;
		case HLTAS::Button::    BACK_LEFT: return 3 * M_PI / 2;
		case HLTAS::Button::         BACK: return -M_PI;
		case HLTAS::Button::   BACK_RIGHT: return -3 * M_PI / 2;
		case HLTAS::Button::        RIGHT: return -M_PI / 2;
		case HLTAS::Button::FORWARD_RIGHT: return -M_PI / 4;
		default: return 0;
		}
	}

	static void SideStrafeGeneral(const PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed,
		const HLTAS::StrafeButtons& strafeButtons, double vel_yaw, double theta, bool right, bool safeguard_yaw, float velocities[2][2], double yaws[2])
	{
		assert(postype != PositionType::WATER);

		HLTAS::Button buttons;
		if (postype == PositionType::AIR) {
			if (right)
				buttons = strafeButtons.AirRight;
			else
				buttons = strafeButtons.AirLeft;
		} else {
			if (right)
				buttons = strafeButtons.GroundRight;
			else
				buttons = strafeButtons.GroundLeft;
		}
		double phi = ButtonsPhi(buttons);
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

	double SideStrafeMaxAccel(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons,
		double vel_yaw, bool right)
	{
		assert(postype != PositionType::WATER);

		double theta = MaxAccelTheta(player, vars, postype, wishspeed);
		float velocities[2][2];
		double yaws[2];
		SideStrafeGeneral(player, vars, postype, wishspeed, strafeButtons, vel_yaw, theta, right, false, velocities, yaws);

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

	double SideStrafeMaxAngle(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons,
		double vel_yaw, bool right)
	{
		assert(postype != PositionType::WATER);

		bool safeguard_yaw;
		double theta = MaxAngleTheta(player, vars, postype, wishspeed, safeguard_yaw);
		float velocities[2][2];
		double yaws[2];
		SideStrafeGeneral(player, vars, postype, wishspeed, strafeButtons, vel_yaw, theta, right, safeguard_yaw, velocities, yaws);

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

	double BestStrafeMaxAccel(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons,
		double vel_yaw)
	{
		assert(postype != PositionType::WATER);

		float temp_vel[2], orig_vel[2];
		double yaws[2];
		VecCopy<float, 2>(player.Velocity, orig_vel);
		yaws[0] = SideStrafeMaxAccel(player, vars, postype, wishspeed, strafeButtons, vel_yaw, false);
		VecCopy<float, 2>(player.Velocity, temp_vel);
		VecCopy<float, 2>(orig_vel, player.Velocity);
		yaws[1] = SideStrafeMaxAccel(player, vars, postype, wishspeed, strafeButtons, vel_yaw, true);

		double speedsqrs[2] = {
			DotProduct<float, float, 2>(temp_vel, temp_vel),
			DotProduct<float, float, 2>(player.Velocity, player.Velocity)
		};

		if (speedsqrs[0] > speedsqrs[1]) {
			VecCopy<float, 2>(temp_vel, player.Velocity);
			return yaws[0];
		} else
			return yaws[1];
	}

	double BestStrafeMaxAngle(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons,
		double vel_yaw)
	{
		assert(postype != PositionType::WATER);

		float temp_vel[2], orig_vel[2];
		double yaws[2];
		VecCopy<float, 2>(player.Velocity, orig_vel);
		yaws[0] = SideStrafeMaxAngle(player, vars, postype, wishspeed, strafeButtons, vel_yaw, false);
		VecCopy<float, 2>(player.Velocity, temp_vel);
		VecCopy<float, 2>(orig_vel, player.Velocity);
		yaws[1] = SideStrafeMaxAngle(player, vars, postype, wishspeed, strafeButtons, vel_yaw, true);

		double old_speed = Length<float, 2>(orig_vel);
		double speeds[2] = { Length<float, 2>(temp_vel), Length<float, 2>(player.Velocity) };
		double cosangles[2] = {
			DotProduct<float, float, 2>(temp_vel, orig_vel) / (old_speed * speeds[0]),
			DotProduct<float, float, 2>(player.Velocity, orig_vel) / (old_speed * speeds[1])
		};

		if (cosangles[0] < cosangles[1]) {
			VecCopy<float, 2>(temp_vel, player.Velocity);
			return yaws[0];
		} else
			return yaws[1];
	}

	double YawStrafeMaxAccel(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons,
		double vel_yaw, double yaw)
	{
		assert(postype != PositionType::WATER);

		double theta = MaxAccelIntoYawTheta(player, vars, postype, wishspeed, vel_yaw, yaw);
		float velocities[2][2];
		double yaws[2];
		SideStrafeGeneral(player, vars, postype, wishspeed, strafeButtons, vel_yaw, std::fabs(theta), (theta < 0), false, velocities, yaws);

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

	double YawStrafeMaxAngle(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons,
		double vel_yaw, double yaw)
	{
		assert(postype != PositionType::WATER);

		bool safeguard_yaw;
		double theta = MaxAngleTheta(player, vars, postype, wishspeed, safeguard_yaw);
		float velocities[2][2];
		double yaws[2];
		if (!IsZero<float, 2>(player.Velocity))
			vel_yaw = Atan2(player.Velocity[1], player.Velocity[0]);
		SideStrafeGeneral(player, vars, postype, wishspeed, strafeButtons, vel_yaw, theta, (NormalizeRad(yaw - vel_yaw) < 0), safeguard_yaw, velocities, yaws);

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

	double PointStrafe(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons,
		double vel_yaw, HLTAS::StrafeType type, float point[2], bool& strafed)
	{
		assert(postype != PositionType::WATER);

		// Covers the case where both vectors are zero.
		if (Distance<float, float, 2>(player.Origin, point) <= 2.0) {
			strafed = false;
			return 0.0;
		}
		strafed = true;

		float difference[2];
		VecSubtract<float, float, 2>(point, player.Origin, difference);
		double yaw = Atan2(difference[1], difference[0]);

		switch (type) {
		case HLTAS::StrafeType::MAXACCEL: return YawStrafeMaxAccel(player, vars, postype, wishspeed, strafeButtons, vel_yaw, yaw);
		default:
		case HLTAS::StrafeType::MAXANGLE: return YawStrafeMaxAngle(player, vars, postype, wishspeed, strafeButtons, vel_yaw, yaw);
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

	PositionType PredictJump(PlayerData& player, PositionType postype, const MovementVars& vars, const CurrentState& curState, ProcessedFrame& out)
	{
		assert(postype != PositionType::WATER);

		if (!out.Jump // Not pressing Jump.
			|| curState.Jump // Jump was already down.
			|| postype != PositionType::GROUND) // Not on ground.
			return postype;

		if (vars.Bhopcap) {
			auto maxscaledspeed = 1.7f * vars.Maxspeed;
			if (maxscaledspeed > 0) {
				auto speed = Length<float, 3>(player.Velocity);
				if (speed > maxscaledspeed)
					VecScale<float, 3>(player.Velocity, (maxscaledspeed / speed) * 0.65, player.Velocity);
			}
		}

		// TODO: duck-when autofuncs.
		// We don't care about the vertical velocity after the jump prediction.
		if (player.InDuckAnimation || player.Ducking) {
			if (player.HasLJModule && out.Duck && player.DuckTime > 0 && Length<float, 3>(player.Velocity) > 50) {
				player.Velocity[0] = static_cast<float>(std::cos(player.Viewangles[1] * M_DEG2RAD) * std::cos(player.Viewangles[0] * M_DEG2RAD) * 350 * 1.6);
				player.Velocity[1] = static_cast<float>(std::sin(player.Viewangles[1] * M_DEG2RAD) * std::cos(player.Viewangles[0] * M_DEG2RAD) * 350 * 1.6);
			}
		}
		CheckVelocity(player, vars);

		return PositionType::AIR;
	}

	void Friction(PlayerData& player, PositionType postype, const MovementVars& vars, TraceFunc traceFunc)
	{
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
	}

	void Autojump(PositionType postype, const HLTAS::Frame& frame, CurrentState& curState, ProcessedFrame& out)
	{
		assert(postype != PositionType::WATER);

		if (!frame.Autojump && !curState.AutojumpsLeft)
			return;

		if (postype == PositionType::GROUND && !curState.Jump && !out.Jump) {
			out.Jump = true;
			if (curState.AutojumpsLeft)
				curState.AutojumpsLeft--;
		}
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

		auto playerCopy = PlayerData(player); // Our copy that we will mess with.
		auto postype = GetPositionType(playerCopy, traceFunc);
		if (postype == PositionType::WATER)
			return out;

		// This order may change.
		// Jumpbug()
		// Dbc()
		// Dbg()
		// Lgagst-ducktap()
		// Ducktap()
		// PredictDuck()
		// Lgagst-jump()
		Autojump(postype, frame, curState, out);
		postype = PredictJump(playerCopy, postype, vars, curState, out);
		Friction(playerCopy, postype, vars, traceFunc);
		CheckVelocity(playerCopy, vars);
		// Strafe()

		curState.Jump = out.Jump;
		curState.Duck = out.Duck;
		return out;
	}

	double GetPitchDifference(float oldpitch, float newpitch)
	{
		return Normalize(static_cast<double>(newpitch) - oldpitch);
	}

	double GetYawDifference(float oldyaw, float newyaw)
	{
		return Normalize(static_cast<double>(newyaw) - oldyaw + M_U_DEG_HALF);
	}

	std::string GetAngleSpeedString(float oldpitch, float oldyaw, float newpitch, float newyaw, double pitchStateMultiplier, double yawStateMultiplier, float frametime)
	{
		std::ostringstream ss;
		ss.setf(std::ios::fixed, std::ios::floatfield);
		ss.precision(std::numeric_limits<double>::digits10);

		if (newpitch != oldpitch) {
			double pitchDifference = std::abs(GetPitchDifference(oldpitch, newpitch));
			double pitchspeed = (pitchDifference / frametime) / pitchStateMultiplier;
			ss << "cl_pitchspeed " << pitchspeed << '\n';
		}

		if (newyaw != oldyaw) {
			double yawDifference = std::abs(GetYawDifference(oldyaw, newyaw));
			double yawspeed = (yawDifference / frametime) / yawStateMultiplier;
			ss << "cl_yawspeed " << yawspeed << '\n';
		}

		return ss.str();
	}
}
