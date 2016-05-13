#include "hlstrafe.hpp"
#include "util.hpp"
#include "vec.hpp"

namespace HLStrafe
{

using namespace prediction;
using namespace strafefuncs;

namespace autofuncs
{
	void Jumpbug(
			const PlayerData& player,
			const MovementVars& vars,
			PositionType postype,
			const HLTAS::Frame& frame,
			ProcessedFrame& out,
			const HLTAS::StrafeButtons& strafeButtons,
			bool useGivenButtons,
			CurrentState& curState,
			TraceFunc traceFunc)
	{
		assert(postype != PositionType::WATER);

		if ((!frame.Jumpbug && !curState.JumpbugsLeft) || postype == PositionType::GROUND)
			return;

		auto playerCopy = PlayerData(player);
		auto outCopy = ProcessedFrame(out);
		auto curStateCopy = CurrentState(curState);

		if (player.Ducking) {
			outCopy.Duck = false;
			postype = PredictDuck(playerCopy, vars, postype, curStateCopy, outCopy, traceFunc);

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

		postype = Strafe(
			playerCopy,
			vars,
			postype,
			frame,
			outCopy,
			false,
			strafeButtons,
			useGivenButtons,
			true,
			traceFunc);

		if (postype == PositionType::GROUND) {
			// Duck now so that in the next frame we can do the jumpbug.
			out.Duck = true;
			out.Jump = false;
		}
	}

	void Ducktap(
			const PlayerData& player,
			PositionType postype,
			const HLTAS::Frame& frame,
			CurrentState& curState,
			ProcessedFrame& out,
			TraceFunc traceFunc)
	{
		assert(postype != PositionType::WATER);

		if ((frame.Lgagst || curState.LgagstsLeft) || (!frame.Ducktap && !curState.DucktapsLeft))
			return;
		if (postype != PositionType::GROUND)
			return;

		// Allow ducktapping with pressed duck - release duck if we're on ground and can unduck.
		// Otherwise we don't want to handle situations where the player is ducking.
		if (player.Ducking) {
			vec newOrigin = player.Origin;
			for (std::size_t i = 0; i < 3; ++i)
				newOrigin[i] += (VEC_DUCK_HULL_MIN[i] - VEC_HULL_MIN[i]);

			auto tr = traceFunc(newOrigin, newOrigin, player.Ducking ? HullType::DUCKED : HullType::NORMAL);
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
			vec newOrigin = player.Origin;
			for (std::size_t i = 0; i < 3; ++i)
				newOrigin[i] += (VEC_DUCK_HULL_MIN[i] - VEC_HULL_MIN[i]);

			auto tr = traceFunc(newOrigin, newOrigin, HullType::NORMAL);
			if (!tr.StartSolid)
				out.Duck = true;
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

	void Dbc(
			const PlayerData& player,
			const MovementVars& vars,
			PositionType postype,
			const HLTAS::Frame& frame,
			ProcessedFrame& out,
			const HLTAS::StrafeButtons& strafeButtons,
			bool useGivenButtons,
			CurrentState& curState,
			TraceFunc traceFunc)
	{
		assert(postype != PositionType::WATER);

		if ((!frame.Dbc && !curState.DbcsLeft)
			|| postype == PositionType::GROUND
			|| out.Duck)
			return;

		// Default to not ceiling.
		float normalzsUnducked[4] =  { 0, 0, 0, 0 };
		// tr.Fraction can never be this high.
		float fractionsUnducked[4] = { 2, 2, 2, 2 };
		float fractionsDucked[4] =   { 2, 2, 2, 2 };

		auto playerCopy = PlayerData(player);
		auto out_temp = ProcessedFrame(out);
		auto curStateCopy = CurrentState(curState);

		postype = PredictDuck(playerCopy, vars, postype, curStateCopy, out, traceFunc);
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
		postype = PredictDuck(playerCopy, vars, postype, curStateCopy, out_temp, traceFunc);
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

	void Dbg(
			const PlayerData& player,
			const MovementVars& vars,
			PositionType postype,
			const HLTAS::Frame& frame,
			ProcessedFrame& out,
			const HLTAS::StrafeButtons& strafeButtons,
			bool useGivenButtons,
			CurrentState& curState,
			TraceFunc traceFunc)
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

		postype = PredictDuck(playerCopy, vars, postype, curState, out_temp, traceFunc);
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

	void LgagstDucktap(
			const PlayerData& player,
			const MovementVars& vars,
			PositionType postype,
			const HLTAS::Frame& frame,
			ProcessedFrame& out,
			bool reduceWishspeed,
			const HLTAS::StrafeButtons& strafeButtons,
			bool useGivenButtons,
			CurrentState& curState,
			TraceFunc traceFunc)
	{
		assert(postype != PositionType::WATER);

		if ((!frame.Lgagst && !curState.LgagstsLeft) || !curState.LgagstType // Lgagst not enabled or LgagstAutojump.
			|| !frame.Strafe || frame.GetType() != HLTAS::StrafeType::MAXACCEL // Not maxaccel strafing.
			|| postype != PositionType::GROUND || player.Ducking || player.InDuckAnimation // Not on ground, ducking or in duck animation.
			|| out.Duck
			|| player.Velocity.as_2d().len() < curState.LgagstMinSpeed)
			return;

		if (vars.Frametime == 0.f) {
			// 0ms frame. If we got here then we need to ducktap.
			out.Duck = true;

			if (curState.LgagstsLeft)
				curState.LgagstsLeft--;

			return;
		}

		// Predict the next frame's origin.
		auto playerCopy = PlayerData(player);
		Friction(playerCopy, postype, vars, traceFunc);
		auto out_temp = ProcessedFrame(out);
		Strafe(playerCopy, vars, postype, frame, out_temp, reduceWishspeed, strafeButtons, useGivenButtons, true, traceFunc);

		// Check if we can ducktap there.
		vec newOrigin = player.Origin;

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
		auto curStateCopy = CurrentState(curState);
		air.InDuckAnimation = true;
		postype = PredictDuck(air, vars, postype, curStateCopy, out_temp, traceFunc);
		Strafe(air, vars, postype, frame, out_temp, false, strafeButtons, useGivenButtons, false, traceFunc);

		auto l_gr = ground.Velocity.as_2d().len_sq();
		auto l_air = air.Velocity.as_2d().len_sq();
		if (l_air > l_gr) {
			out.Duck = true;

			if (curState.LgagstsLeft)
				curState.LgagstsLeft--;
		}
	}

	void LgagstJump(
			const PlayerData& player,
			const MovementVars& vars,
			PositionType postype,
			const HLTAS::Frame& frame,
			ProcessedFrame& out,
			bool reduceWishspeed,
			const HLTAS::StrafeButtons& strafeButtons,
			bool useGivenButtons,
			CurrentState& curState,
			TraceFunc traceFunc)
	{
		assert(postype != PositionType::WATER);

		if ((!frame.Lgagst && !curState.LgagstsLeft) || curState.LgagstType // Lgagst not enabled or LgagstDucktap.
			|| postype != PositionType::GROUND || curState.Jump || out.Jump // Cannot jump / already jumped.
			|| !frame.Strafe || frame.GetType() != HLTAS::StrafeType::MAXACCEL // Not maxaccel strafing.
			|| player.Velocity.len() < curState.LgagstMinSpeed)
			return;

		auto ground = PlayerData(player);
		Friction(ground, postype, vars, traceFunc);
		CheckVelocity(ground, vars);
		auto out_temp = ProcessedFrame(out);
		Strafe(
			ground,
			vars,
			postype,
			frame,
			out_temp,
			reduceWishspeed && !curState.LgagstFullMaxspeed,
			strafeButtons,
			useGivenButtons,
			false,
			traceFunc);

		auto air = PlayerData(player);
		out_temp = ProcessedFrame(out);
		out_temp.Jump = true;
		auto curState_temp = CurrentState(curState);
		postype = PredictJump(air, postype, vars, frame, curState_temp, out_temp, traceFunc);
		Strafe(
			air,
			vars,
			postype,
			frame,
			out_temp,
			reduceWishspeed && !curState_temp.LgagstFullMaxspeed,
			strafeButtons,
			useGivenButtons,
			false,
			traceFunc);

		auto l_gr = ground.Velocity.as_2d().len_sq();
		auto l_air = air.Velocity.as_2d().len_sq();
		if (l_air > l_gr) {
			out.Jump = true;

			if (curState.LgagstsLeft)
				curState.LgagstsLeft--;
		}
	}

	PositionType Strafe(
			PlayerData& player,
			const MovementVars& vars,
			PositionType postype,
			const HLTAS::Frame& frame,
			ProcessedFrame& out,
			bool reduceWishspeed,
			const HLTAS::StrafeButtons& strafeButtons,
			bool useGivenButtons,
			bool predictOrigin,
			TraceFunc traceFunc,
			float fractions[4],
			float normalzs[4])
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
				else if (frame.GetType() == HLTAS::StrafeType::CONSTSPEED)
					out.Yaw = static_cast<float>(BestStrafeConstSpeed(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, usedButton, out.Yaw * M_DEG2RAD) * M_RAD2DEG);
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
				else if (frame.GetType() == HLTAS::StrafeType::CONSTSPEED)
					out.Yaw = static_cast<float>(YawStrafeConstSpeed(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, usedButton, out.Yaw * M_DEG2RAD, frame.GetYaw() * M_DEG2RAD) * M_RAD2DEG);
				else if (frame.GetType() == HLTAS::StrafeType::MAXDECCEL) {
					auto yaw = static_cast<float>(YawStrafeMaxDeccel(player, vars, postype, wishspeed, strafeButtons, useGivenButtons, usedButton, out.Yaw * M_DEG2RAD, frame.GetYaw() * M_DEG2RAD, strafed) * M_RAD2DEG);
					if (strafed)
						out.Yaw = yaw;
				}
				break;

			case HLTAS::StrafeDir::POINT:
			{
				vec2d point(frame.GetX(), frame.GetY());
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

		vec2d wishdir;
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

			wishdir = vec2d(cy * forwardmove + sy * sidemove, sy * forwardmove - cy * sidemove); // TODO: consider pitch & roll.
			wishdir.normalize();

			if (!predictOrigin)
				VectorFME(player, vars, postype, wishspeed, wishdir);
		}

		if (predictOrigin)
			postype = Move(player, vars, postype, wishspeed, traceFunc, !strafed, wishdir, fractions, normalzs);

		return postype;
	}
}

} // namespace HLStrafe
