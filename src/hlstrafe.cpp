#include <algorithm>
#include <cassert>
#include <cmath>

#include "hlstrafe.hpp"
#include "util.hpp"

// #include "../../SPTLib/sptlib.hpp"

namespace HLStrafe
{
	using namespace autofuncs;
	using namespace prediction;
	using namespace strafefuncs;

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

		out.NextFrameIs0ms = false;

		if (frame.PitchPresent)
			out.Pitch = static_cast<float>(frame.GetPitch());
		if (!frame.Strafe && frame.GetYawPresent())
			out.Yaw = static_cast<float>(AngleModDeg(frame.GetYaw()));

		if (frame.Autojump)
			curState.AutojumpsLeft = frame.GetAutojumpTimes();
		if (frame.Ducktap) {
			curState.Ducktap0ms = frame.GetDucktap0ms();
			curState.DucktapsLeft = frame.GetDucktapTimes();
		}
		if (frame.Dbc) {
			curState.DbcCeilings = frame.GetDbcCeilings();
			curState.DbcsLeft = frame.GetDbcTimes();
		}
		if (frame.Dbg)
			curState.DbgsLeft = frame.GetDbgTimes();
		if (frame.Dwj)
			curState.DwjsLeft = frame.GetDwjTimes();
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

		postype = PredictPast0msFrames(playerCopy, vars, postype, out, curState, traceFunc);
		// We only need to set PredictThis on 0ms frames and for that we need to know the previous state.
		// Otherwise, reset it right here.
		if (vars.Frametime != 0.f)
			curState.PredictThis = State0ms::NOTHING;

		bool reduceWishspeed = playerCopy.Ducking;
		// Same as in ReduceTimers().
		playerCopy.DuckTime = std::max(playerCopy.DuckTime - static_cast<int>(vars.Frametime * 1000), 0.f);

		// This order may change.
		Jumpbug(playerCopy, vars, postype, frame, out, strafeButtons, useGivenButtons, curState, traceFunc);
		Dbc(playerCopy, vars, postype, frame, out, strafeButtons, useGivenButtons, curState, traceFunc);
		Dbg(playerCopy, vars, postype, frame, out, strafeButtons, useGivenButtons, curState, traceFunc);
		LgagstDucktap(playerCopy, vars, postype, frame, out, reduceWishspeed, strafeButtons, useGivenButtons, curState, traceFunc);
		Ducktap(playerCopy, postype, frame, curState, out, traceFunc);
		postype = PredictDuck(playerCopy, vars, postype, curState, out, traceFunc);

		// This has to be after PredictDuck() since we may have ducktapped,
		// if this messes with Lgagst-ducktap() then it's the user's problem.
		// (Though it shouldn't, generally).
		if (out.Use && postype == PositionType::GROUND)
			playerCopy.Velocity *= 0.3;

		LgagstJump(playerCopy, vars, postype, frame, out, reduceWishspeed, strafeButtons, useGivenButtons, curState, traceFunc);
		Autojump(postype, frame, curState, out);
		postype = PredictJump(playerCopy, postype, vars, frame, curState, out, traceFunc, true);
		Friction(playerCopy, postype, vars, traceFunc);
		CheckVelocity(playerCopy, vars);
		postype = Strafe(playerCopy, vars, postype, frame, out, reduceWishspeed, strafeButtons, useGivenButtons, true, traceFunc);

		CheckIfNextFrameShouldBe0ms(playerCopy, vars, frame, postype, out, strafeButtons, useGivenButtons, curState, traceFunc);

		//EngineMsg("p po %f\t%f\t%f\t%f\t%f\t%f\n", playerCopy.Origin[0], playerCopy.Origin[1], playerCopy.Origin[2], playerCopy.Velocity[0], playerCopy.Velocity[1], playerCopy.Velocity[2]);
		curState.Jump = out.Jump;
		curState.Duck = out.Duck;
		return out;
	}

	std::string GetAngleSpeedString(float oldpitch, float oldyaw, float newpitch, float newyaw, double pitchStateMultiplier, double yawStateMultiplier, float frametime)
	{
		std::ostringstream ss;
		ss.setf(std::ios::fixed, std::ios::floatfield);
		ss.precision(std::numeric_limits<double>::digits10);

		if (newpitch != oldpitch) {
			double pitchDifference = std::abs(AngleDifference(oldpitch, newpitch));
			double pitchspeed = (pitchDifference / frametime) / pitchStateMultiplier;
			ss << "cl_pitchspeed " << pitchspeed << '\n';
		}

		if (newyaw != oldyaw) {
			auto angleDifference = AngleDifference(oldyaw, newyaw);

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
