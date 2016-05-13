#pragma once
#include <functional>

#include "hltas.hpp"
#include "vec.hpp"

namespace HLStrafe
{
	struct PlayerData {
		vec Origin;
		vec Velocity;
		vec Basevelocity;
		float Viewangles[3];

		bool Ducking;
		bool InDuckAnimation;
		float DuckTime;

		bool HasLJModule;

		double SpeedBeforeFriction; // Internal.
	};

	struct MovementVars {
		float Frametime;
		float Maxvelocity;
		float Maxspeed;
		float Stopspeed;
		float Friction;
		float Edgefriction;
		float EntFriction; // Aka pmove->friction.
		float Accelerate;
		float Airaccelerate;
		float Gravity;
		float EntGravity; // Aka pmove->gravity.
		float Stepsize;
		float Bounce;
		bool Bhopcap;
	};

	struct ProcessedFrame {
		float Pitch;
		float Yaw;

		bool Forward;
		bool Left;
		bool Right;
		bool Back;
		bool Up;
		bool Down;

		bool Jump;
		bool Duck;
		bool Use;
		bool Attack1;
		bool Attack2;
		bool Reload;

		float Forwardspeed;
		float Sidespeed;
		float Backspeed;
		float Upspeed;

		bool NextFrameIs0ms;
	};

	enum class State0ms {
		NOTHING = 0,
		DUCKED,
		UNDUCKED,
		UNDUCKED_AND_DUCKED
	};

	struct CurrentState {
		CurrentState() :
			Jump(false),
			Duck(false),
			JumpbugsLeft(0),
			Ducktap0ms(false),
			DucktapsLeft(0),
			AutojumpsLeft(0),
			DbcCeilings(false),
			DbcsLeft(0),
			DbgsLeft(0),
			DwjsLeft(0),
			LgagstFullMaxspeed(false),
			LgagstType(false),
			LgagstMinSpeed(30.0f),
			LgagstsLeft(0),
			PredictThis(State0ms::NOTHING)
		{};

		bool Jump;
		bool Duck;
		unsigned JumpbugsLeft;
		bool Ducktap0ms;
		unsigned DucktapsLeft;
		unsigned AutojumpsLeft;
		bool DbcCeilings;
		unsigned DbcsLeft;
		unsigned DbgsLeft;
		unsigned DwjsLeft;
		bool LgagstFullMaxspeed;
		bool LgagstType; // False if Autojump, true if Ducktap.
		float LgagstMinSpeed;
		unsigned LgagstsLeft;

		State0ms PredictThis;
	};

	struct TraceResult {
		bool AllSolid;
		bool StartSolid;
		float Fraction;
		vec EndPos;
		vec PlaneNormal;
		int Entity;
	};

	// On ground, in the air or underwater.
	enum class PositionType {
		GROUND = 0,
		AIR,
		WATER
	};

	enum class HullType : int {
		NORMAL = 0,
		DUCKED = 1,
		POINT = 2
	};

	using TraceFunc = std::function<TraceResult(const vec&, const vec&, HLStrafe::HullType)>;

	/*
		All-in-one function that does everything and returns the buttons,
		viewangles and FSU. Modifies curState. You should keep the curState
		and pass it to the function next time it's invoked (next frame usually).
	*/
	ProcessedFrame MainFunc(const PlayerData& player, const MovementVars& vars, const HLTAS::Frame& frame, CurrentState& curState, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, TraceFunc traceFunc);

	/*
		Returns the string to put into the command buffer to change the viewangles.
	*/
	std::string GetAngleSpeedString(float oldpitch, float oldyaw, float newpitch, float newyaw, double pitchStateMultiplier, double yawStateMultiplier, float frametime);

	namespace autofuncs
	{
		/*
			Autofuncs. They modify stuff in curState and also buttons from the ProcessedFrame.
			Autofuncs generally do NOT release any pressed buttons with an exception of Ducktap
			(for the sake of ducktapping while ducking the rest of the time).
		*/
		void Jumpbug(
			const PlayerData& player,
			const MovementVars& vars,
			PositionType postype,
			const HLTAS::Frame& frame,
			ProcessedFrame& out,
			const HLTAS::StrafeButtons& strafeButtons,
			bool useGivenButtons,
			CurrentState& curState,
			TraceFunc traceFunc);

		void Ducktap(
			const PlayerData& player,
			PositionType postype,
			const HLTAS::Frame& frame,
			CurrentState& curState,
			ProcessedFrame& out,
			TraceFunc traceFunc);

		void Autojump(PositionType postype, const HLTAS::Frame& frame, CurrentState& curState, ProcessedFrame& out);

		void Dbc(
			const PlayerData& player,
			const MovementVars& vars,
			PositionType postype,
			const HLTAS::Frame& frame,
			ProcessedFrame& out,
			const HLTAS::StrafeButtons& strafeButtons,
			bool useGivenButtons,
			CurrentState& curState,
			TraceFunc traceFunc);

		void Dbg(
			const PlayerData& player,
			const MovementVars& vars,
			PositionType postype,
			const HLTAS::Frame& frame,
			ProcessedFrame& out,
			const HLTAS::StrafeButtons& strafeButtons,
			bool useGivenButtons,
			CurrentState& curState,
			TraceFunc traceFunc);

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
			TraceFunc traceFunc);

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
			TraceFunc traceFunc);

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
			float fractions[4] = nullptr,
			float normalzs[4] = nullptr);
	}

	namespace anglefuncs
	{
		/*
			Returns the angle in radians - [0; Pi] - between velocity and wishdir that will
			result in maximal speed gain. Postype != WATER.

			Struct requirements:
				Velocity;
				Frametime, Accelerate or Airaccelerate, EntFriction.
		*/
		double MaxAccelTheta(const PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed);

		/*
			Returns the angle between velocity and wishdir in [0, Pi] that will
			keep the speed constant as far as possible. Under certain conditions
			the angle from MaxAccelTheta will be returned. Postype != WATER.

			Struct requirements:
				Velocity;
				Frametime, Accelerate or Airaccelerate, EntFriction.
		*/
		double ConstSpeedTheta(const PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed);

		/*
			Returns the angle in radians - [-Pi; Pi) - between velocity and wishdir that will
			result in maximal speed gain into the given yaw - [-Pi; Pi). If velocity is zero, vel_yaw will
			be used in place of velocity angle. Postype != WATER.

			Struct requirements:
				Velocity;
				Frametime, Accelerate or Airaccelerate, EntFriction.
		*/
		double MaxAccelIntoYawTheta(
			const PlayerData& player,
			const MovementVars& vars,
			PositionType postype,
			double wishspeed,
			double vel_yaw,
			double yaw);

		/*
			Returns the angle in radians - [0; Pi] - between velocity and wishdir that will
			result in maximal velocity angle change. Sets safeguard_yaw to true if the yaw safeguard
			is needed and to false otherwise. Postype != WATER.

			Struct requirements:
				Velocity;
				Frametime, Accelerate or Airaccelerate, EntFriction.
		*/
		double MaxAngleTheta(
			const PlayerData& player,
			const MovementVars& vars,
			PositionType postype,
			double wishspeed,
			bool& safeguard_yaw);

		/*
			Returns the angle in radians - [0; Pi] - between velocity and wishdir that will
			result in maximal decceleration. Sets safeguard_yaw to true if the yaw safeguard
			is needed and to false otherwise. Postype != WATER.

			Struct requirements:
			Velocity;
			Frametime, Accelerate or Airaccelerate, EntFriction.
		*/
		double MaxDeccelTheta(
			const PlayerData& player,
			const MovementVars& vars,
			PositionType postype,
			double wishspeed,
			bool& safeguard_yaw);
	}

	namespace prediction
	{
		/*
			Figures out the player's position type and, if necessary, updates player.Origin and curState.
		*/
		PositionType GetPositionType(PlayerData& player, TraceFunc traceFunc);

		/*
			Limits the velocity components to maxvelocity.
		*/
		void CheckVelocity(PlayerData& player, const MovementVars& vars);

		/*
			Changes the player data the same way as PM_Duck would, returns a new postype.
		*/
		PositionType PredictDuck(
			PlayerData& player,
			const MovementVars& vars,
			PositionType postype,
			CurrentState& curState,
			const ProcessedFrame& out,
			TraceFunc traceFunc);

		/*
			Changes the player data the same way as PM_Jump would, returns a new postype.
			Changes the processed frame in case of some duck-when autofuncs.
		*/
		PositionType PredictJump(
			PlayerData& player,
			PositionType postype,
			const MovementVars& vars,
			const HLTAS::Frame& frame,
			CurrentState& curState,
			ProcessedFrame& out,
			TraceFunc traceFunc,
			bool decreaseDwjTimes = false);

		/*
			Applies the ground friction the same way as PM_Friction would, changing player.Velocity.
		*/
		void Friction(PlayerData& player, PositionType postype, const MovementVars& vars, TraceFunc traceFunc);

		/*
			Computes the new velocity given unit acceleration vector and wishspeed
			and stores the result in player.Velocity. Postype != WATER.

			Struct requirements:
				Velocity;
				Frametime, Accelerate or Airaccelerate, EntFriction.
		*/
		void VectorFME(
			PlayerData& player,
			const MovementVars& vars,
			PositionType postype,
			double wishspeed,
			const vec2d& wishdir);

		/*
			Helpers for the movement prediction, do exactly what PM_ClipVelocity and PM_FlyMove do.
		*/
		int ClipVelocity(vec& velocity, const vec& normal, float overbounce);

		void FlyMove(
			PlayerData& player,
			const MovementVars& vars,
			PositionType postype,
			TraceFunc traceFunc,
			float fractions[4] = nullptr,
			float normalzs[4] = nullptr);

		/*
			Computes the new velocity given unit acceleration vector and wishspeed,
			stores the result in player.Velocity, computes the new position taking the world into consideration
			and stores the result in player.Origin. Postype != WATER.

			Struct requirements:
			Velocity, Basevelocity, Origin;
			Frametime, Accelerate or Airaccelerate, EntFriction, EntGravity, Gravity.
		*/
		PositionType Move(
			PlayerData& player,
			const MovementVars& vars,
			PositionType postype,
			double wishspeed,
			TraceFunc traceFunc,
			bool calcVelocity,
			const vec2d& wishdir,
			float fractions[4] = nullptr,
			float normalzs[4] = nullptr);

		/*
			Checks if the next frame needs to be 0ms.
		*/
		void CheckIfNextFrameShouldBe0ms(
			const PlayerData& player,
			const MovementVars& vars,
			const HLTAS::Frame& frame,
			PositionType postype,
			ProcessedFrame& out,
			const HLTAS::StrafeButtons& strafeButtons,
			bool useGivenButtons,
			const CurrentState& curState,
			TraceFunc traceFunc);

		/*
			Predicts the changes made in past 0ms frames (since those frames didn't run on the server yet).
		*/
		PositionType PredictPast0msFrames(
			PlayerData& player,
			const MovementVars& vars,
			PositionType postype,
			const ProcessedFrame& out,
			const CurrentState& curState,
			TraceFunc traceFunc);
	}

	namespace strafefuncs
	{
		/*
			Finds the best yaw to use for the corresponding strafe type taking the anglemod compensation into account, then
			strafes sideways with that yaw and returns it in radians, given fixed buttons.
			The resulting velocity is stored in player.Velocity.
			Uses vel_yaw instead of the Velocity angle if Velocity is zero.
			Postype != WATER.

			Struct requirements:
				Velocity;
				Frametime, Accelerate or Airaccelerate, EntFriction.
		*/
		double SideStrafeMaxAccel(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, HLTAS::Button& usedButton, double vel_yaw, bool right);
		double SideStrafeMaxAngle(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, HLTAS::Button& usedButton, double vel_yaw, bool right);
		double SideStrafeMaxDeccel(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, HLTAS::Button& usedButton, double vel_yaw, bool right, bool& strafed);
		double SideStrafeConstSpeed(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, HLTAS::Button& usedButton, double vel_yaw, bool right);

		/*
			Finds the best yaw to use for the corresponding strafe type taking the anglemod compensation into account, then
			strafes to the best dir with that yaw and returns it in radians, given fixed buttons.
			The resulting velocity is stored in player.Velocity.
			Uses vel_yaw instead of the Velocity angle if Velocity is zero.
			Postype != WATER.

			Struct requirements:
				Velocity;
				Frametime, Accelerate or Airaccelerate, EntFriction.
		*/
		double BestStrafeMaxAccel(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, HLTAS::Button& usedButton, double vel_yaw);
		double BestStrafeMaxAngle(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, HLTAS::Button& usedButton, double vel_yaw);
		double BestStrafeMaxDeccel(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, HLTAS::Button& usedButton, double vel_yaw, bool& strafed);
		double BestStrafeConstSpeed(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, HLTAS::Button& usedButton, double vel_yaw);

		/*
			Finds the best yaw to use for the corresponding strafe type taking the anglemod compensation into account, then
			strafes to the given yaw with that yaw and returns it in radians, given fixed buttons.
			The resulting velocity is stored in player.Velocity.
			Uses vel_yaw instead of the Velocity angle if Velocity is zero.
			Postype != WATER.

			Struct requirements:
				Velocity;
				Frametime, Accelerate or Airaccelerate, EntFriction.
		*/
		double YawStrafeMaxAccel(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, HLTAS::Button& usedButton, double vel_yaw, double yaw);
		double YawStrafeMaxAngle(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, HLTAS::Button& usedButton, double vel_yaw, double yaw);
		double YawStrafeMaxDeccel(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, HLTAS::Button& usedButton, double vel_yaw, double yaw, bool& strafed);
		double YawStrafeConstSpeed(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, HLTAS::Button& usedButton, double vel_yaw, double yaw);

		/*
			Finds the best yaw to use for the given strafe type taking the anglemod compensation into account, then
			strafes to the given point if needed with that yaw and returns it in radians, given fixed buttons. If not strafing
			is better, set strafed to false, otherwise - to true.
			The resulting velocity is stored in player.Velocity.
			Uses vel_yaw instead of the Velocity angle if Velocity is zero.
			Postype != WATER.

			Struct requirements:
				Velocity, Origin;
				Frametime, Accelerate or Airaccelerate, EntFriction.
		*/
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
			bool& strafed);
	}
}
