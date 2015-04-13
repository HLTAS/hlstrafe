#pragma once
#include <functional>

#include "hltas.hpp"

namespace HLStrafe
{
	struct PlayerData {
		float Origin[3];
		float Velocity[3];
		float Basevelocity[3];
		float Viewangles[3];

		bool Ducking;
		bool InDuckAnimation;
		float DuckTime;

		bool HasLJModule;
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
	};

	struct CurrentState {
		CurrentState() :
			Jump(false),
			Duck(false),
			DucktapsLeft(0),
			AutojumpsLeft(0),
			DbcCeilings(false),
			DbcsLeft(0),
			LgagstFullMaxspeed(false),
			LgagstType(false),
			LgagstMinSpeed(30.0f),
			LgagstsLeft(0)
		{};

		bool Jump;
		bool Duck;
		unsigned DucktapsLeft;
		unsigned AutojumpsLeft;
		bool DbcCeilings;
		unsigned DbcsLeft;
		bool LgagstFullMaxspeed;
		bool LgagstType; // False if Autojump, true if Ducktap.
		float LgagstMinSpeed;
		unsigned LgagstsLeft;
	};

	struct TraceResult {
		bool AllSolid;
		bool StartSolid;
		float Fraction;
		float EndPos[3];
		float PlaneNormal[3];
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

	typedef std::function<TraceResult(const float[3], const float[3], HLStrafe::HullType)> TraceFunc;

	/*
		All-in-one function that does everything and returns the buttons,
		viewangles and FSU. Modifies curState. You should keep the curState
		and pass it to the function next time it's invoked (next frame usually).
	*/
	ProcessedFrame MainFunc(const PlayerData& player, const MovementVars& vars, const HLTAS::Frame& frame, CurrentState& curState, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, TraceFunc traceFunc);

	/*
		Returns the difference between two angles in a [-180; 180) range.
	*/
	double GetAngleDifference(float oldyaw, float newyaw);

	/*
		Returns the string to put into the command buffer to change the viewangles.
	*/
	std::string GetAngleSpeedString(float oldpitch, float oldyaw, float newpitch, float newyaw, double pitchStateMultiplier, double yawStateMultiplier, float frametime);

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
	PositionType PredictDuck(PlayerData& player, PositionType postype, const MovementVars& vars, const CurrentState& curState, const ProcessedFrame& out, TraceFunc traceFunc);

	/*
		Changes the player data the same way as PM_Jump would, returns a new postype.
		Changes the processed frame in case of some duck-when autofuncs.
	*/
	PositionType PredictJump(PlayerData& player, PositionType postype, const MovementVars& vars, const CurrentState& curState, ProcessedFrame& out);

	/*
		Applies the ground friction the same way as PM_Friction would, changing player.Velocity.
	*/
	void Friction(PlayerData& player, PositionType postype, const MovementVars& vars, TraceFunc traceFunc);

	/*
		Autofuncs. They modify stuff in curState and also buttons from the ProcessedFrame.
		Autofuncs generally do NOT release any pressed buttons with an exception of Ducktap
		(for the sake of ducktapping while ducking the rest of the time).
	*/
	void Ducktap(const PlayerData& player, PositionType postype, const HLTAS::Frame& frame, CurrentState& curState, ProcessedFrame& out, TraceFunc traceFunc);
	void Autojump(PositionType postype, const HLTAS::Frame& frame, CurrentState& curState, ProcessedFrame& out);

	void Dbc(const PlayerData& player, const MovementVars& vars, PositionType postype, const HLTAS::Frame& frame, ProcessedFrame& out, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, CurrentState& curState, TraceFunc traceFunc);
	void LgagstDucktap(const PlayerData& player, const MovementVars& vars, PositionType postype, const HLTAS::Frame& frame, ProcessedFrame& out, bool reduceWishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, CurrentState& curState, TraceFunc traceFunc);
	void LgagstJump(const PlayerData& player, const MovementVars& vars, PositionType postype, const HLTAS::Frame& frame, ProcessedFrame& out, bool reduceWishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, CurrentState& curState, TraceFunc traceFunc);
	PositionType Strafe(PlayerData& player, const MovementVars& vars, PositionType postype, const HLTAS::Frame& frame, ProcessedFrame& out, bool reduceWishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, bool predictOrigin, TraceFunc traceFunc, float fractions[4] = nullptr, float normalzs[4] = nullptr);

	/*
		Returns the angle in radians - [0; Pi] - between velocity and wishdir that will
		result in maximal speed gain. Postype != WATER.

		Struct requirements:
			Velocity;
			Frametime, Accelerate or Airaccelerate, EntFriction.
	*/
	double MaxAccelTheta(const PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed);

	/*
		Returns the angle in radians - [-Pi; Pi) - between velocity and wishdir that will
		result in maximal speed gain into the given yaw - [-Pi; Pi). If velocity is zero, vel_yaw will
		be used in place of velocity angle. Postype != WATER.

		Struct requirements:
			Velocity;
			Frametime, Accelerate or Airaccelerate, EntFriction.
	*/
	double MaxAccelIntoYawTheta(const PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, double vel_yaw, double yaw);

	/*
		Returns the angle in radians - [0; Pi] - between velocity and wishdir that will
		result in maximal velocity angle change. Sets safeguard_yaw to true if the yaw safeguard
		is needed and to false otherwise. Postype != WATER.

		Struct requirements:
			Velocity;
			Frametime, Accelerate or Airaccelerate, EntFriction.
	*/
	double MaxAngleTheta(const PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, bool& safeguard_yaw);

	/*
		Computes the new velocity given unit acceleration vector and wishspeed
		and stores the result in player.Velocity. Postype != WATER.

		Struct requirements:
			Velocity;
			Frametime, Accelerate or Airaccelerate, EntFriction.
	*/
	void VectorFME(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const double a[2]);

	/*
		Computes the new velocity given unit acceleration vector and wishspeed,
		stores the result in player.Velocity, computes the new position taking the world into consideration
		and stores the result in player.Origin. Postype != WATER.

		Struct requirements:
		Velocity, Basevelocity, Origin;
		Frametime, Accelerate or Airaccelerate, EntFriction, EntGravity, Gravity.
	*/
	PositionType Move(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, TraceFunc traceFunc, bool calcVelocity = false, const double a[2] = nullptr, float fractions[4] = nullptr, float normalzs[4] = nullptr);

	/*
		Helpers for the movement prediction, do exactly what PM_FlyMove and PM_ClipVelocity do.
	*/
	void FlyMove(PlayerData& player, const MovementVars& vars, PositionType postype, TraceFunc traceFunc, float fractions[4] = nullptr, float normalzs[4] = nullptr);
	int ClipVelocity(float velocity[3], const float normal[3], float overbounce);

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
	double SideStrafeMaxAccel(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, HLTAS::Button& usedButton,
		double vel_yaw, bool right);
	double SideStrafeMaxAngle(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, HLTAS::Button& usedButton,
		double vel_yaw, bool right);

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
	double BestStrafeMaxAccel(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, HLTAS::Button& usedButton,
		double vel_yaw);
	double BestStrafeMaxAngle(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, HLTAS::Button& usedButton,
		double vel_yaw);

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
	double YawStrafeMaxAccel(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, HLTAS::Button& usedButton,
		double vel_yaw, double yaw);
	double YawStrafeMaxAngle(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, HLTAS::Button& usedButton,
		double vel_yaw, double yaw);

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
	double PointStrafe(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons, bool useGivenButtons, HLTAS::Button& usedButton,
		double vel_yaw, HLTAS::StrafeType type, double point[2], bool& strafed);
}
