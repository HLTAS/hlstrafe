#pragma once
#include <functional>

#include "hltas.hpp"

namespace HLStrafe
{
	struct PlayerData {
		float Origin[3];
		float Velocity[3];
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
		bool Jump;
		bool Duck;
		unsigned AutojumpsLeft;
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
		For Yaw takes the anglemod compensation into account.
	*/
	double GetPitchDifference(float oldpitch, float newpitch);
	double GetYawDifference(float oldyaw, float newyaw);

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
		Changes the player data the same way as PM_Jump would, returns a new postype.
		Changes the processed frame in case of some duck-when autofuncs.
	*/
	PositionType PredictJump(PlayerData& player, PositionType postype, const MovementVars& vars, const CurrentState& curState, ProcessedFrame& out);

	/*
		Applies the ground friction the same way as PM_Friction would, changing player.Velocity.
	*/
	void Friction(PlayerData& player, PositionType postype, const MovementVars& vars, TraceFunc traceFunc);

	void Autojump(PositionType postype, const HLTAS::Frame& frame, CurrentState& curState, ProcessedFrame& out);

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
		Finds the best yaw to use for the corresponding strafe type taking the anglemod compensation into account, then
		strafes sideways with that yaw and returns it in radians, given fixed buttons.
		The resulting velocity is stored in player.Velocity.
		Uses vel_yaw instead of the Velocity angle if Velocity is zero.
		Postype != WATER.

		Struct requirements:
			Velocity;
			Frametime, Accelerate or Airaccelerate, EntFriction.
	*/
	double SideStrafeMaxAccel(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons,
		double vel_yaw, bool right);
	double SideStrafeMaxAngle(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons,
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
	double BestStrafeMaxAccel(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons,
		double vel_yaw);
	double BestStrafeMaxAngle(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons,
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
	double YawStrafeMaxAccel(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons,
		double vel_yaw, double yaw);
	double YawStrafeMaxAngle(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons,
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
	double PointStrafe(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const HLTAS::StrafeButtons& strafeButtons,
		double vel_yaw, HLTAS::StrafeType type, float point[2], bool& strafed);
}
