#pragma once
#include "hltas.hpp"

namespace HLStrafe
{
	struct PlayerData {
		float Origin[3];
		float Velocity[3];
		float Viewangles[3];
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
	};

	// On ground, in the air or underwater.
	enum class PositionType {
		GROUND = 0,
		AIR,
		WATER
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

	/*
		All-in-one function that does everything and returns the buttons,
		viewangles and FSU.
	*/
	ProcessedFrame MainFunc(const PlayerData& player, const MovementVars& vars, const HLTAS::Frame& frame);

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
	double SideStrafeMaxAccel(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, HLTAS::Button buttons,
		double vel_yaw, bool right);
	double SideStrafeMaxAngle(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, HLTAS::Button buttons,
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
	double BestStrafeMaxAccel(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, HLTAS::Button buttons,
		double vel_yaw);
	double BestStrafeMaxAngle(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, HLTAS::Button buttons,
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
	double YawStrafeMaxAccel(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, HLTAS::Button buttons,
		double vel_yaw, double yaw);
	double YawStrafeMaxAngle(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, HLTAS::Button buttons,
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
	double PointStrafe(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, HLTAS::Button buttons,
		double vel_yaw, HLTAS::StrafeType type, float point[2], bool& strafed);
}
