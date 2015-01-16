#pragma once
#include "hltas.hpp"

namespace HLStrafe
{
	struct PlayerData {
		float Origin[3];
		float Velocity[3];
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
		result in maximal velocity angle change. Postype != WATER.

		Struct requirements:
			Velocity;
			Frametime, Accelerate or Airaccelerate, EntFriction.
	*/
	double MaxAngleTheta(const PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed);

	/*
		Computes the new velocity given unit acceleration vector and wishspeed
		and stores the result in player.Velocity. Postype != WATER.

		Struct requirements:
			Velocity;
			Frametime, Accelerate or Airaccelerate, EntFriction.
	*/
	void VectorFME(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, const double a[2]);

	/*
		Finds the best yaw to use taking the anglemod compensation into account, then
		strafes sideways with that yaw and returns it in radians, given fixed buttons.
		The resulting velocity is stored in player.Velocity.
		Uses the given yaw instead of the Velocity angle if Velocity is zero.
		Postype != WATER.

		Struct requirements:
			Velocity;
			Frametime, Accelerate or Airaccelerate, EntFriction.
	*/
	void SideStrafeMaxAccel(PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed, HLTAS::Button buttons,
		double& yaw, bool right);
}
