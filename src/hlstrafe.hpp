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
		Returns the angle in radians between velocity and wishdir that will
		result in maximal speed gain. Postype != WATER.

		Struct requirements:
			Velocity;
			Frametime, Accelerate or Airaccelerate, EntFriction.
	*/
	double MaxAccelTheta(const PlayerData& player, const MovementVars& vars, PositionType postype, double wishspeed);

	/*
		Compute new velocity given unit acceleration vector and wishspeed.
		Player.Velocity will be modified. Postype != WATER.

		Struct requirements:
			Velocity;
			Frametime, Accelerate or Airaccelerate, EntFriction.
	*/
	void VectorFME(PlayerData& player, const MovementVars& vars, PositionType postype, const double a[2], double wishspeed);

	/*
		Strafe sideways and returns yaw in radians, given fixed buttons.
		Velocity will be modified. Postype != WATER.

		Struct requirements:
			Velocity;
			Frametime, Accelerate or Airaccelerate, EntFriction.
	*/
	double SideStrafe(HLTAS::Button buttons, PlayerData& player, const MovementVars& vars,
		PositionType postype, double wishspeed, double yaw, double theta, bool right);
}
