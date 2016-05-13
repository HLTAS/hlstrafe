#include "hlstrafe.hpp"
#include "util.hpp"
#include "vec.hpp"

namespace HLStrafe
{

using namespace autofuncs;

namespace prediction
{
	PositionType GetPositionType(PlayerData& player, TraceFunc traceFunc)
	{
		// TODO: Check water. If we're under water, return here.

		// Check ground.
		if (player.Velocity[2] > 180)
			return PositionType::AIR;

		vec point = player.Origin;
		point.z -= 2;

		auto tr = traceFunc(player.Origin, point, player.Ducking ? HullType::DUCKED : HullType::NORMAL);

		if (tr.PlaneNormal.z < 0.7 || tr.Entity == -1)
			return PositionType::AIR;

		if (!tr.StartSolid && !tr.AllSolid)
			player.Origin = tr.EndPos;

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

	PositionType PredictDuck(
		PlayerData& player,
		const MovementVars& vars,
		PositionType postype,
		CurrentState& curState,
		const ProcessedFrame& out,
		TraceFunc traceFunc)
	{
		if (!out.Duck
			&& !player.InDuckAnimation
			&& !player.Ducking)
			return postype;

		if (out.Duck) {
			if (!curState.Duck && !player.Ducking) {
				player.DuckTime = 1000;
				player.InDuckAnimation = true;

				if (vars.Frametime == 0.f) {
					if (curState.PredictThis == State0ms::NOTHING)
						curState.PredictThis = State0ms::DUCKED;
					else
						curState.PredictThis = State0ms::UNDUCKED_AND_DUCKED;
				}
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
			vec newOrigin = player.Origin;

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
					player.Origin = newOrigin;

					if (vars.Frametime == 0.f)
						curState.PredictThis = State0ms::UNDUCKED;

					return GetPositionType(player, traceFunc);
				}
			}
		}

		return postype;
	}

	PositionType PredictJump(
		PlayerData& player,
		PositionType postype,
		const MovementVars& vars,
		const HLTAS::Frame& frame,
		CurrentState& curState,
		ProcessedFrame& out,
		TraceFunc traceFunc,
		bool decreaseDwjTimes)
	{
		assert(postype != PositionType::WATER);

		if (!out.Jump // Not pressing Jump.
			|| curState.Jump // Jump was already down.
			|| postype != PositionType::GROUND) // Not on ground.
			return postype;

		// Check a lot of stuff for DWJ to make sure we don't "spend" DWJ when we are already ducking.
		if ((frame.Dwj || curState.DwjsLeft)
			&& !curState.Duck && !out.Duck
			&& !player.InDuckAnimation && !player.Ducking
			&& player.DuckTime == 0.0f) {
			auto out_backup = ProcessedFrame(out);
			auto curState_backup = CurrentState(curState);

			out.Duck = true;
			postype = PredictDuck(player, vars, postype, curState, out, traceFunc);

			if (postype != PositionType::GROUND) {
				out = out_backup;
				curState = curState_backup;
			} else {
				if (decreaseDwjTimes && curState.DwjsLeft)
					curState.DwjsLeft--;
			}
		}

		if (vars.Bhopcap) {
			auto maxscaledspeed = 1.7f * vars.Maxspeed;

			if (maxscaledspeed > 0) {
				auto speed = player.Velocity.len();

				if (speed > maxscaledspeed)
					player.Velocity *= (maxscaledspeed / speed) * 0.65;
			}
		}

		// We don't care about the vertical velocity after the jump prediction.
		// Except when we need to predict a 0ms frame, then we need the check in Move to correctly
		// report that we are not onground (since we just jumped and our vertical speed is bigger than 180).
		if (player.InDuckAnimation || player.Ducking) {
			if (player.HasLJModule && out.Duck && player.DuckTime > 0 && player.Velocity.len_sq() > 50 * 50) {
				player.Velocity.x = std::cos(player.Viewangles[1] * M_DEG2RAD) * std::cos(player.Viewangles[0] * M_DEG2RAD) * 350 * 1.6;
				player.Velocity.y = std::sin(player.Viewangles[1] * M_DEG2RAD) * std::cos(player.Viewangles[0] * M_DEG2RAD) * 350 * 1.6;
			}
		}
		player.Velocity.z = std::sqrt(2 * 800 * 45.0);

		CheckVelocity(player, vars);

		return PositionType::AIR;
	}

	void Friction(PlayerData& player, PositionType postype, const MovementVars& vars, TraceFunc traceFunc)
	{
		player.SpeedBeforeFriction = player.Velocity.as_2d().len();

		if (postype != PositionType::GROUND)
			return;

		// Doing all this in floats, mismatch is too real otherwise.
		auto speed = static_cast<float>(player.Velocity.len());
		if (speed < 0.1)
			return;

		auto friction = float{ vars.Friction * vars.EntFriction };

		vec start = player.Origin + player.Velocity / speed * 16;
		start.z = player.Origin.z + (player.Ducking ? VEC_DUCK_HULL_MIN[2] : VEC_HULL_MIN[2]);

		vec stop = start;
		stop.z -= 34;

		auto tr = traceFunc(start, stop, player.Ducking ? HullType::DUCKED : HullType::NORMAL);
		if (tr.Fraction == 1.0)
			friction *= vars.Edgefriction;

		auto control = (speed < vars.Stopspeed) ? vars.Stopspeed : speed;
		auto drop = control * friction * vars.Frametime;
		auto newspeed = std::max(speed - drop, 0.f);
		player.Velocity *= newspeed / speed;
	}

	void VectorFME(
		PlayerData& player,
		const MovementVars& vars,
		PositionType postype,
		double wishspeed,
		const vec2d& wishdir)
	{
		assert(postype != PositionType::WATER);

		bool onground = (postype == PositionType::GROUND);
		double wishspeed_capped = onground ? wishspeed : 30;

		double tmp = wishspeed_capped - player.Velocity.as_2d().dot(wishdir);
		if (tmp <= 0.0)
			return;

		double accel = onground ? vars.Accelerate : vars.Airaccelerate;
		double accelspeed = accel * wishspeed * vars.EntFriction * vars.Frametime;
		if (accelspeed <= tmp)
			tmp = accelspeed;

		player.Velocity.as_2d() += wishdir * tmp;
	}

	int ClipVelocity(vec& velocity, const vec& normal, float overbounce)
	{
		static constexpr const auto STOP_EPSILON = 0.1;

		auto backoff = velocity.dot(normal) * overbounce;

		for (size_t i = 0; i < 3; ++i) {
			auto change = normal[i] * backoff;
			velocity[i] -= change;

			if (velocity[i] > -STOP_EPSILON && velocity[i] < STOP_EPSILON)
				velocity[i] = 0;
		}

		if (normal.z > 0)
			return 1;
		else if (normal.z == 0)
			return 2;
		else
			return 0;
	}

	void FlyMove(
		PlayerData& player,
		const MovementVars& vars,
		PositionType postype,
		TraceFunc traceFunc,
		float fractions[4],
		float normalzs[4])
	{
		enum {
			MAX_BUMPS = 4,
			MAX_CLIP_PLANES = 5
		};

		vec originalVelocity = player.Velocity;
		vec savedVelocity = player.Velocity;

		auto timeLeft = vars.Frametime;
		auto allFraction = 0.0f;
		auto numPlanes = 0;
		auto blockedState = 0;
		vec planes[MAX_CLIP_PLANES];

		for (auto bumpCount = 0; bumpCount < MAX_BUMPS; ++bumpCount) {
			if (player.Velocity.is_zero())
				break;

			vec end = player.Origin + timeLeft * player.Velocity;

			auto tr = traceFunc(player.Origin, end, player.Ducking ? HullType::DUCKED : HullType::NORMAL);
			if (fractions)
				fractions[bumpCount] = tr.Fraction;
			if (normalzs)
				normalzs[bumpCount] = tr.PlaneNormal.z;

			allFraction += tr.Fraction;
			if (tr.AllSolid) {
				player.Velocity = vec(0, 0, 0);
				blockedState = 4;
				break;
			}
			if (tr.Fraction > 0) {
				player.Origin = tr.EndPos;
				savedVelocity = player.Velocity;
				numPlanes = 0;
			}
			if (tr.Fraction == 1)
				break;

			if (tr.PlaneNormal.z > 0.7)
				blockedState |= 1;
			else if (tr.PlaneNormal.z == 0)
				blockedState |= 2;

			timeLeft -= timeLeft * tr.Fraction;

			if (numPlanes >= MAX_CLIP_PLANES) {
				player.Velocity = vec(0, 0, 0);
				break;
			}

			planes[numPlanes] = tr.PlaneNormal;
			numPlanes++;

			if (postype != PositionType::GROUND || vars.EntFriction != 1) {
				for (auto i = 0; i < numPlanes; ++i)
					if (planes[i][2] > 0.7)
						ClipVelocity(savedVelocity, planes[i], 1);
					else
						ClipVelocity(savedVelocity, planes[i], static_cast<float>(1.0 + vars.Bounce * (1 - vars.EntFriction)));

				player.Velocity = savedVelocity;
			} else {
				int i = 0;
				for (i = 0; i < numPlanes; ++i) {
					player.Velocity = savedVelocity;
					ClipVelocity(player.Velocity, planes[i], 1);

					int j;
					for (j = 0; j < numPlanes; ++j)
						if (j != i)
							if (player.Velocity.dot(planes[j]) < 0)
								break;

					if (j == numPlanes)
						break;
				}

				if (i == numPlanes) {
					if (numPlanes != 2) {
						player.Velocity = vec(0, 0, 0);
						break;
					}

					vec dir = planes[0].cross(planes[1]);
					player.Velocity = dir * dir.dot(player.Velocity);
				}

				if (player.Velocity.dot(originalVelocity) <= 0) {
					player.Velocity = vec(0, 0, 0);
					break;
				}
			}
		}

		if (allFraction == 0)
			player.Velocity = vec(0, 0, 0);
	}

	PositionType Move(
		PlayerData& player,
		const MovementVars& vars,
		PositionType postype,
		double wishspeed,
		TraceFunc traceFunc,
		bool calcVelocity,
		const vec2d& wishdir,
		float fractions[4],
		float normalzs[4])
	{
		assert(postype != PositionType::WATER);

		bool onground = (postype == PositionType::GROUND);
		CheckVelocity(player, vars);

		// AddCorrectGravity
		float entGravity = vars.EntGravity;
		if (entGravity == 0.0f)
			entGravity = 1.0f;
		player.Velocity.z -= entGravity * vars.Gravity * 0.5 * vars.Frametime;
		player.Velocity.z += player.Basevelocity.z * vars.Frametime;
		player.Basevelocity.z = 0;
		CheckVelocity(player, vars);

		// Move
		wishspeed = std::min(wishspeed, static_cast<double>(vars.Maxspeed));
		if (onground)
			player.Velocity.z = 0;

		// Accelerate
		if (calcVelocity)
			VectorFME(player, vars, postype, wishspeed, wishdir);

		// Move
		player.Velocity += player.Basevelocity;

		if (onground) {
			// WalkMove
			auto spd = player.Velocity.len();

			if (spd < 1) {
				player.Velocity = vec(0, 0, 0);
			} else {
				vec dest = player.Origin + player.Velocity * vars.Frametime;

				auto tr = traceFunc(player.Origin, dest, player.Ducking ? HullType::DUCKED : HullType::NORMAL);
				if (tr.Fraction == 1.0f) {
					player.Origin = tr.EndPos;
				} else {
					// Figure out the end position when trying to walk up a step.
					auto playerUp = PlayerData(player);
					dest.z += vars.Stepsize;

					tr = traceFunc(playerUp.Origin, dest, player.Ducking ? HullType::DUCKED : HullType::NORMAL);
					if (!tr.StartSolid && !tr.AllSolid)
						playerUp.Origin = tr.EndPos;

					FlyMove(playerUp, vars, postype, traceFunc, fractions, normalzs);

					dest = playerUp.Origin;
					dest.z -= vars.Stepsize;

					tr = traceFunc(playerUp.Origin, dest, player.Ducking ? HullType::DUCKED : HullType::NORMAL);
					if (!tr.StartSolid && !tr.AllSolid)
						playerUp.Origin = tr.EndPos;

					// Figure out the end position when _not_ trying to walk up a step.
					auto playerDown = PlayerData(player);
					FlyMove(playerDown, vars, postype, traceFunc);

					// Take whichever move was the furthest.
					auto downdist = (playerDown.Origin - player.Origin).as_2d().len_sq();
					auto updist = (playerUp.Origin - player.Origin).as_2d().len_sq();

					if (tr.PlaneNormal.z < 0.7 || downdist > updist) {
						player.Origin = playerDown.Origin;
						player.Velocity = playerDown.Velocity;
					} else {
						player.Origin = playerUp.Origin;
						player.Velocity = playerUp.Velocity;
						player.Velocity.z = playerDown.Velocity.z;
					}
				}
			}
		} else {
			// AirMove
			FlyMove(player, vars, postype, traceFunc, fractions, normalzs);
		}

		postype = GetPositionType(player, traceFunc);
		player.Velocity -= player.Basevelocity;

		CheckVelocity(player, vars);

		if (postype != PositionType::GROUND && postype != PositionType::WATER) {
			// FixupGravityVelocity
			player.Velocity.z -= entGravity * vars.Gravity * 0.5 * vars.Frametime;
			CheckVelocity(player, vars);
		}

		return postype;
	}


	void CheckIfNextFrameShouldBe0ms(
		const PlayerData& player,
		const MovementVars& vars,
		const HLTAS::Frame& frame,
		PositionType postype,
		ProcessedFrame& out,
		const HLTAS::StrafeButtons& strafeButtons,
		bool useGivenButtons,
		const CurrentState& curState,
		TraceFunc traceFunc)
	{
		if (postype != PositionType::GROUND
			|| !frame.Ducktap || !frame.GetDucktap0ms()
			|| player.InDuckAnimation)
			return;

		if (frame.Lgagst || curState.LgagstsLeft) {
			// Do the lgagst check. Check current player state because our ducktap is going to be 0ms.
			if (player.Velocity.as_2d().len() < curState.LgagstMinSpeed)
				return;

			auto ground = PlayerData(player);
			Friction(ground, postype, vars, traceFunc);
			CheckVelocity(ground, vars);
			auto out_temp = ProcessedFrame(out);
			Strafe(ground, vars, postype, frame, out_temp, false, strafeButtons, useGivenButtons, false, traceFunc);

			auto air = PlayerData(player);
			out_temp = ProcessedFrame(out);
			auto curStateCopy = CurrentState(curState);
			air.InDuckAnimation = true;
			postype = PredictDuck(air, vars, postype, curStateCopy, out_temp, traceFunc);
			Strafe(air, vars, postype, frame, out_temp, false, strafeButtons, useGivenButtons, false, traceFunc);

			auto l_gr = ground.Velocity.as_2d().len();
			auto l_air = air.Velocity.as_2d().len();
			out.NextFrameIs0ms = (l_air > l_gr);
		} else {
			out.NextFrameIs0ms = true;
		}
	}

	PositionType PredictPast0msFrames(
		PlayerData& player,
		const MovementVars& vars,
		PositionType postype,
		const ProcessedFrame& out,
		const CurrentState& curState,
		TraceFunc traceFunc)
	{
		auto out_copy = ProcessedFrame{ out };
		auto curState_copy = CurrentState{ curState };

		if (curState.PredictThis == State0ms::DUCKED) {
			out_copy.Duck = true;
			curState_copy.Duck = false;

			postype = PredictDuck(player, vars, postype, curState_copy, out_copy, traceFunc);
		} else if (curState.PredictThis == State0ms::UNDUCKED) {
			out_copy.Duck = false;

			postype = PredictDuck(player, vars, postype, curState_copy, out_copy, traceFunc);
		} else if (curState.PredictThis == State0ms::UNDUCKED_AND_DUCKED) {
			out_copy.Duck = false;

			postype = PredictDuck(player, vars, postype, curState_copy, out_copy, traceFunc);

			out_copy.Duck = true;
			curState_copy.Duck = false;

			postype = PredictDuck(player, vars, postype, curState_copy, out_copy, traceFunc);
		}

		return postype;
	}
}

} // namespace HLStrafe
