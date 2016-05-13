#pragma once
#include <cmath>

namespace HLStrafe
{
#ifndef M_PI
	static constexpr const double M_PI = 3.14159265358979323846;
#endif
	static constexpr const double M_RAD2DEG = 180 / M_PI;
	static constexpr const double M_DEG2RAD = M_PI / 180;
	static constexpr const double M_U_RAD = M_PI / 32768;
	static constexpr const double M_U_DEG = 360.0 / 65536;
	static constexpr const double M_INVU_RAD = 32768 / M_PI;
	static constexpr const double M_INVU_DEG = 65536.0 / 360;
	static constexpr const double M_U_DEG_HALF = 180.0 / 65536;

	static constexpr const float VEC_HULL_MIN[3] = { -16, -16, -36 };
	static constexpr const float VEC_HULL_MAX[3] = { 16,  16,  36 };
	static constexpr const float VEC_DUCK_HULL_MIN[3] = { -16, -16, -18 };
	static constexpr const float VEC_DUCK_HULL_MAX[3] = { 16,  16,  18 };

	inline double AngleModRad(double a)
	{
		return M_U_RAD * (static_cast<int>(a * M_INVU_RAD) & 0xffff);
	}

	inline double AngleModDeg(double a)
	{
		return M_U_DEG * (static_cast<int>(a * M_INVU_DEG) & 0xffff);
	}

	// Return angle in [-Pi; Pi).
	inline double NormalizeRad(double a)
	{
		a = std::fmod(a, M_PI * 2);
		if (a >= M_PI)
			a -= 2 * M_PI;
		else if (a < -M_PI)
			a += 2 * M_PI;
		return a;
	}

	inline double NormalizeDeg(double a)
	{
		a = std::fmod(a, 360.0);
		if (a >= 180.0)
			a -= 360.0;
		else if (a < -180.0)
			a += 360.0;
		return a;
	}
	
	/*
		Returns the difference between two angles in a [-180; 180) range.
	*/
	inline double AngleDifference(float oldang, float newang)
	{
		return NormalizeDeg(static_cast<double>(newang) - oldang);
	}

	// Convert both arguments to doubles.
	inline double Atan2(double a, double b)
	{
		return std::atan2(a, b);
	}

	inline double ButtonsPhi(HLTAS::Button button)
	{
		switch (button) {
		case HLTAS::Button::      FORWARD: return 0;
		case HLTAS::Button:: FORWARD_LEFT: return M_PI / 4;
		case HLTAS::Button::         LEFT: return M_PI / 2;
		case HLTAS::Button::    BACK_LEFT: return 3 * M_PI / 4;
		case HLTAS::Button::         BACK: return -M_PI;
		case HLTAS::Button::   BACK_RIGHT: return -3 * M_PI / 4;
		case HLTAS::Button::        RIGHT: return -M_PI / 2;
		case HLTAS::Button::FORWARD_RIGHT: return -M_PI / 4;
		default: return 0;
		}
	}

	inline HLTAS::Button GetBestButtons(double theta, bool right)
	{
		if (theta < M_PI / 8)
			return HLTAS::Button::FORWARD;
		else if (theta < 3 * M_PI / 8)
			return right ? HLTAS::Button::FORWARD_RIGHT : HLTAS::Button::FORWARD_LEFT;
		else if (theta < 5 * M_PI / 8)
			return right ? HLTAS::Button::RIGHT : HLTAS::Button::LEFT;
		else if (theta < 7 * M_PI / 8)
			return right ? HLTAS::Button::BACK_RIGHT : HLTAS::Button::BACK_LEFT;
		else
			return HLTAS::Button::BACK;
	}
}
