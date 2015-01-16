#pragma once
#include <cmath>
#include <cstddef>

namespace HLStrafe
{
#ifndef M_PI
	const double M_PI = 3.14159265358979323846;
#endif
	const double M_RAD2DEG = 180 / M_PI;
	const double M_DEG2RAD = M_PI / 180;
	const double M_U_RAD = M_PI / 32768;
	const double M_INVU_RAD = 32768 / M_PI;

	template<typename T, std::size_t size = 3>
	inline bool IsZero(const T vec[])
	{
		for (std::size_t i = 0; i < size; ++i)
			if (vec[i] != 0)
				return false;

		return true;
	}

	template<typename T, std::size_t size = 3>
	inline double Length(const T vec[])
	{
		double squared = 0.0;
		for (std::size_t i = 0; i < size; ++i)
			squared += vec[i] * vec[i];
		return std::sqrt(squared);
	}

	template<typename T1, typename T2, std::size_t size = 3>
	inline double DotProduct(const T1 a[], const T2 b[])
	{
		double result = 0.0;
		for (size_t i = 0; i < size; ++i)
			result += a[i] * b[i];
		return result;
	}

	inline double AngleModRad(double a)
	{
		return M_U_RAD * ((int)(a * M_INVU_RAD) & 0xffff);
	}
}
