#pragma once
#include <cmath>
#include <cstddef>

namespace HLStrafe
{
	const double M_PI = 3.14159265358979323846;
	const double M_RAD2DEG = 180 / M_PI;
	const double M_DEG2RAD = M_PI / 180;

	template<typename T, std::size_t size>
	inline bool IsZero(const T vec[size])
	{
		for (std::size_t i = 0; i < size; ++i)
			if (vec[i] != 0)
				return false;

		return true;
	}

	template<typename T, std::size_t size>
	inline double Length(const T vec[size])
	{
		double squared = 0.0;
		for (std::size_t i = 0; i < size; ++i)
			squared += vec[i] * vec[i];
		return std::sqrt(squared);
	}
}
