#include <cassert>
#include <cmath>
#include <iostream>

#include "vec.hpp"

bool vec2d::is_zero() const
{
	return x == 0.0 && y == 0.0;
}

double vec2d::len_sq() const
{
	return x * x + y * y;
}

double vec2d::len() const
{
	return std::hypot(x, y);
}

double vec2d::normalize()
{
	auto length = len();

	x /= length;
	y /= length;

	return length;
}

double vec2d::dot(const vec2d& v) const
{
	return x * v.x + y * v.y;
}

bool vec2d::operator==(const vec2d& rhs) const
{
	return x == rhs.x && y == rhs.y;
}

bool vec2d::operator!=(const vec2d& rhs) const
{
	return !(*this == rhs);
}

vec2d vec2d::operator-() const
{
	return vec2d(-x, -y);
}

vec2d& vec2d::operator+=(const vec2d& rhs)
{
	x += rhs.x;
	y += rhs.y;

	return *this;
}

vec2d& vec2d::operator-=(const vec2d& rhs)
{
	x -= rhs.x;
	y -= rhs.y;

	return *this;
}

vec2d& vec2d::operator*=(double scale)
{
	x *= scale;
	y *= scale;

	return *this;
}

vec2d& vec2d::operator/=(double scale)
{
	x /= scale;
	y /= scale;

	return *this;
}

vec2d operator+(vec2d lhs, const vec2d& rhs)
{
	lhs += rhs;
	return lhs;
}

vec2d operator-(vec2d lhs, const vec2d& rhs)
{
	lhs -= rhs;
	return lhs;
}

vec2d operator*(vec2d lhs, double scale)
{
	lhs *= scale;
	return lhs;
}

vec2d operator*(double scale, vec2d rhs)
{
	rhs *= scale;
	return rhs;
}

vec2d operator/(vec2d lhs, double scale)
{
	lhs /= scale;
	return lhs;
}

bool vec::is_zero() const
{
	return x == 0.0 && y == 0.0 && z == 0.0;
}

double vec::len_sq() const
{
	return x * x + y * y + z * z;
}

double vec::len() const
{
	return std::sqrt(len_sq());
}

double vec::normalize()
{
	auto length = len();

	x /= length;
	y /= length;
	z /= length;

	return length;
}

double vec::dot(const vec& v) const
{
	return x * v.x + y * v.y + z * v.z;
}

vec vec::cross(const vec& v) const
{
	return vec(
		y * v.z - z * v.y,
		z * v.x - x * v.z,
		x * v.y - y * v.x
	);
}

bool vec::operator==(const vec& rhs) const
{
	return x == rhs.x && y == rhs.y && z == rhs.z;
}

bool vec::operator!=(const vec& rhs) const
{
	return !(*this == rhs);
}

vec2d& vec::as_2d()
{
	return static_cast<vec2d&>(*this);
}

const vec2d& vec::as_2d() const
{
	return static_cast<const vec2d&>(*this);
}

vec vec::operator-() const
{
	return vec(-x, -y, -z);
}

vec& vec::operator+=(const vec& rhs)
{
	x += rhs.x;
	y += rhs.y;
	z += rhs.z;

	return *this;
}

vec& vec::operator-=(const vec& rhs)
{
	x -= rhs.x;
	y -= rhs.y;
	z -= rhs.z;

	return *this;
}

vec& vec::operator*=(double scale)
{
	x *= scale;
	y *= scale;
	z *= scale;

	return *this;
}

vec& vec::operator/=(double scale)
{
	x /= scale;
	y /= scale;
	z /= scale;

	return *this;
}

vec operator+(vec lhs, const vec& rhs)
{
	lhs += rhs;
	return lhs;
}

vec operator-(vec lhs, const vec& rhs)
{
	lhs -= rhs;
	return lhs;
}

vec operator*(vec lhs, double scale)
{
	lhs *= scale;
	return lhs;
}

vec operator*(double scale, vec rhs)
{
	rhs *= scale;
	return rhs;
}

vec operator/(vec lhs, double scale)
{
	lhs /= scale;
	return lhs;
}
