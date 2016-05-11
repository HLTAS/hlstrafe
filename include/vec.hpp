#pragma once

struct vec2d {
	double x, y;

	vec2d() {}
	vec2d(double x_, double y_) : x(x_), y(y_) {}

	bool is_zero() const;
	double len_sq() const;
	double len() const;
	double normalize();

	double dot(const vec2d& v) const;

	bool operator==(const vec2d& rhs) const;
	bool operator!=(const vec2d& rhs) const;

	vec2d operator-() const;
	vec2d& operator+=(const vec2d& rhs);
	vec2d& operator-=(const vec2d& rhs);
	vec2d& operator*=(double scale);
	vec2d& operator/=(double scale);

	friend vec2d operator+(vec2d lhs, const vec2d& rhs);
	friend vec2d operator-(vec2d lhs, const vec2d& rhs);
	friend vec2d operator*(vec2d lhs, double scale);
	friend vec2d operator*(double scale, vec2d rhs);
	friend vec2d operator/(vec2d lhs, double scale);
};

struct vec : vec2d {
	double z;

	vec() {}
	vec(double x_, double y_, double z_) : vec2d(x_, y_), z(z_) {}

	bool is_zero() const;
	double len_sq() const;
	double len() const;
	double normalize();

	double dot(const vec& v) const;
	vec cross(const vec& v) const;

	bool operator==(const vec& rhs) const;
	bool operator!=(const vec& rhs) const;

	vec2d& as_2d();
	const vec2d& as_2d() const;

	vec operator-() const;
	vec& operator+=(const vec& rhs);
	vec& operator-=(const vec& rhs);
	vec& operator*=(double scale);
	vec& operator/=(double scale);

	friend vec operator+(vec lhs, const vec& rhs);
	friend vec operator-(vec lhs, const vec& rhs);
	friend vec operator*(vec lhs, double scale);
	friend vec operator*(double scale, vec rhs);
	friend vec operator/(vec lhs, double scale);
};
