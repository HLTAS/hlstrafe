#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include <cmath>

#include "vec.hpp"

using namespace HLStrafe;

TEST_CASE( "vec2d tests", "[vec]" ) {
	vec2d v(1, 2);

	REQUIRE( v.x == 1.0 );
	REQUIRE( v.y == 2.0 );
	REQUIRE( v[0] == 1.0 );
	REQUIRE( v[1] == 2.0 );

	REQUIRE( v == vec2d(1, 2) );
	REQUIRE( v != vec2d(3, 2) );

	REQUIRE_FALSE( v.is_zero() );
	REQUIRE( vec2d(0, 0).is_zero() );

	REQUIRE( v.len_sq() == 5.0 );
	REQUIRE( v.len() == Approx(std::sqrt(5.0)) );

	REQUIRE( v + vec2d(2, 3) == vec2d(3, 5) );
	REQUIRE( v - vec2d(2, 3) == vec2d(-1, -1) );
	REQUIRE( v * 2 == vec2d(2, 4) );
	REQUIRE( 2 * v == vec2d(2, 4) );
	REQUIRE( (v / 2 - vec2d(0.5, 1)).len_sq() == Approx(0.0) );
	REQUIRE( -v == vec2d(-1, -2) );

	v += vec2d(2, 3);
	REQUIRE( v == vec2d(3, 5) );

	v -= vec2d(2, 3);
	REQUIRE( v == vec2d(1, 2) );

	v *= 2;
	REQUIRE( v == vec2d(2, 4) );

	v /= 2;
	REQUIRE( v == vec2d(1, 2) );

	REQUIRE( v.dot(-v) == -5.0 );

	auto length = v.len();
	REQUIRE( v.normalize() == length );
	REQUIRE( (v - vec2d(std::sqrt(1/5.0), std::sqrt(4/5.0))).len_sq() == Approx(0.0) );
}

TEST_CASE( "vec tests", "[vec]" ) {
	vec v(1, 2, 3);

	REQUIRE( v.x == 1.0 );
	REQUIRE( v.y == 2.0 );
	REQUIRE( v.z == 3.0 );
	REQUIRE( v[0] == 1.0 );
	REQUIRE( v[1] == 2.0 );
	REQUIRE( v[2] == 3.0 );

	REQUIRE( v == vec(1, 2, 3) );
	REQUIRE( v != vec(3, 2, 1) );

	REQUIRE_FALSE( v.is_zero() );
	REQUIRE( vec(0, 0, 0).is_zero() );

	REQUIRE( v.len_sq() == 14.0 );
	REQUIRE( v.len() == Approx(std::sqrt(14.0)) );

	REQUIRE( v + vec(2, 3, 4) == vec(3, 5, 7) );
	REQUIRE( v - vec(2, 3, 4) == vec(-1, -1, -1) );
	REQUIRE( v * 2 == vec(2, 4, 6) );
	REQUIRE( 2 * v == vec(2, 4, 6) );
	REQUIRE( (v / 2 - vec(0.5, 1, 1.5)).len_sq() == Approx(0.0) );
	REQUIRE( -v == vec(-1, -2, -3) );

	v += vec(2, 3, 4);
	REQUIRE( v == vec(3, 5, 7) );

	v -= vec(2, 3, 4);
	REQUIRE( v == vec(1, 2, 3) );

	v *= 2;
	REQUIRE( v == vec(2, 4, 6) );

	v /= 2;
	REQUIRE( v == vec(1, 2, 3) );

	REQUIRE( v.dot(-v) == -14.0 );
	REQUIRE( v.cross(vec(5, 6, 7)) == vec(-4, 8, -4) );

	REQUIRE( v.as_2d() == vec2d(1, 2) );
	REQUIRE( vec(0, 0, 1).as_2d().is_zero() );

	auto length = v.len();
	REQUIRE( v.normalize() == length );
	REQUIRE( (v - vec(std::sqrt(1/14.0), std::sqrt(2/7.0), std::sqrt(9/14.0))).len_sq() == Approx(0.0) );
}
