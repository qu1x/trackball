// Copyright © 2021-2026 Rouven Spreckels <rs@qu1x.dev>
// SPDX-License-Identifier: MIT OR Apache-2.0
/// \file
/// \brief API source file.

#include "trackball.h"

#include <tgmath.h>

/// \cond
#define PI_2_f 1.57079632679489661923f
#define PI_2_d 1.57079632679489661923
#define PI_2_l 1.570796326794896619231321691639751442l
/// \endcond

/// Scalar.
typedef float num_f;

/// Scalar.
typedef double num_d;

/// Scalar.
typedef long double num_l;

/// Vector inclusive its length or quaternion.
typedef struct {
	/// x-component.
	num_f x;
	/// y-component.
	num_f y;
	/// z-component.
	num_f z;
	/// w-component of quaternion or length of vector.
	num_f w;
} vec_f;

/// Vector inclusive its length or quaternion.
typedef struct {
	/// x-component.
	num_d x;
	/// y-component.
	num_d y;
	/// z-component.
	num_d z;
	/// w-component of quaternion or length of vector.
	num_d w;
} vec_d;

/// Vector inclusive its length or quaternion.
typedef struct {
	/// x-component.
	num_l x;
	/// y-component.
	num_l y;
	/// z-component.
	num_l z;
	/// w-component of quaternion or length of vector.
	num_l w;
} vec_l;

/// \cond
// Ensures casting between structure and array of scalar.
_Static_assert(sizeof (vec_f) == sizeof (num_f) * 4, "weird padding");
_Static_assert(sizeof (vec_d) == sizeof (num_d) * 4, "weird padding");
_Static_assert(sizeof (vec_l) == sizeof (num_l) * 4, "weird padding");
/// \endcond

/// Clamps value between minimum and maximum.
#define clamp(val, min, max) _Generic((val), \
	num_f: clamp_f, \
	num_d: clamp_d, \
	num_l: clamp_l \
)(val, min, max)

/// Associated implementation of generic selection macro \ref clamp.
static num_f
clamp_f(num_f val, num_f min, num_f max) {
	return val < min ? min : val > max ? max : val;
}

/// Associated implementation of generic selection macro \ref clamp.
static num_d
clamp_d(num_d val, num_d min, num_d max) {
	return val < min ? min : val > max ? max : val;
}

/// Associated implementation of generic selection macro \ref clamp.
static num_l
clamp_l(num_l val, num_l min, num_l max) {
	return val < min ? min : val > max ? max : val;
}

/// Computes vector norm before normalizing it.
#define normalize(v) _Generic((v), \
	vec_f*: normalize_f, \
	vec_d*: normalize_d, \
	vec_l*: normalize_l \
)(v)

/// Associated implementation of generic selection macro \ref normalize.
static num_f
normalize_f(vec_f* v) {
	v->w = sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
	if (v->w) {
		v->x /= v->w;
		v->y /= v->w;
		v->z /= v->w;
	}
	return v->w;
}

/// Associated implementation of generic selection macro \ref normalize.
static num_d
normalize_d(vec_d* v) {
	v->w = sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
	if (v->w) {
		v->x /= v->w;
		v->y /= v->w;
		v->z /= v->w;
	}
	return v->w;
}

/// Associated implementation of generic selection macro \ref normalize.
static num_l
normalize_l(vec_l* v) {
	v->w = sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
	if (v->w) {
		v->x /= v->w;
		v->y /= v->w;
		v->z /= v->w;
	}
	return v->w;
}

/// Computes cross product.
#define cross(a, b) _Generic((a), \
	vec_f: cross_f, \
	vec_d: cross_d, \
	vec_l: cross_l \
)(a, b)

/// Associated implementation of generic selection macro \ref cross.
static vec_f
cross_f(vec_f a, vec_f b) {
	return (vec_f) {
		.x = a.y * b.z - a.z * b.y,
		.y = a.z * b.x - a.x * b.z,
		.z = a.x * b.y - a.y * b.x,
	};
}

/// Associated implementation of generic selection macro \ref cross.
static vec_d
cross_d(vec_d a, vec_d b) {
	return (vec_d) {
		.x = a.y * b.z - a.z * b.y,
		.y = a.z * b.x - a.x * b.z,
		.z = a.x * b.y - a.y * b.x,
	};
}

/// Associated implementation of generic selection macro \ref cross.
static vec_l
cross_l(vec_l a, vec_l b) {
	return (vec_l) {
		.x = a.y * b.z - a.z * b.y,
		.y = a.z * b.x - a.x * b.z,
		.z = a.x * b.y - a.y * b.x,
	};
}

// Associated implementation of generic selection macro `trackball_orbit`.
void
trackball_orbit_f(
	num_f xyzw[static restrict 4],
	num_f xyzm[static restrict 4],
	const num_f xy[static restrict 2],
	const num_f wh[static restrict 2]
) {
	// Identity quaternion with no previous position or zero displacement.
	vec_f* rot = (vec_f*)xyzw;
	*rot = (vec_f) { .w = 1.0f };
	// Previous trackball point with zero length for no position.
	vec_f* old = (vec_f*)xyzm;
	// Maximum centered position as half the screen's width and height.
	vec_f max = { .x = wh[0] * 0.5f, .y = wh[1] * 0.5f };
	// Current centered position from left to right and bottom to top.
	vec_f pos = {
		.x = clamp(xy[0], 0.0f, wh[0]) - max.x,
		.y = max.y - clamp(xy[1], 0.0f, wh[1]),
	};
	// Trackball's radius mapping maximum of half the screen's width or height
	// onto quarter of its circumference.
	max.w = fmax(max.x, max.y) / PI_2_f;
	// Polar angle of current position, beyond equator for screen corners.
	pos.w = normalize(&pos) / max.w;
	num_f s = sin(pos.w), c = cos(pos.w);
	// Exponential map of current position onto trackball.
	vec_f exp = { .x = s * pos.x, .y = s * pos.y, .z = c, .w = 1.0f };
	// Get previous and replace with current trackball point.
	vec_f vec = *old;
	*old = exp;
	// No rotation with no previous position or zero displacement.
	if (!vec.w || (vec.x == exp.x && vec.y == exp.y && vec.z == exp.z))
		return;
	// Shortest arc from current to previous trackball point as rotation of
	// camera which in turn carries previous to current trackball point,
	// keeping it under the cursor/finger.
	*rot = cross(exp, vec);
	rot->w = 1.0f + exp.x * vec.x + exp.y * vec.y + exp.z * vec.z;
	num_f len = sqrt(rot->x * rot->x + rot->y * rot->y + rot->z * rot->z
		+ rot->w * rot->w);
	if (len) {
		*rot = (vec_f) { rot->x / len, rot->y / len, rot->z / len, rot->w / len };
	} else {
		// Shortest arc is ambiguous for antipodal trackball points as any half
		// turn about an axis orthogonal to them carries one onto the other.
		// Choose the half turn on the great circle through the pole as if
		// displacing straight through the screen's center.
		*rot = (vec_f) { .x = pos.y, .y = -pos.x };
	}
}

// Associated implementation of generic selection macro `trackball_orbit`.
void
trackball_orbit_d(
	num_d xyzw[static restrict 4],
	num_d xyzm[static restrict 4],
	const num_d xy[static restrict 2],
	const num_d wh[static restrict 2]
) {
	// Identity quaternion with no previous position or zero displacement.
	vec_d* rot = (vec_d*)xyzw;
	*rot = (vec_d) { .w = 1.0 };
	// Previous trackball point with zero length for no position.
	vec_d* old = (vec_d*)xyzm;
	// Maximum centered position as half the screen's width and height.
	vec_d max = { .x = wh[0] * 0.5, .y = wh[1] * 0.5 };
	// Current centered position from left to right and bottom to top.
	vec_d pos = {
		.x = clamp(xy[0], 0.0, wh[0]) - max.x,
		.y = max.y - clamp(xy[1], 0.0, wh[1]),
	};
	// Trackball's radius mapping maximum of half the screen's width or height
	// onto quarter of its circumference.
	max.w = fmax(max.x, max.y) / PI_2_d;
	// Polar angle of current position, beyond equator for screen corners.
	pos.w = normalize(&pos) / max.w;
	num_d s = sin(pos.w), c = cos(pos.w);
	// Exponential map of current position onto trackball.
	vec_d exp = { .x = s * pos.x, .y = s * pos.y, .z = c, .w = 1.0 };
	// Get previous and replace with current trackball point.
	vec_d vec = *old;
	*old = exp;
	// No rotation with no previous position or zero displacement.
	if (!vec.w || (vec.x == exp.x && vec.y == exp.y && vec.z == exp.z))
		return;
	// Shortest arc from current to previous trackball point as rotation of
	// camera which in turn carries previous to current trackball point,
	// keeping it under the cursor/finger.
	*rot = cross(exp, vec);
	rot->w = 1.0 + exp.x * vec.x + exp.y * vec.y + exp.z * vec.z;
	num_d len = sqrt(rot->x * rot->x + rot->y * rot->y + rot->z * rot->z
		+ rot->w * rot->w);
	if (len) {
		*rot = (vec_d) { rot->x / len, rot->y / len, rot->z / len, rot->w / len };
	} else {
		// Shortest arc is ambiguous for antipodal trackball points as any half
		// turn about an axis orthogonal to them carries one onto the other.
		// Choose the half turn on the great circle through the pole as if
		// displacing straight through the screen's center.
		*rot = (vec_d) { .x = pos.y, .y = -pos.x };
	}
}

// Associated implementation of generic selection macro `trackball_orbit`.
void
trackball_orbit_l(
	num_l xyzw[static restrict 4],
	num_l xyzm[static restrict 4],
	const num_l xy[static restrict 2],
	const num_l wh[static restrict 2]
) {
	// Identity quaternion with no previous position or zero displacement.
	vec_l* rot = (vec_l*)xyzw;
	*rot = (vec_l) { .w = 1.0l };
	// Previous trackball point with zero length for no position.
	vec_l* old = (vec_l*)xyzm;
	// Maximum centered position as half the screen's width and height.
	vec_l max = { .x = wh[0] * 0.5l, .y = wh[1] * 0.5l };
	// Current centered position from left to right and bottom to top.
	vec_l pos = {
		.x = clamp(xy[0], 0.0l, wh[0]) - max.x,
		.y = max.y - clamp(xy[1], 0.0l, wh[1]),
	};
	// Trackball's radius mapping maximum of half the screen's width or height
	// onto quarter of its circumference.
	max.w = fmax(max.x, max.y) / PI_2_l;
	// Polar angle of current position, beyond equator for screen corners.
	pos.w = normalize(&pos) / max.w;
	num_l s = sin(pos.w), c = cos(pos.w);
	// Exponential map of current position onto trackball.
	vec_l exp = { .x = s * pos.x, .y = s * pos.y, .z = c, .w = 1.0l };
	// Get previous and replace with current trackball point.
	vec_l vec = *old;
	*old = exp;
	// No rotation with no previous position or zero displacement.
	if (!vec.w || (vec.x == exp.x && vec.y == exp.y && vec.z == exp.z))
		return;
	// Shortest arc from current to previous trackball point as rotation of
	// camera which in turn carries previous to current trackball point,
	// keeping it under the cursor/finger.
	*rot = cross(exp, vec);
	rot->w = 1.0l + exp.x * vec.x + exp.y * vec.y + exp.z * vec.z;
	num_l len = sqrt(rot->x * rot->x + rot->y * rot->y + rot->z * rot->z
		+ rot->w * rot->w);
	if (len) {
		*rot = (vec_l) { rot->x / len, rot->y / len, rot->z / len, rot->w / len };
	} else {
		// Shortest arc is ambiguous for antipodal trackball points as any half
		// turn about an axis orthogonal to them carries one onto the other.
		// Choose the half turn on the great circle through the pole as if
		// displacing straight through the screen's center.
		*rot = (vec_l) { .x = pos.y, .y = -pos.x };
	}
}
