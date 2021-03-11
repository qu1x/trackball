/// \file
/// \brief API source file.

#include "trackball.h"

#include <math.h>

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

/// Matrix of column vectors.
typedef struct {
	/// x-column vector.
	vec_f x;
	/// y-column vector.
	vec_f y;
	/// z-column vector.
	vec_f z;
} mat_f;

/// Matrix of column vectors.
typedef struct {
	/// x-column vector.
	vec_d x;
	/// y-column vector.
	vec_d y;
	/// z-column vector.
	vec_d z;
} mat_d;

/// Matrix of column vectors.
typedef struct {
	/// x-column vector.
	vec_l x;
	/// y-column vector.
	vec_l y;
	/// z-column vector.
	vec_l z;
} mat_l;

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
	return (v->w);
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
	return (v->w);
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
	return (v->w);
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

/// Applies matrix to vector.
#define mul(a, b) _Generic((b), \
	vec_f: mul_f, \
	vec_d: mul_d, \
	vec_l: mul_l \
)(a, b)

/// Associated implementation of generic selection macro \ref mul.
static vec_f
mul_f(mat_f m, vec_f v) {
	return (vec_f) {
		.x = m.x.x * v.x + m.y.x * v.y + m.z.x * v.z,
		.y = m.x.y * v.x + m.y.y * v.y + m.z.y * v.z,
		.z = m.x.z * v.x + m.y.z * v.y + m.z.z * v.z,
	};
}

/// Associated implementation of generic selection macro \ref mul.
static vec_d
mul_d(mat_d m, vec_d v) {
	return (vec_d) {
		.x = m.x.x * v.x + m.y.x * v.y + m.z.x * v.z,
		.y = m.x.y * v.x + m.y.y * v.y + m.z.y * v.z,
		.z = m.x.z * v.x + m.y.z * v.y + m.z.z * v.z,
	};
}

/// Associated implementation of generic selection macro \ref mul.
static vec_l
mul_l(mat_l m, vec_l v) {
	return (vec_l) {
		.x = m.x.x * v.x + m.y.x * v.y + m.z.x * v.z,
		.y = m.x.y * v.x + m.y.y * v.y + m.z.y * v.z,
		.z = m.x.z * v.x + m.y.z * v.y + m.z.z * v.z,
	};
}

/// Applies transpose of matrix to vector.
#define tr_mul(a, b) _Generic((b), \
	vec_f: tr_mul_f, \
	vec_d: tr_mul_d, \
	vec_l: tr_mul_l \
)(a, b)

/// Associated implementation of generic selection macro \ref tr_mul.
static vec_f
tr_mul_f(mat_f m, vec_f v) {
	return (vec_f) {
		.x = m.x.x * v.x + m.x.y * v.y + m.x.z * v.z,
		.y = m.y.x * v.x + m.y.y * v.y + m.y.z * v.z,
		.z = m.z.x * v.x + m.z.y * v.y + m.z.z * v.z,
	};
}

/// Associated implementation of generic selection macro \ref tr_mul.
static vec_d
tr_mul_d(mat_d m, vec_d v) {
	return (vec_d) {
		.x = m.x.x * v.x + m.x.y * v.y + m.x.z * v.z,
		.y = m.y.x * v.x + m.y.y * v.y + m.y.z * v.z,
		.z = m.z.x * v.x + m.z.y * v.y + m.z.z * v.z,
	};
}

/// Associated implementation of generic selection macro \ref tr_mul.
static vec_l
tr_mul_l(mat_l m, vec_l v) {
	return (vec_l) {
		.x = m.x.x * v.x + m.x.y * v.y + m.x.z * v.z,
		.y = m.y.x * v.x + m.y.y * v.y + m.y.z * v.z,
		.z = m.z.x * v.x + m.z.y * v.y + m.z.z * v.z,
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
	// Previous position with zero length for no position.
	vec_f* old = (vec_f*)xyzm;
	// Positive z-axis pointing from far to near.
	vec_f pza = { .z = 1.0f, .w = 1.0f };
	// Maximum centered position as half the screen's width and height.
	vec_f max = { .x = wh[0] * 0.5f, .y = wh[1] * 0.5f };
	// Current centered position from left to right and bottom to top.
	vec_f vec = (vec_f) {
		.x = clamp(xy[0], 0.0f, wh[0]) - max.x,
		.y = max.y - clamp(xy[1], 0.0f, wh[1]),
	};
	// Distinguish no position from origin position by assigning z-axis.
	if (!normalize(&vec))
		vec = pza;
	// Get previous and replace with current position.
	vec_f pos = *old;
	*old = vec;
	// No rotation with no previous position.
	if (!pos.w)
		return;
	// Displacement vector from previous to current position.
	vec.x = vec.x * vec.w - pos.x * pos.w;
	vec.y = vec.y * vec.w - pos.y * pos.w;
	// No rotation with zero displacement.
	if (!normalize(&vec))
		return;
	// Treat maximum of half the screen's width or height as trackball's radius.
	max.w = fmax(max.x, max.y);
	// Map trackball's diameter onto half its circumference for start positions
	// so that only screen corners are mapped to lower hemisphere which induces
	// less intuitive rotations.
	pos.w = pos.w / max.w * PI_2_f;
	num_f s = sin(pos.w), c = cos(pos.w);
	// Exponential map of start position.
	vec_f exp = { .x = s * pos.x, .y = s * pos.y, .z = c };
	// Tangent unit vector of geodesic at exponential map.
	vec_f tan = { .x = c * pos.x, .y = c * pos.y, .z = -s };
	// Cross product of z-axis and start position for orthonormal frames.
	vec_f zxp = { .x = -pos.y, .y = pos.x };
	// Orthonormal frame as argument of differential of exponential map.
	mat_f arg = { pza, pos, zxp };
	// Orthonormal frame as image of differential of exponential map.
	mat_f img = { exp, tan, zxp };
	// Compute differential of exponential map by its argument and image and
	// apply it to displacement vector which in turn spans rotation plane
	// together with exponential map.
	*rot = cross(mul(img, tr_mul(arg, vec)), exp);
	normalize(rot);
	// Angle of rotation is displacement length divided by radius.
	rot->w = vec.w / max.w;
	// Convert axis-angle representation into unit quaternion.
	rot->w *= 0.5f;
	num_f im = sin(rot->w), re = cos(rot->w);
	*rot = (vec_f) { rot->x * im, rot->y * im, rot->z * im, re };
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
	// Previous position with zero length for no position.
	vec_d* old = (vec_d*)xyzm;
	// Positive z-axis pointing from far to near.
	vec_d pza = { .z = 1.0, .w = 1.0 };
	// Maximum centered position as half the screen's width and height.
	vec_d max = { .x = wh[0] * 0.5, .y = wh[1] * 0.5 };
	// Current centered position from left to right and bottom to top.
	vec_d vec = (vec_d) {
		.x = clamp(xy[0], 0.0, wh[0]) - max.x,
		.y = max.y - clamp(xy[1], 0.0, wh[1]),
	};
	// Distinguish no position from origin position by assigning z-axis.
	if (!normalize(&vec))
		vec = pza;
	// Get previous and replace with current position.
	vec_d pos = *old;
	*old = vec;
	// No rotation with no previous position.
	if (!pos.w)
		return;
	// Displacement vector from previous to current position.
	vec.x = vec.x * vec.w - pos.x * pos.w;
	vec.y = vec.y * vec.w - pos.y * pos.w;
	// No rotation with zero displacement.
	if (!normalize(&vec))
		return;
	// Treat maximum of half the screen's width or height as trackball's radius.
	max.w = fmax(max.x, max.y);
	// Map trackball's diameter onto half its circumference for start positions
	// so that only screen corners are mapped to lower hemisphere which induces
	// less intuitive rotations.
	pos.w = pos.w / max.w * PI_2_d;
	num_d s = sin(pos.w), c = cos(pos.w);
	// Exponential map of start position.
	vec_d exp = { .x = s * pos.x, .y = s * pos.y, .z = c };
	// Tangent unit vector of geodesic at exponential map.
	vec_d tan = { .x = c * pos.x, .y = c * pos.y, .z = -s };
	// Cross product of z-axis and start position for orthonormal frames.
	vec_d zxp = { .x = -pos.y, .y = pos.x };
	// Orthonormal frame as argument of differential of exponential map.
	mat_d arg = { pza, pos, zxp };
	// Orthonormal frame as image of differential of exponential map.
	mat_d img = { exp, tan, zxp };
	// Compute differential of exponential map by its argument and image and
	// apply it to displacement vector which in turn spans rotation plane
	// together with exponential map.
	*rot = cross(mul(img, tr_mul(arg, vec)), exp);
	normalize(rot);
	// Angle of rotation is displacement length divided by radius.
	rot->w = vec.w / max.w;
	// Convert axis-angle representation into unit quaternion.
	rot->w *= 0.5;
	num_d im = sin(rot->w), re = cos(rot->w);
	*rot = (vec_d) { rot->x * im, rot->y * im, rot->z * im, re };
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
	// Previous position with zero length for no position.
	vec_l* old = (vec_l*)xyzm;
	// Positive z-axis pointing from far to near.
	vec_l pza = { .z = 1.0l, .w = 1.0l };
	// Maximum centered position as half the screen's width and height.
	vec_l max = { .x = wh[0] * 0.5l, .y = wh[1] * 0.5l };
	// Current centered position from left to right and bottom to top.
	vec_l vec = (vec_l) {
		.x = clamp(xy[0], 0.0l, wh[0]) - max.x,
		.y = max.y - clamp(xy[1], 0.0l, wh[1]),
	};
	// Distinguish no position from origin position by assigning z-axis.
	if (!normalize(&vec))
		vec = pza;
	// Get previous and replace with current position.
	vec_l pos = *old;
	*old = vec;
	// No rotation with no previous position.
	if (!pos.w)
		return;
	// Displacement vector from previous to current position.
	vec.x = vec.x * vec.w - pos.x * pos.w;
	vec.y = vec.y * vec.w - pos.y * pos.w;
	// No rotation with zero displacement.
	if (!normalize(&vec))
		return;
	// Treat maximum of half the screen's width or height as trackball's radius.
	max.w = fmax(max.x, max.y);
	// Map trackball's diameter onto half its circumference for start positions
	// so that only screen corners are mapped to lower hemisphere which induces
	// less intuitive rotations.
	pos.w = pos.w / max.w * PI_2_l;
	num_l s = sin(pos.w), c = cos(pos.w);
	// Exponential map of start position.
	vec_l exp = { .x = s * pos.x, .y = s * pos.y, .z = c };
	// Tangent unit vector of geodesic at exponential map.
	vec_l tan = { .x = c * pos.x, .y = c * pos.y, .z = -s };
	// Cross product of z-axis and start position for orthonormal frames.
	vec_l zxp = { .x = -pos.y, .y = pos.x };
	// Orthonormal frame as argument of differential of exponential map.
	mat_l arg = { pza, pos, zxp };
	// Orthonormal frame as image of differential of exponential map.
	mat_l img = { exp, tan, zxp };
	// Compute differential of exponential map by its argument and image and
	// apply it to displacement vector which in turn spans rotation plane
	// together with exponential map.
	*rot = cross(mul(img, tr_mul(arg, vec)), exp);
	normalize(rot);
	// Angle of rotation is displacement length divided by radius.
	rot->w = vec.w / max.w;
	// Convert axis-angle representation into unit quaternion.
	rot->w *= 0.5l;
	num_l im = sin(rot->w), re = cos(rot->w);
	*rot = (vec_l) { rot->x * im, rot->y * im, rot->z * im, re };
}
