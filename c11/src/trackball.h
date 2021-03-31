/// \file
/// \brief API header file.

/// \mainpage Virtual Trackball Orbiting via the Exponential Map
///
/// This is an alternative trackball technique using exponential map and
/// parallel transport to preserve distances and angles for inducing coherent
/// and intuitive trackball rotations. For instance, displacements on straight
/// radial lines through the screen's center are carried to arcs of the same
/// length on great circles of the trackball. This is in contrast to
/// state-of-the-art techniques using orthogonal projection which distorts
/// radial distances further away from the screen's center. This implementation
/// strictly follows the recipe given in the paper of Stantchev, G.. “Virtual
/// Trackball Modeling and the Exponential Map.” . [S2CID][] [44199608][].
///
/// [S2CID]: https://en.wikipedia.org/wiki/S2CID_(identifier)
/// [44199608]: https://api.semanticscholar.org/CorpusID:44199608
///
/// \version v0.2.2
/// \author Rouven Spreckels <rs@qu1x.dev>
/// \copyright BSD-3-Clause
///
/// # Example
///
/// A trackball camera mode implementation can be as easy as this by delegating
/// events of your 3D graphics library of choice to the \ref trackball_orbit
/// operation implementation along with other implementations for common
/// trackball camera mode operations like slide, scale, and focus.
///
/// \include example.c

#ifndef TRACKBALL_H
#define TRACKBALL_H

/// Computes rotation between previous and current cursor/finger position.
///
/// Normalization of previous position is cached and has to be discarded on
/// button/finger release by resetting it to zero. Current position is clamped
/// between origin and maximum position as screen's width and height.
///
/// Screen space with origin in top left corner:
///
///   * x-axis from left to right,
///   * y-axis from top to bottom.
///
/// Camera space with origin at its target, the trackball's center:
///
///   * x-axis from left to right,
///   * y-axis from bottom to top,
///   * z-axis from far to near.
///
/// Parameter types are either `float*`, `double*` or `long double*` and point
/// to arrays or structures comprising vector components in the order they are
/// mentioned in their parameter name with the type of the output parameter \p
/// xyzw controlling the generic selection of the implementation's scalar type.
///
/// \param[out]    xyzw Induced rotation as unit quaternion.
/// \param[in,out] xyzm Cached normalization of previous position.
/// \param[in]     xy   Current position.
/// \param[in]     wh   Maximum position as screen's width and height.
#define trackball_orbit(xyzw, xyzm, xy, wh) _Generic((xyzw), \
	float*: trackball_orbit_f, \
	double*: trackball_orbit_d, \
	long double*: trackball_orbit_l \
)(xyzw, xyzm, xy, wh)

/// Associated implementation of generic selection macro \ref trackball_orbit.
void
trackball_orbit_f(
	float xyzw[static restrict 4],
	float xyzm[static restrict 4],
	const float xy[static restrict 2],
	const float wh[static restrict 2]
);

/// Associated implementation of generic selection macro \ref trackball_orbit.
void
trackball_orbit_d(
	double xyzw[static restrict 4],
	double xyzm[static restrict 4],
	const double xy[static restrict 2],
	const double wh[static restrict 2]
);

/// Associated implementation of generic selection macro \ref trackball_orbit.
void
trackball_orbit_l(
	long double xyzw[static restrict 4],
	long double xyzm[static restrict 4],
	const long double xy[static restrict 2],
	const long double wh[static restrict 2]
);

#endif // TRACKBALL_H
