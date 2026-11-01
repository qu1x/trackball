// Copyright © 2021-2026 Rouven Spreckels <rs@qu1x.dev>
// SPDX-License-Identifier: MIT OR Apache-2.0
/// \file
/// \brief API header file.
///
/// \mainpage Virtual Trackball Orbiting via the Exponential Map
///
/// This is an alternative trackball technique using the exponential map for
/// inducing coherent and intuitive trackball rotations based on the paper of
/// Stantchev, G.. “Virtual Trackball Modeling and the Exponential Map.” .
/// [S2CID][] [44199608][]. Displacements on straight radial lines through the
/// screen's center are carried to arcs of the same length on great circles of
/// the trackball, whose half circumference is the maximum of the screen's width
/// and height. This is in contrast to state-of-the-art techniques using
/// orthogonal projection which distorts radial distances further away from the
/// screen's center. Each rotation carries the trackball's point under the
/// previous cursor/finger position along the shortest arc to the one under the
/// current position, keeping the grabbed point under the cursor/finger. This
/// follows the paper's differential of the exponential map rather than its
/// recipe, which takes the differential to be parallel transport, preserving
/// the lengths of displacements but letting the grabbed point drift. Screen
/// corners beyond the trackball's border are mapped beyond its equator, keeping
/// radial displacements linear.
///
/// [S2CID]: https://en.wikipedia.org/wiki/S2CID_(identifier)
/// [44199608]: https://api.semanticscholar.org/CorpusID:44199608
///
/// \version v0.19.1
/// \author Rouven Spreckels <rs@qu1x.dev>
/// \copyright MIT OR Apache-2.0
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
/// Trackball point of previous position is cached and has to be discarded on
/// button/finger release by resetting its length to zero. Current position is
/// clamped between origin and maximum position as screen's width and height.
///
/// Maps positions via the exponential map onto a target-centered trackball with
/// radius of maximum of the screen's width and height divided by π and rotates
/// along the shortest arc between their trackball points so that the
/// trackball's point under the cursor/finger stays under it. Positions in
/// screen corners beyond the trackball's border are mapped beyond its equator.
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
/// \param[in,out] xyzm Cached trackball point of previous position with its
///                     length or zero for no position.
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
