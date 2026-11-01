//! Rotations of [`Orbit`].

use core::f32::consts::{FRAC_1_SQRT_2, FRAC_PI_2};
use trackball::{
	Orbit,
	glam::{Quat, Vec2, Vec3},
};

#[test]
fn starts_at_center() {
	let max = Vec2::new(800.0, 600.0);
	let mut orbit = Orbit::default();
	assert_eq!(orbit.compute(max * 0.5, max), None);
	// Displacement to the right rotates about negative y-axis by displacement over radius.
	let rot = orbit
		.compute(max * 0.5 + Vec2::new(40.0, 0.0), max)
		.unwrap();
	assert!(
		rot.abs_diff_eq(Quat::from_rotation_y(-0.1 * FRAC_PI_2), 1e-6),
		"{rot:?}"
	);
}

#[test]
fn transports_displacement() {
	let max = Vec2::new(800.0, 600.0);
	let mut orbit = Orbit::default();
	// Start at right edge, mapped onto trackball's equator, and displace upwards.
	assert_eq!(orbit.compute(Vec2::new(800.0, 300.0), max), None);
	let rot = orbit.compute(Vec2::new(800.0, 260.0), max).unwrap();
	// Rotation about x-axis as a pole start would induce becomes rotation about negative z-axis.
	assert!(
		rot.abs_diff_eq(Quat::from_axis_angle(Vec3::NEG_Z, 0.1 * FRAC_PI_2), 1e-6),
		"{rot:?}"
	);
}

#[test]
fn sticks_to_cursor_on_radial_displacement() {
	let max = Vec2::new(800.0, 600.0);
	let mut orbit = Orbit::default();
	// Start halfway left of center mapped onto trackball at 45° from its pole.
	assert_eq!(orbit.compute(Vec2::new(200.0, 300.0), max), None);
	// Displace through center to right edge mapped onto trackball's equator.
	let rot = orbit.compute(Vec2::new(800.0, 300.0), max).unwrap();
	// Scene rotates inversely to orbiting camera carrying trackball's point along with cursor.
	let point = rot.inverse() * Vec3::new(-FRAC_1_SQRT_2, 0.0, FRAC_1_SQRT_2);
	assert!(point.abs_diff_eq(Vec3::X, 1e-6), "{point:?}");
}
