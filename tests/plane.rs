//! Projections onto and angles within [`Plane`].

use core::f32::consts::PI;
use trackball::{Plane, glam::Vec3};

#[test]
fn project_point_and_vector_onto_offset_plane() {
	let plane = Plane::new(Vec3::Y, 5.0);
	let point = Vec3::new(1.0, 2.0, 3.0);
	// Points are moved onto the plane.
	assert_eq!(plane.project_point(point), Vec3::new(1.0, 5.0, 3.0));
	// Vectors and axes are made parallel to the plane regardless of its bias.
	assert_eq!(plane.project_vector(point), Vec3::new(1.0, 0.0, 3.0));
	assert_eq!(plane.project_axis(Vec3::new(0.0, 2.0, 3.0)), Vec3::Z);
}

#[test]
fn measures_small_and_nearly_opposite_signed_angles() {
	let plane = Plane::new(Vec3::Z, 5.0);
	for angle in [1e-6, 1e-4, 1e-2, 1.0, PI - 1e-2, PI - 1e-4] {
		for angle in [angle, -angle] {
			// Vectors in the plane need not be unit vectors.
			let a = Vec3::X * 2.0;
			let b = Vec3::new(angle.cos(), angle.sin(), 0.0) * 3.0;
			let error = (plane.angle_between(a, b) - angle).abs();
			let tolerance = angle.abs() * 2.0 * f32::EPSILON;
			assert!(error <= tolerance, "{angle}: {error}");
		}
	}
}
