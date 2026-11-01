//! Projections onto [`Plane`].

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
