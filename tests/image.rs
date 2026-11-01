//! Cache invalidation of [`Image`].

use trackball::{
	Frame, Image, Scope,
	glam::{Vec2, Vec3},
};

#[test]
fn recomputes_only_on_changes() {
	let frame = Frame::look_at(Vec3::new(1.0, 2.0, 5.0), Vec3::ZERO, Vec3::Y);
	let scope = Scope::default();
	let mut image = Image::new(frame, scope, Vec2::new(800.0, 600.0));
	assert_eq!(image.view_rigid(), frame.view());
	assert_eq!(image.compute(frame, scope), None);
	// Passive view.
	image.set_passive(true);
	assert_eq!(image.compute(frame, scope), Some(true));
	assert_eq!(image.view_rigid(), frame.inverse_view());
	assert_eq!(image.compute(frame, scope), None);
	// Resized screen.
	let projection = image.projection();
	image.set_max(Vec2::new(600.0, 800.0));
	assert_eq!(image.compute(frame, scope), Some(true));
	assert_ne!(image.projection(), projection);
	assert_eq!(image.compute(frame, scope), None);
	// Re-enabled transformations.
	image.set_compute(false, false);
	assert_eq!(image.compute(frame, scope), Some(true));
	image.set_compute(true, true);
	assert_eq!(image.compute(frame, scope), Some(true));
	assert_eq!(image.transformation(), image.projection() * image.view());
}
