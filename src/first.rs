use glam::{Vec2, Vec3};

/// First person view induced by displacement on screen.
///
/// Implements [`Default`] and can be created with `First::default()`.
///
/// All methods except [`Self::enabled()`] must be invoked on matching events fired by your 3D
/// graphics library of choice.
#[derive(Debug, Clone, Default)]
pub struct First {
	/// Caches captured yaw axis.
	ray: Option<Vec3>,
}

impl First {
	/// Captures current yaw axis when entering first person view.
	pub const fn capture(&mut self, yaw_axis: Vec3) {
		self.ray = Some(yaw_axis);
	}
	/// Computes pitch and yaw from cursor/finger displacement vector in screen space.
	///
	/// Carries cursor/finger displacements to arcs of the same length on great circles of an
	/// eye-centered trackball with radius of maximum of half the screen's width and height in
	/// compliance with [`crate::Orbit`] except that its trackball is target-centered.
	#[must_use]
	pub fn compute(&self, vec: Vec2, max: Vec2) -> Option<(f32, f32, Vec3)> {
		self.ray.map(|ray| {
			let max = max.x.max(max.y) * 0.5;
			(vec.y / max, vec.x / max, ray)
		})
	}
	/// Discards captured yaw axis when leaving first person view.
	pub const fn discard(&mut self) {
		self.ray = None;
	}
	/// Whether a yaw axis has been captured.
	#[must_use]
	pub const fn enabled(&self) -> bool {
		self.ray.is_some()
	}
	/// Captured yaw axis.
	#[must_use]
	pub const fn yaw_axis(&self) -> Option<Vec3> {
		self.ray
	}
}
