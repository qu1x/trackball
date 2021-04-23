use nalgebra::{convert, Point2, RealField, Unit, Vector2, Vector3};

/// First person view induced by displacement on screen.
///
/// Implements [`Default`] and can be created with `First::default()`.
///
/// All methods except [`Self::enabled()`] must be invoked on matching events fired by your 3D
/// graphics library of choice.
#[derive(Debug, Clone, Default)]
pub struct First<N: RealField> {
	/// Caches captured yaw axis.
	ray: Option<Unit<Vector3<N>>>,
}

impl<N: RealField> First<N> {
	/// Captures current yaw axis when entering first person view.
	pub fn capture(&mut self, yaw_axis: Unit<Vector3<N>>) {
		self.ray = Some(yaw_axis);
	}
	/// Computes pitch and yaw from cursor/finger displacement vector in screen space.
	///
	/// Carries cursor/finger displacements to arcs of the same length on great circles of an
	/// eye-centered trackball with radius of maximum of half the screen's width and height in
	/// compliance with [`crate::Orbit`] except that its trackball is target-centered.
	pub fn compute(
		&mut self,
		vec: &Vector2<N>,
		max: &Point2<N>,
	) -> Option<(N, N, &Unit<Vector3<N>>)> {
		self.ray.as_ref().map(|ray| {
			let max = max.x.max(max.y) * convert(0.5);
			(vec.y / max, vec.x / max, ray)
		})
	}
	/// Discards captured yaw axis when leaving first person view.
	pub fn discard(&mut self) {
		self.ray = None;
	}
	/// Whether a yaw axis has been captured.
	pub fn enabled(&self) -> bool {
		self.ray.is_some()
	}
}
