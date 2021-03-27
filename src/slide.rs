use nalgebra::{Point2, RealField, Vector2};

/// Slide induced by displacement on screen.
///
/// Implements [`Default`] and can be created with `Slide::default()`.
///
/// Both its methods must be invoked on matching events fired by your 3D graphics library of choice.
#[derive(Debug, Clone, Default)]
pub struct Slide<N: RealField> {
	/// Caches previous cursor/finger position.
	vec: Option<Point2<N>>,
}

impl<N: RealField> Slide<N> {
	/// Computes slide between previous and current cursor/finger position in screen space.
	pub fn compute(&mut self, pos: &Point2<N>) -> Option<Vector2<N>> {
		self.vec.replace(pos.clone()).map(|old| old - pos)
	}
	/// Discards cached previous cursor/finger position on button/finger release.
	pub fn discard(&mut self) {
		self.vec = None;
	}
}
