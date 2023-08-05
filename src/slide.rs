use nalgebra::{Point2, RealField, Vector2};
use simba::scalar::SubsetOf;

/// Slide induced by displacement on screen.
///
/// Implements [`Default`] and can be created with `Slide::default()`.
///
/// Both its methods must be invoked on matching events fired by your 3D graphics library of choice.
#[derive(Debug, Clone, Default)]
pub struct Slide<N: Copy + RealField> {
	/// Caches previous cursor/finger position.
	pos: Option<Point2<N>>,
}

impl<N: Copy + RealField> Slide<N> {
	/// Computes slide between previous and current cursor/finger position in screen space.
	pub fn compute(&mut self, pos: Point2<N>) -> Option<Vector2<N>> {
		self.pos.replace(pos).map(|old| old - pos)
	}
	/// Discards cached previous cursor/finger position on button/finger release.
	pub fn discard(&mut self) {
		self.pos = None;
	}
	/// Casts components to another type, e.g., between [`f32`] and [`f64`].
	pub fn cast<M: Copy + RealField>(self) -> Slide<M>
	where
		N: SubsetOf<M>,
	{
		Slide {
			pos: self.pos.map(Point2::cast),
		}
	}
}
