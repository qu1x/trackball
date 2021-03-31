use crate::{Frame, Scene};
use nalgebra::RealField;
use std::mem::replace;

/// Clamp as user boundary conditions of [`Frame`].
///
/// Implements [`Default`] and can be created with `Clamp::default()`.
#[derive(Clone)]
pub struct Clamp<N: RealField> {
	/// Ensures user boundary conditions.
	ubc: fn(frame: &Frame<N>, scene: &Scene<N>) -> Frame<N>,
}

impl<N: RealField> Default for Clamp<N> {
	fn default() -> Self {
		Self {
			ubc: Self::zcp_collision,
		}
	}
}

impl<N: RealField> Clamp<N> {
	/// Computes clamped [`Frame`] wrt to user boundary conditions.
	pub fn compute(&mut self, frame: &Frame<N>, scene: &Scene<N>) -> Frame<N> {
		(self.ubc)(frame, scene)
	}
	/// Replace with new and return old user boundary conditions.
	pub fn replace(
		&mut self,
		ubc: fn(frame: &Frame<N>, scene: &Scene<N>) -> Frame<N>,
	) -> fn(frame: &Frame<N>, scene: &Scene<N>) -> Frame<N> {
		replace(&mut self.ubc, ubc)
	}
	/// Boundary conditions preventing clip plane collisions.
	pub fn zcp_collision(frame: &Frame<N>, scene: &Scene<N>) -> Frame<N> {
		let mut frame = frame.clone();
		if scene.scale() {
			let zat = frame.distance();
			let (znear, _zfar) = scene.clip_planes(N::zero());
			frame.set_distance(zat.max(-znear * (N::one() + N::default_epsilon().sqrt())));
		}
		frame
	}
}
