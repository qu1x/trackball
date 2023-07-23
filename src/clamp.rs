use crate::{Delta, Frame, Scene};
use core::mem::replace;
use nalgebra::RealField;

/// Clamp as user boundary conditions of [`Frame`].
///
/// Implements [`Default`] and can be created with `Clamp::default()`.
#[derive(Debug, Clone)]
pub struct Clamp<N: Copy + RealField> {
	/// Ensures user boundary conditions. Default is [`Self::zcp_collision()`].
	#[allow(clippy::type_complexity)]
	ubc: fn(delta: Delta<N>, frame: &Frame<N>, scene: &Scene<N>) -> Delta<N>,
}

impl<N: Copy + RealField> Default for Clamp<N> {
	fn default() -> Self {
		Self {
			ubc: Self::zcp_collision,
		}
	}
}

impl<N: Copy + RealField> Clamp<N> {
	/// Computes clamped [`Frame`] wrt to user boundary conditions.
	pub fn compute(&self, delta: Delta<N>, frame: &Frame<N>, scene: &Scene<N>) -> Delta<N> {
		(self.ubc)(delta, frame, scene)
	}
	/// Replace with new and return old user boundary conditions.
	#[allow(clippy::type_complexity)]
	pub fn replace(
		&mut self,
		ubc: fn(delta: Delta<N>, frame: &Frame<N>, scene: &Scene<N>) -> Delta<N>,
	) -> fn(delta: Delta<N>, frame: &Frame<N>, scene: &Scene<N>) -> Delta<N> {
		replace(&mut self.ubc, ubc)
	}
	/// Default boundary conditions preventing clip plane collisions.
	pub fn zcp_collision(mut delta: Delta<N>, frame: &Frame<N>, scene: &Scene<N>) -> Delta<N> {
		if scene.scale() {
			if let Delta::Scale { rat, pos } = delta {
				let (znear, _zfar) = scene.clip_planes(N::zero());
				let old_zat = frame.distance();
				let zat = (old_zat * rat).max(-znear * (N::one() + N::default_epsilon().sqrt()));
				let rat = zat / old_zat;
				delta = Delta::Scale { rat, pos };
			}
		}
		delta
	}
}
