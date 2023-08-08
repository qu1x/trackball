use crate::{Clamp, Frame, Plane};
use core::fmt::Debug;
use nalgebra::{Isometry3, Point3, RealField, UnitQuaternion, Vector3};

/// Orthogonal boundary conditions implementing [`Clamp`].
///
/// Implements [`Default`] and can be created with `Bound::default()`.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
	feature = "rkyv",
	derive(rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)
)]
pub struct Bound<N: Copy + RealField> {
	/// Isometry in world space of bound inversely transforming target and eye positions.
	pub transform: Isometry3<N>,
	/// Minimum components of target position in world space. Default splats `N::MIN`.
	pub min_target: Point3<N>,
	/// Maximum components of target position in world space. Default splats `N::MAX`.
	pub max_target: Point3<N>,
	/// Minimum components of eye position in world space. Default splats `N::MIN`.
	pub min_eye: Point3<N>,
	/// Maximum components of eye position in world space. Default splats `N::MAX`.
	pub max_eye: Point3<N>,
	/// Minimum components of up axis in world space following yaw. Default splats `N::MIN`.
	pub min_up: Point3<N>,
	/// Maximum components of up axis in world space following yaw. Default splats `N::MAX`.
	pub max_up: Point3<N>,
	/// Minimum distance of eye from target. Default is `N::zero()`.
	pub min_distance: N,
	/// Maximum distance of eye from target. Default is `N::MAX`.
	pub max_distance: N,
	/// Epsilon allowing clamped [`Delta`] to more likely pass revalidation.
	///
	/// Default is [`AbsDiffEq::default_epsilon()`]`.sqrt()`.
	///
	/// [`Delta`]: crate::Delta
	/// [`AbsDiffEq::default_epsilon()`]: approx::AbsDiffEq::default_epsilon()
	pub hysteresis: N,
}

impl<N: Copy + RealField> Default for Bound<N> {
	fn default() -> Self {
		let min = N::min_value().unwrap();
		let max = N::max_value().unwrap();
		Self {
			transform: Isometry3::default(),
			hysteresis: N::default_epsilon().sqrt(),
			min_target: Point3::new(min, min, min),
			max_target: Point3::new(max, max, max),
			min_eye: Point3::new(min, min, min),
			max_eye: Point3::new(max, max, max),
			min_up: Point3::new(min, min, min),
			max_up: Point3::new(max, max, max),
			min_distance: N::zero(),
			max_distance: max,
		}
	}
}

impl<N: Copy + RealField> Clamp<N> for Bound<N> {
	/// Using lower loop limit for flat boundary conditions.
	fn loops(&self) -> usize {
		10
	}
	/// Find any boundary plane exceeded by target position.
	fn target(&self, frame: &Frame<N>) -> Option<Plane<N>> {
		let target = self.transform.inverse() * frame.target();
		let axes = [Vector3::x_axis(), Vector3::y_axis(), Vector3::z_axis()];
		for (distance, axis) in axes.into_iter().enumerate() {
			let min_plane = Plane::new(axis, self.min_target[distance]);
			if min_plane.distance_from(&target) > self.hysteresis {
				return Some(min_plane);
			}
			let max_plane = Plane::new(axis, self.max_target[distance]);
			if max_plane.distance_from(&target) < -self.hysteresis {
				return Some(max_plane);
			}
		}
		None
	}
	/// Find any boundary plane exceeded by eye position.
	fn eye(&self, frame: &Frame<N>) -> Option<Plane<N>> {
		let distance = frame.distance();
		if (self.min_distance - distance) > self.hysteresis {
			return Some(Plane::new(frame.roll_axis(), self.min_distance));
		}
		if (self.max_distance - distance) < -self.hysteresis {
			return Some(Plane::new(frame.roll_axis(), self.max_distance));
		}
		let eye = self.transform.inverse() * frame.eye();
		let axes = [Vector3::x_axis(), Vector3::y_axis(), Vector3::z_axis()];
		for (distance, axis) in axes.into_iter().enumerate() {
			let min_plane = Plane::new(axis, self.min_eye[distance]);
			if min_plane.distance_from(&eye) > self.hysteresis {
				return Some(min_plane);
			}
			let max_plane = Plane::new(axis, self.max_eye[distance]);
			if max_plane.distance_from(&eye) < -self.hysteresis {
				return Some(max_plane);
			}
		}
		None
	}
	/// Find any boundary plane exceeded by up position.
	fn up(&self, frame: &Frame<N>) -> Option<Plane<N>> {
		let roll_axis = frame.roll_axis();
		let yaw = UnitQuaternion::from_axis_angle(
			&frame.local_yaw_axis(),
			roll_axis.x.atan2(roll_axis.z),
		);
		let up = yaw * frame.yaw_axis();
		let axes = [Vector3::x_axis(), Vector3::y_axis(), Vector3::z_axis()];
		for (distance, axis) in axes.into_iter().enumerate() {
			let min_plane = Plane::new(yaw.inverse() * axis, self.min_up[distance]);
			if min_plane.distance_from(&Point3::from(up.into_inner())) > self.hysteresis {
				return Some(min_plane);
			}
			let max_plane = Plane::new(yaw.inverse() * axis, self.max_up[distance]);
			if max_plane.distance_from(&Point3::from(up.into_inner())) < -self.hysteresis {
				return Some(max_plane);
			}
		}
		None
	}
}
