use crate::{Clamp, Frame, Plane, Rigid};
use approx::AbsDiffEq;
use glam::{Quat, Vec3};
#[cfg_attr(test, allow(unused_imports))]
use num_traits::Float;

/// Orthogonal boundary conditions implementing [`Clamp`].
///
/// Implements [`Default`] and can be created with `Bound::default()`.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
	feature = "rkyv",
	derive(rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)
)]
pub struct Bound {
	/// Isometry in world space of bound inversely transforming target and eye positions.
	pub transform: Rigid,
	/// Minimum components of target position in world space. Default splats `f32::MIN`.
	pub min_target: Vec3,
	/// Maximum components of target position in world space. Default splats `f32::MAX`.
	pub max_target: Vec3,
	/// Minimum components of eye position in world space. Default splats `f32::MIN`.
	pub min_eye: Vec3,
	/// Maximum components of eye position in world space. Default splats `f32::MAX`.
	pub max_eye: Vec3,
	/// Minimum components of up axis in world space following yaw. Default splats `f32::MIN`.
	pub min_up: Vec3,
	/// Maximum components of up axis in world space following yaw. Default splats `f32::MAX`.
	pub max_up: Vec3,
	/// Minimum distance of eye from target. Default is `0.0`.
	pub min_distance: f32,
	/// Maximum distance of eye from target. Default is `f32::MAX`.
	pub max_distance: f32,
	/// Epsilon allowing clamped [`Delta`] to more likely pass revalidation.
	///
	/// Default is <code>[AbsDiffEq::default_epsilon()].sqrt()</code>.
	///
	/// [`Delta`]: crate::Delta
	/// [AbsDiffEq::default_epsilon()]: approx::AbsDiffEq::default_epsilon()
	pub hysteresis: f32,
}

impl Default for Bound {
	fn default() -> Self {
		Self {
			transform: Rigid::IDENTITY,
			hysteresis: f32::default_epsilon().sqrt(),
			min_target: Vec3::MIN,
			max_target: Vec3::MAX,
			min_eye: Vec3::MIN,
			max_eye: Vec3::MAX,
			min_up: Vec3::MIN,
			max_up: Vec3::MAX,
			min_distance: 0.0,
			max_distance: f32::MAX,
		}
	}
}

impl Clamp for Bound {
	/// Using lower loop limit for flat boundary conditions.
	fn loops(&self) -> usize {
		10
	}
	/// Find any boundary plane exceeded by target position.
	fn target(&self, frame: &Frame) -> Option<Plane> {
		let target = self.transform.inverse() * frame.target();
		let axes = [Vec3::X, Vec3::Y, Vec3::Z];
		for (distance, axis) in axes.into_iter().enumerate() {
			let min_plane = Plane::new(axis, self.min_target[distance]);
			if min_plane.distance_from(target) > self.hysteresis {
				return Some(min_plane);
			}
			let max_plane = Plane::new(axis, self.max_target[distance]);
			if max_plane.distance_from(target) < -self.hysteresis {
				return Some(max_plane);
			}
		}
		None
	}
	/// Find any boundary plane exceeded by eye position.
	fn eye(&self, frame: &Frame) -> Option<Plane> {
		let distance = frame.distance();
		if (self.min_distance - distance) > self.hysteresis {
			return Some(Plane::new(frame.roll_axis(), self.min_distance));
		}
		if (self.max_distance - distance) < -self.hysteresis {
			return Some(Plane::new(frame.roll_axis(), self.max_distance));
		}
		let eye = self.transform.inverse() * frame.eye();
		let axes = [Vec3::X, Vec3::Y, Vec3::Z];
		for (distance, axis) in axes.into_iter().enumerate() {
			let min_plane = Plane::new(axis, self.min_eye[distance]);
			if min_plane.distance_from(eye) > self.hysteresis {
				return Some(min_plane);
			}
			let max_plane = Plane::new(axis, self.max_eye[distance]);
			if max_plane.distance_from(eye) < -self.hysteresis {
				return Some(max_plane);
			}
		}
		None
	}
	/// Find any boundary plane exceeded by up position.
	fn up(&self, frame: &Frame) -> Option<Plane> {
		let roll_axis = frame.roll_axis();
		let yaw = Quat::from_axis_angle(frame.local_yaw_axis(), roll_axis.x.atan2(roll_axis.z));
		let up = yaw * frame.yaw_axis();
		let axes = [Vec3::X, Vec3::Y, Vec3::Z];
		for (distance, axis) in axes.into_iter().enumerate() {
			let min_plane = Plane::new(yaw.inverse() * axis, self.min_up[distance]);
			if min_plane.distance_from(up) > self.hysteresis {
				return Some(min_plane);
			}
			let max_plane = Plane::new(yaw.inverse() * axis, self.max_up[distance]);
			if max_plane.distance_from(up) < -self.hysteresis {
				return Some(max_plane);
			}
		}
		None
	}
}
