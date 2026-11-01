use crate::{Rigid, float};
use core::f32::consts::PI;
use glam::{Quat, Vec3, camera::rh::view::look_to_quat};

/// Frame wrt camera eye and target.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
	feature = "rkyv",
	derive(rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)
)]
pub struct Frame {
	/// Target position in world space.
	pos: Vec3,
	/// Eye rotation from camera to world space around target.
	rot: Quat,
	/// Target distance from eye.
	zat: f32,
}

impl Frame {
	/// Sets eye position inclusive its roll attitude and target position in world space.
	#[must_use]
	pub fn look_at(target: Vec3, eye: Vec3, up: Vec3) -> Self {
		let (dir, zat) = (target - eye).normalize_and_length();
		Self {
			pos: target,
			rot: look_to_quat(dir, up).inverse(),
			zat,
		}
	}
	/// Eye position in world space.
	#[must_use]
	pub fn eye(&self) -> Vec3 {
		self.pos + self.rot * Vec3::Z * self.zat
	}
	/// Sets eye position inclusive its roll attitude in world space preserving target position.
	pub fn set_eye(&mut self, eye: Vec3, up: Vec3) {
		*self = Self::look_at(self.pos, eye, up);
	}
	/// Target position in world space.
	#[must_use]
	pub const fn target(&self) -> Vec3 {
		self.pos
	}
	/// Sets target position in world space preserving eye position inclusive its roll attitude.
	///
	/// Allows to track a moving object.
	pub fn set_target(&mut self, pos: Vec3) {
		let eye = self.eye();
		let (new_dir, zat) = (pos - eye).normalize_and_length();
		let old_dir = (self.pos - eye).normalize();
		// Rotate about yaw axis first if nearly opposite to preserve roll attitude and precision.
		let yaw = if old_dir.dot(new_dir) < float::sqrt(f32::EPSILON) - 1.0 {
			Quat::from_axis_angle(self.yaw_axis(), PI)
		} else {
			Quat::IDENTITY
		};
		// Shortest arc without snapping small angles to identity unlike
		// `Quat::from_rotation_arc()`.
		let old_dir = yaw * old_dir;
		let axis = old_dir.cross(new_dir);
		let arc = Quat::from_xyzw(axis.x, axis.y, axis.z, 1.0 + old_dir.dot(new_dir)).normalize();
		let rot = (arc * yaw * self.rot).normalize();
		*self = Self { pos, rot, zat }
	}
	/// Distance between eye and target.
	#[must_use]
	pub const fn distance(&self) -> f32 {
		self.zat
	}
	/// Sets distance between eye and target preserving target position.
	pub const fn set_distance(&mut self, zat: f32) {
		self.zat = zat;
	}
	/// Scales distance between eye and target by ratio preserving target position.
	pub fn scale(&mut self, rat: f32) {
		self.zat *= rat;
	}
	/// Scales distance between eye and point in camera space by ratio preserving target position.
	pub fn local_scale_around(&mut self, rat: f32, pos: Vec3) {
		self.local_slide(pos - pos * rat);
		self.scale(rat);
	}
	/// Scales distance between eye and point in world space by ratio preserving target position.
	pub fn scale_around(&mut self, rat: f32, pos: Vec3) {
		let pos = pos - self.pos;
		self.slide(pos - pos * rat);
		self.scale(rat);
	}
	/// Slides camera eye and target by vector in camera space.
	pub fn local_slide(&mut self, vec: Vec3) {
		self.pos += self.rot * vec;
	}
	/// Slides camera eye and target by vector in world space.
	pub fn slide(&mut self, vec: Vec3) {
		self.pos += vec;
	}
	/// Orbits eye by rotation in camera space around target.
	pub fn local_orbit(&mut self, rot: Quat) {
		self.rot *= rot;
	}
	/// Orbits eye by rotation in camera space around point in camera space.
	pub fn local_orbit_around(&mut self, rot: Quat, pos: Vec3) {
		self.local_slide(pos - rot * pos);
		self.local_orbit(rot);
	}
	/// Orbits eye by rotation in world space around target.
	pub fn orbit(&mut self, rot: Quat) {
		self.rot = rot * self.rot;
	}
	/// Orbits eye by rotation in world space around point in world space.
	pub fn orbit_around(&mut self, rot: Quat, pos: Vec3) {
		let pos = pos - self.pos;
		self.slide(pos - rot * pos);
		self.orbit(rot);
	}
	/// Orbits target around eye by pitch and yaw preserving roll attitude aka first person view.
	///
	/// Use fixed [`Self::yaw_axis()`] by capturing it when entering first person view.
	pub fn look_around(&mut self, pitch: f32, yaw: f32, yaw_axis: Vec3) {
		let pitch = Quat::from_axis_angle(self.pitch_axis(), pitch);
		let yaw = Quat::from_axis_angle(yaw_axis, yaw);
		self.orbit_around(yaw * pitch, self.eye());
	}
	/// Positive x-axis in camera space pointing from left to right.
	#[allow(clippy::unused_self)]
	#[must_use]
	pub const fn local_pitch_axis(&self) -> Vec3 {
		Vec3::X
	}
	/// Positive y-axis in camera space pointing from bottom to top.
	#[allow(clippy::unused_self)]
	#[must_use]
	pub const fn local_yaw_axis(&self) -> Vec3 {
		Vec3::Y
	}
	/// Positive z-axis in camera space pointing from back to front.
	#[allow(clippy::unused_self)]
	#[must_use]
	pub const fn local_roll_axis(&self) -> Vec3 {
		Vec3::Z
	}
	/// Positive x-axis in world space pointing from left to right.
	#[must_use]
	pub fn pitch_axis(&self) -> Vec3 {
		self.rot * self.local_pitch_axis()
	}
	/// Positive y-axis in world space pointing from bottom to top.
	#[must_use]
	pub fn yaw_axis(&self) -> Vec3 {
		self.rot * self.local_yaw_axis()
	}
	/// Positive z-axis in world space pointing from back to front.
	#[must_use]
	pub fn roll_axis(&self) -> Vec3 {
		self.rot * self.local_roll_axis()
	}
	/// Interpolates between two frames using linear interpolation for the translation part, and
	/// spherical linear interpolation along the shortest path for the rotation part.
	///
	/// # Arguments
	///
	///   * `self`: The initial frame to interpolate from.
	///   * `other`: The final frame to interpolate toward.
	///   * `t`: The interpolation parameter between 0 and 1.
	#[must_use]
	pub fn lerp_slerp(&self, other: &Self, t: f32) -> Self {
		Self {
			pos: self.pos.lerp(other.pos, t),
			rot: self.rot.slerp(other.rot, t),
			zat: float::mul_add(other.zat, t, self.zat * (1.0 - t)),
		}
	}
	/// Renormalizes eye rotation and returns its norm.
	pub fn renormalize(&mut self) -> f32 {
		let norm = self.rot.length();
		self.rot /= norm;
		norm
	}
	/// View transformation from camera to world space.
	#[must_use]
	pub fn view(&self) -> Rigid {
		Rigid::from_parts(
			// Eye position in world space with origin in camera space.
			self.eye(),
			// Eye rotation from camera to world space around target.
			self.rot,
		)
	}
	/// Inverse view transformation from world to camera space.
	///
	/// Uses less computations than <code>[Self::view()].inverse()</code>.
	#[must_use]
	pub fn inverse_view(&self) -> Rigid {
		// Eye rotation from world to camera space around target.
		let rot = self.rot.inverse();
		// Eye position in camera space with origin in world space.
		let eye = rot * self.pos + Vec3::Z * self.zat;
		// Translate in such a way that the eye position with origin in world space vanishes.
		Rigid::from_parts(-eye, rot)
	}
	/// Whether all components are approximately equal within absolute difference `max_abs_diff`.
	#[must_use]
	pub fn abs_diff_eq(&self, other: &Self, max_abs_diff: f32) -> bool {
		self.pos.abs_diff_eq(other.pos, max_abs_diff)
			&& self.rot.abs_diff_eq(other.rot, max_abs_diff)
			&& (self.zat - other.zat).abs() <= max_abs_diff
	}
}
