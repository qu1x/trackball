use approx::{AbsDiffEq, RelativeEq, UlpsEq};
use core::ops::{Mul, MulAssign};
use glam::{Mat4, Quat, Vec3};

/// Rigid transformation as rotation followed by translation, i.e., a direct isometry.
///
/// Implements [`Default`] and can be created with `Rigid::default()` returning [`Self::IDENTITY`].
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
	feature = "rkyv",
	derive(rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)
)]
#[repr(C)]
pub struct Rigid {
	/// Rotational part.
	pub rotation: Quat,
	/// Translational part.
	pub translation: Vec3,
}

impl Rigid {
	/// Identity transformation.
	pub const IDENTITY: Self = Self {
		rotation: Quat::IDENTITY,
		translation: Vec3::ZERO,
	};

	/// Rigid transformation from `translation` and `axisangle` as rotation axis scaled by angle.
	#[must_use]
	pub fn new(translation: Vec3, axisangle: Vec3) -> Self {
		Self::from_parts(translation, Quat::from_scaled_axis(axisangle))
	}
	/// Rigid transformation from `translation` and `rotation`.
	#[must_use]
	pub const fn from_parts(translation: Vec3, rotation: Quat) -> Self {
		Self {
			rotation,
			translation,
		}
	}
	/// Inverse transformation.
	#[must_use]
	pub fn inverse(&self) -> Self {
		let rotation = self.rotation.inverse();
		Self {
			rotation,
			translation: rotation * -self.translation,
		}
	}
	/// Transforms point.
	#[must_use]
	pub fn transform_point(&self, point: Vec3) -> Vec3 {
		self.rotation * point + self.translation
	}
	/// Transforms vector, i.e., rotation only.
	#[must_use]
	pub fn transform_vector(&self, vector: Vec3) -> Vec3 {
		self.rotation * vector
	}
	/// Homogeneous transformation matrix.
	#[must_use]
	pub fn to_mat4(&self) -> Mat4 {
		Mat4::from_rotation_translation(self.rotation, self.translation)
	}
}

impl From<Rigid> for Mat4 {
	fn from(rigid: Rigid) -> Self {
		rigid.to_mat4()
	}
}

impl Mul for Rigid {
	type Output = Self;

	fn mul(self, rhs: Self) -> Self {
		Self {
			rotation: self.rotation * rhs.rotation,
			translation: self.transform_point(rhs.translation),
		}
	}
}

impl MulAssign for Rigid {
	fn mul_assign(&mut self, rhs: Self) {
		*self = *self * rhs;
	}
}

impl Mul<Vec3> for Rigid {
	type Output = Vec3;

	/// Transforms point, see [`Rigid::transform_point()`].
	fn mul(self, rhs: Vec3) -> Vec3 {
		self.transform_point(rhs)
	}
}

impl AbsDiffEq for Rigid {
	type Epsilon = f32;

	fn default_epsilon() -> f32 {
		f32::default_epsilon()
	}

	fn abs_diff_eq(&self, other: &Self, epsilon: f32) -> bool {
		self.rotation.abs_diff_eq(other.rotation, epsilon)
			&& self.translation.abs_diff_eq(other.translation, epsilon)
	}
}

impl RelativeEq for Rigid {
	fn default_max_relative() -> f32 {
		f32::default_max_relative()
	}

	fn relative_eq(&self, other: &Self, epsilon: f32, max_relative: f32) -> bool {
		self.rotation
			.relative_eq(&other.rotation, epsilon, max_relative)
			&& self
				.translation
				.relative_eq(&other.translation, epsilon, max_relative)
	}
}

impl UlpsEq for Rigid {
	fn default_max_ulps() -> u32 {
		f32::default_max_ulps()
	}

	fn ulps_eq(&self, other: &Self, epsilon: f32, max_ulps: u32) -> bool {
		self.rotation.ulps_eq(&other.rotation, epsilon, max_ulps)
			&& self
				.translation
				.ulps_eq(&other.translation, epsilon, max_ulps)
	}
}
