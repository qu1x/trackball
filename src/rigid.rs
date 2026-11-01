use core::ops::{Mul, MulAssign};
use glam::{Mat4, Quat, Vec3};

/// Rigid transformation as rotation followed by translation, i.e., a direct isometry.
///
/// Implements [`Default`] and can be created with `Rigid::default()` returning [`Self::IDENTITY`].
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
	feature = "rkyv-08",
	derive(rkyv_08::Archive, rkyv_08::Serialize, rkyv_08::Deserialize),
	rkyv(crate = rkyv_08)
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

    /// Rigid transformation from `rotation` followed by `translation`.
    #[must_use]
    pub const fn from_rotation_translation(rotation: Quat, translation: Vec3) -> Self {
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
    /// Whether all components are approximately equal within absolute difference `max_abs_diff`.
    #[must_use]
    pub fn abs_diff_eq(&self, other: Self, max_abs_diff: f32) -> bool {
        self.rotation.abs_diff_eq(other.rotation, max_abs_diff)
            && self
                .translation
                .abs_diff_eq(other.translation, max_abs_diff)
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
