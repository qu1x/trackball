use crate::{Rigid, float};
use glam::{Quat, Vec3};

/// Plane encoding position with signed bias along unit normal.
///
/// Realizes plane equation `a*x+b*y+c*z+d=0` with unit normal `[x, y, z]` and signed bias `d`.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "rkyv-08",
    derive(rkyv_08::Archive, rkyv_08::Serialize, rkyv_08::Deserialize),
    rkyv(crate = rkyv_08)
)]
#[repr(C)]
pub struct Plane {
    /// Plane unit normal.
    pub normal: Vec3,
    /// Signed bias along unit normal.
    pub bias: f32,
}

impl Plane {
    /// Plane from unit `normal` and signed `distance` from the origin.
    ///
    /// ```
    /// use trackball::{Plane, glam::Vec3};
    ///
    /// // Plane intersecting y-axis at `5.0`.
    /// let plane = Plane::new(Vec3::Y, 5.0);
    ///
    /// // Origin projected onto plane where plane intersects y-axis.
    /// let point = plane.project_point(Vec3::ZERO);
    /// assert_eq!(point, Vec3::new(0.0, 5.0, 0.0));
    ///
    /// // Bias is negated distance.
    /// assert_eq!(plane.distance(), 5.0);
    /// assert_eq!(plane.bias, -5.0);
    ///
    /// // Plane intersecting y-axis at `-5.0`.
    /// let plane = Plane::new(Vec3::Y, -5.0);
    ///
    /// // Origin projected onto plane where plane intersects y-axis.
    /// let point = plane.project_point(Vec3::ZERO);
    /// assert_eq!(point, Vec3::new(0.0, -5.0, 0.0));
    ///
    /// // Bias is negated distance.
    /// assert_eq!(plane.distance(), -5.0);
    /// assert_eq!(plane.bias, 5.0);
    /// ```
    #[must_use]
    pub const fn new(normal: Vec3, distance: f32) -> Self {
        Self {
            normal,
            bias: -distance,
        }
    }
    /// Plane from unit normal with point in plane.
    #[must_use]
    pub fn with_point(normal: Vec3, point: Vec3) -> Self {
        Self::new(normal, normal.dot(point))
    }
    /// Signed orthogonal distance from the origin.
    #[must_use]
    pub const fn distance(&self) -> f32 {
        -self.bias
    }
    /// Signed orthogonal distance from `point`.
    #[must_use]
    pub fn distance_from(&self, point: Vec3) -> f32 {
        self.distance() - self.normal.dot(point)
    }
    /// Projects point onto plane.
    #[must_use]
    pub fn project_point(&self, point: Vec3) -> Vec3 {
        point - self.normal * (self.normal.dot(point) + self.bias)
    }
    /// Projects axis onto plane regardless of bias.
    #[must_use]
    pub fn project_axis(&self, axis: Vec3) -> Vec3 {
        self.project_vector(axis).normalize()
    }
    /// Projects vector onto plane regardless of bias.
    #[must_use]
    pub fn project_vector(&self, vector: Vec3) -> Vec3 {
        vector - self.normal * self.normal.dot(vector)
    }
    /// Signed angle from `a` to `b` where both vectors are in the plane.
    ///
    /// Unlike `Vec3::angle_between()`, small and nearly opposite angles are precise.
    #[must_use]
    pub fn angle_between(&self, a: Vec3, b: Vec3) -> f32 {
        float::atan2(self.normal.dot(a.cross(b)), a.dot(b))
    }
    /// Rotates plane.
    #[must_use]
    pub fn rotate_by(self, rot: Quat) -> Self {
        Self {
            normal: rot * self.normal,
            bias: self.bias,
        }
    }
    /// Translates plane.
    #[must_use]
    pub fn translate_by(self, vec: Vec3) -> Self {
        Self {
            normal: self.normal,
            bias: self.bias - self.normal.dot(vec),
        }
    }
    /// Transforms plane by direct isometry, i.e., rotation followed by translation.
    ///
    /// ```
    /// use core::f32::consts::FRAC_PI_2;
    /// use trackball::{
    ///     Plane, Rigid,
    ///     glam::{Quat, Vec3},
    /// };
    ///
    /// // Plane intersecting y-axis at `5.0`.
    /// let plane = Plane::new(Vec3::Y, 5.0).transform_by(Rigid::from_rotation_translation(
    ///     // Rotation by 90 degrees before translation.
    ///     Quat::from_rotation_z(FRAC_PI_2),
    ///     // Translation after rotation.
    ///     Vec3::new(-5.0, 0.0, 0.0),
    /// ));
    /// // Plane intersecting x-axis at `-10.0`.
    /// assert!(plane.abs_diff_eq(Plane::new(-Vec3::X, 10.0), 1e-6));
    /// ```
    #[must_use]
    pub fn transform_by(self, rigid: Rigid) -> Self {
        self.rotate_by(rigid.rotation)
            .translate_by(rigid.translation)
    }
    /// Whether all components are approximately equal within absolute difference `max_abs_diff`.
    #[must_use]
    pub fn abs_diff_eq(&self, other: Self, max_abs_diff: f32) -> bool {
        self.normal.abs_diff_eq(other.normal, max_abs_diff)
            && (self.bias - other.bias).abs() <= max_abs_diff
    }
}
