use crate::{Clamp, Frame, Plane, Rigid, float};
use glam::{Quat, Vec3};

/// Orthogonal boundary conditions implementing [`Clamp`].
///
/// Implements [`Default`] and can be created with `Bound::default()`.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
	feature = "rkyv-08",
	derive(rkyv_08::Archive, rkyv_08::Serialize, rkyv_08::Deserialize),
	rkyv(crate = rkyv_08)
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
    /// Minimum components of up axis in bound space following yaw. Default splats `f32::MIN`.
    ///
    /// See [`Clamp::up()`] implementation of [`Bound`].
    pub min_up: Vec3,
    /// Maximum components of up axis in bound space following yaw. Default splats `f32::MAX`.
    ///
    /// See [`Clamp::up()`] implementation of [`Bound`].
    pub max_up: Vec3,
    /// Minimum distance of eye from target. Default is `0.0`.
    pub min_distance: f32,
    /// Maximum distance of eye from target. Default is `f32::MAX`.
    pub max_distance: f32,
    /// Epsilon allowing clamped [`Delta`] to more likely pass revalidation.
    ///
    /// Default is <code>[f32::EPSILON].sqrt()</code>.
    ///
    /// [`Delta`]: crate::Delta
    pub hysteresis: f32,
}

impl Default for Bound {
    fn default() -> Self {
        Self {
            transform: Rigid::IDENTITY,
            hysteresis: float::sqrt(f32::EPSILON),
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
    fn target(&self, frame: Frame) -> Option<Plane> {
        let target = self.transform.inverse() * frame.target();
        self.exceeded(target, self.min_target, self.max_target, Quat::IDENTITY)
            .map(|plane| plane.transform_by(self.transform))
    }
    /// Find any boundary plane exceeded by eye position.
    ///
    /// Distance planes are orthogonal to the roll axis at minimum or maximum distance from target.
    fn eye(&self, frame: Frame) -> Option<Plane> {
        let distance = frame.distance();
        let (target, roll_axis) = (frame.target(), frame.roll_axis());
        if (self.min_distance - distance) > self.hysteresis {
            return Some(Plane::with_point(
                roll_axis,
                target + roll_axis * self.min_distance,
            ));
        }
        if (self.max_distance - distance) < -self.hysteresis {
            return Some(Plane::with_point(
                roll_axis,
                target + roll_axis * self.max_distance,
            ));
        }
        let eye = self.transform.inverse() * frame.eye();
        self.exceeded(eye, self.min_eye, self.max_eye, Quat::IDENTITY)
            .map(|plane| plane.transform_by(self.transform))
    }
    /// Find any boundary plane exceeded by up position.
    ///
    /// Up position is expressed in bound space following yaw, i.e., rotated around y-axis by the
    /// heading whose x-axis is the horizontal pitch axis. Its y-component bounds the tilt (e.g.,
    /// `min_up.y = 0.0` prevents turning upside down but allows rolling sideways over the pole)
    /// whereas its x-component bounds the roll (e.g., `min_up.x = max_up.x = 0.0` prevents rolling
    /// and stops at the pole like a turntable) and its z-component bounds the forward tilt.
    fn up(&self, frame: Frame) -> Option<Plane> {
        let rotation = self.transform.rotation;
        let pitch_axis = rotation.inverse() * frame.pitch_axis();
        let heading = Quat::from_rotation_y(float::atan2(-pitch_axis.z, pitch_axis.x));
        let up = heading.inverse() * rotation.inverse() * frame.yaw_axis();
        self.exceeded(up, self.min_up, self.max_up, rotation * heading)
    }
}

impl Bound {
    /// Find any boundary plane exceeded by position in bound space and rotate it by `rotation`.
    fn exceeded(&self, position: Vec3, min: Vec3, max: Vec3, rotation: Quat) -> Option<Plane> {
        let axes = [Vec3::X, Vec3::Y, Vec3::Z];
        for (index, axis) in axes.into_iter().enumerate() {
            let min_plane = Plane::new(axis, min[index]);
            if min_plane.distance_from(position) > self.hysteresis {
                return Some(min_plane.rotate_by(rotation));
            }
            let max_plane = Plane::new(axis, max[index]);
            if max_plane.distance_from(position) < -self.hysteresis {
                return Some(max_plane.rotate_by(rotation));
            }
        }
        None
    }
}
