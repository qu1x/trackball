// Generated `rkyv` resolver lacks docs of its variant fields.
#![cfg_attr(feature = "rkyv-08", allow(missing_docs))]

use crate::{Frame, float};
use glam::{Quat, Vec3};

/// Delta transform from initial to final [`Frame`].
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "rkyv-08",
    derive(rkyv_08::Archive, rkyv_08::Serialize, rkyv_08::Deserialize),
    rkyv(crate = rkyv_08)
)]
#[non_exhaustive]
pub enum Delta {
    /// Yields frame as identity transform (default).
    #[default]
    Frame,
    /// Orbits target around eye by pitch and yaw preserving roll attitude aka first person view.
    ///
    /// See [`Frame::look_around()`].
    First {
        /// Pitch angle.
        pitch: f32,
        /// Yaw angle.
        yaw: f32,
        /// Yaw axis.
        yaw_axis: Vec3,
    },
    /// Tracks target which slides by vector in world space.
    ///
    /// Preserves eye position inclusive its roll attitude.
    Track {
        /// Vector in world space of a sliding target to track.
        vector: Vec3,
    },
    /// Orbits eye by rotation in camera space around point in camera space.
    ///
    /// See [`Frame::local_orbit_around()`].
    Orbit {
        /// Rotation in camera space.
        rotation: Quat,
        /// Point in camera space.
        point: Vec3,
    },
    /// Slides camera eye and target by vector in camera space.
    ///
    /// See [`Frame::local_slide()`].
    Slide {
        /// Vector in camera space.
        vector: Vec3,
    },
    /// Scales distance between eye and point in camera space by ratio preserving target position.
    ///
    /// See [`Frame::local_scale_around()`].
    Scale {
        /// Scale ratio.
        ratio: f32,
        /// Point in camera space.
        point: Vec3,
    },
}

impl Delta {
    /// Transforms from initial to final frame.
    #[must_use]
    pub fn transform(&self, frame: Frame) -> Frame {
        let mut frame = frame;
        match *self {
            Self::Frame => {}
            Self::First {
                pitch,
                yaw,
                yaw_axis,
            } => frame.look_around(pitch, yaw, yaw_axis),
            Self::Track { vector } => frame.set_target(frame.target() + vector),
            Self::Orbit { rotation, point } => frame.local_orbit_around(rotation, point),
            Self::Slide { vector } => frame.local_slide(vector),
            Self::Scale { ratio, point } => frame.local_scale_around(ratio, point),
        }
        frame
    }
    /// Inverses delta transform.
    ///
    /// Effectively swaps initial with final frame.
    #[must_use]
    pub fn inverse(&self) -> Self {
        match *self {
            Self::Frame => Self::Frame,
            Self::First {
                pitch,
                yaw,
                yaw_axis,
            } => Self::First {
                pitch: -pitch,
                yaw: -yaw,
                yaw_axis,
            },
            Self::Track { vector } => Self::Track { vector: -vector },
            Self::Orbit { rotation, point } => Self::Orbit {
                rotation: rotation.inverse(),
                point,
            },
            Self::Slide { vector } => Self::Slide { vector: -vector },
            // Point in camera space of final frame is scaled by ratio.
            Self::Scale { ratio, point } => Self::Scale {
                ratio: ratio.recip(),
                point: point * ratio,
            },
        }
    }
    /// Interpolates delta transform to a fraction using linear interpolation for the translation
    /// part, and spherical linear interpolation for the rotation part.
    ///
    /// # Arguments
    ///
    ///   * `self`: The delta transform to interpolate from.
    ///   * `t`: The interpolation parameter between 0 and 1.
    #[must_use]
    pub fn lerp_slerp(&self, t: f32) -> Self {
        match *self {
            Self::Frame => Self::Frame,
            Self::First {
                pitch,
                yaw,
                yaw_axis,
            } => Self::First {
                pitch: pitch * t,
                yaw: yaw * t,
                yaw_axis,
            },
            Self::Track { vector } => Self::Track { vector: vector * t },
            Self::Orbit { rotation, point } => Self::Orbit {
                rotation: Quat::IDENTITY.slerp(rotation, t),
                point,
            },
            Self::Slide { vector } => Self::Slide { vector: vector * t },
            Self::Scale { ratio, point } => Self::Scale {
                ratio: float::mul_add(ratio - 1.0, t, 1.0),
                point,
            },
        }
    }
}
