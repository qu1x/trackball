// Generated `rkyv` resolver lacks docs of its variant fields.
#![cfg_attr(feature = "rkyv", allow(missing_docs))]

use crate::{Frame, float};
use glam::{Quat, Vec3};

/// Delta transform from initial to final [`Frame`].
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
	feature = "rkyv",
	derive(rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)
)]
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
		vec: Vec3,
	},
	/// Orbits eye by rotation in camera space around point in camera space.
	///
	/// See [`Frame::local_orbit_around()`].
	Orbit {
		/// Rotation in camera space.
		rot: Quat,
		/// Point in camera space.
		pos: Vec3,
	},
	/// Slides camera eye and target by vector in camera space.
	///
	/// See [`Frame::local_slide()`].
	Slide {
		/// Vector in camera space.
		vec: Vec3,
	},
	/// Scales distance between eye and point in camera space by ratio preserving target position.
	///
	/// See [`Frame::local_scale_around()`].
	Scale {
		/// Scale ratio.
		rat: f32,
		/// Point in camera space.
		pos: Vec3,
	},
}

impl Delta {
	/// Transforms from initial to final frame.
	#[must_use]
	pub fn transform(&self, frame: &Frame) -> Frame {
		let mut frame = *frame;
		match *self {
			Self::Frame => {}
			Self::First {
				pitch,
				yaw,
				yaw_axis,
			} => frame.look_around(pitch, yaw, yaw_axis),
			Self::Track { vec } => frame.set_target(frame.target() + vec),
			Self::Orbit { rot, pos } => frame.local_orbit_around(rot, pos),
			Self::Slide { vec } => frame.local_slide(vec),
			Self::Scale { rat, pos } => frame.local_scale_around(rat, pos),
		}
		frame
	}
	/// Inverses delta transform.
	///
	/// Effectively swaps initial with final frame.
	#[must_use]
	pub fn inverse(self) -> Self {
		match self {
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
			Self::Track { vec } => Self::Track { vec: -vec },
			Self::Orbit { rot, pos } => Self::Orbit {
				rot: rot.inverse(),
				pos,
			},
			Self::Slide { vec } => Self::Slide { vec: -vec },
			Self::Scale { rat, pos } => Self::Scale {
				rat: 2.0 - rat,
				pos,
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
			Self::Track { vec } => Self::Track { vec: vec * t },
			Self::Orbit { rot, pos } => Self::Orbit {
				rot: Quat::IDENTITY.slerp(rot, t),
				pos,
			},
			Self::Slide { vec } => Self::Slide { vec: vec * t },
			Self::Scale { rat, pos } => Self::Scale {
				rat: float::mul_add(rat - 1.0, t, 1.0),
				pos,
			},
		}
	}
}
