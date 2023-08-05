use crate::Frame;
use nalgebra::{Point3, RealField, Unit, UnitQuaternion, Vector3};
use simba::scalar::SubsetOf;

/// Delta transform from initial to final [`Frame`].
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
	feature = "rkyv",
	derive(rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)
)]
pub enum Delta<N: Copy + RealField> {
	/// Yields frame as identity transform (default).
	Frame,
	/// Orbits target around eye by pitch and yaw preserving roll attitude aka first person view.
	///
	/// See [`Frame::look_around()`].
	First {
		/// Pitch angle.
		pitch: N,
		/// Yaw angle.
		yaw: N,
		/// Yaw axis.
		yaw_axis: Unit<Vector3<N>>,
	},
	/// Orbits eye by rotation in camera space around point in camera space.
	///
	/// See [`Frame::local_orbit_around()`].
	Orbit {
		/// Rotation in camera space.
		rot: UnitQuaternion<N>,
		/// Point in camera space.
		pos: Point3<N>,
	},
	/// Slides camera eye and target by vector in camera space.
	///
	/// See [`Frame::local_slide()`].
	Slide {
		/// Vector in camera space.
		vec: Vector3<N>,
	},
	/// Scales distance between eye and point in camera space by ratio preserving target position.
	///
	/// See [`Frame::local_scale_around()`].
	Scale {
		/// Scale ratio.
		rat: N,
		/// Point in camera space.
		pos: Point3<N>,
	},
}

impl<N: Copy + RealField> Delta<N> {
	/// Transforms from initial to final frame.
	pub fn transform(&self, frame: &Frame<N>) -> Frame<N> {
		let mut frame = frame.clone();
		match self {
			Self::Frame => {}
			Self::First {
				pitch,
				yaw,
				ref yaw_axis,
			} => frame.look_around(*pitch, *yaw, yaw_axis),
			Self::Orbit { rot, pos } => frame.local_orbit_around(rot, pos),
			Self::Slide { vec } => frame.local_slide(vec),
			Self::Scale { rat, pos } => frame.local_scale_around(*rat, pos),
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
			Self::Orbit { rot, pos } => Self::Orbit {
				rot: rot.inverse(),
				pos,
			},
			Self::Slide { vec } => Self::Slide { vec: -vec },
			Self::Scale { rat, pos } => Self::Scale {
				rat: N::one() + N::one() - rat,
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
	pub fn lerp_slerp(&self, t: N) -> Self {
		match self.clone() {
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
			Self::Orbit { rot, pos } => Self::Orbit {
				rot: rot.powf(t),
				pos,
			},
			Self::Slide { vec } => Self::Slide { vec: vec * t },
			Self::Scale { rat, pos } => Self::Scale {
				rat: (rat - N::one()) * t + N::one(),
				pos,
			},
		}
	}
	/// Computes common minimum of two delta transforms.
	///
	/// Assumes `other` to be similar (e.g., a clamped `self`).
	///
	/// Returns `None` if transforms are of different variants and neither is [`Self::Frame`].
	pub fn minimum(&self, other: &Self) -> Option<Self> {
		match (self, other) {
			(Self::Frame, _) => Some(other.clone()),
			(_, Self::Frame) => Some(self.clone()),
			(
				Self::First {
					pitch: self_pitch,
					yaw: self_yaw,
					yaw_axis: self_yaw_axis,
				},
				Self::First {
					pitch: other_pitch,
					yaw: other_yaw,
					..
				},
			) => Some(Self::First {
				pitch: *if self_pitch.abs() <= other_pitch.abs() {
					self_pitch
				} else {
					other_pitch
				},
				yaw: *if self_yaw.abs() <= other_yaw.abs() {
					self_yaw
				} else {
					other_yaw
				},
				yaw_axis: *self_yaw_axis,
			}),
			(
				Self::Orbit {
					rot: self_rot,
					pos: self_pos,
				},
				Self::Orbit { rot: other_rot, .. },
			) => Some(Self::Orbit {
				rot: *if self_rot.angle() <= other_rot.angle() {
					self_rot
				} else {
					other_rot
				},
				pos: *self_pos,
			}),
			(Self::Slide { vec: self_vec }, Self::Slide { vec: other_vec }) => Some(Self::Slide {
				vec: Vector3::new(
					if self_vec.x.abs() <= other_vec.x.abs() {
						self_vec.x
					} else {
						other_vec.x
					},
					if self_vec.y.abs() <= other_vec.y.abs() {
						self_vec.y
					} else {
						other_vec.y
					},
					if self_vec.z.abs() <= other_vec.z.abs() {
						self_vec.z
					} else {
						other_vec.z
					},
				),
			}),
			(
				Self::Scale {
					rat: self_rat,
					pos: self_pos,
				},
				Self::Scale { rat: other_rat, .. },
			) => Some(Self::Scale {
				rat: if (*self_rat - N::one()).abs() <= (*other_rat - N::one()).abs() {
					*self_rat
				} else {
					*other_rat
				},
				pos: *self_pos,
			}),
			(_, _) => None,
		}
	}
	/// Casts components to another type, e.g., between [`f32`] and [`f64`].
	pub fn cast<M: Copy + RealField>(self) -> Delta<M>
	where
		N: SubsetOf<M>,
	{
		match self {
			Self::Frame => Delta::Frame,
			Self::First {
				pitch,
				yaw,
				yaw_axis,
			} => Delta::First {
				pitch: pitch.to_superset(),
				yaw: yaw.to_superset(),
				yaw_axis: yaw_axis.cast(),
			},
			Self::Orbit { rot, pos } => Delta::Orbit {
				rot: rot.cast(),
				pos: pos.cast(),
			},
			Self::Slide { vec } => Delta::Slide { vec: vec.cast() },
			Self::Scale { rat, pos } => Delta::Scale {
				rat: rat.to_superset(),
				pos: pos.cast(),
			},
		}
	}
}

impl<N: Copy + RealField> Default for Delta<N> {
	fn default() -> Self {
		Self::Frame
	}
}
