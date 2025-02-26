use crate::Frame;
use nalgebra::{Point3, RealField, Unit, UnitQuaternion, Vector3};
use simba::scalar::SubsetOf;

/// Delta transform from initial to final [`Frame`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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
	/// Tracks target which slides by vector in world space.
	///
	/// Preserves eye position inclusive its roll attitude.
	Track {
		/// Vector in world space of a sliding target to track.
		vec: Vector3<N>,
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
	#[must_use]
	pub fn transform(&self, frame: &Frame<N>) -> Frame<N> {
		let mut frame = *frame;
		match self {
			Self::Frame => {}
			Self::First {
				pitch,
				yaw,
				yaw_axis,
			} => frame.look_around(*pitch, *yaw, yaw_axis),
			Self::Track { vec } => frame.set_target(frame.target() + vec),
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
			Self::Track { vec } => Self::Track { vec: -vec },
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
	/// Casts components to another type, e.g., between [`f32`] and [`f64`].
	#[must_use]
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
			Self::Track { vec } => Delta::Track { vec: vec.cast() },
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

#[cfg(feature = "rkyv")]
impl<N: Copy + RealField> rkyv::Archive for Delta<N> {
	type Archived = Self;
	type Resolver = ();

	#[inline]
	#[allow(unsafe_code)]
	unsafe fn resolve(&self, _: usize, (): Self::Resolver, out: *mut Self::Archived) {
		unsafe {
			out.write(rkyv::to_archived!(*self as Self));
		}
	}
}

#[cfg(feature = "rkyv")]
impl<Ser: rkyv::Fallible + ?Sized, N: Copy + RealField> rkyv::Serialize<Ser> for Delta<N> {
	#[inline]
	fn serialize(&self, _: &mut Ser) -> Result<Self::Resolver, Ser::Error> {
		Ok(())
	}
}

#[cfg(feature = "rkyv")]
impl<De: rkyv::Fallible + ?Sized, N: Copy + RealField> rkyv::Deserialize<Self, De> for Delta<N> {
	#[inline]
	fn deserialize(&self, _: &mut De) -> Result<Self, De::Error> {
		Ok(rkyv::from_archived!(*self))
	}
}
