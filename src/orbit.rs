use glam::{Quat, Vec2, Vec3};

#[cfg(not(feature = "cc"))]
use crate::Image;

/// Orbit induced by displacement on screen.
///
/// Implements [`Default`] and can be created with `Orbit::default()`.
///
/// Both its methods must be invoked on matching events fired by your 3D graphics library of choice.
#[derive(Debug, Clone, Default)]
pub struct Orbit {
	/// Caches normalization of previous cursor/finger position.
	vec: Option<(Vec3, f32)>,
}

#[cfg(not(feature = "cc"))]
use core::f32::consts::FRAC_PI_2;

impl Orbit {
	/// Computes rotation between previous and current cursor/finger position.
	///
	/// Normalization of previous position is cached and has to be discarded on button/finger
	/// release via [`Self::discard()`]. Current position `pos` is clamped between origin and
	/// maximum position `max` as screen's width and height.
	///
	/// Screen space with origin in top left corner:
	///
	///   * x-axis from left to right,
	///   * y-axis from top to bottom.
	///
	/// Camera space with origin at its target, the trackball's center:
	///
	///   * x-axis from left to right,
	///   * y-axis from bottom to top,
	///   * z-axis from far to near.
	///
	/// Returns `None`:
	///
	///   * on first invocation and after [`Self::discard()`] as there is no previous position yet,
	///   * in the unlikely case that a position event fires twice resulting in zero displacements.
	#[cfg(not(feature = "cc"))]
	pub fn compute(&mut self, pos: Vec2, max: Vec2) -> Option<Quat> {
		// Clamped cursor/finger position from left to right and top to bottom.
		let pos = Image::clamp_pos_wrt_max(pos, max);
		// Centered cursor/finger position and its maximum from left to right and bottom to top.
		let (pos, max) = Image::transform_pos_and_max_wrt_max(pos, max);
		// New position as ray and length on xy-plane or any ray on xy-plane for origin position.
		let (ray, len) = (
			pos.try_normalize().unwrap_or(Vec2::X).extend(0.0),
			pos.length(),
		);
		// Get old ray and length as start position and offset and replace with new ray and length.
		let (pos, off) = self.vec.replace((ray, len))?;
		// Displacement vector from old to new ray and length.
		let vec = ray * len - pos * off;
		// Shadow new ray and length as normalized displacement vector.
		let (ray, len) = (vec.try_normalize()?, vec.length());
		// Treat maximum of half the screen's width or height as trackball's radius.
		let max = max.x.max(max.y);
		// Parallel transport along geodesic from trackball's pole on z-axis to exponential map of
		// start position. Maps trackball's diameter onto half its circumference for start positions
		// so that only screen corners are mapped to lower hemisphere which induces less intuitive
		// rotations.
		let transport = Quat::from_axis_angle(Vec3::Z.cross(pos), off / max * FRAC_PI_2);
		// Rotation axis as if displacement starts at pole, then parallel transported.
		let axis = transport * ray.cross(Vec3::Z);
		// Angle of rotation is displacement length divided by radius.
		Some(Quat::from_axis_angle(axis, len / max))
	}
	/// Computes rotation between previous and current cursor/finger position.
	///
	/// Normalization of previous position is cached and has to be discarded on button/finger
	/// release via [`Self::discard()`]. Current position `pos` is clamped between origin and
	/// maximum position `max` as screen's width and height.
	///
	/// Screen space with origin in top left corner:
	///
	///   * x-axis from left to right,
	///   * y-axis from top to bottom.
	///
	/// Camera space with origin at its target, the trackball's center:
	///
	///   * x-axis from left to right,
	///   * y-axis from bottom to top,
	///   * z-axis from far to near.
	///
	/// Returns `None`:
	///
	///   * on first invocation and after [`Self::discard()`] as there is no previous position yet,
	///   * in the unlikely case that a position event fires twice resulting in zero displacements.
	#[cfg(feature = "cc")]
	pub fn compute(&mut self, pos: Vec2, max: Vec2) -> Option<Quat> {
		let mut rot = Quat::IDENTITY.to_array();
		let mut old = self
			.vec
			.map(|(ray, len)| ray.extend(len).to_array())
			.unwrap_or_default();
		let (pos, max) = (pos.to_array(), max.to_array());
		#[allow(unsafe_code)]
		unsafe {
			trackball_orbit_f(
				rot.as_mut_ptr(),
				old.as_mut_ptr(),
				pos.as_ptr(),
				max.as_ptr(),
			);
		}
		let [x, y, z, w] = old;
		self.vec = Some((Vec3::new(x, y, z), w));
		#[allow(clippy::float_cmp)]
		(rot[3] != 1.0).then(|| Quat::from_array(rot))
	}
	/// Discards cached normalization of previous cursor/finger position on button/finger release.
	pub const fn discard(&mut self) {
		self.vec = None;
	}
}

#[cfg(feature = "cc")]
#[allow(unsafe_code)]
unsafe extern "C" {
	fn trackball_orbit_f(xyzw: *mut f32, xyzm: *mut f32, xy: *const f32, wh: *const f32);
}
