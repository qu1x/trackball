use nalgebra::{Point2, RealField, Unit, UnitQuaternion, Vector3};
use simba::scalar::SubsetOf;

#[cfg(not(feature = "cc"))]
use crate::Image;

/// Orbit induced by displacement on screen.
///
/// Implements [`Default`] and can be created with `Orbit::default()`.
///
/// Both its methods must be invoked on matching events fired by your 3D graphics library of choice.
#[derive(Debug, Clone, Default)]
pub struct Orbit<N: Copy + RealField> {
	/// Caches normalization of previous cursor/finger position.
	vec: Option<(Unit<Vector3<N>>, N)>,
}

#[cfg(not(feature = "cc"))]
use nalgebra::Matrix3;

#[cfg(not(feature = "cc"))]
impl<N: Copy + RealField> Orbit<N> {
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
	pub fn compute(&mut self, pos: &Point2<N>, max: &Point2<N>) -> Option<UnitQuaternion<N>> {
		// Clamped cursor/finger position from left to right and top to bottom.
		let pos = Image::clamp_pos_wrt_max(pos, max);
		// Centered cursor/finger position and its maximum from left to right and bottom to top.
		let (pos, max) = Image::transform_pos_and_max_wrt_max(&pos, max);
		// Positive z-axis pointing from far to near.
		let (pos, pza) = (pos.coords.push(N::zero()), Vector3::z_axis());
		// New position as ray and length on xy-plane or z-axis of zero length for origin position.
		let (ray, len) = Unit::try_new_and_get(pos, N::zero()).unwrap_or((pza, N::zero()));
		// Get old ray and length as start position and offset and replace with new ray and length.
		let (pos, off) = self.vec.replace((ray, len))?;
		// Displacement vector from old to new ray and length.
		let vec = ray.into_inner() * len - pos.into_inner() * off;
		// Shadow new ray and length as normalized displacement vector.
		let (ray, len) = Unit::try_new_and_get(vec, N::zero())?;
		// Treat maximum of half the screen's width or height as trackball's radius.
		let max = max.x.max(max.y);
		// Map trackball's diameter onto half its circumference for start positions so that only
		// screen corners are mapped to lower hemisphere which induces less intuitive rotations.
		let (sin, cos) = (off / max * N::frac_pi_2()).sin_cos();
		// Exponential map of start position.
		let exp = Vector3::new(sin * pos.x, sin * pos.y, cos);
		// Tangent ray of geodesic at exponential map.
		let tan = Vector3::new(cos * pos.x, cos * pos.y, -sin);
		// Cross product of z-axis and start position to construct orthonormal frames.
		let zxp = Vector3::new(-pos.y, pos.x, N::zero());
		// Orthonormal frame as argument of differential of exponential map.
		let arg = Matrix3::from_columns(&[pza.into_inner(), pos.into_inner(), zxp]);
		// Orthonormal frame as image of differential of exponential map.
		let img = Matrix3::from_columns(&[exp, tan, zxp]);
		// Compute differential of exponential map by its argument and image and apply it to
		// displacement vector which in turn spans rotation plane together with exponential map.
		let vec = (img * arg.tr_mul(&ray.into_inner())).cross(&exp);
		// Angle of rotation is displacement length divided by radius.
		Unit::try_new(vec, N::zero()).map(|ray| UnitQuaternion::from_axis_angle(&ray, len / max))
	}
	/// Discards cached normalization of previous cursor/finger position on button/finger release.
	pub fn discard(&mut self) {
		self.vec = None;
	}
	/// Casts components to another type, e.g., between [`f32`] and [`f64`].
	pub fn cast<M: Copy + RealField>(self) -> Orbit<M>
	where
		N: SubsetOf<M>,
	{
		Orbit {
			vec: self.vec.map(|(ray, len)| (ray.cast(), len.to_superset())),
		}
	}
}

#[cfg(feature = "cc")]
use nalgebra::Quaternion;

#[cfg(feature = "cc")]
impl Orbit<f32> {
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
	pub fn compute(&mut self, pos: &Point2<f32>, max: &Point2<f32>) -> Option<UnitQuaternion<f32>> {
		let mut rot = Quaternion::identity();
		let mut old = self
			.vec
			.map(|(ray, len)| ray.into_inner().push(len))
			.unwrap_or_default();
		unsafe {
			trackball_orbit_f(
				rot.as_vector_mut().as_mut_ptr(),
				old.as_mut_ptr(),
				pos.coords.as_ptr(),
				max.coords.as_ptr(),
			);
		}
		self.vec = Some((Unit::new_unchecked(old.xyz()), old.w));
		#[allow(clippy::float_cmp)]
		(rot.w != 1.0).then(|| UnitQuaternion::new_unchecked(rot))
	}
	/// Discards cached normalization of previous cursor/finger position on button/finger release.
	pub fn discard(&mut self) {
		self.vec = None;
	}
	/// Casts components to another type, e.g., to [`f64`].
	pub fn cast<M: Copy + RealField>(self) -> Orbit<M>
	where
		f32: SubsetOf<M>,
	{
		Orbit {
			vec: self.vec.map(|(ray, len)| (ray.cast(), len.to_superset())),
		}
	}
}

#[cfg(feature = "cc")]
impl Orbit<f64> {
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
	pub fn compute(&mut self, pos: &Point2<f64>, max: &Point2<f64>) -> Option<UnitQuaternion<f64>> {
		let mut rot = Quaternion::identity();
		let mut old = self
			.vec
			.map(|(ray, len)| ray.into_inner().push(len))
			.unwrap_or_default();
		unsafe {
			trackball_orbit_d(
				rot.as_vector_mut().as_mut_ptr(),
				old.as_mut_ptr(),
				pos.coords.as_ptr(),
				max.coords.as_ptr(),
			);
		}
		self.vec = Some((Unit::new_unchecked(old.xyz()), old.w));
		#[allow(clippy::float_cmp)]
		(rot.w != 1.0).then(|| UnitQuaternion::new_unchecked(rot))
	}
	/// Discards cached normalization of previous cursor/finger position on button/finger release.
	pub fn discard(&mut self) {
		self.vec = None;
	}
	/// Casts components to another type, e.g., to [`f32`].
	pub fn cast<M: Copy + RealField>(self) -> Orbit<M>
	where
		f64: SubsetOf<M>,
	{
		Orbit {
			vec: self.vec.map(|(ray, len)| (ray.cast(), len.to_superset())),
		}
	}
}

#[cfg(feature = "cc")]
extern "C" {
	fn trackball_orbit_f(xyzw: *mut f32, xyzm: *mut f32, xy: *const f32, wh: *const f32);
	fn trackball_orbit_d(xyzw: *mut f64, xyzm: *mut f64, xy: *const f64, wh: *const f64);
}
