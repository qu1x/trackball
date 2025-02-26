use crate::Fixed;
use nalgebra::{Matrix4, Point2, RealField, convert};
use simba::scalar::SubsetOf;

/// Scope defining enclosing viewing frustum.
///
/// Implements [`Default`] and can be created with `Scope::default()`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Scope<N: Copy + RealField> {
	/// Fixed quantity wrt field of view.
	///
	/// Default is fixed vertical field of view of π/4.
	fov: Fixed<N>,
	/// Clip plane distances.
	///
	/// Near and far clip plane distances from either target or eye whether [`Self::oim`]. Default
	/// is `(1e-1, 1e+3)` measured from eye.
	zcp: (N, N),
	/// Object inspection mode.
	///
	/// Scales clip plane distances by measuring from target instead of eye. Default is `false`.
	oim: bool,
	/// Orthographic projection mode.
	///
	/// Computes scale-identical orthographic instead of perspective projection. Default is `false`.
	opm: bool,
}

impl<N: Copy + RealField> Default for Scope<N> {
	fn default() -> Self {
		Self {
			fov: Fixed::default(),
			zcp: (convert(1e-1), convert(1e+3)),
			oim: false,
			opm: false,
		}
	}
}

impl<N: Copy + RealField> Scope<N> {
	/// Fixed quantity wrt field of view, see [`Self::set_fov()`].
	#[must_use]
	pub const fn fov(&self) -> Fixed<N> {
		self.fov
	}
	/// Sets fixed quantity wrt field of view.
	///
	/// Default is fixed vertical field of view of π/4.
	///
	/// ```
	/// use nalgebra::Point2;
	/// use trackball::Scope;
	///
	/// // Current screen size.
	/// let max = Point2::new(800, 600);
	/// // Default scope with fixed vertical field of view of π/4:
	/// //
	/// //   * Increasing width increases horizontal field of view (more can be seen).
	/// //   * Increasing height scales scope zooming in as vertical field of view is fixed.
	/// let mut scope = Scope::default();
	/// // Unfix vertical field of view by fixing current unit per pixel on focus plane at distance
	/// // from eye of one, that is effectively `upp` divided by `zat` to make it scale-independant:
	/// //
	/// //   * Increasing width increases horizontal field of view (more can be seen).
	/// //   * Increasing height increases vertical field of view (more can be seen).
	/// scope.set_fov(scope.fov().to_upp(&max.cast::<f32>()));
	/// ```
	pub fn set_fov(&mut self, fov: impl Into<Fixed<N>>) {
		self.fov = fov.into();
	}
	/// Clip plane distances from eye regardless of [`Self::scale()`] wrt to distance between eye
	/// and target.
	///
	/// Default is `(1e-1, 1e+3)` measured from eye.
	#[must_use]
	pub fn clip_planes(&self, zat: N) -> (N, N) {
		if self.oim {
			let (znear, zfar) = self.zcp;
			(zat - znear, zat + zfar)
		} else {
			self.zcp
		}
	}
	/// Sets clip plane distances from target or eye whether [`Self::scale()`].
	///
	/// Default is `(1e-1, 1e+3)` measured from eye.
	pub const fn set_clip_planes(&mut self, znear: N, zfar: N) {
		self.zcp = (znear, zfar);
	}
	/// Object inspection mode.
	///
	/// Scales clip plane distances by measuring from target instead of eye. Default is `false`.
	#[must_use]
	pub const fn scale(&self) -> bool {
		self.oim
	}
	/// Sets object inspection mode.
	///
	/// Scales clip plane distances by measuring from target instead of eye. Default is `false`.
	pub const fn set_scale(&mut self, oim: bool) {
		self.oim = oim;
	}
	/// Orthographic projection mode.
	///
	/// Computes scale-identical orthographic instead of perspective projection. Default is `false`.
	#[must_use]
	pub const fn ortho(&self) -> bool {
		self.opm
	}
	/// Sets orthographic projection mode.
	///
	/// Computes scale-identical orthographic instead of perspective projection. Default is `false`.
	pub const fn set_ortho(&mut self, opm: bool) {
		self.opm = opm;
	}
	/// Projection transformation and unit per pixel on focus plane wrt distance between eye and
	/// target and maximum position in screen space.
	#[must_use]
	pub fn projection_and_upp(&self, zat: N, max: &Point2<N>) -> (Matrix4<N>, N) {
		let (znear, zfar) = self.clip_planes(zat);
		if self.opm {
			let (max, upp) = self.fov.max_and_upp(zat, max);
			let mat = Matrix4::new_orthographic(-max.x, max.x, -max.y, max.y, znear, zfar);
			(mat, upp)
		} else {
			let fov = self.fov.to_ver(max).into_inner();
			let (max, upp) = self.fov.max_and_upp(zat, max);
			let mat = Matrix4::new_perspective(max.x / max.y, fov, znear, zfar);
			(mat, upp)
		}
	}
	/// Casts components to another type, e.g., between [`f32`] and [`f64`].
	#[must_use]
	pub fn cast<M: Copy + RealField>(self) -> Scope<M>
	where
		N: SubsetOf<M>,
	{
		let (near, far) = self.zcp;
		Scope {
			fov: self.fov.cast(),
			zcp: (near.to_superset(), far.to_superset()),
			oim: self.oim,
			opm: self.opm,
		}
	}
}

#[cfg(feature = "rkyv")]
impl<N: Copy + RealField> rkyv::Archive for Scope<N> {
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
impl<Ser: rkyv::Fallible + ?Sized, N: Copy + RealField> rkyv::Serialize<Ser> for Scope<N> {
	#[inline]
	fn serialize(&self, _: &mut Ser) -> Result<Self::Resolver, Ser::Error> {
		Ok(())
	}
}

#[cfg(feature = "rkyv")]
impl<De: rkyv::Fallible + ?Sized, N: Copy + RealField> rkyv::Deserialize<Self, De> for Scope<N> {
	#[inline]
	fn deserialize(&self, _: &mut De) -> Result<Self, De::Error> {
		Ok(rkyv::from_archived!(*self))
	}
}
