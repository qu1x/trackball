use crate::Fixed;
use glam::{Mat4, Vec2, camera::rh::proj::opengl};

/// Scope defining enclosing viewing frustum.
///
/// Implements [`Default`] and can be created with `Scope::default()`.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
	feature = "rkyv-08",
	derive(rkyv_08::Archive, rkyv_08::Serialize, rkyv_08::Deserialize),
	rkyv(crate = rkyv_08)
)]
pub struct Scope {
	/// Fixed quantity wrt field of view.
	///
	/// Default is fixed vertical field of view of π/4.
	fov: Fixed,
	/// Clip plane distances.
	///
	/// Near and far clip plane distances from either target or eye whether [`Self::oim`]. Default
	/// is `(1e-1, 1e+3)` measured from eye.
	zcp: (f32, f32),
	/// Object inspection mode.
	///
	/// Scales clip plane distances by measuring from target instead of eye. Default is `false`.
	oim: bool,
	/// Orthographic projection mode.
	///
	/// Computes scale-identical orthographic instead of perspective projection. Default is `false`.
	opm: bool,
}

impl Default for Scope {
	fn default() -> Self {
		Self {
			fov: Fixed::default(),
			zcp: (1e-1, 1e+3),
			oim: false,
			opm: false,
		}
	}
}

impl Scope {
	/// Fixed quantity wrt field of view, see [`Self::set_fov()`].
	#[must_use]
	pub const fn fov(&self) -> Fixed {
		self.fov
	}
	/// Sets fixed quantity wrt field of view.
	///
	/// Default is fixed vertical field of view of π/4.
	///
	/// ```
	/// use trackball::{Scope, glam::Vec2};
	///
	/// // Current screen size.
	/// let max = Vec2::new(800.0, 600.0);
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
	/// scope.set_fov(scope.fov().to_upp(max));
	/// ```
	pub fn set_fov(&mut self, fov: impl Into<Fixed>) {
		self.fov = fov.into();
	}
	/// Clip plane distances from eye regardless of [`Self::inspect()`] wrt to distance between eye
	/// and target.
	///
	/// Default is `(1e-1, 1e+3)` measured from eye.
	#[must_use]
	pub fn clip_planes(&self, zat: f32) -> (f32, f32) {
		if self.oim {
			let (znear, zfar) = self.zcp;
			(zat - znear, zat + zfar)
		} else {
			self.zcp
		}
	}
	/// Sets clip plane distances from target or eye whether [`Self::inspect()`].
	///
	/// Default is `(1e-1, 1e+3)` measured from eye.
	pub const fn set_clip_planes(&mut self, znear: f32, zfar: f32) {
		self.zcp = (znear, zfar);
	}
	/// Object inspection mode.
	///
	/// Scales clip plane distances by measuring from target instead of eye. Default is `false`.
	#[must_use]
	pub const fn inspect(&self) -> bool {
		self.oim
	}
	/// Sets object inspection mode.
	///
	/// Scales clip plane distances by measuring from target instead of eye. Default is `false`.
	pub const fn set_inspect(&mut self, oim: bool) {
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
	pub fn projection_and_upp(&self, zat: f32, max: Vec2) -> (Mat4, f32) {
		let (znear, zfar) = self.clip_planes(zat);
		if self.opm {
			let (max, upp) = self.fov.max_and_upp(zat, max);
			let mat = opengl::orthographic(-max.x, max.x, -max.y, max.y, znear, zfar);
			(mat, upp)
		} else {
			let fov = self.fov.to_ver(max).into_inner();
			let (max, upp) = self.fov.max_and_upp(zat, max);
			let mat = opengl::perspective(fov, max.x / max.y, znear, zfar);
			(mat, upp)
		}
	}
}
