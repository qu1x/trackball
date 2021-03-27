use nalgebra::{convert, Matrix4, Orthographic3, Perspective3, Point2, RealField};

/// Scene wrt enclosing viewing frustum.
///
/// Implements [`Default`] and can be created with `Scene::default()`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Scene<N: RealField> {
	/// Field of view y-axis.
	///
	/// Angle in yz-plane. Default is [`RealField::frac_pi_4()`].
	fov: N,
	/// Clip plane distances.
	///
	/// Near and far clip plane distances from either target or eye whether [`Self::oim`]. Defaults
	/// to `(1e-1, 1e+6)` measured from eye.
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

impl<N: RealField> Default for Scene<N> {
	fn default() -> Self {
		Self {
			fov: N::frac_pi_4(),
			zcp: (convert(1e-1), convert(1e+6)),
			oim: false,
			opm: false,
		}
	}
}

impl<N: RealField> Scene<N> {
	/// Field of view y-axis.
	///
	/// Angle in yz-plane. Default is [`RealField::frac_pi_4()`].
	pub fn fov(&self) -> N {
		self.fov
	}
	/// Sets field of view y-axis.
	///
	/// Angle in yz-plane. Default is [`RealField::frac_pi_4()`].
	pub fn set_fov(&mut self, fov: N) {
		self.fov = fov;
	}
	/// Clip plane distances from eye regardless of [`Self::scale()`].
	///
	/// Defaults to `(1e-1, 1e+6)` measured from eye.
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
	/// Defaults to `(1e-1, 1e+6)` measured from eye.
	pub fn set_clip_planes(&mut self, znear: N, zfar: N) {
		self.zcp = (znear, zfar);
	}
	/// Object inspection mode.
	///
	/// Scales clip plane distances by measuring from target instead of eye. Default is `false`.
	pub fn scale(&self) -> bool {
		self.oim
	}
	/// Sets object inspection mode.
	///
	/// Scales clip plane distances by measuring from target instead of eye. Default is `false`.
	pub fn set_scale(&mut self, oim: bool) {
		self.oim = oim
	}
	/// Orthographic projection mode.
	///
	/// Computes scale-identical orthographic instead of perspective projection. Default is `false`.
	pub fn ortho(&self) -> bool {
		self.opm
	}
	/// Sets orthographic projection mode.
	///
	/// Computes scale-identical orthographic instead of perspective projection. Default is `false`.
	pub fn set_ortho(&mut self, opm: bool) {
		self.opm = opm
	}
	/// Projection transformation and unit per pixel on focus plane.
	pub fn projection(&self, zat: N, max: &Point2<N>) -> (Matrix4<N>, N) {
		let (znear, zfar) = self.clip_planes(zat);
		let aspect = max.x / max.y;
		let two = N::one() + N::one();
		let top = zat * (self.fov / two).tan();
		let right = aspect * top;
		let upp = right * two / max.x;
		let mat = if self.opm {
			Orthographic3::new(-right, right, -top, top, znear, zfar).into_inner()
		} else {
			Perspective3::new(aspect, self.fov, znear, zfar).into_inner()
		};
		(mat, upp)
	}
}
