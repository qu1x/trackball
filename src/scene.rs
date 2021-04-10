use nalgebra::{convert, Matrix4, Point2, RealField};

/// Scene wrt enclosing viewing frustum.
///
/// Implements [`Default`] and can be created with `Scene::default()`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Scene<N: RealField> {
	/// Fixed quantity wrt field of view.
	///
	/// Default is fixed vertical field of view of π/4.
	fov: Fixed<N>,
	/// Clip plane distances.
	///
	/// Near and far clip plane distances from either target or eye whether [`Self::oim`]. Default
	/// is `(1e-1, 1e+6)` measured from eye.
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
			fov: Fixed::default(),
			zcp: (convert(1e-1), convert(1e+6)),
			oim: false,
			opm: false,
		}
	}
}

impl<N: RealField> Scene<N> {
	/// Fixed quantity wrt field of view, see [`Self::set_fov()`].
	pub fn fov(&self) -> Fixed<N> {
		self.fov
	}
	/// Sets fixed quantity wrt field of view.
	///
	/// Default is fixed vertical field of view of π/4.
	///
	/// ```
	/// use nalgebra::Point2;
	/// use trackball::Scene;
	///
	/// // Default screen size.
	/// let max = Point2::new(800, 600);
	/// // Default scene with fixed vertical field of view of π/4:
	/// //
	/// //   * Increasing width increases horizontal field of view (more can be seen).
	/// //   * Increasing height scales scene zooming in as vertical field of view is fixed.
	/// let mut scene = Scene::default();
	/// // Unfix vertical field of view by fixing current unit per pixel on focus plane at distance
	/// // from eye of one, that is effectively `upp` divided by `zat` to make it scale-independant:
	/// //
	/// //   * Increasing width increases horizontal field of view (more can be seen).
	/// //   * Increasing height increases vertical field of view (more can be seen).
	/// scene.set_fov(scene.fov().to_upp(&max.cast::<f32>()));
	/// ```
	pub fn set_fov(&mut self, fov: impl Into<Fixed<N>>) {
		self.fov = fov.into();
	}
	/// Clip plane distances from eye regardless of [`Self::scale()`] wrt to distance between eye
	/// and target.
	///
	/// Default is `(1e-1, 1e+6)` measured from eye.
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
	/// Default is `(1e-1, 1e+6)` measured from eye.
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
	/// Projection transformation and unit per pixel on focus plane wrt distance between eye and
	/// target and maximum position in screen space.
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
}

/// Fixed quantity wrt field of view.
///
///   * Implements [`Default`] and can be created with `Fixed::default()` returning
///     `Fixed::Ver(N::frac_pi_4())`.
///   * Implements `From<N>` and can be created with `N::into()` returning `Fixed::Ver()`.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Fixed<N: RealField> {
	/// Fixed horizontal field of view aka Vert- scaling.
	Hor(N),
	/// Fixed vertical field of view aka Hor+ scaling.
	Ver(N),
	/// Fixed unit per pixel on focus plane at distance from eye of one aka Pixel-based scaling.
	Upp(N),
}

impl<N: RealField> Default for Fixed<N> {
	fn default() -> Self {
		N::frac_pi_4().into()
	}
}

impl<N: RealField> From<N> for Fixed<N> {
	fn from(fov: N) -> Self {
		Self::Ver(fov)
	}
}

impl<N: RealField> Fixed<N> {
	/// Converts to fixed horizontal field of view wrt maximum position in screen space.
	pub fn to_hor(&self, max: &Point2<N>) -> Self {
		let two = N::one() + N::one();
		Self::Hor(match self {
			&Self::Hor(fov) => fov,
			&Self::Ver(fov) => (max.x / max.y * (fov / two).tan()).atan() * two,
			&Self::Upp(upp) => (max.x / two * upp).atan() * two,
		})
	}
	/// Converts to fixed vertical field of view wrt maximum position in screen space.
	pub fn to_ver(&self, max: &Point2<N>) -> Self {
		let two = N::one() + N::one();
		Self::Ver(match self {
			&Self::Hor(fov) => (max.y / max.x * (fov / two).tan()).atan() * two,
			&Self::Ver(fov) => fov,
			&Self::Upp(upp) => (max.y / two * upp).atan() * two,
		})
	}
	/// Converts to fixed unit per pixel on focus plane at distance from eye of one wrt maximum
	/// position in screen space.
	pub fn to_upp(&self, max: &Point2<N>) -> Self {
		let two = N::one() + N::one();
		Self::Upp(match self {
			&Self::Hor(fov) => (fov / two).tan() * two / max.x,
			&Self::Ver(fov) => (fov / two).tan() * two / max.y,
			&Self::Upp(upp) => upp,
		})
	}
	/// Maximum position in camera space and unit per pixel on focus plane wrt distance between
	/// eye and target and maximum position in screen space.
	pub fn max_and_upp(&self, zat: N, max: &Point2<N>) -> (Point2<N>, N) {
		let two = N::one() + N::one();
		match self {
			&Self::Hor(fov) => {
				let x = zat * (fov / two).tan();
				let y = max.y / max.x * x;
				(Point2::new(x, y), x * two / max.x)
			}
			&Self::Ver(fov) => {
				let y = zat * (fov / two).tan();
				let x = max.x / max.y * y;
				(Point2::new(x, y), y * two / max.y)
			}
			&Self::Upp(upp) => {
				let upp = upp * zat;
				(max / two * upp, upp)
			}
		}
	}
	/// Underlying quantity.
	pub fn into_inner(self) -> N {
		match self {
			Self::Hor(fov) => fov,
			Self::Ver(fov) => fov,
			Self::Upp(upp) => upp,
		}
	}
}
