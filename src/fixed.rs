use nalgebra::{Point2, RealField};
use simba::scalar::SubsetOf;

/// Fixed quantity wrt field of view.
///
///   * Implements [`Default`] and can be created with `Fixed::default()` returning
///     `Fixed::Ver(N::frac_pi_4())`.
///   * Implements `From<N>` and can be created with `N::into()` returning `Fixed::Ver()`.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
	feature = "rkyv",
	derive(rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)
)]
pub enum Fixed<N: Copy + RealField> {
	/// Fixed horizontal field of view aka Vert- scaling.
	Hor(N),
	/// Fixed vertical field of view aka Hor+ scaling.
	Ver(N),
	/// Fixed unit per pixel on focus plane at distance from eye of one aka Pixel-based scaling.
	Upp(N),
}

impl<N: Copy + RealField> Default for Fixed<N> {
	fn default() -> Self {
		N::frac_pi_4().into()
	}
}

impl<N: Copy + RealField> From<N> for Fixed<N> {
	fn from(fov: N) -> Self {
		Self::Ver(fov)
	}
}

impl<N: Copy + RealField> Fixed<N> {
	/// Converts to fixed horizontal field of view wrt maximum position in screen space.
	#[must_use]
	pub fn to_hor(self, max: &Point2<N>) -> Self {
		let two = N::one() + N::one();
		Self::Hor(match self {
			Self::Hor(fov) => fov,
			Self::Ver(fov) => (max.x / max.y * (fov / two).tan()).atan() * two,
			Self::Upp(upp) => (max.x / two * upp).atan() * two,
		})
	}
	/// Converts to fixed vertical field of view wrt maximum position in screen space.
	#[must_use]
	pub fn to_ver(self, max: &Point2<N>) -> Self {
		let two = N::one() + N::one();
		Self::Ver(match self {
			Self::Hor(fov) => (max.y / max.x * (fov / two).tan()).atan() * two,
			Self::Ver(fov) => fov,
			Self::Upp(upp) => (max.y / two * upp).atan() * two,
		})
	}
	/// Converts to fixed unit per pixel on focus plane at distance from eye of one wrt maximum
	/// position in screen space.
	#[must_use]
	pub fn to_upp(self, max: &Point2<N>) -> Self {
		let two = N::one() + N::one();
		Self::Upp(match self {
			Self::Hor(fov) => (fov / two).tan() * two / max.x,
			Self::Ver(fov) => (fov / two).tan() * two / max.y,
			Self::Upp(upp) => upp,
		})
	}
	/// Maximum position in camera space and unit per pixel on focus plane wrt distance between
	/// eye and target and maximum position in screen space.
	#[must_use]
	pub fn max_and_upp(&self, zat: N, max: &Point2<N>) -> (Point2<N>, N) {
		let two = N::one() + N::one();
		match *self {
			Self::Hor(fov) => {
				let x = zat * (fov / two).tan();
				let y = max.y / max.x * x;
				(Point2::new(x, y), x * two / max.x)
			}
			Self::Ver(fov) => {
				let y = zat * (fov / two).tan();
				let x = max.x / max.y * y;
				(Point2::new(x, y), y * two / max.y)
			}
			Self::Upp(upp) => {
				let upp = upp * zat;
				(max / two * upp, upp)
			}
		}
	}
	/// Underlying quantity.
	pub const fn into_inner(self) -> N {
		match self {
			Self::Hor(fov) | Self::Ver(fov) => fov,
			Self::Upp(upp) => upp,
		}
	}
	/// Casts components to another type, e.g., between [`f32`] and [`f64`].
	pub fn cast<M: Copy + RealField>(self) -> Fixed<M>
	where
		N: SubsetOf<M>,
	{
		match self {
			Self::Hor(fov) => Fixed::Hor(fov.to_superset()),
			Self::Ver(fov) => Fixed::Ver(fov.to_superset()),
			Self::Upp(fov) => Fixed::Upp(fov.to_superset()),
		}
	}
}
