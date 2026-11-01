use core::f32::consts::FRAC_PI_4;
use glam::Vec2;
#[cfg_attr(test, allow(unused_imports))]
use num_traits::Float;

/// Fixed quantity wrt field of view.
///
///   * Implements [`Default`] and can be created with `Fixed::default()` returning
///     `Fixed::Ver(FRAC_PI_4)`.
///   * Implements `From<f32>` and can be created with `f32::into()` returning `Fixed::Ver()`.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
	feature = "rkyv",
	derive(rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)
)]
pub enum Fixed {
	/// Fixed horizontal field of view aka Vert- scaling.
	Hor(f32),
	/// Fixed vertical field of view aka Hor+ scaling.
	Ver(f32),
	/// Fixed unit per pixel on focus plane at distance from eye of one aka Pixel-based scaling.
	Upp(f32),
}

impl Default for Fixed {
	fn default() -> Self {
		FRAC_PI_4.into()
	}
}

impl From<f32> for Fixed {
	fn from(fov: f32) -> Self {
		Self::Ver(fov)
	}
}

impl Fixed {
	/// Converts to fixed horizontal field of view wrt maximum position in screen space.
	#[must_use]
	pub fn to_hor(self, max: Vec2) -> Self {
		let two = 2.0;
		Self::Hor(match self {
			Self::Hor(fov) => fov,
			Self::Ver(fov) => (max.x / max.y * (fov / two).tan()).atan() * two,
			Self::Upp(upp) => (max.x / two * upp).atan() * two,
		})
	}
	/// Converts to fixed vertical field of view wrt maximum position in screen space.
	#[must_use]
	pub fn to_ver(self, max: Vec2) -> Self {
		let two = 2.0;
		Self::Ver(match self {
			Self::Hor(fov) => (max.y / max.x * (fov / two).tan()).atan() * two,
			Self::Ver(fov) => fov,
			Self::Upp(upp) => (max.y / two * upp).atan() * two,
		})
	}
	/// Converts to fixed unit per pixel on focus plane at distance from eye of one wrt maximum
	/// position in screen space.
	#[must_use]
	pub fn to_upp(self, max: Vec2) -> Self {
		let two = 2.0;
		Self::Upp(match self {
			Self::Hor(fov) => (fov / two).tan() * two / max.x,
			Self::Ver(fov) => (fov / two).tan() * two / max.y,
			Self::Upp(upp) => upp,
		})
	}
	/// Maximum position in camera space and unit per pixel on focus plane wrt distance between
	/// eye and target and maximum position in screen space.
	#[must_use]
	pub fn max_and_upp(&self, zat: f32, max: Vec2) -> (Vec2, f32) {
		let two = 2.0;
		match *self {
			Self::Hor(fov) => {
				let x = zat * (fov / two).tan();
				let y = max.y / max.x * x;
				(Vec2::new(x, y), x * two / max.x)
			}
			Self::Ver(fov) => {
				let y = zat * (fov / two).tan();
				let x = max.x / max.y * y;
				(Vec2::new(x, y), y * two / max.y)
			}
			Self::Upp(upp) => {
				let upp = upp * zat;
				(max / two * upp, upp)
			}
		}
	}
	/// Underlying quantity.
	#[must_use]
	pub const fn into_inner(self) -> f32 {
		match self {
			Self::Hor(fov) | Self::Ver(fov) => fov,
			Self::Upp(upp) => upp,
		}
	}
}
