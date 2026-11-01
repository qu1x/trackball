//! Math backend dispatching to either `std` or `libm` float functions like `glam`.

#[cfg(any(feature = "libm", all(feature = "nostd-libm", not(feature = "std"))))]
mod libm_math {
	#[inline]
	pub fn sqrt(f: f32) -> f32 {
		libm::sqrtf(f)
	}

	#[inline]
	pub fn tan(f: f32) -> f32 {
		libm::tanf(f)
	}

	#[inline]
	pub fn atan(f: f32) -> f32 {
		libm::atanf(f)
	}

	#[inline]
	pub fn atan2(f: f32, other: f32) -> f32 {
		libm::atan2f(f, other)
	}

	#[inline]
	pub fn exp(f: f32) -> f32 {
		libm::expf(f)
	}

	#[cfg(not(feature = "cc"))]
	#[inline]
	pub fn sin_cos(f: f32) -> (f32, f32) {
		libm::sincosf(f)
	}

	#[inline]
	pub fn mul_add(f: f32, a: f32, b: f32) -> f32 {
		libm::fmaf(f, a, b)
	}
}

#[cfg(all(not(feature = "libm"), feature = "std"))]
mod std_math {
	#[inline]
	pub fn sqrt(f: f32) -> f32 {
		f32::sqrt(f)
	}

	#[inline]
	pub fn tan(f: f32) -> f32 {
		f32::tan(f)
	}

	#[inline]
	pub fn atan(f: f32) -> f32 {
		f32::atan(f)
	}

	#[inline]
	pub fn atan2(f: f32, other: f32) -> f32 {
		f32::atan2(f, other)
	}

	#[inline]
	pub fn exp(f: f32) -> f32 {
		f32::exp(f)
	}

	#[cfg(not(feature = "cc"))]
	#[inline]
	pub fn sin_cos(f: f32) -> (f32, f32) {
		f32::sin_cos(f)
	}

	#[inline]
	pub fn mul_add(f: f32, a: f32, b: f32) -> f32 {
		f32::mul_add(f, a, b)
	}
}

#[cfg(any(feature = "libm", all(feature = "nostd-libm", not(feature = "std"))))]
pub use libm_math::*;

#[cfg(all(not(feature = "libm"), feature = "std"))]
pub use std_math::*;
