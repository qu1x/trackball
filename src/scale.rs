use nalgebra::{convert, RealField};

/// Scale induced by relative input.
///
/// Implements [`Default`] and can be created with `Scale::default()`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Scale<N: Copy + RealField> {
	/// Denominator. Default is scroll unit of `120.0`.
	den: N,
}

impl<N: Copy + RealField> Default for Scale<N> {
	fn default() -> Self {
		Self {
			den: convert(120.0),
		}
	}
}

impl<N: Copy + RealField> Scale<N> {
	/// Computes scale ratio from relative value.
	pub fn compute(&self, num: N) -> N {
		N::one() - num / self.den
	}
	/// Denominator. Default is scroll unit of `120.0`.
	pub const fn denominator(&self) -> N {
		self.den
	}
	/// Sets denominator. Default is scroll unit of `120.0`.
	pub fn set_denominator(&mut self, den: N) {
		self.den = den;
	}
}
