use crate::float;

/// Scale induced by relative input.
///
/// Implements [`Default`] and can be created with `Scale::default()`.
#[derive(Debug, Clone, PartialEq)]
pub struct Scale {
	/// Denominator. Default is scroll unit of `120.0`.
	den: f32,
}

impl Default for Scale {
	fn default() -> Self {
		Self { den: 120.0 }
	}
}

impl Scale {
	/// Computes scale ratio from relative value.
	///
	/// The ratio is exponential in the relative value `num` over the denominator, that is
	/// `exp(-num / den)`, so that a relative value equal to the denominator scales by `1 / e`.
	/// Hence, scaling in and out by the same relative value restores the distance and the ratio of
	/// a relative value is the product of the ratios of its parts, regardless of how the input is
	/// split into events or frames. The ratio is always positive.
	#[must_use]
	pub fn compute(&self, num: f32) -> f32 {
		float::exp(-num / self.den)
	}
	/// Denominator. Default is scroll unit of `120.0`.
	#[must_use]
	pub const fn denominator(&self) -> f32 {
		self.den
	}
	/// Sets denominator. Default is scroll unit of `120.0`.
	pub const fn set_denominator(&mut self, den: f32) {
		self.den = den;
	}
}
