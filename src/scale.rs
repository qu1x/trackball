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
	#[must_use]
	pub fn compute(&self, num: f32) -> f32 {
		1.0 - num / self.den
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
