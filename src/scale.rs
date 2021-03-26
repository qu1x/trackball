use nalgebra::{convert, RealField};

/// Scale induced by relative input.
///
/// Implements [`Default`] and can be created with `Scale::default()`.
#[derive(Debug, Clone)]
pub struct Scale<N: RealField> {
	/// Denominator. Default is scroll unit of `120.0`.
	pub rel: N,
}

impl<N: RealField> Default for Scale<N> {
	fn default() -> Self {
		Self {
			rel: convert(120.0),
		}
	}
}

impl<N: RealField> Scale<N> {
	/// Computes scale ratio from relative value.
	pub fn compute(&self, val: N) -> N {
		N::one() - val / self.rel
	}
}
