use nalgebra::{RealField, convert};
use simba::scalar::SubsetOf;

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
	#[must_use]
	pub fn compute(&self, num: N) -> N {
		N::one() - num / self.den
	}
	/// Denominator. Default is scroll unit of `120.0`.
	#[must_use]
	pub const fn denominator(&self) -> N {
		self.den
	}
	/// Sets denominator. Default is scroll unit of `120.0`.
	pub const fn set_denominator(&mut self, den: N) {
		self.den = den;
	}
	/// Casts components to another type, e.g., between [`f32`] and [`f64`].
	#[must_use]
	pub fn cast<M: Copy + RealField>(self) -> Scale<M>
	where
		N: SubsetOf<M>,
	{
		Scale {
			den: self.den.to_superset(),
		}
	}
}
