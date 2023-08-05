use approx::{AbsDiffEq, RelativeEq, UlpsEq};
use nalgebra::{Isometry3, Point3, RealField, Reflection3, Unit, UnitQuaternion, Vector3};
use simba::scalar::SubsetOf;

/// Plane encoding position with singed bias along unit normal.
///
/// Realizes plane equation `a*x+b*y+c*z+d=0` with unit normal `[x, y, z]` and signed bias `d`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
	feature = "rkyv",
	derive(rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)
)]
#[repr(C)]
pub struct Plane<N: Copy + RealField> {
	/// Plane unit normal.
	pub normal: Unit<Vector3<N>>,
	/// Signed bias along unit normal.
	pub bias: N,
}

impl<N: Copy + RealField> Plane<N> {
	/// Plane from unit `normal` and signed `distance` from the origin.
	///
	/// ```
	/// use trackball::{
	///     nalgebra::{Point3, Vector3},
	///     Plane,
	/// };
	///
	/// // Plane intersecting y-axis at `5.0`.
	/// let plane = Plane::new(Vector3::y_axis(), 5.0);
	///
	/// // Origin projected onto plane where plane intersects y-axis.
	/// let point = plane.project_point(&Point3::origin());
	/// assert_eq!(point, Point3::new(0.0, 5.0, 0.0));
	///
	/// // Bias is negated distance.
	/// assert_eq!(plane.distance(), 5.0);
	/// assert_eq!(plane.bias, -5.0);
	///
	/// // Plane intersecting y-axis at `-5.0`.
	/// let plane = Plane::new(Vector3::y_axis(), -5.0);
	///
	/// // Origin projected onto plane where plane intersects y-axis.
	/// let point = plane.project_point(&Point3::origin());
	/// assert_eq!(point, Point3::new(0.0, -5.0, 0.0));
	///
	/// // Bias is negated distance.
	/// assert_eq!(plane.distance(), -5.0);
	/// assert_eq!(plane.bias, 5.0);
	/// ```
	pub fn new(normal: Unit<Vector3<N>>, distance: N) -> Self {
		Self {
			normal,
			bias: -distance,
		}
	}
	/// Plane from unit normal with point in plane.
	pub fn with_point(normal: Unit<Vector3<N>>, point: &Point3<N>) -> Self {
		Self::new(normal, normal.dot(&point.coords))
	}
	/// Signed orthogonal distance from the origin.
	pub fn distance(&self) -> N {
		-self.bias
	}
	/// Signed orthogonal distance from `point`.
	pub fn distance_from(&self, point: &Point3<N>) -> N {
		self.distance() - self.normal.dot(&point.coords)
	}
	/// Projects point onto plane.
	pub fn project_point(&self, point: &Point3<N>) -> Point3<N> {
		self.project_vector(&point.coords).into()
	}
	/// Projects axis onto plane.
	pub fn project_axis(&self, axis: &Unit<Vector3<N>>) -> Unit<Vector3<N>> {
		Unit::new_normalize(self.project_vector(&axis.into_inner()))
	}
	/// Projects vector onto plane.
	pub fn project_vector(&self, vector: &Vector3<N>) -> Vector3<N> {
		vector - self.normal.into_inner() * (self.normal.dot(vector) + self.bias)
	}
	/// Singed angle from `a` to `b` where both vectors are in the plane.
	pub fn angle_between(&self, a: &Vector3<N>, b: &Vector3<N>) -> N {
		let angle = a.angle(&b);
		let axis = a.cross(&b);
		if self.normal.dot(&axis).is_sign_negative() {
			-angle
		} else {
			angle
		}
	}
	/// Rotates plane.
	pub fn rotate_by(self, rot: &UnitQuaternion<N>) -> Self {
		Self {
			normal: Unit::new_unchecked(rot.transform_vector(&self.normal)),
			bias: self.bias,
		}
	}
	/// Translates plane.
	pub fn translate_by(self, vec: &Vector3<N>) -> Self {
		Self {
			normal: self.normal,
			bias: self.bias - self.normal.dot(vec),
		}
	}
	/// Transforms plane by direct isometry, i.e., rotation followed by translation.
	///
	/// ```
	/// use core::f64::{
	///     consts::FRAC_PI_2,
	///     EPSILON,
	/// };
	/// use trackball::{
	///     approx::AbsDiffEq,
	///     nalgebra::{Isometry3, Vector3},
	///     Plane,
	/// };
	///
	/// // Plane intersecting y-axis at `5.0`.
	/// let plane = Plane::new(Vector3::y_axis(), 5.0)
	///     .transform_by(&Isometry3::new(
	///         // Translation by after rotation.
	///         Vector3::new(-5.0, 0.0, 0.0),
	///         // Rotation by 90 degrees before translation.
	///         Vector3::new(0.0, 0.0, FRAC_PI_2),
	///     ));
	/// // Plane intersecting x-axis at `-10.0`.
	/// assert!(plane.abs_diff_eq(&Plane::new(-Vector3::x_axis(), 10.0), EPSILON));
	/// ```
	pub fn transform_by(self, iso: &Isometry3<N>) -> Self {
		self.rotate_by(&iso.rotation)
			.translate_by(&iso.translation.vector)
	}
	/// Casts components to another type, e.g., between [`f32`] and [`f64`].
	pub fn cast<M: Copy + RealField>(self) -> Plane<M>
	where
		N: SubsetOf<M>,
	{
		Plane {
			normal: self.normal.cast(),
			bias: self.bias.to_superset(),
		}
	}
}

impl<N: Copy + RealField> From<Reflection3<N>> for Plane<N> {
	fn from(reflection: Reflection3<N>) -> Self {
		Self::new(Unit::new_unchecked(*reflection.axis()), reflection.bias())
	}
}

impl<N: Copy + RealField> From<Plane<N>> for Reflection3<N> {
	fn from(plane: Plane<N>) -> Self {
		Self::new(plane.normal, plane.distance())
	}
}

impl<N: Copy + RealField + AbsDiffEq> AbsDiffEq for Plane<N>
where
	N::Epsilon: Copy,
{
	type Epsilon = N::Epsilon;

	fn default_epsilon() -> N::Epsilon {
		N::default_epsilon()
	}

	fn abs_diff_eq(&self, other: &Self, epsilon: N::Epsilon) -> bool {
		self.normal.abs_diff_eq(&other.normal, epsilon)
			&& self.bias.abs_diff_eq(&other.bias, epsilon)
	}
}

impl<N: Copy + RealField + RelativeEq> RelativeEq for Plane<N>
where
	N::Epsilon: Copy,
{
	fn default_max_relative() -> N::Epsilon {
		N::default_max_relative()
	}

	fn relative_eq(&self, other: &Self, epsilon: N::Epsilon, max_relative: N::Epsilon) -> bool {
		self.normal
			.relative_eq(&other.normal, epsilon, max_relative)
			&& self.bias.relative_eq(&other.bias, epsilon, max_relative)
	}
}

impl<N: Copy + RealField + UlpsEq> UlpsEq for Plane<N>
where
	N::Epsilon: Copy,
{
	fn default_max_ulps() -> u32 {
		N::default_max_ulps()
	}

	fn ulps_eq(&self, other: &Self, epsilon: N::Epsilon, max_ulps: u32) -> bool {
		self.normal.ulps_eq(&other.normal, epsilon, max_ulps)
			&& self.bias.ulps_eq(&other.bias, epsilon, max_ulps)
	}
}
