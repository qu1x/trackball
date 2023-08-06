use approx::{AbsDiffEq, RelativeEq, UlpsEq};
use nalgebra::{Isometry3, Point3, RealField, Unit, UnitQuaternion, Vector3};
use simba::scalar::SubsetOf;

/// Frame wrt camera eye and target.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
	feature = "rkyv",
	derive(rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)
)]
pub struct Frame<N: Copy + RealField> {
	/// Target position in world space.
	pos: Point3<N>,
	/// Eye rotation from camera to world space around target.
	rot: UnitQuaternion<N>,
	/// Target distance from eye.
	zat: N,
}

impl<N: Copy + RealField> Frame<N> {
	/// Sets eye position inclusive its roll attitude and target position in world space.
	#[must_use]
	pub fn look_at(target: Point3<N>, eye: &Point3<N>, up: &Vector3<N>) -> Self {
		let dir = target - eye;
		Self {
			pos: target,
			rot: UnitQuaternion::face_towards(&-dir, up),
			zat: dir.norm(),
		}
	}
	/// Eye position in world space.
	#[must_use]
	pub fn eye(&self) -> Point3<N> {
		self.pos + self.rot * Vector3::z_axis().into_inner() * self.zat
	}
	/// Sets eye position inclusive its roll attitude in world space preserving target position.
	pub fn set_eye(&mut self, eye: &Point3<N>, up: &Vector3<N>) {
		*self = Self::look_at(self.pos, eye, up);
	}
	/// Target position in world space.
	#[must_use]
	pub const fn target(&self) -> &Point3<N> {
		&self.pos
	}
	/// Sets target position in world space preserving eye position inclusive its roll attitude.
	pub fn set_target(&mut self, target: Point3<N>) {
		let eye = self.eye();
		self.pos = target;
		self.zat = (self.pos - eye).norm();
	}
	/// Distance between eye and target.
	#[must_use]
	pub const fn distance(&self) -> N {
		self.zat
	}
	/// Sets distance between eye and target preserving target position.
	pub fn set_distance(&mut self, zat: N) {
		self.zat = zat;
	}
	/// Scales distance between eye and target by ratio preserving target position.
	pub fn scale(&mut self, rat: N) {
		self.zat *= rat;
	}
	/// Scales distance between eye and point in camera space by ratio preserving target position.
	pub fn local_scale_around(&mut self, rat: N, pos: &Point3<N>) {
		self.local_slide(&(pos - pos * rat));
		self.scale(rat);
	}
	/// Scales distance between eye and point in world space by ratio preserving target position.
	pub fn scale_around(&mut self, rat: N, pos: &Point3<N>) {
		let pos = pos - self.pos.coords;
		self.slide(&(pos - pos * rat));
		self.scale(rat);
	}
	/// Slides camera eye and target by vector in camera space.
	pub fn local_slide(&mut self, vec: &Vector3<N>) {
		self.pos += self.rot * vec;
	}
	/// Slides camera eye and target by vector in world space.
	pub fn slide(&mut self, vec: &Vector3<N>) {
		self.pos += vec;
	}
	/// Orbits eye by rotation in camera space around target.
	pub fn local_orbit(&mut self, rot: &UnitQuaternion<N>) {
		self.rot *= rot;
	}
	/// Orbits eye by rotation in camera space around point in camera space.
	pub fn local_orbit_around(&mut self, rot: &UnitQuaternion<N>, pos: &Point3<N>) {
		self.local_slide(&(pos - rot * pos));
		self.local_orbit(rot);
	}
	/// Orbits eye by rotation in world space around target.
	pub fn orbit(&mut self, rot: &UnitQuaternion<N>) {
		self.rot = rot * self.rot;
	}
	/// Orbits eye by rotation in world space around point in world space.
	pub fn orbit_around(&mut self, rot: &UnitQuaternion<N>, pos: &Point3<N>) {
		let pos = pos - self.pos.coords;
		self.slide(&(pos - rot * pos));
		self.orbit(rot);
	}
	/// Orbits target around eye by pitch and yaw preserving roll attitude aka first person view.
	///
	/// Use fixed [`Self::yaw_axis()`] by capturing it when entering first person view.
	pub fn look_around(&mut self, pitch: N, yaw: N, yaw_axis: &Unit<Vector3<N>>) {
		let pitch = UnitQuaternion::from_axis_angle(&self.pitch_axis(), pitch);
		let yaw = UnitQuaternion::from_axis_angle(yaw_axis, yaw);
		self.orbit_around(&(yaw * pitch), &self.eye());
	}
	/// Positive x-axis in camera space pointing from left to right.
	#[allow(clippy::unused_self)]
	#[must_use]
	pub fn local_pitch_axis(&self) -> Unit<Vector3<N>> {
		Vector3::x_axis()
	}
	/// Positive y-axis in camera space pointing from bottom to top.
	#[allow(clippy::unused_self)]
	#[must_use]
	pub fn local_yaw_axis(&self) -> Unit<Vector3<N>> {
		Vector3::y_axis()
	}
	/// Positive z-axis in camera space pointing from back to front.
	#[allow(clippy::unused_self)]
	#[must_use]
	pub fn local_roll_axis(&self) -> Unit<Vector3<N>> {
		Vector3::z_axis()
	}
	/// Positive x-axis in world space pointing from left to right.
	#[must_use]
	pub fn pitch_axis(&self) -> Unit<Vector3<N>> {
		self.rot * self.local_pitch_axis()
	}
	/// Positive y-axis in world space pointing from bottom to top.
	#[must_use]
	pub fn yaw_axis(&self) -> Unit<Vector3<N>> {
		self.rot * self.local_yaw_axis()
	}
	/// Positive z-axis in world space pointing from back to front.
	#[must_use]
	pub fn roll_axis(&self) -> Unit<Vector3<N>> {
		self.rot * self.local_roll_axis()
	}
	/// Attempts to interpolate between two frames using linear interpolation for the translation
	/// part, and spherical linear interpolation for the rotation part.
	///
	/// Returns `None` if the angle between both rotations is 180 degrees (in which case the
	/// interpolation is not well-defined).
	///
	/// # Arguments
	///
	///   * `self`: The initial frame to interpolate from.
	///   * `other`: The final frame to interpolate toward.
	///   * `t`: The interpolation parameter between 0 and 1.
	///   * `epsilon`: The value below which the sinus of the angle separating both quaternion
	///     must be to return `None`.
	#[must_use]
	pub fn try_lerp_slerp(&self, other: &Self, t: N, epsilon: N) -> Option<Self> {
		Some(Self {
			pos: self.pos.lerp(&other.pos, t),
			rot: self.rot.try_slerp(&other.rot, t, epsilon)?,
			zat: self.zat * (N::one() - t) + other.zat * t,
		})
	}
	/// Renormalizes eye rotation and returns its norm.
	pub fn renormalize(&mut self) -> N {
		self.rot.renormalize()
	}
	/// View transformation from camera to world space.
	#[must_use]
	pub fn view(&self) -> Isometry3<N> {
		Isometry3::from_parts(
			// Eye position in world space with origin in camera space.
			self.eye().into(),
			// Eye rotation from camera to world space around target.
			self.rot,
		)
	}
	/// Inverse view transformation from world to camera space.
	///
	/// Uses less computations than [`Self::view()`]`.inverse()`.
	#[must_use]
	pub fn inverse_view(&self) -> Isometry3<N> {
		// Eye rotation from world to camera space around target.
		let rot = self.rot.inverse();
		// Eye position in camera space with origin in world space.
		let eye = rot * self.pos + Vector3::z_axis().into_inner() * self.zat;
		// Translate in such a way that the eye position with origin in world space vanishes.
		Isometry3::from_parts((-eye.coords).into(), rot)
	}
	/// Casts components to another type, e.g., between [`f32`] and [`f64`].
	#[must_use]
	pub fn cast<M: Copy + RealField>(self) -> Frame<M>
	where
		N: SubsetOf<M>,
	{
		Frame {
			pos: self.pos.cast(),
			rot: self.rot.cast(),
			zat: self.zat.to_superset(),
		}
	}
}

impl<N: Copy + RealField + AbsDiffEq> AbsDiffEq for Frame<N>
where
	N::Epsilon: Copy,
{
	type Epsilon = N::Epsilon;

	fn default_epsilon() -> N::Epsilon {
		N::default_epsilon()
	}

	fn abs_diff_eq(&self, other: &Self, epsilon: N::Epsilon) -> bool {
		self.pos.abs_diff_eq(&other.pos, epsilon)
			&& self.rot.abs_diff_eq(&other.rot, epsilon)
			&& self.zat.abs_diff_eq(&other.zat, epsilon)
	}
}

impl<N: Copy + RealField + RelativeEq> RelativeEq for Frame<N>
where
	N::Epsilon: Copy,
{
	fn default_max_relative() -> N::Epsilon {
		N::default_max_relative()
	}

	fn relative_eq(&self, other: &Self, epsilon: N::Epsilon, max_relative: N::Epsilon) -> bool {
		self.pos.relative_eq(&other.pos, epsilon, max_relative)
			&& self.rot.relative_eq(&other.rot, epsilon, max_relative)
			&& self.zat.relative_eq(&other.zat, epsilon, max_relative)
	}
}

impl<N: Copy + RealField + UlpsEq> UlpsEq for Frame<N>
where
	N::Epsilon: Copy,
{
	fn default_max_ulps() -> u32 {
		N::default_max_ulps()
	}

	fn ulps_eq(&self, other: &Self, epsilon: N::Epsilon, max_ulps: u32) -> bool {
		self.pos.ulps_eq(&other.pos, epsilon, max_ulps)
			&& self.rot.ulps_eq(&other.rot, epsilon, max_ulps)
			&& self.zat.ulps_eq(&other.zat, epsilon, max_ulps)
	}
}
