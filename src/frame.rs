use nalgebra::{Isometry3, Point3, RealField, Unit, UnitQuaternion, Vector3};

/// Frame wrt camera eye and target.
#[derive(Debug, Clone, PartialEq, Eq)]
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
	pub fn look_at(target: Point3<N>, eye: &Point3<N>, up: &Vector3<N>) -> Self {
		let dir = target - eye;
		Self {
			pos: target,
			rot: UnitQuaternion::face_towards(&-dir, up),
			zat: dir.norm(),
		}
	}
	/// Eye position in world space.
	pub fn eye(&self) -> Point3<N> {
		self.pos + self.rot * Vector3::z_axis().into_inner() * self.zat
	}
	/// Sets eye position inclusive its roll attitude in world space preserving target position.
	pub fn set_eye(&mut self, eye: &Point3<N>, up: &Vector3<N>) {
		*self = Self::look_at(self.pos, eye, up);
	}
	/// Target position in world space.
	pub fn target(&self) -> &Point3<N> {
		&self.pos
	}
	/// Sets target position in world space preserving eye position inclusive its roll attitude.
	pub fn set_target(&mut self, target: Point3<N>) {
		let eye = self.eye();
		self.pos = target;
		self.zat = (self.pos - eye).norm();
	}
	/// Distance between eye and target.
	pub fn distance(&self) -> N {
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
		let pitch = UnitQuaternion::from_axis_angle(&self.local_pitch_axis(), pitch);
		let yaw = UnitQuaternion::from_axis_angle(yaw_axis, yaw);
		self.local_orbit_around(&pitch, &Point3::new(N::zero(), N::zero(), self.zat));
		self.orbit_around(&yaw, &self.eye());
	}
	/// Positive x-axis in camera space pointing from left to right.
	pub fn local_pitch_axis(&self) -> Unit<Vector3<N>> {
		Vector3::x_axis()
	}
	/// Positive y-axis in camera space pointing from bottom to top.
	pub fn local_yaw_axis(&self) -> Unit<Vector3<N>> {
		Vector3::y_axis()
	}
	/// Positive z-axis in camera space pointing from back to front.
	pub fn local_roll_axis(&self) -> Unit<Vector3<N>> {
		Vector3::z_axis()
	}
	/// Positive x-axis in world space pointing from left to right.
	pub fn pitch_axis(&self) -> Unit<Vector3<N>> {
		self.rot * self.local_pitch_axis()
	}
	/// Positive y-axis in world space pointing from bottom to top.
	pub fn yaw_axis(&self) -> Unit<Vector3<N>> {
		self.rot * self.local_yaw_axis()
	}
	/// Positive z-axis in world space pointing from back to front.
	pub fn roll_axis(&self) -> Unit<Vector3<N>> {
		self.rot * self.local_roll_axis()
	}
	/// Eye attitude via intrinsic `(pitch, yaw, roll)` angles, see [`Self::set_angles()`].
	pub fn angles(&self) -> (N, N, N) {
		self.rot.euler_angles()
	}
	/// Sets eye attitude via intrinsic angles.
	///
	/// Eye rotation occurs in the sense prescribed by the right-hand rule in the following order:
	///
	///  1. roll about [`Self::roll_axis()`] within ±π,
	///  2. pitch about rolled [`Self::pitch_axis()`] within ±π,
	///  3. yaw about rolled and pitched [`Self::yaw_axis()`] within ±π/2.
	pub fn set_angles(&mut self, pitch: N, yaw: N, roll: N) {
		self.rot = UnitQuaternion::from_euler_angles(pitch, yaw, roll);
	}
	/// View transformation from world to camera space.
	pub fn view(&self) -> Isometry3<N> {
		// Eye rotation from world to camera space around target.
		let rot = self.rot.inverse();
		// Eye position in camera space with origin in world space.
		let eye = rot * self.pos + Vector3::z_axis().into_inner() * self.zat;
		// Translate in such a way that the eye position with origin in world space vanishes.
		Isometry3::from_parts((-eye.coords).into(), rot)
	}
}
