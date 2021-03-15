//! Virtual Trackball Orbiting via the Exponential Map
//!
//! This is an alternative trackball technique using exponential map and parallel transport to
//! preserve distances and angles for inducing coherent and intuitive trackball rotations. For
//! instance, displacements on straight radial lines through the screen's center are carried to arcs
//! of the same length on great circles of the trackball. This is in contrast to state-of-the-art
//! techniques using orthogonal projection which distorts radial distances further away from the
//! screen's center. This implementation strictly follows the recipe given in the paper of
//! Stantchev, G.. “Virtual Trackball Modeling and the Exponential Map.” . [S2CID] [44199608].
//!
//! [S2CID]: https://en.wikipedia.org/wiki/S2CID_(identifier)
//! [44199608]: https://api.semanticscholar.org/CorpusID:44199608
//!
//! # Status
//!
//! Currently only [`nalgebra`] is supported as underlying linear algebra library but others will be
//! supported behind feature gates so that only your library of choice becomes a dependency. The
//! [`Orbit`] operation handler will be complemented with other handlers for common trackball camera
//! mode operations like slide, scale, and focus. Projection view matrices will be computed as well
//! with scale preserving transitions between orthographic and perspective projection mode.
//!
//! # Example
//!
//! A trackball camera mode implementation can be as easy as this by delegating events of your 3D
//! graphics library of choice to the [`Orbit`] operation handler along with other handlers for
//! common trackball camera mode operations like slide, scale, and focus.
//!
//! ```
//! use nalgebra::{Point2, UnitQuaternion, Vector3};
//! use std::f32::consts::PI;
//! use trackball::Orbit;
//!
//! /// Trackball camera mode.
//! pub struct Trackball {
//! 	// Camera eye alignment.
//! 	align: UnitQuaternion<f32>,
//! 	// Orbit operation handler along with slide, scale, and focus operation handlers.
//! 	orbit: Orbit<f32>,
//! 	// Maximum cursor/finger position as screen's width and height.
//! 	frame: Point2<f32>,
//! }
//!
//! impl Trackball {
//! 	// Usually, a cursor position event with left mouse button being pressed.
//! 	fn handle_left_button_displacement(&mut self, pos: &Point2<f32>) {
//! 		// Optionally, do a coordinate system transformation like flipping x-axis/z-axis.
//! 		let camera_space = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), PI);
//! 		// Or directly apply this induced rotation.
//! 		let rotation = self.orbit.compute(&pos, &self.frame).unwrap_or_default();
//! 		// Post-multiply rotation to total camera alignment.
//! 		self.align *= camera_space * rotation * camera_space.inverse();
//! 	}
//! 	// Event when left mouse button is released again.
//! 	fn handle_left_button_release(&mut self) {
//! 		// Can also or instead be invoked on `Self::handle_left_button_press()`.
//! 		self.orbit.discard();
//! 	}
//! }
//! ```

use nalgebra::{Point2, RealField, Unit, UnitQuaternion, Vector3};

/// Orbit operation handler.
///
/// Implements [`Default`] and can be created with `Orbit::default()`.
///
/// Both its methods must be invoked on matching events fired by your 3D graphics library of choice.
#[derive(Debug, Clone, Default)]
pub struct Orbit<N: RealField> {
	/// Caches normalization of previous cursor/finger position.
	pub vec: Option<(Unit<Vector3<N>>, N)>,
}

#[cfg(not(feature = "cc"))]
use nalgebra::Matrix3;

#[cfg(not(feature = "cc"))]
impl<N: RealField> Orbit<N> {
	/// Computes rotation between previous and current cursor/finger position.
	///
	/// Normalization of previous position is cached and has to be discarded on button/finger
	/// release via [`Self::discard()`]. Current position `pos` is clamped between origin and
	/// maximum position `max` as screen's width and height.
	///
	/// Screen coordinate system with origin in top left corner:
	///
	///   * x-axis from left to right,
	///   * y-axis from top to bottom.
	///
	/// Trackball coordinate system with origin in trackball's center:
	///
	///   * x-axis from left to right,
	///   * y-axis from bottom to top,
	///   * z-axis from far to near.
	///
	/// Returns `None`:
	///
	///   * on first invocation and after [`Self::discard()`] as there is no previous position yet,
	///   * in the unlikely case that a position event fires twice resulting in zero displacements.
	pub fn compute(&mut self, pos: &Point2<N>, max: &Point2<N>) -> Option<UnitQuaternion<N>> {
		// Clamp position between screen's left top and right bottom corner.
		let pos = Point2::new(pos.x.clamp(N::zero(), max.x), pos.y.clamp(N::zero(), max.y));
		// Maximum centered cursor/finger position as half the screen's width and height.
		let max = max / (N::one() + N::one());
		// Current centered cursor/finger position from left to right and bottom to top.
		let pos = Vector3::new(pos.x - max.x, max.y - pos.y, N::zero());
		// Positive z-axis pointing from far to near.
		let pza = Vector3::z_axis();
		// New position as ray and length on xy-plane or z-axis of zero length for origin position.
		let (ray, len) = Unit::try_new_and_get(pos, N::zero()).unwrap_or((pza, N::zero()));
		// Get old ray and length as start position and offset and replace with new ray and length.
		let (pos, off) = self.vec.replace((ray, len))?;
		// Displacement vector from old to new ray and length.
		let vec = ray.into_inner() * len - pos.into_inner() * off;
		// Shadow new ray and length as normalized displacement vector.
		let (ray, len) = Unit::try_new_and_get(vec, N::zero())?;
		// Treat maximum of half the screen's width or height as trackball's radius.
		let max = max.x.max(max.y);
		// Map trackball's diameter onto half its circumference for start positions so that only
		// screen corners are mapped to lower hemisphere which induces less intuitive rotations.
		let (sin, cos) = (off / max * N::frac_pi_2()).sin_cos();
		// Exponential map of start position.
		let exp = Vector3::new(sin * pos.x, sin * pos.y, cos);
		// Tangent ray of geodesic at exponential map.
		let tan = Vector3::new(cos * pos.x, cos * pos.y, -sin);
		// Cross product of z-axis and start position to construct orthonormal frames.
		let zxp = Vector3::new(-pos.y, pos.x, N::zero());
		// Orthonormal frame as argument of differential of exponential map.
		let arg = Matrix3::from_columns(&[pza.into_inner(), pos.into_inner(), zxp]);
		// Orthonormal frame as image of differential of exponential map.
		let img = Matrix3::from_columns(&[exp, tan, zxp]);
		// Compute differential of exponential map by its argument and image and apply it to
		// displacement vector which in turn spans rotation plane together with exponential map.
		let vec = (img * arg.tr_mul(&ray.into_inner())).cross(&exp);
		// Angle of rotation is displacement length divided by radius.
		Unit::try_new(vec, N::zero()).map(|ray| UnitQuaternion::from_axis_angle(&ray, len / max))
	}
	/// Discards cached normalization of previous cursor/finger position on button/finger release.
	pub fn discard(&mut self) {
		self.vec = None;
	}
}

#[cfg(feature = "cc")]
use nalgebra::Quaternion;

#[cfg(feature = "cc")]
impl Orbit<f32> {
	/// Computes rotation between previous and current cursor/finger position.
	///
	/// Normalization of previous position is cached and has to be discarded on button/finger
	/// release via [`Self::discard()`]. Current position `pos` is clamped between origin and
	/// maximum position `max` as screen's width and height.
	///
	/// Screen coordinate system with origin in top left corner:
	///
	///   * x-axis from left to right,
	///   * y-axis from top to bottom.
	///
	/// Trackball coordinate system with origin in trackball's center:
	///
	///   * x-axis from left to right,
	///   * y-axis from bottom to top,
	///   * z-axis from far to near.
	///
	/// Returns `None`:
	///
	///   * on first invocation and after [`Self::discard()`] as there is no previous position yet,
	///   * in the unlikely case that a position event fires twice resulting in zero displacements.
	pub fn compute(&mut self, pos: &Point2<f32>, max: &Point2<f32>) -> Option<UnitQuaternion<f32>> {
		let mut rot = Quaternion::identity();
		let mut old = self
			.vec
			.map(|(ray, len)| ray.into_inner().push(len))
			.unwrap_or_default();
		unsafe {
			trackball_orbit_f(
				rot.as_vector_mut().as_mut_ptr(),
				old.as_mut_ptr(),
				pos.coords.as_ptr(),
				max.coords.as_ptr(),
			);
		}
		self.vec = Some((Unit::new_unchecked(old.xyz()), old.w));
		(rot.w != 1.0).then(|| UnitQuaternion::new_unchecked(rot))
	}
	/// Discards cached normalization of previous cursor/finger position on button/finger release.
	pub fn discard(&mut self) {
		self.vec = None;
	}
}

#[cfg(feature = "cc")]
impl Orbit<f64> {
	/// Computes rotation between previous and current cursor/finger position.
	///
	/// Normalization of previous position is cached and has to be discarded on button/finger
	/// release via [`Self::discard()`]. Current position `pos` is clamped between origin and
	/// maximum position `max` as screen's width and height.
	///
	/// Screen coordinate system with origin in top left corner:
	///
	///   * x-axis from left to right,
	///   * y-axis from top to bottom.
	///
	/// Trackball coordinate system with origin in trackball's center:
	///
	///   * x-axis from left to right,
	///   * y-axis from bottom to top,
	///   * z-axis from far to near.
	///
	/// Returns `None`:
	///
	///   * on first invocation and after [`Self::discard()`] as there is no previous position yet,
	///   * in the unlikely case that a position event fires twice resulting in zero displacements.
	pub fn compute(&mut self, pos: &Point2<f64>, max: &Point2<f64>) -> Option<UnitQuaternion<f64>> {
		let mut rot = Quaternion::identity();
		let mut old = self
			.vec
			.map(|(ray, len)| ray.into_inner().push(len))
			.unwrap_or_default();
		unsafe {
			trackball_orbit_d(
				rot.as_vector_mut().as_mut_ptr(),
				old.as_mut_ptr(),
				pos.coords.as_ptr(),
				max.coords.as_ptr(),
			);
		}
		self.vec = Some((Unit::new_unchecked(old.xyz()), old.w));
		(rot.w != 1.0).then(|| UnitQuaternion::new_unchecked(rot))
	}
	/// Discards cached normalization of previous cursor/finger position on button/finger release.
	pub fn discard(&mut self) {
		self.vec = None;
	}
}

#[cfg(feature = "cc")]
extern "C" {
	fn trackball_orbit_f(xyzw: *mut f32, xyzm: *mut f32, xy: *const f32, wh: *const f32);
	fn trackball_orbit_d(xyzw: *mut f64, xyzm: *mut f64, xy: *const f64, wh: *const f64);
}

/// Slide operation handler.
///
/// Implements [`Default`] and can be created with `Frame::default()`.
#[derive(Debug, Clone, Default)]
pub struct Slide<N: RealField> {
	/// Previous cursor/finger position scaled to focus plane.
	pub pos: Option<Point2<N>>,
}

impl<N: RealField> Slide<N> {
	/// Computes slide on focus plane from previous to current cursor/finger position.
	pub fn compute(&mut self, pos: &Point2<N>, sat: &Vector2<N>) -> Option<Vector3<N>> {
		let new = Point2::from(pos.coords.component_mul(sat));
		self.pos.replace(new).map(|old| (new - old).push(N::zero()))
	}
	/// Discards previous cursor/finger position scaled to focus plane on button/finger release.
	pub fn discard(&mut self) {
		self.pos = None;
	}
	/// Effective slide on focus plane to localize orbit operation.
	pub fn orbit_at(rot: UnitQuaternion<N>, pos: &Point2<N>, sat: &Vector2<N>) -> Vector3<N> {
		let pos = pos.coords.component_mul(sat).push(N::zero());
		pos - rot.transform_vector(&pos)
	}
	/// Effective slide on focus plane to localize scale operation.
	pub fn scale_at(rat: N, pos: &Point2<N>, sat: &Vector2<N>) -> Vector3<N> {
		pos.coords.component_mul(sat).push(N::zero()) * (N::one() - rat)
	}
}

use nalgebra::{Matrix4, Orthographic3, Perspective3, Vector2};

/// Frame operation handler.
///
/// Implements [`Default`] and can be created with `Frame::default()`.
#[derive(Debug, Clone)]
pub struct Frame<N: RealField> {
	/// Scale identical orthographic or perspective projection matrix.
	pub mat: Matrix4<N>,
	/// Maximum position on focus plane as frame's width and height.
	pub max: Point2<N>,
}

impl<N: RealField> Default for Frame<N> {
	fn default() -> Self {
		Self {
			mat: Matrix4::zeros(),
			max: Point2::origin(),
		}
	}
}

impl<N: RealField> Frame<N> {
	/// Computes projection matrix and maximum position on focus plane as frame's width and height.
	///
	/// Extended frustrum parameters are:
	///
	///   * `aspect`: screen's width divided by its height,
	///   * `fovy`: field of view yaw axis (y/up-axis), usually [`RealField::frac_pi_4()`],
	///   * `znear`: distance of near clip plane from camera eye,
	///   * `zat`: distance of focus plane from camera eye,
	///   * `zfar`: distance of far clip plane from camera eye,
	///   * `ortho`: scale preserving transition between orthographic and perspective projection.
	pub fn compute(&mut self, aspect: N, fovy: N, znear: N, zat: N, zfar: N, ortho: bool) {
		let two = N::one() + N::one();
		let top = zat * (fovy / two).tan();
		let right = aspect * top;
		self.max = Point2::new(right, top) * two;
		self.mat = if ortho {
			Orthographic3::new(-right, right, -top, top, znear, zfar).into_inner()
		} else {
			Perspective3::new(aspect, fovy, znear, zfar).into_inner()
		};
	}
	/// Screen to focus plane scale.
	pub fn scale(&self, max: &Point2<N>) -> Vector2<N> {
		self.max.coords.component_div(&max.coords)
	}
}

use nalgebra::center;
use std::collections::BTreeMap;

/// Touch operation handler.
///
/// Implements [`Default`] and can be created with `Frame::default()`.
#[derive(Debug, Clone, Default)]
pub struct Touch<I: Ord, N: RealField> {
	/// Position by finger ID.
	pub pos: BTreeMap<I, Point2<N>>,
	/// Cached normalization of previous two-finger vector.
	pub vec: Option<(Unit<Vector3<N>>, N)>,
}

impl<I: Ord, N: RealField> Touch<I, N> {
	/// Computes roll, zoom, and centroid of two fingers for orbit, scale, and slide operations.
	pub fn compute(&mut self, id: I, pos: Point2<N>) -> Option<(UnitQuaternion<N>, N, Point2<N>)> {
		// Insert or update finger position.
		let _old_pos = self.pos.insert(id, pos);
		// Wait for two fingers.
		(self.pos.len() == 2).then(|| ())?;
		// Position of first and second finger.
		let pos_one = &self.pos[self.pos.keys().min().unwrap()];
		let pos_two = &self.pos[self.pos.keys().max().unwrap()];
		// Ray and its length pointing from first to second finger.
		let (new_ray, new_len) = Unit::new_and_get((pos_two - pos_one).push(N::zero()));
		// Get old and replace with new vector.
		let (old_ray, old_len) = self.vec.replace((new_ray, new_len))?;
		// Camera roll in opposite direction at centroid.
		let roll = UnitQuaternion::rotation_between_axis(&new_ray, &old_ray).unwrap_or_default();
		// Camera zoom at centroid.
		Some((roll, old_len / new_len, center(pos_one, pos_two)))
	}
	/// Removes finger position and discards cached normalization of previous two-finger vector.
	pub fn discard(&mut self, id: I) {
		self.pos.remove(&id).expect("Unknown touch ID");
		self.vec = None;
	}
}
