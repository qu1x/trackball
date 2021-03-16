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

use nalgebra::Point3;

/// Camera eye and target alignment.
#[derive(Debug, Clone, Copy)]
pub struct Align<N: RealField> {
	/// Target position in world space.
	pub pos: Point3<N>,
	/// Eye rotation at target from camera to world space.
	pub rot: UnitQuaternion<N>,
	/// Target distance from eye.
	pub zat: N,
}

impl<N: RealField> Align<N> {
	/// Sets eye inclusive its attitude and target point.
	pub fn look_at(eye: &Point3<N>, at: &Point3<N>, up: &Vector3<N>) -> Self {
		let dir = at - eye;
		Self {
			pos: at.clone(),
			rot: UnitQuaternion::look_at_rh(&dir, up),
			zat: dir.norm(),
		}
	}
	/// Eye position in world space.
	pub fn eye(&self) -> Point3<N> {
		self.pos + self.rot * Vector3::z_axis().into_inner() * self.zat
	}
	/// Sets eye point inclusive its attitude.
	///
	/// Clamps eye point between [`Self::min_dist()`] and [`Self::max_dist()`].
	pub fn set_eye(&mut self, eye: &Point3<N>, up: &Vector3<N>) {
		*self = Self::look_at(eye, &self.pos, up);
	}
	/// Target position in world space.
	pub fn at(&self) -> Point3<N> {
		self.pos
	}
	/// Sets target position in world space.
	pub fn set_at(&mut self, at: &Point3<N>) {
		self.pos = at.clone();
	}
	/// Slides camera eye and target by vector in world space.
	///
	/// Use following axes to slide in camera space:
	///
	///   * [`Self::roll_axis()`]
	///   * [`Self::pitch_axis()`]
	///   * [`Self::yaw_axis()`]
	pub fn slide(&mut self, vec: &Vector3<N>) {
		self.pos += vec;
	}
	/// Orbit eye at target by `rot` in world space.
	///
	/// Use following axes to orbit in camera space:
	///
	///   * [`Self::roll_axis()`]
	///   * [`Self::pitch_axis()`]
	///   * [`Self::yaw_axis()`]
	pub fn orbit(&mut self, rot: &UnitQuaternion<N>) {
		self.rot = rot * self.rot;
	}
	/// Orbits eye by `rot` at `pos` in world space, see [`Self::orbit()`].
	pub fn orbit_at(&mut self, rot: &UnitQuaternion<N>, pos: &Point3<N>) {
		self.orbit(rot);
		self.slide(&(pos - rot * pos));
	}
	/// Scales target distance from eye by `rat`.
	pub fn scale(&mut self, rat: N) {
		self.zat *= rat;
	}
	/// Scales target distance from eye by `rat` at `pos` in world space, see [`Self::scale()`].
	pub fn scale_at(&mut self, rat: N, pos: &Point3<N>) {
		self.scale(rat);
		self.slide(&(pos - pos * rat));
	}
	/// Negative z-axis in camera space pointing from front to back.
	pub fn roll_axis(&self) -> Unit<Vector3<N>> {
		self.rot * -Vector3::z_axis()
	}
	/// Positive x-axis in camera space pointing from left to right.
	pub fn pitch_axis(&self) -> Unit<Vector3<N>> {
		self.rot * Vector3::x_axis()
	}
	/// Negative y-axis in camera space pointing from top to bottom.
	pub fn yaw_axis(&self) -> Unit<Vector3<N>> {
		self.rot * -Vector3::y_axis()
	}
	/// Eye attitude via intrinsic roll, pitch, and yaw angles applied in the order mentioned.
	///
	/// Angles ranges are `(±PI, ±FRAC_PI_2, ±PI)` with `(0.0, 0.0, 0.0)` pointing along negative
	/// z-axis from front to back.
	pub fn angles(&self) -> (N, N, N) {
		self.rot.euler_angles()
	}
	/// Sets eye attitude via intrinsic roll, pitch, and yaw angles applied in the order mentioned.
	pub fn set_angles(&mut self, roll: N, pitch: N, yaw: N) {
		self.rot = UnitQuaternion::from_euler_angles(roll, pitch, yaw);
	}
	/// View transformation from world to camera space.
	pub fn view(&self) -> Isometry3<N> {
		// Eye rotation at target from world to camera space.
		let rot = self.rot.inverse();
		// Eye position in camera space with origin in world space.
		let eye = rot * self.pos + Vector3::z_axis().into_inner() * self.zat;
		// Translate in such a way that the eye position with origin in world coordinates vanishes.
		Isometry3::from_parts(eye.coords.neg().into(), rot)
	}
}

/// Camera projection.
///
/// Implements [`Default`] and can be created with `Frame::default()`.
#[derive(Debug, Clone, Copy)]
pub struct Frame<N: RealField> {
	/// Field of view y-axis in radians.
	///
	/// Angle in yz-plane. Default is [`RealField::frac_pi_4()`].
	pub fov: N,
	/// Clip plane distances.
	///
	/// Near and far clip plane distances from either target or eye whether [`Self::oim`]. Defaults
	/// to `(1e-1, 1e+6)` measured from eye.
	pub zcp: (N, N),
	/// Object inspection mode.
	///
	/// Scales clip plane distances by measuring from target instead of eye. Default is `false`.
	pub oim: bool,
	/// Orthographic projection mode.
	///
	/// Computes scale-identical orthographic instead of perspective projection. Default is `false`.
	pub opm: bool,
}

impl<N: RealField> Default for Frame<N> {
	fn default() -> Self {
		Self {
			fov: N::frac_pi_4(),
			zcp: (convert(1e-1), convert(1e+6)),
			oim: false,
			opm: false,
		}
	}
}

use nalgebra::Vector2;

use nalgebra::{Matrix4, Orthographic3, Perspective3};

#[derive(Debug, Clone, Copy)]
pub struct Image<N: RealField> {
	/// Current position in screen space of hovering input or pointing device.
	pub pos: Point2<N>,
	/// Maximum position in screen space as screen's width and height.
	pub max: Point2<N>,
	/// Unit per pixel on focus plane to scale slide operations and localize orbit/scale operations.
	pub upp: N,
	/// Cached view isometry from world to camera space coinciding with right-handed look-at space.
	pub view_iso: Isometry3<N>,
	/// Cached homogeneous view matrix computed from view isometry.
	pub view_mat: Matrix4<N>,
	/// Cached scale-identical orthographic or perspective projection matrix.
	pub proj_mat: Matrix4<N>,
	/// Cached transformation applying view and projection matrix.
	pub proj_view_mat: Matrix4<N>,
	/// Cached inverse of transformation for ray-tracing screen positions onto world objects.
	pub proj_view_inv: Matrix4<N>,
}

use nalgebra::{convert, zero, Isometry3};
use std::ops::Neg;

impl<N: RealField> Image<N> {
	pub fn new(align: &Align<N>, frame: &Frame<N>, max: Point2<N>) -> Self {
		let mut image = Self {
			pos: Point2::origin(),
			max,
			upp: zero(),
			view_iso: Isometry3::identity(),
			view_mat: zero(),
			proj_mat: zero(),
			proj_view_mat: zero(),
			proj_view_inv: zero(),
		};
		image.view(align);
		image.projection(align, frame);
		image.transformation();
		let _is_ok = image.inverse_transformation();
		image
	}
	/// Computes view matrix from camera eye and target alignment.
	///
	/// Camera eye and target alignment is defined with:
	///
	///   * `pos` as target position in world space,
	///   * `rot` as eye rotation at target from camera to world space,
	///   * `zat` as target distance from eye.
	pub fn view(&mut self, align: &Align<N>) {
		self.view_iso = align.view();
		self.view_mat = self.view_iso.to_homogeneous();
	}
	/// Computes projection matrix and unit per pixel on focus plane.
	///
	/// Extended frustum parameters are:
	///
	///   * `wh`: screen's width and height,
	///   * `fovy`: field of view yaw axis (y/up-axis), usually [`RealField::frac_pi_4()`],
	///   * `znear`: distance of near clip plane from camera eye,
	///   * `zat`: distance of focus plane from camera eye,
	///   * `zfar`: distance of far clip plane from camera eye,
	///   * `ortho`: scale preserving transition between orthographic and perspective projection.
	pub fn projection(&mut self, align: &Align<N>, frame: &Frame<N>) {
		let &Align { zat, .. } = align;
		let &Frame { fov, zcp, oim, opm } = frame;
		let (znear, zfar) = if oim {
			let (znear, zfar) = zcp;
			(zat - znear, zat + zfar)
		} else {
			zcp
		};
		let aspect = self.max.x / self.max.y;
		let two = N::one() + N::one();
		let top = zat * (fov / two).tan();
		let right = aspect * top;
		self.upp = right * two / self.max.x;
		self.proj_mat = if opm {
			Orthographic3::new(-right, right, -top, top, znear, zfar).into_inner()
		} else {
			Perspective3::new(aspect, fov, znear, zfar).into_inner()
		};
	}
	/// Computes transformation applying view and projection matrix.
	pub fn transformation(&mut self) {
		self.proj_view_mat = self.proj_mat * self.view_mat
	}
	/// Computes inverse of transformation for ray-tracing screen positions onto world objects.
	#[must_use = "return value is `true` on success"]
	pub fn inverse_transformation(&mut self) -> bool {
		self.proj_view_mat.try_inverse_mut()
	}
	/// Clamps position in screen space with respect to its maximum in screen space.
	pub fn clamp_pos_wrt_max(pos: &Point2<N>, max: &Point2<N>) -> Point2<N> {
		Point2::new(pos.x.clamp(N::zero(), max.x), pos.y.clamp(N::zero(), max.y))
	}
	/// Clamps position in screen space.
	pub fn clamp_pos(&self, pos: &Point2<N>) -> Point2<N> {
		Self::clamp_pos_wrt_max(pos, &self.max)
	}
	/// Transforms position from screen to camera space with respect to its maximum in screen space.
	pub fn transform_pos_wrt_max(pos: &Point2<N>, max: &Point2<N>) -> (Point2<N>, Point2<N>) {
		let max = max * convert(0.5);
		(Point2::new(pos.x - max.x, max.y - pos.y), max)
	}
	/// Transforms position from screen to camera space.
	pub fn transform_pos(&self, pos: &Point2<N>) -> Point2<N> {
		Self::transform_pos_wrt_max(pos, &self.max).0
	}
	/// Transforms vector from screen to camera space.
	pub fn transform_vec(pos: &Vector2<N>) -> Vector2<N> {
		Vector2::new(pos.x, -pos.y)
	}
	/// Transforms position from screen to camera space and projects it onto focus plane.
	pub fn project_pos(&self, pos: &Point2<N>) -> Point3<N> {
		self.transform_pos(pos).coords.scale(self.upp).push(N::zero()).into()
	}
	/// Transforms vector from screen to camera space and projects it onto focus plane.
	pub fn project_vec(&self, vec: &Vector2<N>) -> Vector3<N> {
		Self::transform_vec(vec).scale(self.upp).push(N::zero())
	}
}

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
		// Clamped cursor/finger position and its maximum from left to right and bottom to top.
		let (pos, max) = Image::transform_pos_wrt_max(&Image::clamp_pos_wrt_max(pos, &max), &max);
		// Positive z-axis pointing from far to near.
		let (pos, pza) = (pos.coords.push(N::zero()), Vector3::z_axis());
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

/// Scale operation handler.
///
/// Implements [`Default`] and can be created with `Orbit::default()`.
#[derive(Debug, Clone)]
pub struct Scale<N: RealField> {
	/// Caches previous cursor/finger position.
	pub scu: N,
}

impl<N: RealField> Default for Scale<N> {
	fn default() -> Self {
		Self {
			scu: convert(120.0),
		}
	}
}

impl<N: RealField> Scale<N> {
	pub fn compute(&self, delta: N) -> N {
		N::one() - delta / self.scu
	}
}

/// Slide operation handler.
///
/// Implements [`Default`] and can be created with `Orbit::default()`.
///
/// Both its methods must be invoked on matching events fired by your 3D graphics library of choice.
#[derive(Debug, Clone, Default)]
pub struct Slide<N: RealField> {
	/// Caches previous cursor/finger position.
	pub vec: Option<Point2<N>>,
}

impl<N: RealField> Slide<N> {
	/// Computes slide between previous and current cursor/finger position in screen space.
	pub fn compute(&mut self, pos: &Point2<N>) -> Option<Vector2<N>> {
		self.vec.replace(pos.clone()).map(|old| old - pos)
	}
	/// Discards cached previous cursor/finger position on button/finger release.
	pub fn discard(&mut self) {
		self.vec = None;
	}
}

use std::collections::BTreeMap;

/// Touch operation handler.
///
/// Implements [`Default`] and can be created with `Frame::default()`.
#[derive(Debug, Clone, Default)]
pub struct Touch<F: Ord, N: RealField> {
	/// Finger positions ordered by finger IDs.
	pub pos: BTreeMap<F, Point2<N>>,
	/// Centroid position and cached normalization of previous two-finger vector.
	pub vec: Option<(Unit<Vector2<N>>, N)>,
	/// Centroid position of potential finger tap gesture.
	pub tap: Option<(usize, Point2<N>)>,
	/// Number of total finger moves per potential finger tap gesture.
	pub mvs: usize,
}

impl<F: Ord, N: RealField> Touch<F, N> {
	/// Computes centroid position, roll angle, and scale ratio from finger gestures.
	pub fn compute(
		&mut self,
		fid: F,
		pos: Point2<N>,
		mvs: usize,
	) -> Option<(usize, Point2<N>, N, N)> {
		// Insert or update finger position.
		let _old_pos = self.pos.insert(fid, pos);
		// Current number of fingers.
		let num = self.pos.len();
		// Maximum number of fingers seen per potential tap.
		let max = self.tap.map_or(1, |(tap, _pos)| tap).max(num);
		// Centroid position.
		let pos = self
			.pos
			.values()
			.map(|pos| pos.coords)
			.sum::<Vector2<N>>()
			// TODO Is this still a generic integer to float cast? Way to avoid concrete type?
			.unscale(convert(num as f64))
			.into();
		// Cancel potential tap if more moves than number of finger starts plus optional number of
		// moves per finger for debouncing tap gesture. Debouncing would delay non-tap gestures.
		if self.mvs >= max + mvs * max {
			// Make sure to not resume cancelled tap when fingers are discarded.
			self.mvs = std::usize::MAX;
			// Cancel potential tap.
			self.tap = None;
		} else {
			// Count total moves per potential tap.
			self.mvs += 1;
			// Insert or update potential tap as long as fingers are not discarded.
			if num >= max {
				self.tap = Some((num, pos));
			}
		}
		// Inhibit finger gestures for given number of moves per finger. No delay with zero `mvs`.
		if self.mvs >= mvs * max {
			// Identity roll angle and scale ratio.
			let (rot, rat) = (N::zero(), N::one());
			// Roll and scale only with two-finger gesture, otherwise orbit or slide via centroid.
			if num == 2 {
				// Position of first and second finger.
				let mut val = self.pos.values();
				let one_pos = val.next().unwrap();
				let two_pos = val.next().unwrap();
				// Ray and its length pointing from first to second finger.
				let (new_ray, new_len) = Unit::new_and_get(two_pos - one_pos);
				// Get old and replace with new vector.
				if let Some((old_ray, old_len)) = self.vec.replace((new_ray, new_len)) {
					// Roll angle in opposite direction at centroid.
					let rot = new_ray.perp(&old_ray).atan2(new_ray.dot(&old_ray));
					// Scale ratio at centroid.
					let rat = old_len / new_len;
					// Induced two-finger slide, roll, and scale.
					Some((num, pos, rot, rat))
				} else {
					// Start position of slide.
					Some((num, pos, rot, rat))
				}
			} else {
				// Induced one-finger or more than two-finger orbit or slide.
				Some((num, pos, rot, rat))
			}
		} else {
			// Gesture inhibited.
			None
		}
	}
	/// Removes finger position and returns centroid position of finger tap gesture.
	pub fn discard(&mut self, fid: F) -> Option<(usize, Point2<N>)> {
		self.pos.remove(&fid).expect("Unknown touch ID");
		self.vec = None;
		if self.pos.is_empty() {
			self.mvs = 0;
			self.tap.take()
		} else {
			None
		}
	}
}
