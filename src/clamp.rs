use crate::{Delta, Frame, Plane, Scope};
use core::fmt::Debug;
use nalgebra::{Point3, RealField, UnitQuaternion};

/// Clamp wrt abstract boundary conditions of [`Frame`] and [`Scope`].
///
/// Specific boundary conditions are defined by trait implementations (e.g., [`Bound`]).
///
/// Exceeding a boundary condition is communicated by specifying an exceeded plane. If the plane is
/// orthogonal to [`Delta`], it is completely stopped. If not, it glides along the plane. In this
/// case, the direction of [`Delta`] is changed by projecting the exceeded position onto the
/// boundary plane and finding the [`Delta`] from initial to projected position. This projected
/// [`Delta`] is repeatedly revalidated wrt boundary conditions until no new boundary plane is
/// exceeded. For orthogonal boundary conditions (e.g., a box), revalidation usually passes after
/// one, two, or three loops whenever zero, one, or two boundary conditions intersect (i.e., face,
/// edge, or corner).
///
/// [`Bound`]: crate::Bound
pub trait Clamp<N: Copy + RealField>: Send + Sync + Debug + 'static {
	/// Maximum loops due to maximum possible boundary plane intersections.
	///
	/// Measure to break out of validation loop as last resort. Default is `100`. Round boundary
	/// conditions require more loops whereas flat ones should stop with the 3rd validation
	/// (i.e., a corner) for each validated position (e.g., target, eye).
	#[must_use]
	fn loops(&self) -> usize {
		100
	}

	/// Exceeded boundary plane for target position in world space.
	///
	/// Must return `None` if target position satisfies all boundary conditions.
	#[must_use]
	fn target(&self, frame: &Frame<N>) -> Option<Plane<N>>;
	/// Exceeded boundary plane for eye position in world space.
	///
	/// Must return `None` if eye position satisfies all boundary conditions.
	#[must_use]
	fn eye(&self, frame: &Frame<N>) -> Option<Plane<N>>;
	/// Exceeded boundary plane for up position in world space.
	///
	/// Must return `None` if up position satisfies all boundary conditions.
	#[must_use]
	fn up(&self, frame: &Frame<N>) -> Option<Plane<N>>;

	/// Computes clamped [`Delta`] wrt abstract boundary conditions of [`Frame`] and [`Scope`].
	///
	/// Returns `None` if [`Delta`] satisfies all boundary conditions.
	#[allow(clippy::too_many_lines)]
	#[must_use]
	fn compute(
		&self,
		frame: &Frame<N>,
		scope: &Scope<N>,
		delta: &Delta<N>,
	) -> Option<(Delta<N>, usize)> {
		match delta {
			Delta::Frame => None,
			&Delta::First {
				pitch: _,
				yaw: _,
				yaw_axis,
			} => {
				let eye = frame.eye();
				let distance = frame.distance();
				let pitch_axis = frame.pitch_axis();
				// Old target position in eye space.
				let old_target = frame.target() - eye;
				let mut min_delta = *delta;
				let mut loops = 0;
				loop {
					let frame = min_delta.transform(frame);
					let mut bound = false;
					if let Some(plane) = self.target(&frame) {
						bound = true;
						// Center of spherical cap in world space.
						let center = plane.project_point(&eye);
						// Height of spherical cap.
						let height = distance - (center - eye).norm();
						// Radius of spherical cap.
						let radius = (height * (distance * (N::one() + N::one()) - height)).sqrt();
						// New clamped target position in spherical cap space.
						let new_target = (plane.project_point(frame.target()) - center)
							.normalize()
							.scale(radius);
						// New clamped target position in world space.
						let new_target = center + new_target;
						// New clamped target position in eye space.
						let new_target = new_target - eye;

						// Extract new signed pitch.
						let pitch_plane = Plane::with_point(pitch_axis, &eye);
						let old_pitch_target = pitch_plane.project_vector(&old_target);
						let new_pitch_target = pitch_plane.project_vector(&new_target);
						let pitch = pitch_plane.angle_between(&old_pitch_target, &new_pitch_target);
						// Apply signed pitch to old target.
						let pitch_rot = UnitQuaternion::from_axis_angle(&pitch_axis, pitch);
						let old_target = pitch_rot * old_target;
						// Extract left-over signed yaw.
						let yaw_plane = Plane::with_point(yaw_axis, &eye);
						let old_yaw_target = yaw_plane.project_vector(&old_target);
						let new_yaw_target = yaw_plane.project_vector(&new_target);
						let yaw = yaw_plane.angle_between(&old_yaw_target, &new_yaw_target);

						// FIXME It stutters and seems that roll attitude isn't preserved.
						#[allow(clippy::no_effect_underscore_binding)]
						let _min_delta = Delta::First {
							pitch,
							yaw,
							yaw_axis,
						};
						min_delta = Delta::Frame;
					}
					if bound {
						if loops == self.loops() {
							break;
						}
						loops += 1;
					} else {
						break;
					}
				}
				(min_delta != *delta).then_some((min_delta, loops))
			}
			Delta::Track { vec: _ } => {
				let old_frame = frame;
				let old_target = frame.target();
				let old_rot_inverse = frame.view().rotation.inverse();
				let mut min_delta = *delta;
				let mut loops = 0;
				loop {
					let frame = min_delta.transform(old_frame);
					let mut bound = false;
					if let Some(plane) = self.target(&frame) {
						bound = true;
						let new_target = plane.project_point(frame.target());
						let vec = old_rot_inverse * (new_target - old_target);
						min_delta = Delta::Track { vec };
					}
					let frame = min_delta.transform(old_frame);
					if let Some(_plane) = self.up(&frame) {
						bound = true;
						// TODO Implement gliding.
						min_delta = Delta::Frame;
					}
					if bound {
						if loops == self.loops() {
							break;
						}
						loops += 1;
					} else {
						break;
					}
				}
				(min_delta != *delta).then_some((min_delta, loops))
			}
			&Delta::Orbit { rot: _, pos } => {
				if pos != Point3::origin() {
					return Some((Delta::Frame, 0));
				}
				let old_frame = frame;
				let distance = frame.distance();
				let target = frame.target();
				// Rotation from world to camera space for eye in target space.
				let old_rot_inverse = frame.view().rotation.inverse();
				// Old eye position in camera space.
				let old_eye = old_rot_inverse * (frame.eye() - target);
				let mut min_delta = *delta;
				let mut loops = 0;
				loop {
					let mut bound = false;
					let frame = min_delta.transform(old_frame);
					if let Some(plane) = self.eye(&frame) {
						bound = true;
						// Center of spherical cap in world space.
						let center = plane.project_point(target);
						// Height of spherical cap.
						let height = distance - (center - target).norm();
						// Radius of spherical cap.
						let radius = (height * (distance * (N::one() + N::one()) - height)).sqrt();
						// New clamped eye position in spherical cap space.
						let new_eye = (plane.project_point(&frame.eye()) - center)
							.normalize()
							.scale(radius);
						// New clamped eye position in world space.
						let new_eye = center + new_eye;
						// New clamped eye position in camera space.
						let new_eye = old_rot_inverse * (new_eye - target);
						// New delta rotation in camera space.
						let rot = UnitQuaternion::rotation_between(&old_eye, &new_eye)
							.unwrap_or_default();
						min_delta = Delta::Orbit { rot, pos };
					}
					let frame = min_delta.transform(old_frame);
					if let Some(_plane) = self.up(&frame) {
						bound = true;
						// TODO Implement gliding.
						min_delta = Delta::Frame;
					}
					if bound {
						if loops == self.loops() {
							break;
						}
						loops += 1;
					} else {
						break;
					}
				}
				(min_delta != *delta).then_some((min_delta, loops))
			}
			Delta::Slide { vec: _ } => {
				let old_frame = frame;
				let old_target = frame.target();
				let old_rot_inverse = frame.view().rotation.inverse();
				let old_eye = frame.eye();
				let mut min_delta = *delta;
				let mut loops = 0;
				loop {
					let frame = min_delta.transform(old_frame);
					let mut bound = false;
					if let Some(plane) = self.target(&frame) {
						bound = true;
						let new_target = plane.project_point(frame.target());
						let vec = old_rot_inverse * (new_target - old_target);
						min_delta = Delta::Slide { vec };
					}
					let frame = min_delta.transform(old_frame);
					if let Some(plane) = self.eye(&frame) {
						bound = true;
						let new_eye = plane.project_point(&frame.eye());
						let vec = old_rot_inverse * (new_eye - old_eye);
						min_delta = Delta::Slide { vec };
					}
					if bound {
						if loops == self.loops() {
							break;
						}
						loops += 1;
					} else {
						break;
					}
				}
				(min_delta != *delta).then_some((min_delta, loops))
			}
			&Delta::Scale { mut rat, pos } => {
				let old_zat = frame.distance();
				let mut min_delta = *delta;
				let mut loops = 0;
				loop {
					let frame = min_delta.transform(frame);
					let mut bound = false;
					if let Some(_plane) = self.eye(&frame) {
						bound = true;
						// TODO Implement gliding.
						min_delta = Delta::Frame;
					}
					if scope.scale() {
						let (znear, _zfar) = scope.clip_planes(N::zero());
						let min_zat = -znear * (N::one() + N::default_epsilon().sqrt());
						let new_zat = old_zat * rat;
						if new_zat < min_zat {
							bound = true;
							// TODO Implement gliding.
							rat = min_zat / old_zat;
							min_delta = Delta::Scale { rat, pos };
						}
					}
					if bound {
						if loops == self.loops() {
							break;
						}
						loops += 1;
					} else {
						break;
					}
				}
				(min_delta != *delta).then_some((min_delta, loops))
			}
		}
	}
}
