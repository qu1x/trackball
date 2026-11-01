use crate::{Delta, Frame, Plane, Scope, float};
use core::fmt::Debug;
use glam::{Quat, Vec3};

/// Clamp wrt abstract boundary conditions of [`Frame`] and [`Scope`].
///
/// Specific boundary conditions are defined by trait implementations (e.g., [`Bound`]).
///
/// Exceeding a boundary condition is communicated by specifying an exceeded plane in world space.
/// If the plane is orthogonal to [`Delta`], it is completely stopped. If not, it glides along the
/// plane. In this case, the direction of [`Delta`] is changed by projecting the exceeded position
/// onto the boundary plane and finding the [`Delta`] from initial to projected position. This
/// projected [`Delta`] is repeatedly revalidated wrt boundary conditions until no new boundary
/// plane is exceeded. For orthogonal boundary conditions (e.g., a box), revalidation usually
/// passes after one, two, or three loops whenever zero, one, or two boundary conditions intersect
/// (i.e., face, edge, or corner).
///
/// Positions moving on spheres (e.g., the eye orbiting around the target) are projected onto the
/// circle where the sphere intersects the boundary plane. The up axis is treated as position on
/// the unit sphere. [`Delta::Scale`] glides by keeping its ratio but moving its pivot such that
/// the exceeded position is projected onto the boundary plane, i.e., scaling turns partially into
/// sliding. It stops where boundary planes moving along with the frame (e.g., distance bounds)
/// are reached.
///
/// [`Bound`]: crate::Bound
pub trait Clamp: Send + Sync + Debug + 'static {
    /// Maximum loops due to maximum possible boundary plane intersections.
    ///
    /// Measure to break out of validation loop as last resort. Default is `100`. Round boundary
    /// conditions require more loops whereas flat ones should stop with the 3rd validation
    /// (i.e., a corner) for each validated position (e.g., target, eye). When reached, [`Delta`]
    /// stops at the boundary instead of gliding along it.
    #[must_use]
    fn loops(&self) -> usize {
        100
    }

    /// Exceeded boundary plane for target position in world space.
    ///
    /// Must return `None` if target position satisfies all boundary conditions.
    #[must_use]
    fn target(&self, frame: Frame) -> Option<Plane>;
    /// Exceeded boundary plane for eye position in world space.
    ///
    /// Must return `None` if eye position satisfies all boundary conditions.
    #[must_use]
    fn eye(&self, frame: Frame) -> Option<Plane>;
    /// Exceeded boundary plane for up position in world space.
    ///
    /// The up position is the up axis (i.e., [`Frame::yaw_axis()`]) as position on the unit
    /// sphere.
    ///
    /// Must return `None` if up position satisfies all boundary conditions.
    #[must_use]
    fn up(&self, frame: Frame) -> Option<Plane>;

    /// Computes clamped [`Delta`] wrt abstract boundary conditions of [`Frame`] and [`Scope`].
    ///
    /// Returns `None` if [`Delta`] satisfies all boundary conditions.
    #[allow(clippy::too_many_lines)]
    #[must_use]
    fn compute(&self, frame: Frame, scope: Scope, delta: Delta) -> Option<(Delta, usize)> {
        let mut min_delta = delta;
        let mut loops = 0;
        match delta {
            Delta::Frame => return None,
            Delta::First { yaw_axis, .. } => {
                // Target orbits around fixed eye.
                let eye = frame.eye();
                let distance = frame.distance();
                loop {
                    let new_frame = min_delta.transform(frame);
                    let new_target = if let Some(plane) = self.target(new_frame) {
                        cap(eye, distance, plane, new_frame.target())
                    } else if let Some(plane) = self.up(new_frame) {
                        let up = new_frame.yaw_axis();
                        cap(Vec3::ZERO, 1.0, plane, up)
                            .map(|new_up| eye + arc(up, new_up) * (new_frame.target() - eye))
                    } else {
                        break;
                    };
                    min_delta = new_target
                        .and_then(|new_target| look_around(frame, yaw_axis, new_target))
                        .map_or(Delta::Frame, |(pitch, yaw)| Delta::First {
                            pitch,
                            yaw,
                            yaw_axis,
                        });
                    if loops == self.loops() {
                        min_delta = stop(self, frame, delta).unwrap_or(min_delta);
                        break;
                    }
                    loops += 1;
                }
            }
            Delta::Track { .. } => {
                // Target slides in world space while eye is fixed.
                let old_target = frame.target();
                let eye = frame.eye();
                // Fixed eye cannot be moved inside if it is already outside.
                let eye_bound = self.eye(frame).is_none();
                loop {
                    let new_frame = min_delta.transform(frame);
                    let new_target = if let Some(plane) = self.target(new_frame) {
                        Some(plane.project_point(new_frame.target()))
                    } else if let Some(plane) = self.eye(new_frame).filter(|_| eye_bound) {
                        // Move target inversely to projected eye, e.g., to satisfy distance.
                        let new_eye = new_frame.eye();
                        Some(new_frame.target() - (plane.project_point(new_eye) - new_eye))
                    } else if let Some(plane) = self.up(new_frame) {
                        let up = new_frame.yaw_axis();
                        cap(Vec3::ZERO, 1.0, plane, up)
                            .map(|new_up| eye + arc(up, new_up) * (new_frame.target() - eye))
                    } else {
                        break;
                    };
                    min_delta = new_target.map_or(Delta::Frame, |new_target| Delta::Track {
                        vector: new_target - old_target,
                    });
                    if loops == self.loops() {
                        min_delta = stop(self, frame, delta).unwrap_or(min_delta);
                        break;
                    }
                    loops += 1;
                }
            }
            Delta::Orbit {
                rotation: mut rot,
                point: pos,
            } => {
                // Eye and target orbit around pivot in world space.
                let old_rot = frame.view().rotation;
                let pivot = frame.target() + old_rot * pos;
                // Target only moves if pivot is not the target.
                let target_bound = pos != Vec3::ZERO;
                loop {
                    let new_frame = min_delta.transform(frame);
                    let correction =
                        if let Some(plane) = self.target(new_frame).filter(|_| target_bound) {
                            let new_target = new_frame.target();
                            let radius = (new_target - pivot).length();
                            cap(pivot, radius, plane, new_target)
                                .map(|target| arc(new_target - pivot, target - pivot))
                        } else if let Some(plane) = self.eye(new_frame) {
                            let new_eye = new_frame.eye();
                            let radius = (new_eye - pivot).length();
                            cap(pivot, radius, plane, new_eye)
                                .map(|eye| arc(new_eye - pivot, eye - pivot))
                        } else if let Some(plane) = self.up(new_frame) {
                            let up = new_frame.yaw_axis();
                            cap(Vec3::ZERO, 1.0, plane, up).map(|new_up| arc(up, new_up))
                        } else {
                            break;
                        };
                    // Apply world space correction after delta rotation around same pivot.
                    min_delta = correction.map_or(Delta::Frame, |correction| {
                        rot = (old_rot.inverse() * correction * old_rot * rot).normalize();
                        Delta::Orbit {
                            rotation: rot,
                            point: pos,
                        }
                    });
                    if loops == self.loops() {
                        min_delta = stop(self, frame, delta).unwrap_or(min_delta);
                        break;
                    }
                    loops += 1;
                }
            }
            Delta::Slide { .. } => {
                // Eye and target slide in camera space.
                let old_target = frame.target();
                let old_rot_inverse = frame.view().rotation.inverse();
                let old_eye = frame.eye();
                loop {
                    let new_frame = min_delta.transform(frame);
                    let vec = if let Some(plane) = self.target(new_frame) {
                        plane.project_point(new_frame.target()) - old_target
                    } else if let Some(plane) = self.eye(new_frame) {
                        plane.project_point(new_frame.eye()) - old_eye
                    } else {
                        break;
                    };
                    min_delta = Delta::Slide {
                        vector: old_rot_inverse * vec,
                    };
                    if loops == self.loops() {
                        min_delta = stop(self, frame, delta).unwrap_or(min_delta);
                        break;
                    }
                    loops += 1;
                }
            }
            Delta::Scale {
                ratio: mut rat,
                point: pos,
            } => {
                // Eye and target linearly move towards pivot in world space by `1.0 - rat`.
                let old_target = frame.target();
                let old_eye = frame.eye();
                let old_rot = frame.view().rotation;
                if scope.inspect() {
                    let (znear, _zfar) = scope.clip_planes(0.0);
                    let min_zat = -znear * (1.0 + float::sqrt(f32::EPSILON));
                    let old_zat = frame.distance();
                    if old_zat * rat < min_zat {
                        rat = min_zat / old_zat;
                        min_delta = Delta::Scale {
                            ratio: rat,
                            point: pos,
                        };
                    }
                }
                let scale = |pivot: Vec3, rat: f32| Delta::Scale {
                    ratio: rat,
                    point: old_rot.inverse() * (pivot - old_target),
                };
                // Exceeded plane of moved target or eye with its new and old position.
                let exceeded = |delta: Delta| {
                    let frame = delta.transform(frame);
                    self.target(frame)
                        .map(|plane| (plane, frame.target(), old_target, true))
                        .or_else(|| {
                            self.eye(frame)
                                .map(|plane| (plane, frame.eye(), old_eye, false))
                        })
                };
                // Keep ratio but move pivot such that exceeded position is projected onto plane.
                let glide = |rat: f32| {
                    let fraction = 1.0 - rat;
                    let mut delta = Delta::Scale {
                        ratio: rat,
                        point: pos,
                    };
                    for loops in 0..=self.loops() {
                        let Some((plane, new, old, target)) = exceeded(delta) else {
                            return Some((delta, loops));
                        };
                        let glided = old + (plane.project_point(new) - old) / fraction;
                        delta = scale(glided, rat);
                        // Plane moving along with frame (e.g., distance bound) cannot be glided
                        // along.
                        let moving = exceeded(delta).is_some_and(|(other, _, _, other_target)| {
                            other_target == target
                                && other.normal.dot(plane.normal) > 1.0 - float::sqrt(f32::EPSILON)
                        });
                        if moving || fraction == 0.0 {
                            return None;
                        }
                    }
                    None
                };
                if let Some((delta, glide_loops)) = glide(rat) {
                    (min_delta, loops) = (delta, glide_loops);
                } else if exceeded(Delta::Frame).is_none() {
                    // Stop at largest ratio still gliding along remaining planes.
                    let fraction = 1.0 - rat;
                    let (mut min, mut max) = (0.0, 1.0);
                    for _ in 0..f32::MANTISSA_DIGITS {
                        let mid = f32::midpoint(min, max);
                        if glide(float::mul_add(fraction, -mid, 1.0)).is_some() {
                            min = mid;
                        } else {
                            max = mid;
                        }
                    }
                    if let Some((delta, glide_loops)) = glide(float::mul_add(fraction, -min, 1.0)) {
                        (min_delta, loops) = (delta, glide_loops);
                    }
                }
            }
        }
        (min_delta != delta).then_some((min_delta, loops))
    }
}

/// Stops `delta` at largest fraction satisfying boundary conditions of `clamp` by bisection.
///
/// Returns `None` if boundary conditions are not satisfied initially.
fn stop<C: Clamp + ?Sized>(clamp: &C, frame: Frame, delta: Delta) -> Option<Delta> {
    let inside = |delta: Delta| {
        let frame = delta.transform(frame);
        clamp.target(frame).is_none() && clamp.eye(frame).is_none() && clamp.up(frame).is_none()
    };
    inside(Delta::Frame).then(|| {
        let (mut min, mut max) = (0.0, 1.0);
        for _ in 0..f32::MANTISSA_DIGITS {
            let mid = f32::midpoint(min, max);
            if inside(delta.lerp_slerp(mid)) {
                min = mid;
            } else {
                max = mid;
            }
        }
        delta.lerp_slerp(min)
    })
}

/// Projects `point` onto circle where sphere at `center` with `radius` intersects with `plane`.
///
/// Returns `None` if sphere and plane do not intersect or if projection is ambiguous.
fn cap(center: Vec3, radius: f32, plane: Plane, point: Vec3) -> Option<Vec3> {
    let distance = plane.normal.dot(center) + plane.bias;
    let square = float::mul_add(radius, radius, -distance * distance);
    let ray = (plane.project_point(point) - plane.project_point(center)).try_normalize()?;
    (square > 0.0).then(|| plane.project_point(center) + ray * float::sqrt(square))
}

/// Shortest arc rotation from `from` to `to` or identity if opposite or if either is zero.
///
/// Unlike `Quat::from_rotation_arc()`, small angles are not snapped to identity.
fn arc(from: Vec3, to: Vec3) -> Quat {
    match (from.try_normalize(), to.try_normalize()) {
        (Some(from), Some(to)) if 1.0 + from.dot(to) >= 2.0 * f32::EPSILON => {
            let axis = from.cross(to);
            Quat::from_xyzw(axis.x, axis.y, axis.z, 1.0 + from.dot(to)).normalize()
        }
        _ => Quat::IDENTITY,
    }
}

/// Decomposes rotation of target around eye into pitch and yaw of [`Delta::First`].
///
/// Pitch is the change of elevation in the vertical plane spanned by `yaw_axis` and the forward
/// direction orthogonal to the pitch axis. Yaw is the remaining change of azimuth around
/// `yaw_axis`. Returns `None` if the new target coincides with the eye.
fn look_around(frame: Frame, yaw_axis: Vec3, new_target: Vec3) -> Option<(f32, f32)> {
    let eye = frame.eye();
    let old_ray = (frame.target() - eye).try_normalize()?;
    let new_ray = (new_target - eye).try_normalize()?;
    // Horizontal forward direction orthogonal to pitch axis, defined even when looking straight
    // up or down.
    let forward = yaw_axis.cross(frame.pitch_axis()).normalize();
    // Signed elevation of old target from yaw axis within vertical plane.
    let old_elevation = float::atan2(old_ray.dot(forward), old_ray.dot(yaw_axis));
    // Elevation of new target from yaw axis on same side as old target.
    let horizontal = new_ray - yaw_axis * new_ray.dot(yaw_axis);
    let new_elevation =
        float::atan2(horizontal.length(), new_ray.dot(yaw_axis)).copysign(old_elevation);
    // Rotating by positive pitch around pitch axis decreases elevation.
    let pitch = old_elevation - new_elevation;
    // Remaining azimuth from pitched forward direction to new target around yaw axis.
    let yaw = horizontal.try_normalize().map_or(0.0, |horizontal| {
        let forward = forward * new_elevation.signum();
        float::atan2(
            yaw_axis.dot(forward.cross(horizontal)),
            forward.dot(horizontal),
        )
    });
    Some((pitch, yaw))
}
