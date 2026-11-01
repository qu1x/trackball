//! Gliding clamp of all [`Delta`] variants wrt [`Bound`].

use core::f32::consts::FRAC_PI_4;
use trackball::{
	Bound, Clamp, Delta, Frame, Rigid, Scope,
	glam::{Quat, Vec3},
};

/// Tolerance of positions wrt boundary planes.
const TOL: f32 = 1e-3;

/// Applies clamped delta like `bevy_trackball` does and returns new frame and loops.
fn step(bound: &Bound, frame: &Frame, delta: &Delta) -> (Frame, usize) {
	let scope = Scope::default();
	let (delta, loops) = bound.compute(frame, &scope, delta).unwrap_or((*delta, 0));
	let mut frame = delta.transform(frame);
	frame.renormalize();
	(frame, loops)
}

/// Ground bound similar to `gliding_clamp` example of `bevy_trackball`.
fn ground() -> Bound {
	let mut bound = Bound::default();
	bound.min_target.y = 0.0;
	bound.min_eye.y = 0.3;
	bound.min_distance = 6.0;
	bound.max_distance = 50.0;
	bound
}

/// Asserts all boundary conditions of `bound` are satisfied by `frame`.
fn assert_inside(bound: &Bound, frame: &Frame) {
	assert!(bound.target(frame).is_none(), "target outside: {frame:?}");
	assert!(bound.eye(frame).is_none(), "eye outside: {frame:?}");
	assert!(bound.up(frame).is_none(), "up outside: {frame:?}");
}

/// Asserts that consecutive displacements of a gliding position neither reverse nor stall.
fn assert_smooth(positions: &[Vec3]) {
	let steps = positions
		.windows(2)
		.map(|w| w[1] - w[0])
		.collect::<Vec<_>>();
	for pair in steps.windows(2) {
		let (a, b) = (pair[0], pair[1]);
		assert!(a.dot(b) > 0.0, "reversing glide: {a} then {b}");
		let ratio = b.length() / a.length();
		assert!((0.5..2.0).contains(&ratio), "jittering glide: {a} then {b}");
	}
}

#[test]
fn first_glides_target_along_ground() {
	let bound = ground();
	let mut frame = Frame::look_at(Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 2.0, 8.0), Vec3::Y);
	let eye = frame.eye();
	let yaw_axis = frame.yaw_axis();
	let delta = Delta::First {
		pitch: -0.02,
		yaw: 0.01,
		yaw_axis,
	};
	let mut targets = Vec::new();
	for _ in 0..200 {
		(frame, _) = step(&bound, &frame, &delta);
		assert_inside(&bound, &frame);
		// Eye is fixed and roll attitude is preserved.
		assert!(frame.eye().distance(eye) < TOL);
		assert!(frame.pitch_axis().dot(yaw_axis).abs() < TOL);
		if frame.target().y < TOL {
			targets.push(frame.target());
		}
	}
	// Target reached ground and kept gliding instead of stopping.
	assert!(targets.len() > 100);
	assert!(targets.iter().all(|target| target.y.abs() < TOL));
	assert!(targets.first().unwrap().distance(*targets.last().unwrap()) > 1.0);
	assert_smooth(&targets);
}

#[test]
fn first_stops_at_vertical_with_up_bound() {
	let mut bound = Bound::default();
	bound.min_up.y = 0.0;
	let mut frame = Frame::look_at(Vec3::ZERO, Vec3::new(0.0, 0.0, 10.0), Vec3::Y);
	let yaw_axis = frame.yaw_axis();
	let delta = Delta::First {
		pitch: -0.05,
		yaw: 0.0,
		yaw_axis,
	};
	for _ in 0..100 {
		(frame, _) = step(&bound, &frame, &delta);
		assert_inside(&bound, &frame);
	}
	// Looking straight down without turning upside down.
	assert!(frame.roll_axis().dot(Vec3::Y) > 1.0 - TOL);
	assert!(frame.yaw_axis().y > -TOL);
}

#[test]
fn orbit_glides_eye_along_ground() {
	let bound = ground();
	let mut frame = Frame::look_at(Vec3::Y, Vec3::new(0.0, 7.0, 14.0), Vec3::Y);
	let target = frame.target();
	// Orbit eye downwards and sideways in camera space.
	let rot = Quat::from_axis_angle(Vec3::new(-1.0, 0.5, 0.0).normalize(), 0.02);
	let delta = Delta::Orbit {
		rot,
		pos: Vec3::ZERO,
	};
	let mut eyes = Vec::new();
	for _ in 0..300 {
		(frame, _) = step(&bound, &frame, &delta);
		assert_inside(&bound, &frame);
		assert!(frame.target().distance(target) < TOL);
		if frame.eye().y < 0.3 + TOL {
			eyes.push(frame.eye());
		}
	}
	assert!(eyes.len() > 100);
	assert!(eyes.first().unwrap().distance(*eyes.last().unwrap()) > 1.0);
	assert_smooth(&eyes);
}

#[test]
fn orbit_glides_over_pole_without_turning_upside_down() {
	let mut bound = Bound::default();
	bound.min_up.y = 0.0;
	let mut frame = Frame::look_at(Vec3::ZERO, Vec3::new(0.0, 0.0, 10.0), Vec3::Y);
	// Orbit eye upwards over the pole and sideways in camera space.
	let rot = Quat::from_axis_angle(Vec3::new(-1.0, 0.3, 0.0).normalize(), 0.05);
	let delta = Delta::Orbit {
		rot,
		pos: Vec3::ZERO,
	};
	let mut eyes = Vec::new();
	for _ in 0..100 {
		(frame, _) = step(&bound, &frame, &delta);
		assert_inside(&bound, &frame);
		if frame.yaw_axis().y < TOL {
			eyes.push(frame.eye());
		}
	}
	// Up axis glided along horizon while rolling sideways over the pole.
	assert!(eyes.len() > 50);
	assert_smooth(&eyes);
}

#[test]
fn orbit_stops_at_pole_like_turntable() {
	let bound = Bound {
		min_up: Vec3::new(0.0, 0.0, f32::MIN),
		max_up: Vec3::new(0.0, f32::MAX, f32::MAX),
		..Bound::default()
	};
	let mut frame = Frame::look_at(Vec3::ZERO, Vec3::new(0.0, 0.0, 10.0), Vec3::Y);
	// Orbit eye upwards over the pole and sideways around world up axis.
	let up = Delta::Orbit {
		rot: Quat::from_axis_angle(Vec3::X, -0.05),
		pos: Vec3::ZERO,
	};
	for _ in 0..100 {
		(frame, _) = step(&bound, &frame, &up);
		assert_inside(&bound, &frame);
		assert!(frame.pitch_axis().y.abs() < TOL);
	}
	// Eye stopped at pole without rolling.
	assert!(frame.eye().y > 10.0 - TOL);
	// Turning around the vertical still works at the pole.
	let turn = Delta::Orbit {
		rot: Quat::from_axis_angle(frame.view().rotation.inverse() * Vec3::Y, 0.05),
		pos: Vec3::ZERO,
	};
	let pitch_axis = frame.pitch_axis();
	for _ in 0..10 {
		(frame, _) = step(&bound, &frame, &turn);
		assert_inside(&bound, &frame);
	}
	assert!(frame.pitch_axis().dot(pitch_axis) < 0.9);
}

#[test]
fn orbit_around_pivot_respects_bounds() {
	let bound = ground();
	let mut frame = Frame::look_at(Vec3::Y, Vec3::new(0.0, 4.0, 10.0), Vec3::Y);
	// Two-finger roll and tilt around a point on the focus plane.
	let rot = Quat::from_axis_angle(Vec3::new(1.0, 0.0, 1.0).normalize(), 0.05);
	let delta = Delta::Orbit {
		rot,
		pos: Vec3::new(2.0, -1.0, 0.0),
	};
	let mut moved = false;
	for _ in 0..200 {
		let old = frame;
		(frame, _) = step(&bound, &frame, &delta);
		assert_inside(&bound, &frame);
		moved |= !frame.abs_diff_eq(&old, TOL);
	}
	assert!(moved);
}

#[test]
fn scale_stops_at_distance_bounds() {
	let bound = ground();
	let frame = Frame::look_at(Vec3::Y, Vec3::new(0.0, 7.0, 14.0), Vec3::Y);
	let zoom_in = Delta::Scale {
		rat: 0.1,
		pos: Vec3::ZERO,
	};
	let (new_frame, _) = step(&bound, &frame, &zoom_in);
	assert!((new_frame.distance() - 6.0).abs() < TOL);
	assert!(new_frame.target().distance(frame.target()) < TOL);
	let zoom_out = Delta::Scale {
		rat: 10.0,
		pos: Vec3::ZERO,
	};
	let (new_frame, _) = step(&bound, &frame, &zoom_out);
	assert!((new_frame.distance() - 50.0).abs() < TOL);
}

#[test]
fn scale_glides_along_ground() {
	let mut bound = Bound::default();
	bound.min_eye.y = 0.3;
	bound.min_distance = 1.0;
	let mut frame = Frame::look_at(Vec3::ZERO, Vec3::new(0.0, 1.0, 10.0), Vec3::Y);
	let roll_axis = frame.roll_axis();
	let delta = Delta::Scale {
		rat: 0.95,
		pos: Vec3::ZERO,
	};
	let mut eyes = Vec::new();
	for _ in 0..100 {
		let old = frame;
		(frame, _) = step(&bound, &frame, &delta);
		assert_inside(&bound, &frame);
		// Rotation is preserved and zoom ratio is kept until minimum distance is reached.
		assert!(frame.roll_axis().distance(roll_axis) < TOL);
		if old.distance() * 0.95 > 1.0 {
			assert!(old.distance().mul_add(-0.95, frame.distance()).abs() < TOL);
		}
		if frame.eye().y < 0.3 + TOL && frame.distance() > 1.0 + TOL {
			eyes.push(frame.eye());
		}
	}
	// Eye glided along ground instead of stopping at a distance of about 3.
	assert!((frame.distance() - 1.0).abs() < TOL);
	assert!(eyes.len() > 10);
	assert_smooth(&eyes);
}

#[test]
fn scale_glides_around_pivot_and_stops_at_distance() {
	let mut bound = ground();
	bound.min_distance = 0.0;
	let frame = Frame::look_at(Vec3::Y, Vec3::new(0.0, 7.0, 14.0), Vec3::Y);
	// Zoom towards point below target on focus plane which drags eye and target below ground.
	let delta = Delta::Scale {
		rat: 0.3,
		pos: Vec3::new(0.0, -4.0, 0.0),
	};
	let (new_frame, _) = step(&bound, &frame, &delta);
	assert_inside(&bound, &new_frame);
	assert!(frame.distance().mul_add(-0.3, new_frame.distance()).abs() < TOL);
	assert!(new_frame.roll_axis().distance(frame.roll_axis()) < TOL);
	// Stops at minimum distance instead.
	bound.min_distance = 6.0;
	let (new_frame, loops) = step(&bound, &frame, &delta);
	assert_inside(&bound, &new_frame);
	assert!((new_frame.distance() - 6.0).abs() < TOL);
	assert!(loops < bound.loops());
}

#[test]
fn slide_glides_along_ground() {
	let bound = ground();
	let mut frame = Frame::look_at(Vec3::Y, Vec3::new(0.0, 7.0, 14.0), Vec3::Y);
	let delta = Delta::Slide {
		vec: Vec3::new(0.05, -0.05, 0.0),
	};
	let mut targets = Vec::new();
	for _ in 0..200 {
		(frame, _) = step(&bound, &frame, &delta);
		assert_inside(&bound, &frame);
		if frame.target().y < TOL {
			targets.push(frame.target());
		}
	}
	assert!(targets.len() > 100);
	assert_smooth(&targets);
}

#[test]
fn track_glides_target_in_world_space() {
	let bound = ground();
	// Rotated camera reveals mixing up camera and world space.
	let mut frame = Frame::look_at(Vec3::Y, Vec3::new(10.0, 7.0, 3.0), Vec3::Y);
	let eye = frame.eye();
	let delta = Delta::Track {
		vec: Vec3::new(0.0, -0.02, 0.02),
	};
	let mut targets = Vec::new();
	for _ in 0..200 {
		(frame, _) = step(&bound, &frame, &delta);
		assert_inside(&bound, &frame);
		assert!(frame.eye().distance(eye) < TOL);
		if frame.target().y < TOL {
			targets.push(frame.target());
		}
	}
	assert!(targets.len() > 100);
	// Glides along world z-axis as requested.
	let glide = *targets.last().unwrap() - *targets.first().unwrap();
	assert!(glide.z > 1.0 && glide.x.abs() < TOL);
	assert_smooth(&targets);
}

#[test]
fn transformed_bound_clamps_in_bound_space() {
	let mut bound = Bound::default();
	bound.min_eye.y = 0.0;
	// Tilted ground through point (0, 1, 0).
	bound.transform = Rigid::new(Vec3::Y, Vec3::new(0.0, 0.0, FRAC_PI_4 / 2.0));
	let mut frame = Frame::look_at(Vec3::new(0.0, 2.0, 0.0), Vec3::new(0.0, 8.0, 10.0), Vec3::Y);
	let rot = Quat::from_axis_angle(Vec3::X, -0.02);
	let delta = Delta::Orbit {
		rot,
		pos: Vec3::ZERO,
	};
	for _ in 0..200 {
		(frame, _) = step(&bound, &frame, &delta);
		assert_inside(&bound, &frame);
		let eye = bound.transform.inverse() * frame.eye();
		assert!(eye.y > -TOL);
	}
}

/// Deterministic pseudo-random numbers in `[-1.0, 1.0]`.
struct Random(u32);

impl Random {
	#[allow(clippy::cast_precision_loss)]
	fn next(&mut self) -> f32 {
		self.0 ^= self.0 << 13;
		self.0 ^= self.0 >> 17;
		self.0 ^= self.0 << 5;
		(self.0 as f32 / u32::MAX as f32).mul_add(2.0, -1.0)
	}
	fn below(&mut self, max: u32) -> u32 {
		self.next();
		self.0 % max
	}
	fn vec(&mut self) -> Vec3 {
		Vec3::new(self.next(), self.next(), self.next())
	}
}

#[test]
fn random_deltas_stay_inside() {
	let bound = Bound {
		min_target: Vec3::new(-5.0, 0.0, -5.0),
		max_target: Vec3::new(5.0, 5.0, 5.0),
		min_eye: Vec3::new(-20.0, 0.3, -20.0),
		max_eye: Vec3::new(20.0, 20.0, 20.0),
		min_up: Vec3::new(f32::MIN, 0.0, f32::MIN),
		min_distance: 2.0,
		max_distance: 15.0,
		..Bound::default()
	};
	let mut random = Random(0x2545_f491);
	let mut stops = 0;
	for _ in 0..200 {
		let mut frame = Frame::look_at(Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 5.0, 8.0), Vec3::Y);
		for _ in 0..50 {
			let yaw_axis = frame.yaw_axis();
			let delta = match random.below(6) {
				0 => Delta::First {
					pitch: random.next() * 0.2,
					yaw: random.next() * 0.2,
					yaw_axis,
				},
				1 => Delta::Track { vec: random.vec() },
				2 => Delta::Orbit {
					rot: Quat::from_scaled_axis(random.vec() * 0.2),
					pos: Vec3::ZERO,
				},
				3 => Delta::Orbit {
					rot: Quat::from_scaled_axis(random.vec() * 0.1),
					pos: random.vec().with_z(0.0),
				},
				4 => Delta::Slide { vec: random.vec() },
				_ => Delta::Scale {
					rat: (random.next() * 0.5).exp(),
					pos: random.vec().with_z(0.0),
				},
			};
			let loops;
			(frame, loops) = step(&bound, &frame, &delta);
			// Stops instead of gliding when not converging.
			if loops == bound.loops() {
				stops += 1;
			}
			assert_inside(&bound, &frame);
			assert!(frame.yaw_axis().y > -TOL && frame.roll_axis().is_finite());
		}
	}
	assert!(stops < 50, "{stops} stops instead of glides");
}
