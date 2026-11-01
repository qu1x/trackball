//! Inverse of all [`Delta`] variants.

use trackball::{
	Delta, Frame,
	glam::{Quat, Vec3},
};

#[test]
fn inverse_restores_frame() {
	let frame = Frame::look_at(
		Vec3::new(3.0, 2.0, 10.0),
		Vec3::new(1.0, 0.5, -2.0),
		Vec3::Y,
	);
	let deltas = [
		Delta::Frame,
		Delta::First {
			pitch: 0.3,
			yaw: -0.7,
			yaw_axis: frame.yaw_axis(),
		},
		Delta::Track {
			vector: Vec3::new(0.5, -1.0, 2.0),
		},
		Delta::Orbit {
			rotation: Quat::from_scaled_axis(Vec3::new(0.2, -0.4, 0.1)),
			point: Vec3::new(1.0, 2.0, -3.0),
		},
		Delta::Slide {
			vector: Vec3::new(-1.0, 0.5, 2.0),
		},
		Delta::Scale {
			ratio: 0.5,
			point: Vec3::new(1.0, 2.0, 0.0),
		},
		Delta::Scale {
			ratio: 3.0,
			point: Vec3::new(-2.0, 1.0, 0.5),
		},
	];
	for delta in deltas {
		let back = delta.inverse().transform(delta.transform(frame));
		assert!(back.abs_diff_eq(frame, 1e-5), "{delta:?}: {back:?}");
	}
}
