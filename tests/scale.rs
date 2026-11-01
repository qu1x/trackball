//! Ratios of [`Scale`].

use core::iter::repeat_n;
use trackball::Scale;

#[test]
fn scales_reversibly() {
	let scale = Scale::default();
	// Scaling in and out by the same relative value restores the distance.
	let ratio = scale.compute(42.0) * scale.compute(-42.0);
	assert!((ratio - 1.0).abs() < 1e-6, "{ratio}");
}

#[test]
fn scales_regardless_of_splitting() {
	let mut scale = Scale::default();
	scale.set_denominator(24.0);
	// One wheel turn of 24 clicks scales by `1 / e` whether in 24 events or a single one.
	let clicks = repeat_n(scale.compute(1.0), 24).product::<f32>();
	let turn = scale.compute(24.0);
	assert!((clicks - (-1.0f32).exp()).abs() < 1e-5, "{clicks}");
	assert!((turn - (-1.0f32).exp()).abs() < 1e-6, "{turn}");
}

#[test]
fn scales_positively() {
	let scale = Scale::default();
	// Large relative values neither collapse nor flip the distance.
	assert!(scale.compute(10.0 * scale.denominator()) > 0.0);
}
