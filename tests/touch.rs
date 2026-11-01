//! Finger gestures of [`Touch`].

use std::panic::{AssertUnwindSafe, catch_unwind};
use trackball::{Touch, glam::Vec2};

#[test]
fn two_finger_tap() {
    let mut touch = Touch::<u64>::default();
    assert!(touch.compute(1, Vec2::new(0.0, 0.0), 0).is_some());
    assert!(touch.compute(2, Vec2::new(2.0, 0.0), 0).is_some());
    // Unchanged position is ignored.
    assert_eq!(touch.compute(2, Vec2::new(2.0, 0.0), 0), None);
    assert_eq!(touch.fingers(), 2);
    assert_eq!(touch.discard(1), None);
    assert_eq!(touch.discard(2), Some((2, Vec2::new(1.0, 0.0))));
    assert_eq!(touch.fingers(), 0);
}

#[test]
fn two_finger_roll_and_scale() {
    let mut touch = Touch::<u64>::default();
    touch.compute(1, Vec2::new(-1.0, 0.0), 0);
    touch.compute(2, Vec2::new(1.0, 0.0), 0);
    // Rotate first finger by 45° and shorten distance to `2.0.sqrt()`.
    let (num, pos, rot, rat) = touch.compute(1, Vec2::new(0.0, -1.0), 0).unwrap();
    assert_eq!(num, 2);
    assert_eq!(pos, Vec2::new(0.5, -0.5));
    assert!((rot - core::f32::consts::FRAC_PI_4).abs() < 1e-6);
    assert!((rat - 2.0f32.sqrt()).abs() < 1e-6);
}

#[test]
fn unknown_finger_discards_all() {
    let mut touch = Touch::<u64>::default();
    touch.compute(1, Vec2::new(0.0, 0.0), 0);
    touch.compute(2, Vec2::new(1.0, 0.0), 0);
    assert_eq!(touch.discard(9), None);
    assert_eq!(touch.fingers(), 0);
}

#[test]
fn freed_slot_is_reused() {
    let mut touch = Touch::<u64, 2>::default();
    touch.compute(1, Vec2::new(0.0, 0.0), 0);
    touch.compute(2, Vec2::new(1.0, 0.0), 0);
    assert_eq!(touch.discard(1), None);
    touch.compute(3, Vec2::new(2.0, 0.0), 0);
    assert_eq!(touch.fingers(), 2);
}

#[test]
fn capacity_limits_fingers() {
    let mut touch = Touch::<u8, 3>::default();
    for id in 0..3 {
        touch.compute(id, Vec2::splat(f32::from(id)), 0);
    }
    assert_eq!(touch.fingers(), 3);
    let fourth = catch_unwind(AssertUnwindSafe(|| touch.compute(3, Vec2::ZERO, 0)));
    assert!(fourth.is_err());
}
