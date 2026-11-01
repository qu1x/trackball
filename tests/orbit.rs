//! Rotations of [`Orbit`].

use core::f32::consts::{FRAC_1_SQRT_2, FRAC_PI_2};
use trackball::{
    Orbit,
    glam::{Quat, Vec2, Vec3},
};

const MAX: Vec2 = Vec2::new(800.0, 600.0);

/// Trackball point of screen position.
fn exp(pos: Vec2) -> Vec3 {
    let pos = Vec2::new(pos.x - 400.0, 300.0 - pos.y);
    let (sin, cos) = (pos.length() / 400.0 * FRAC_PI_2).sin_cos();
    (pos.normalize_or_zero() * sin).extend(cos)
}

#[test]
fn starts_at_center() {
    let mut orbit = Orbit::default();
    assert_eq!(orbit.compute(MAX * 0.5, MAX), None);
    // Displacement to the right rotates about negative y-axis by displacement over radius.
    let rot = orbit
        .compute(MAX * 0.5 + Vec2::new(40.0, 0.0), MAX)
        .unwrap();
    let rot_y = Quat::from_rotation_y(-0.1 * FRAC_PI_2);
    assert!(rot.abs_diff_eq(rot_y, 1e-6), "{rot:?}");
}

#[test]
fn sticks_to_cursor_on_radial_displacement() {
    let mut orbit = Orbit::default();
    // Start halfway left of center mapped onto trackball at 45° from its pole.
    assert_eq!(orbit.compute(Vec2::new(200.0, 300.0), MAX), None);
    // Displace through center to right edge mapped onto trackball's equator.
    let rot = orbit.compute(Vec2::new(800.0, 300.0), MAX).unwrap();
    // Scene rotates inversely to orbiting camera carrying trackball's point along with cursor.
    let point = rot.inverse() * Vec3::new(-FRAC_1_SQRT_2, 0.0, FRAC_1_SQRT_2);
    assert!(point.abs_diff_eq(Vec3::X, 1e-6), "{point:?}");
}

#[test]
fn sticks_to_cursor_on_any_path() {
    let path = (0..=1000u16).map(|step| {
        let t = f32::from(step) / 1000.0;
        Vec2::new(
            450.0f32.mul_add(t, 150.0),
            150.0f32.mul_add((6.0 * t).sin(), 250.0),
        )
    });
    let (first, last) = (
        path.clone().next().unwrap(),
        path.clone().next_back().unwrap(),
    );
    let mut orbit = Orbit::default();
    let rot = path.fold(Quat::IDENTITY, |rot, pos| {
        orbit
            .compute(pos, MAX)
            .map_or(rot, |delta| (rot * delta).normalize())
    });
    // Scene rotates inversely to orbiting camera carrying trackball's point along with cursor.
    let point = rot.inverse() * exp(first);
    assert!(point.abs_diff_eq(exp(last), 1e-5), "{point:?}");
}

#[test]
fn rolls_like_cursor_along_border() {
    let mut orbit = Orbit::default();
    // Start at right edge and displace along border, both mapped onto trackball's equator.
    assert_eq!(orbit.compute(Vec2::new(800.0, 300.0), MAX), None);
    let rot = orbit.compute(Vec2::new(720.0, 60.0), MAX).unwrap();
    // Rolls about negative z-axis by angle the cursor turns around screen's center.
    let roll = Quat::from_axis_angle(Vec3::NEG_Z, 0.6f32.atan2(0.8));
    assert!(rot.abs_diff_eq(roll, 1e-6), "{rot:?}");
}

#[test]
fn rotates_radially_in_corners() {
    let mut orbit = Orbit::default();
    // Start in top right corner 450 away from center beyond trackball's border 400 away from it.
    assert_eq!(orbit.compute(Vec2::new(760.0, 30.0), MAX), None);
    // Radial displacement further outwards rotates by displacement over radius like elsewhere.
    let rot = orbit.compute(Vec2::new(776.0, 18.0), MAX).unwrap();
    let rot_xy = Quat::from_axis_angle(Vec3::new(0.6, -0.8, 0.0), 0.05 * FRAC_PI_2);
    assert!(rot.abs_diff_eq(rot_xy, 1e-6), "{rot:?}");
    // Scene rotates inversely to orbiting camera carrying trackball's point along with cursor.
    let point = rot.inverse() * exp(Vec2::new(760.0, 30.0));
    assert!(
        point.abs_diff_eq(exp(Vec2::new(776.0, 18.0)), 1e-6),
        "{point:?}"
    );
}

#[test]
fn half_turns_across_screen() {
    let mut orbit = Orbit::default();
    // Start at left edge and jump to right edge, both mapped onto trackball's equator.
    assert_eq!(orbit.compute(Vec2::new(0.0, 300.0), MAX), None);
    let rot = orbit.compute(Vec2::new(800.0, 300.0), MAX).unwrap();
    // Half turn on great circle through pole carrying trackball's point along with cursor.
    assert!(
        (rot.inverse() * Vec3::NEG_X).abs_diff_eq(Vec3::X, 1e-6),
        "{rot:?}"
    );
    assert!((rot * Vec3::Z).abs_diff_eq(Vec3::NEG_Z, 1e-6), "{rot:?}");
}
