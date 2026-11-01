use glam::{Quat, Vec2, Vec3};

#[cfg(not(feature = "cc"))]
use crate::{Image, float};

/// Orbit induced by displacement on screen.
///
/// Implements [`Default`] and can be created with `Orbit::default()`.
///
/// Both its methods must be invoked on matching events fired by your 3D graphics library of choice.
#[derive(Debug, Clone, Default)]
pub struct Orbit {
    /// Caches trackball point of previous cursor/finger position.
    exp: Option<Vec3>,
}

#[cfg(not(feature = "cc"))]
use core::f32::consts::FRAC_PI_2;

impl Orbit {
    /// Computes rotation between previous and current cursor/finger position.
    ///
    /// Trackball point of previous position is cached and has to be discarded on button/finger
    /// release via [`Self::discard()`]. Current position `pos` is clamped between origin and
    /// maximum position `max` as screen's width and height.
    ///
    /// Maps positions via the exponential map onto a target-centered trackball with radius of
    /// maximum of the screen's width and height divided by π and rotates along the shortest arc
    /// between their trackball points so that the trackball's point under the cursor/finger stays
    /// under it. Positions in screen corners beyond the trackball's border are mapped beyond its
    /// equator.
    ///
    /// Screen space with origin in top left corner:
    ///
    ///   * x-axis from left to right,
    ///   * y-axis from top to bottom.
    ///
    /// Camera space with origin at its target, the trackball's center:
    ///
    ///   * x-axis from left to right,
    ///   * y-axis from bottom to top,
    ///   * z-axis from far to near.
    ///
    /// Returns `None`:
    ///
    ///   * on first invocation and after [`Self::discard()`] as there is no previous position yet,
    ///   * for displacements resulting in the same trackball point, e.g., when a position event
    ///     fires twice.
    #[cfg(not(feature = "cc"))]
    pub fn compute(&mut self, pos: Vec2, max: Vec2) -> Option<Quat> {
        // Clamped cursor/finger position from left to right and top to bottom.
        let pos = Image::clamp_pos_wrt_max(pos, max);
        // Centered cursor/finger position and its maximum from left to right and bottom to top.
        let (pos, max) = Image::transform_pos_and_max_wrt_max(pos, max);
        // Trackball's radius mapping maximum of half the screen's width or height onto quarter of
        // its circumference.
        let rad = max.x.max(max.y) / FRAC_PI_2;
        // Polar angle of position, beyond equator for screen corners.
        let pol = pos.length() / rad;
        // Exponential map of position onto trackball.
        let (ray, (sin, cos)) = (pos.normalize_or_zero(), float::sin_cos(pol));
        let exp = (ray * sin).extend(cos);
        // Get old and replace with new trackball point unless they coincide.
        let old = self.exp.replace(exp).filter(|&old| old != exp)?;
        // Shortest arc from new to old trackball point as rotation of camera which in turn carries
        // old to new trackball point, keeping it under the cursor/finger.
        let rot = exp
            .cross(old)
            .extend(1.0 + exp.dot(old))
            .try_normalize()
            .map_or_else(
                // Shortest arc is ambiguous for antipodal trackball points as any half turn about
                // an axis orthogonal to them carries one onto the other. Choose the half turn on
                // the great circle through the pole as if displacing straight through the screen's
                // center.
                || Quat::from_xyzw(ray.y, -ray.x, 0.0, 0.0),
                Quat::from_vec4,
            );
        Some(rot)
    }
    /// Computes rotation between previous and current cursor/finger position.
    ///
    /// Trackball point of previous position is cached and has to be discarded on button/finger
    /// release via [`Self::discard()`]. Current position `pos` is clamped between origin and
    /// maximum position `max` as screen's width and height.
    ///
    /// Maps positions via the exponential map onto a target-centered trackball with radius of
    /// maximum of the screen's width and height divided by π and rotates along the shortest arc
    /// between their trackball points so that the trackball's point under the cursor/finger stays
    /// under it. Positions in screen corners beyond the trackball's border are mapped beyond its
    /// equator.
    ///
    /// Screen space with origin in top left corner:
    ///
    ///   * x-axis from left to right,
    ///   * y-axis from top to bottom.
    ///
    /// Camera space with origin at its target, the trackball's center:
    ///
    ///   * x-axis from left to right,
    ///   * y-axis from bottom to top,
    ///   * z-axis from far to near.
    ///
    /// Returns `None`:
    ///
    ///   * on first invocation and after [`Self::discard()`] as there is no previous position yet,
    ///   * for displacements resulting in the same trackball point, e.g., when a position event
    ///     fires twice.
    #[cfg(feature = "cc")]
    pub fn compute(&mut self, pos: Vec2, max: Vec2) -> Option<Quat> {
        let mut rot = Quat::IDENTITY.to_array();
        let mut old = self.exp.map_or([0.0; 4], |exp| exp.extend(1.0).to_array());
        let (pos, max) = (pos.to_array(), max.to_array());
        #[allow(unsafe_code)]
        unsafe {
            trackball_orbit_f(
                rot.as_mut_ptr(),
                old.as_mut_ptr(),
                pos.as_ptr(),
                max.as_ptr(),
            );
        }
        let [x, y, z, _len] = old;
        self.exp = Some(Vec3::new(x, y, z));
        #[allow(clippy::float_cmp)]
        (rot != Quat::IDENTITY.to_array()).then(|| Quat::from_array(rot))
    }
    /// Discards cached trackball point of previous cursor/finger position on button/finger
    /// release.
    pub const fn discard(&mut self) {
        self.exp = None;
    }
}

#[cfg(feature = "cc")]
#[allow(unsafe_code)]
unsafe extern "C" {
    fn trackball_orbit_f(xyzw: *mut f32, xyzm: *mut f32, xy: *const f32, wh: *const f32);
}
