# Version 0.19.0 (unreleased)

  * Migrate from `nalgebra` to `glam` and drop `glam` conversion feature.
  * Swap arguments of `Frame::look_at()` to `eye`, `target`, and `up` like `glam`. Existing calls
    still compile, so check their argument order.
  * Drop generic scalar in favor of `f32` and remove `cast()` methods.
  * Pass `glam` types by value, e.g., `Vec2`, `Vec3`, and `Quat`.
  * Add `Rigid` transformation replacing `Isometry3` and construct it from rotation and
    translation via `Rigid::from_rotation_translation()` like `glam`.
  * Replace `Frame::try_lerp_slerp()` with infallible `Frame::lerp_slerp()`.
  * Fix precision of `Frame::set_target()` for small and nearly opposite target movements.
  * Drop `Plane` conversions from and to `Reflection3`.
  * Bump `rkyv` to version 0.8 deriving its traits.
  * Drop `approx` in favor of inherent `abs_diff_eq()` methods like `glam`.
  * Drop `num-traits` in favor of `std` or `libm` math backends selected like `glam` via `std`
    (default), `libm`, or `nostd-libm` feature. Use `default-features = false` and `nostd-libm`
    for `no_std` environments.
  * Drop `heapless` in favor of array-based finger positions of `Touch`.
  * Complete gliding `Clamp` for `Delta::First`, `Delta::Track`, `Delta::Orbit` around arbitrary
    points, `Delta::Scale`, and up axis bounds.
  * Stop `Delta` at boundary instead of partially clamping it when `Clamp::loops()` is reached.
  * Fix `Delta::Track` gliding in world space instead of camera space.
  * Return `Bound` planes in world space wrt `Bound::transform` and distance planes wrt target.
  * Define heading of `Bound` up axis by pitch axis which is well-defined at the poles.
  * Fix `Delta::inverse()` of `Delta::Scale` by inverting ratio and scaling point.
  * Fix `Plane::project_vector()` and `Plane::project_axis()` to disregard bias.
  * Replace `rkyv` feature with `rkyv-08` feature forwarding to `glam/rkyv-08` as `glam/rkyv` is
    deprecated.
  * Mark `Delta` as non-exhaustive and rename its fields to `rotation`, `point`, `vector`, and
    `ratio`.
  * Rename `Scope::scale()` and `Scope::set_scale()` to `Scope::inspect()` and
    `Scope::set_inspect()`.
  * Make computations of `Image` private except `Image::compute()` which now considers changes
    by `Image::set_max()`, `Image::set_passive()`, and `Image::set_compute()`.
  * Rename `Image::view_isometry()` to `Image::view_rigid()` and return matrices by value.
  * Add capacity parameter to `Touch` defaulting to ten fingers.
  * Pass operands by value, e.g., `Frame`, `Scope`, `Delta`, `Plane`, and `Rigid` of `Clamp`
    methods.
  * Compute `Orbit` as rotation of a displacement starting at the trackball's pole conjugated by
    the parallel transport along the geodesic to its actual start position.
  * Fix `Orbit` ignoring (Rust) or distorting (C) displacements starting at the screen's center.
  * Rotate `Orbit` and `First` by displacement length over the same trackball radius that maps
    start positions, i.e., by a factor of π/2 faster, keeping the trackball's point under the
    cursor on radial displacements.
  * Use type-generic math of `<tgmath.h>` in C implementation so that `long double` is computed
    in its own precision instead of in `double` precision.
  * Rebuild C implementation of `cc` feature on source changes.

# Version 0.18.0 (2026-06-22)

  * Bump MSRV.
  * Bump dependencies.

# Version 0.17.1 (2025-10-19)

  * Add `Image::set_passive()`.

# Version 0.17.0 (2025-10-05)

  * Bump MSRV.
  * Bump dependencies.

# Version 0.16.0 (2025-02-22)

  * Rust Edition 2024.
  * Bump MSRV.
  * Bump dependencies.
  * Fix build with `cc` feature.

# Version 0.15.0 (2024-10-29)

  * Support latest `glam`.
  * Bump `nalgebra` to latest version.
  * Bump MSRV.

# Version 0.14.0 (2024-07-06)

  * Bump `nalgebra` to latest version.

# Version 0.13.0 (2024-06-12)

  * Bump dependencies.

# Version 0.12.0 (2024-02-21)

  * Support latest `glam`.
  * Implement `Delta::Track`, changing behavior of `Frame::set_target`.

# Version 0.11.2 (2023-08-19)

  * Make some types `Copy`.
  * Fix `rkyv` by implementing `Archive` for `Self`.
  * Enable `rkyv` for `docsrs` again.

# Version 0.11.1 (2023-08-09)

  * Fix scale boundary in `Scope::scale()` mode.
  * Disable `rkyv` for `docsrs`.

# Version 0.11.0 (2023-08-08)

  * Refactor `Clamp` as trait realizing gliding along a boundary `Plane`. This
    is implemented for `Delta::Orbit` and `Delta::Slide`. Other variants
    currently just stop the movement.
  * Implement boundary `Plane` and projecting points onto it.
  * Implement `Clamp` with `Bound` defining orthogonal boundary conditions.
  * Add `cast` methods to cast between `f32` and `f64`.
  * Rename `Scene` to `Scope`.

# Version 0.10.0 (2023-07-25)

  * Support `serde` and `rkyv`.
  * Switch to `MIT OR Apache-2.0`.
  * Add `Delta` transform.
  * Support and re-export `approx`.
  * Add Lerp/Slerp.
  * Remove Euler angles.
  * Avoid panic on unknown finger ID.
  * Support `bevy`:
     * Update scene defaults (e.g., clip planes).
     * Invert view transformation.
     * Support `glam` conversions.

# Version 0.9.0 (2022-05-05)

  * Bump `nalgebra` to latest version.

# Version 0.8.1 (2022-03-22)

  * Re-export `nalgebra`.

# Version 0.8.0 (2022-01-07)

  * Bump `nalgebra` to latest version.

# Version 0.7.0 (2021-11-12)

  * Bump `nalgebra` to latest version.
  * Use latest edition.

# Version 0.6.0 (2021-08-24)

  * Prepare for latest `nalgebra`.
  * Make crate `no_std` by limiting `Touch` to ten fingers.

# Version 0.5.1 (2021-05-09)

  * Fix `Image::compute_inverse_transformation()`. By [@Graph-Donte-Crypto].

# Version 0.5.0 (2021-04-26)

  * Adhere to lints except preferring hard tabs.
  * Use move semantics whenever otherwise cloning borrowed method arguments.
  * Reorder arguments of `Frame::look_at()` matching `Frame::set_eye()`.

# Version 0.4.0 (2021-04-23)

  * Add `First` person view.
  * Fix `Frame::local_orbit_at()` and `Frame::orbit_at()`.
  * Use `around` over `at` for scale/orbit operation.

# Version 0.3.0 (2021-04-13)

  * Add `Fixed` quantity wrt field of view.
  * Update dependencies.

# Version 0.2.3 (2021-04-08)

  * Switch to [BSD-2-Clause-Patent](LICENSES/BSD-2-Clause-Patent.md).
  * Ignore resize events with unchanged screen size.
  * Rephrase `Frame` documentation.

# Version 0.2.2 (2021-03-31)

  * Add clamp operation handler.

# Version 0.2.1 (2021-03-28)

  * Fix image distortion when resizing.
  * Fix documentation.

# Version 0.2.0 (2021-03-27)

  * Add several operation handlers.

# Version 0.1.2 (2021-03-12)

  * Fix zero literal suffixes in [C11 implementation](c11).
  * Use `num_f`, `num_d`, and `num_l` type definitions.
  * Use `None` for identity quaternion.
  * Update build script.
  * Update [README.md](README.md).

# Version 0.1.1 (2021-03-11)

  * Reliably build documentation at <https://doc.qu1x.dev/trackball>.
  * Clamp cursor/finger position between zero and maximum position.
  * Add identical [C11 implementation](c11).

# Version 0.1.0 (2021-03-06)

  * Add orbit operation handler.

[@Graph-Donte-Crypto]: https://github.com/Graph-Donte-Crypto
