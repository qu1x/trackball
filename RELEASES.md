# Version 0.11.2 (2023-08-16)

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
