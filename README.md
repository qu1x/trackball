# trackball

Virtual Trackball Orbiting via the Exponential Map

[![Build][]](https://github.com/qu1x/trackball/actions/workflows/build.yml)
[![Documentation][]](https://docs.rs/trackball)
[![Downloads][]](https://crates.io/crates/trackball)
[![Version][]](https://crates.io/crates/trackball)
[![Rust][]](https://www.rust-lang.org)
[![License][]](https://opensource.org/licenses)

[Build]: https://github.com/qu1x/trackball/actions/workflows/build.yml/badge.svg
[Documentation]: https://docs.rs/trackball/badge.svg
[Downloads]: https://img.shields.io/crates/d/trackball.svg
[Version]: https://img.shields.io/crates/v/trackball.svg
[Rust]: https://img.shields.io/badge/rust-v1.89.0-brightgreen.svg
[License]: https://img.shields.io/badge/License-MIT%20OR%20Apache--2.0-blue.svg

This is an alternative trackball technique using the exponential map for inducing coherent and
intuitive trackball rotations.[^1] Displacements on straight radial lines through the screen's
center are carried to arcs of the same length on great circles of the trackball, whose half
circumference is the maximum of the screen's width and height. This is in contrast to
state-of-the-art techniques using orthogonal projection which distorts radial distances further
away from the screen's center. Each rotation carries the trackball's point under the previous
cursor/finger position along the shortest arc to the one under the current position, keeping the
grabbed point under the cursor/finger. This follows the paper's differential of the exponential
map rather than its recipe, which takes the differential to be parallel transport, preserving
the lengths of displacements but letting the grabbed point drift. Screen corners beyond the
trackball's border are mapped beyond its equator, keeping radial displacements linear.

[^1]: G. Stantchev, “Virtual Trackball Modeling and the Exponential Map”, [S2CID 44199608 (2004)
](https://api.semanticscholar.org/CorpusID:44199608), [Archived PDF
](https://web.archive.org/web/2/http://www.math.umd.edu:80/~gogo/Papers/trackballExp.pdf)

## Features

  * Common trackball operations split into several operation handlers.
  * Coherent and intuitive orbiting via the exponential map, see [`Orbit`] operation handler.
  * Identical C11 implementation for [`Orbit`] operation handler behind `cc` feature gate.
  * Coherent [`First`] person view aka free look or mouse look wrt [`Orbit`] operation handler.
  * Observer [`Frame`] with [`Frame::slide()`], [`Frame::orbit()`], [`Frame::scale()`]
    operations in world space and their local complements in camera space and with orbit and
    slide operations around arbitrary points in either world or camera space.
  * Gliding [`Clamp`] operation handler trait ensuring boundary conditions of observer
    [`Frame`]. When [`Delta`] between initial and final [`Frame`] is not orthogonal to a
    boundary [`Plane`], [`Delta`] is changed in such a way that the clamped movement glides
    along the plane.
  * [`Bound`] implementing [`Clamp`] providing customizable orthogonal boundary conditions.
  * Object inspection mode scaling clip plane distances by measuring from target instead of eye.
  * Scale-preserving transitioning between orthographic and perspective projection mode.
  * Converting between [`Fixed`] quantities wrt to field of view, see [`Scope::set_fov()`].
  * Time-free [`Touch`] gesture recognition for slide, orbit, scale, and focus operations.

[`Frame::slide()`]: https://docs.rs/trackball/latest/trackball/struct.Frame.html#method.slide
[`Frame::orbit()`]: https://docs.rs/trackball/latest/trackball/struct.Frame.html#method.orbit
[`Frame::scale()`]: https://docs.rs/trackball/latest/trackball/struct.Frame.html#method.scale

[`First`]: https://docs.rs/trackball/latest/trackball/struct.First.html
[`Frame`]: https://docs.rs/trackball/latest/trackball/struct.Frame.html
[`Clamp`]: https://docs.rs/trackball/latest/trackball/struct.Clamp.html
[`Delta`]: https://docs.rs/trackball/latest/trackball/struct.Delta.html
[`Bound`]: https://docs.rs/trackball/latest/trackball/struct.Bound.html
[`Plane`]: https://docs.rs/trackball/latest/trackball/struct.Plane.html
[`Scope`]: https://docs.rs/trackball/latest/trackball/struct.Scope.html
[`Touch`]: https://docs.rs/trackball/latest/trackball/struct.Touch.html

[`Fixed`]: https://docs.rs/trackball/latest/trackball/enum.Fixed.html
[`Scope::set_fov()`]: https://docs.rs/trackball/latest/trackball/struct.Scope.html#method.set_fov

## Math Backend

Like `glam`, this crate requires one of the following math backends, forwarded to `glam`:

  * `std` (default) for using the math functions of the standard library.
  * `libm` for using `libm` instead of `std`, e.g., for consistent results across platforms.
  * `nostd-libm` for using `libm` only if `std` is disabled, e.g., in `no_std` environments.

This crate is `no_std` with `default-features = false` and `libm` or `nostd-libm` enabled.

## Optional Features

Following features are disabled unless their corresponding feature gate is enabled:

  * `serde` for `serde` support of various structures of this crate and its dependencies.
  * `rkyv-08` for `rkyv` 0.8 support of various structures of this crate and its dependencies.
  * `cc` for testing the behaviorally identical C implementation of the exponential map.

See the [release history](RELEASES.md) to keep track of the development.

## Example

A trackball camera mode implementation can be as easy as this by delegating events of your 3D
graphics library of choice to the [`Orbit`] operation handler along with other handlers.

```rust
use trackball::{Frame, Image, Orbit, glam::Vec2};

/// Trackball camera mode.
pub struct Trackball {
	// Frame wrt camera eye and target.
	frame: Frame,
	// Image as projection of `Scope` wrt `Frame`.
	image: Image,
	// Orbit induced by displacement on screen.
	orbit: Orbit,
}

impl Trackball {
	// Usually, a cursor position event with left mouse button being pressed.
	fn handle_left_button_displacement(&mut self, pos: Vec2) {
		// Maximum position as screen's width and height.
		let max = self.image.max();
		// Induced rotation in camera space.
		let rot = self.orbit.compute(pos, max).unwrap_or_default();
		// Apply induced rotation to local observer frame.
		self.frame.local_orbit(rot);
	}
	// Event when left mouse button is released again.
	fn handle_left_button_release(&mut self) {
		// Can also or instead be invoked on `Self::handle_left_button_press()`.
		self.orbit.discard();
	}
}
```

## C11 Implementation

Use identical [C11 implementation](c11) for [`Orbit`] operation handler behind `cc` feature gate.

[`Orbit`]: https://docs.rs/trackball/latest/trackball/struct.Orbit.html

## License

Copyright © 2021-2026 Rouven Spreckels <rs@qu1x.dev>

This project is licensed under either of

 * Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or
   https://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or https://opensource.org/licenses/MIT)

at your option.

# Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
this project by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
