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
[Rust]: https://img.shields.io/badge/rust-v1.79.0-brightgreen.svg
[License]: https://img.shields.io/badge/License-MIT%20OR%20Apache--2.0-blue.svg

This is an alternative trackball technique using exponential map and parallel transport to
preserve distances and angles for inducing coherent and intuitive trackball rotations. For
instance, displacements on straight radial lines through the screen's center are carried to arcs
of the same length on great circles of the trackball. This is in contrast to state-of-the-art
techniques using orthogonal projection which distorts radial distances further away from the
screen's center. This implementation strictly follows the recipe given in the paper of
Stantchev, G.. “Virtual Trackball Modeling and the Exponential Map.” . [S2CID] [44199608].

[S2CID]: https://en.wikipedia.org/wiki/S2CID_(identifier)
[44199608]: https://api.semanticscholar.org/CorpusID:44199608

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

See the [release history](RELEASES.md) to keep track of the development.

## Example

A trackball camera mode implementation can be as easy as this by delegating events of your 3D
graphics library of choice to the [`Orbit`] operation handler along with other handlers.

```rust
use trackball::{
	nalgebra::{Point2, Vector3},
	Frame, Image, Orbit,
};

/// Trackball camera mode.
pub struct Trackball {
	// Frame wrt camera eye and target.
	frame: Frame<f32>,
	// Image as projection of `Scope` wrt `Frame`.
	image: Image<f32>,
	// Orbit induced by displacement on screen.
	orbit: Orbit<f32>,
}

impl Trackball {
	// Usually, a cursor position event with left mouse button being pressed.
	fn handle_left_button_displacement(&mut self, pos: &Point2<f32>) {
		// Maximum position as screen's width and height.
		let max = self.image.max();
		// Induced rotation in camera space.
		let rot = self.orbit.compute(&pos, max).unwrap_or_default();
		// Apply induced rotation to local observer frame.
		self.frame.local_orbit(&rot);
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

Copyright © 2021-2024 Rouven Spreckels <rs@qu1x.dev>

This project is licensed under either of

 * Apache License, Version 2.0, ([LICENSES/Apache-2.0](LICENSES/Apache-2.0) or
   https://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSES/MIT](LICENSES/MIT) or https://opensource.org/licenses/MIT)

at your option.

# Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
this project by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
