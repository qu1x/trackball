# trackball

Virtual Trackball Orbiting via the Exponential Map

[![Build Status][]](https://travis-ci.org/qu1x/trackball)
[![Downloads][]](https://crates.io/crates/trackball)
[![Rust][]](https://www.rust-lang.org)
[![Version][]](https://crates.io/crates/trackball)
[![Documentation][]](https://doc.qu1x.dev/trackball)
[![License][]](https://opensource.org/licenses/BSD-3-Clause)

[Build Status]: https://travis-ci.org/qu1x/trackball.svg
[Downloads]: https://img.shields.io/crates/d/trackball.svg
[Rust]: https://img.shields.io/badge/rust-stable-brightgreen.svg
[Version]: https://img.shields.io/crates/v/trackball.svg
[Documentation]: https://docs.rs/trackball/badge.svg
[License]: https://img.shields.io/crates/l/trackball.svg

This is an alternative trackball technique using exponential map and parallel transport to
preserve distances and angles for inducing coherent and intuitive trackball rotations. For
instance, displacements on straight radial lines through the screen's center are carried to arcs
of the same length on great circles of the trackball. This is in contrast to state-of-the-art
techniques using orthogonal projection which distorts radial distances further away from the
screen's center. This implementation strictly follows the recipe given in the paper of
Stantchev, G.. “Virtual Trackball Modeling and the Exponential Map.” . [S2CID] [44199608].

[S2CID]: https://en.wikipedia.org/wiki/S2CID_(identifier)
[44199608]: https://api.semanticscholar.org/CorpusID:44199608

## Status

Currently only [`nalgebra`] is supported as underlying linear algebra library but others will be
supported behind feature gates so that only your library of choice becomes a dependency. The
[`Orbit`] operation handler will be complemented with other handlers for common trackball camera
mode operations like slide, scale, and focus. Projection view matrices will be computed as well
with scale preserving transitions between orthographic and perspective projection mode.

[`nalgebra`]: https://doc.qu1x.dev/trackball/nalgebra/index.html

## Example

A trackball camera mode implementation can be as easy as this by delegating events of your 3D
graphics library of choice to the [`Orbit`] operation handler along with other handlers for
common trackball camera mode operations like slide, scale, and focus.

[`Orbit`]: https://doc.qu1x.dev/trackball/trackball/struct.Orbit.html

```rust
use nalgebra::{Point2, UnitQuaternion, Vector3};
use std::f32::consts::PI;
use trackball::Orbit;

/// Trackball camera mode.
pub struct Trackball {
	// Camera eye alignment.
	align: UnitQuaternion<f32>,
	// Orbit operation handler along with slide, scale, and focus operation handlers.
	orbit: Orbit<f32>,
	// Maximum cursor/finger position as screen's width and height.
	frame: Point2<f32>,
}

impl Trackball {
	// Usually, a cursor position event with left mouse button being pressed.
	fn handle_left_button_displacement(&mut self, pos: &Point2<f32>) {
		// Optionally, do a coordinate system transformation like flipping x-axis/z-axis.
		let camera_space = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), PI);
		// Or directly apply this induced rotation.
		let rotation = self.orbit.compute(&pos, &self.frame).unwrap_or_default();
		// Post-multiply rotation to total camera alignment.
		self.align *= camera_space * rotation * camera_space.inverse();
	}
	// Event when left mouse button is released again.
	fn handle_left_button_release(&mut self) {
		// Can also or instead be invoked on `Self::handle_left_button_press()`.
		self.orbit.discard();
	}
}
```

## C11 Implementation

An identical [C11 implementation](c11) can be used instead by enabling the `cc` feature as in:

```toml
[dependencies]
trackball = { version = "0.1", features = ["cc"] }
```

## License

[BSD-3-Clause](LICENSE.md)

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion
in the works by you shall be licensed as above, without any additional terms or conditions.
