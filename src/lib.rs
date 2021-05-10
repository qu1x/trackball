//! Virtual Trackball Orbiting via the Exponential Map
//!
//! This is an alternative trackball technique using exponential map and parallel transport to
//! preserve distances and angles for inducing coherent and intuitive trackball rotations. For
//! instance, displacements on straight radial lines through the screen's center are carried to arcs
//! of the same length on great circles of the trackball. This is in contrast to state-of-the-art
//! techniques using orthogonal projection which distorts radial distances further away from the
//! screen's center. This implementation strictly follows the recipe given in the paper of
//! Stantchev, G.. “Virtual Trackball Modeling and the Exponential Map.” . [S2CID] [44199608].
//!
//! [S2CID]: https://en.wikipedia.org/wiki/S2CID_(identifier)
//! [44199608]: https://api.semanticscholar.org/CorpusID:44199608
//!
//! # Features
//!
//!   * Common trackball operations split into several operation handlers.
//!   * Coherent and intuitive orbiting via the exponential map, see [`Orbit`] operation handler.
//!   * Identical C11 implementation for [`Orbit`] operation handler behind `cc` feature gate.
//!   * Coherent [`First`] person view aka free look or mouse look wrt [`Orbit`] operation handler.
//!   * Observer [`Frame`] with [`Frame::slide()`], [`Frame::orbit()`], [`Frame::scale()`]
//!     operations in world space and their local complements in camera space and with orbit and
//!     slide operations around arbitrary points in either world or camera space.
//!   * [`Clamp`] operation handler ensuring user boundary conditions of observer [`Frame`].
//!   * Object inspection mode scaling clip plane distances by measuring from target instead of eye.
//!   * Scale-preserving transitioning between orthographic and perspective projection mode.
//!   * Converting between [`Fixed`] quantities wrt to field of view, see [`Scene::set_fov()`].
//!   * Time-free [`Touch`] gesture recognition for slide, orbit, scale, and focus operations.
//!
//! # Example
//!
//! A trackball camera mode implementation can be as easy as this by delegating events of your 3D
//! graphics library of choice to the [`Orbit`] operation handler along with other handlers.
//!
//! ```
//! use nalgebra::{Point2, Vector3};
//! use trackball::{Frame, Image, Orbit};
//!
//! /// Trackball camera mode.
//! pub struct Trackball {
//! 	// Frame wrt camera eye and target.
//! 	frame: Frame<f32>,
//! 	// Image as projection of `Scene` wrt `Frame`.
//! 	image: Image<f32>,
//! 	// Orbit induced by displacement on screen.
//! 	orbit: Orbit<f32>,
//! }
//!
//! impl Trackball {
//! 	// Usually, a cursor position event with left mouse button being pressed.
//! 	fn handle_left_button_displacement(&mut self, pos: &Point2<f32>) {
//! 		// Maximum position as screen's width and height.
//! 		let max = self.image.max();
//! 		// Induced rotation in camera space.
//! 		let rot = self.orbit.compute(&pos, max).unwrap_or_default();
//! 		// Apply induced rotation to local observer frame.
//! 		self.frame.local_orbit(&rot);
//! 	}
//! 	// Event when left mouse button is released again.
//! 	fn handle_left_button_release(&mut self) {
//! 		// Can also or instead be invoked on `Self::handle_left_button_press()`.
//! 		self.orbit.discard();
//! 	}
//! }
//! ```

#![forbid(missing_docs)]
#![allow(clippy::tabs_in_doc_comments)]

mod clamp;
mod first;
mod frame;
mod image;
mod orbit;
mod scale;
mod scene;
mod slide;
mod touch;

pub use clamp::*;
pub use first::*;
pub use frame::*;
pub use image::*;
pub use orbit::*;
pub use scale::*;
pub use scene::*;
pub use slide::*;
pub use touch::*;
