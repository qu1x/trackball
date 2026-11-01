use glam::Vec2;

/// Slide induced by displacement on screen.
///
/// Implements [`Default`] and can be created with `Slide::default()`.
///
/// Both its methods must be invoked on matching events fired by your 3D graphics library of choice.
#[derive(Debug, Clone, Default)]
pub struct Slide {
	/// Caches previous cursor/finger position.
	pos: Option<Vec2>,
}

impl Slide {
	/// Computes slide between previous and current cursor/finger position in screen space.
	pub fn compute(&mut self, pos: Vec2) -> Option<Vec2> {
		self.pos.replace(pos).map(|old| old - pos)
	}
	/// Discards cached previous cursor/finger position on button/finger release.
	pub const fn discard(&mut self) {
		self.pos = None;
	}
}
