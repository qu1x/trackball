use nalgebra::{convert, Point2, RealField, Unit, Vector2};
use std::collections::BTreeMap;

/// Touch gestures inducing slide, orbit, scale, and focus.
///
/// Implements [`Default`] and can be created with `Scene::default()`.
///
/// Both its methods must be invoked on matching events fired by your 3D graphics library of choice.
#[derive(Debug, Clone, Default)]
pub struct Touch<F: Ord, N: RealField> {
	/// Finger positions ordered by finger IDs.
	pos: BTreeMap<F, Point2<N>>,
	/// Centroid position and cached normalization of previous two-finger vector.
	vec: Option<(Unit<Vector2<N>>, N)>,
	/// Centroid position of potential finger tap gesture.
	tap: Option<(usize, Point2<N>)>,
	/// Number of total finger moves per potential finger tap gesture.
	mvs: usize,
}

impl<F: Ord, N: RealField> Touch<F, N> {
	/// Computes centroid position, roll angle, and scale ratio from finger gestures.
	///
	/// Parameters are:
	///
	///   * `fid` as generic finger ID like `Some(id)` for touch and `None` for mouse events,
	///   * `pos` as current cursor/finger position in screen space,
	///   * `mvs` as number of finger moves for debouncing potential finger tap gesture with zero
	///     resulting in no delay of non-tap gestures while tap gesture can still be recognized. Use
	///     zero unless tap gestures are hardly recognized.
	///
	/// Returns number of fingers, centroid position, roll angle, and scale ratio in screen space in
	/// the order mentioned or `None` when debouncing tap gesture with non-vanishing `mvs`. See
	/// [`Self::discard()`] for tap gesture result.
	pub fn compute(
		&mut self,
		fid: F,
		pos: &Point2<N>,
		mvs: usize,
	) -> Option<(usize, Point2<N>, N, N)> {
		// Insert or update finger position.
		let _old_pos = self.pos.insert(fid, pos.clone());
		// Current number of fingers.
		let num = self.pos.len();
		// Maximum number of fingers seen per potential tap.
		let max = self.tap.map_or(1, |(tap, _pos)| tap).max(num);
		// Centroid position.
		let pos = self
			.pos
			.values()
			.map(|pos| pos.coords)
			.sum::<Vector2<N>>()
			// TODO Is this still a generic integer-to-float cast? Way to avoid concrete type?
			.unscale(convert(num as f64))
			.into();
		// Cancel potential tap if more moves than number of finger starts plus optional number of
		// moves per finger for debouncing tap gesture. Debouncing would delay non-tap gestures.
		if self.mvs >= max + mvs * max {
			// Make sure to not resume cancelled tap when fingers are discarded.
			self.mvs = std::usize::MAX;
			// Cancel potential tap.
			self.tap = None;
		} else {
			// Count total moves per potential tap.
			self.mvs += 1;
			// Insert or update potential tap as long as fingers are not discarded.
			if num >= max {
				self.tap = Some((num, pos));
			}
		}
		// Inhibit finger gestures for given number of moves per finger. No delay with zero `mvs`.
		if self.mvs >= mvs * max {
			// Identity roll angle and scale ratio.
			let (rot, rat) = (N::zero(), N::one());
			// Roll and scale only with two-finger gesture, otherwise orbit or slide via centroid.
			if num == 2 {
				// Position of first and second finger.
				let mut val = self.pos.values();
				let one_pos = val.next().unwrap();
				let two_pos = val.next().unwrap();
				// Ray and its length pointing from first to second finger.
				let (new_ray, new_len) = Unit::new_and_get(two_pos - one_pos);
				// Get old and replace with new vector.
				if let Some((old_ray, old_len)) = self.vec.replace((new_ray, new_len)) {
					// Roll angle in opposite direction at centroid.
					let rot = old_ray.perp(&new_ray).atan2(old_ray.dot(&new_ray));
					// Scale ratio at centroid.
					let rat = old_len / new_len;
					// Induced two-finger slide, roll, and scale.
					Some((num, pos, rot, rat))
				} else {
					// Start position of slide.
					Some((num, pos, rot, rat))
				}
			} else {
				// Induced one-finger or more than two-finger orbit or slide.
				Some((num, pos, rot, rat))
			}
		} else {
			// Gesture inhibited.
			None
		}
	}
	/// Removes finger position and returns number of fingers and centroid position of tap gesture.
	///
	/// Returns `None` as long as there are finger positions or no tap gesture has been recognized.
	///
	/// Panics if generic finger ID `fid` is unknown.
	pub fn discard(&mut self, fid: F) -> Option<(usize, Point2<N>)> {
		self.pos.remove(&fid).expect("Unknown touch ID");
		self.vec = None;
		if self.pos.is_empty() {
			self.mvs = 0;
			self.tap.take()
		} else {
			None
		}
	}
}
