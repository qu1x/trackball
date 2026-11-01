use crate::{Frame, Rigid, Scope};
use glam::{Mat4, Vec2, Vec3};

/// Image as projection of [`Scope`] wrt [`Frame`].
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
	feature = "rkyv",
	derive(rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)
)]
pub struct Image {
	/// Current position in screen space of hovering input or pointing device.
	pos: Vec2,
	/// Maximum position in screen space as screen's width and height.
	max: Vec2,
	/// Cached unit per pixel on focus plane to scale/project positions/vectors onto focus plane.
	upp: f32,
	/// Cached previous frame.
	frame: Frame,
	/// Cached previous scope.
	scope: Scope,
	/// Cached view isometry from world to camera space coinciding with right-handed look-at space.
	view_iso: Rigid,
	/// Cached homogeneous view matrix computed from view isometry.
	view_mat: Mat4,
	/// Cached scale-identical orthographic or perspective projection matrix.
	proj_mat: Mat4,
	/// Cached transformation.
	proj_view_mat: Mat4,
	/// Cached inverse of transformation.
	proj_view_inv: Mat4,
	/// Whether to compute transformation. Default is `true`.
	compute_mat: bool,
	/// Whether to compute inverse transformation. Default is `true`.
	compute_inv: bool,
	/// Whether to use passive transformations. Default is `false`.
	use_passive: bool,
}

impl Image {
	/// Computes initial transformations from frame, scope, and screen's width and height.
	#[must_use]
	pub fn new(frame: &Frame, scope: &Scope, max: Vec2) -> Self {
		let mut image = Self {
			pos: Vec2::ZERO,
			max,
			upp: 0.0,
			frame: *frame,
			scope: *scope,
			view_iso: Rigid::IDENTITY,
			view_mat: Mat4::ZERO,
			proj_mat: Mat4::ZERO,
			proj_view_mat: Mat4::ZERO,
			proj_view_inv: Mat4::ZERO,
			compute_mat: true,
			compute_inv: true,
			use_passive: false,
		};
		if frame.distance() != 0.0 {
			image.compute_view(frame);
			image.compute_projection_and_upp(frame.distance(), scope);
			image.compute_transformation();
			image.compute_inverse_transformation();
		}
		image
	}
	/// Recomputes only cached matrices whose parameters have changed, see [`Self::set_compute()`].
	///
	/// Returns `Some(true)` on success, `Some(false)` on failure, and `None` with no changes.
	#[allow(clippy::useless_let_if_seq, clippy::float_cmp)]
	pub fn compute(&mut self, frame: Frame, scope: Scope) -> Option<bool> {
		let mut compute = false;
		if self.frame != frame {
			self.compute_view(&frame);
			compute = true;
		}
		if self.frame.distance() != frame.distance() || self.scope != scope {
			self.compute_projection_and_upp(frame.distance(), &scope);
			compute = true;
		}
		self.frame = frame;
		self.scope = scope;
		compute.then(|| {
			if self.compute_mat || self.compute_inv {
				self.compute_transformation();
			}
			if self.compute_inv {
				self.compute_inverse_transformation()
			} else {
				true
			}
		})
	}
	/// Sets whether to compute transformation and inverse transformation with [`Self::compute()`].
	///
	/// Default is `(true, true)`.
	pub const fn set_compute(&mut self, compute_mat: bool, compute_inv: bool) {
		self.compute_mat = compute_mat;
		self.compute_inv = compute_inv;
	}
	/// Sets whether to use passive transformations.
	///
	/// Default is `false`.
	pub const fn set_passive(&mut self, use_passive: bool) {
		self.use_passive = use_passive;
	}
	/// Current position in screen space of hovering input or pointing device.
	#[must_use]
	pub const fn pos(&self) -> Vec2 {
		self.pos
	}
	/// Sets current position in screen space of hovering input or pointing device.
	pub const fn set_pos(&mut self, pos: Vec2) {
		self.pos = pos;
	}
	/// Maximum position in screen space as screen's width and height.
	#[must_use]
	pub const fn max(&self) -> Vec2 {
		self.max
	}
	/// Sets maximum position in screen space as screen's width and height.
	pub fn set_max(&mut self, max: Vec2) {
		// Let `Self::compute()` recompute projection matrix by invalidating cached previous scope.
		if self.max != max {
			self.scope.set_fov(0.0);
		}
		self.max = max;
	}
	/// Cached unit per pixel on focus plane to scale/project positions/vectors onto focus plane.
	#[must_use]
	pub const fn upp(&self) -> f32 {
		self.upp
	}
	/// Cached view isometry.
	#[must_use]
	pub const fn view_isometry(&self) -> &Rigid {
		&self.view_iso
	}
	/// Cached view matrix.
	#[must_use]
	pub const fn view(&self) -> &Mat4 {
		&self.view_mat
	}
	/// Computes view isometry and matrix from frame wrt camera eye and target.
	pub fn compute_view(&mut self, frame: &Frame) {
		self.view_iso = if self.use_passive {
			frame.inverse_view()
		} else {
			frame.view()
		};
		self.view_mat = self.view_iso.to_mat4();
	}
	/// Cached projection matrix.
	#[must_use]
	pub const fn projection(&self) -> &Mat4 {
		&self.proj_mat
	}
	/// Computes projection matrix and unit per pixel on focus plane.
	pub fn compute_projection_and_upp(&mut self, zat: f32, scope: &Scope) {
		let (mat, upp) = scope.projection_and_upp(zat, self.max);
		self.upp = upp;
		self.proj_mat = mat;
	}
	/// Cached projection view matrix.
	#[must_use]
	pub const fn transformation(&self) -> &Mat4 {
		&self.proj_view_mat
	}
	/// Computes projection view matrix.
	pub fn compute_transformation(&mut self) {
		self.proj_view_mat = self.proj_mat * self.view_mat;
	}
	/// Cached inverse projection view matrix.
	#[must_use]
	pub const fn inverse_transformation(&self) -> &Mat4 {
		&self.proj_view_inv
	}
	/// Computes inverse of projection view matrix.
	///
	/// Returns `true` on success.
	pub fn compute_inverse_transformation(&mut self) -> bool {
		let det = self.proj_view_mat.determinant();
		let inv = det != 0.0 && det.is_finite();
		if inv {
			self.proj_view_inv = self.proj_view_mat.inverse();
		}
		inv
	}
	/// Clamps position in screen space wrt its maximum in screen space.
	#[must_use]
	pub const fn clamp_pos_wrt_max(pos: Vec2, max: Vec2) -> Vec2 {
		Vec2::new(pos.x.clamp(0.0, max.x), pos.y.clamp(0.0, max.y))
	}
	/// Clamps position in screen space.
	#[must_use]
	pub const fn clamp_pos(&self, pos: Vec2) -> Vec2 {
		Self::clamp_pos_wrt_max(pos, self.max)
	}
	/// Transforms position and its maximum from screen to camera space wrt its maximum.
	#[must_use]
	pub fn transform_pos_and_max_wrt_max(pos: Vec2, max: Vec2) -> (Vec2, Vec2) {
		let max = max * 0.5;
		(Vec2::new(pos.x - max.x, max.y - pos.y), max)
	}
	/// Transforms position from screen to camera space.
	#[must_use]
	pub fn transform_pos(&self, pos: Vec2) -> Vec2 {
		Self::transform_pos_and_max_wrt_max(pos, self.max).0
	}
	/// Transforms vector from screen to camera space.
	#[must_use]
	pub const fn transform_vec(vec: Vec2) -> Vec2 {
		Vec2::new(vec.x, -vec.y)
	}
	/// Transforms position from screen to camera space and projects it onto focus plane.
	#[must_use]
	pub fn project_pos(&self, pos: Vec2) -> Vec3 {
		(self.transform_pos(pos) * self.upp).extend(0.0)
	}
	/// Transforms vector from screen to camera space and projects it onto focus plane.
	#[must_use]
	pub fn project_vec(&self, vec: Vec2) -> Vec3 {
		(Self::transform_vec(vec) * self.upp).extend(0.0)
	}
}
