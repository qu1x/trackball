use crate::{Frame, Scope};
use nalgebra::{convert, zero, Isometry3, Matrix4, Point2, Point3, RealField, Vector2, Vector3};
use simba::scalar::SubsetOf;

/// Image as projection of [`Scope`] wrt [`Frame`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Image<N: Copy + RealField> {
	/// Current position in screen space of hovering input or pointing device.
	pos: Point2<N>,
	/// Maximum position in screen space as screen's width and height.
	max: Point2<N>,
	/// Cached unit per pixel on focus plane to scale/project positions/vectors onto focus plane.
	upp: N,
	/// Cached previous frame.
	frame: Frame<N>,
	/// Cached previous scope.
	scope: Scope<N>,
	/// Cached view isometry from world to camera space coinciding with right-handed look-at space.
	view_iso: Isometry3<N>,
	/// Cached homogeneous view matrix computed from view isometry.
	view_mat: Matrix4<N>,
	/// Cached scale-identical orthographic or perspective projection matrix.
	proj_mat: Matrix4<N>,
	/// Cached transformation.
	proj_view_mat: Matrix4<N>,
	/// Cached inverse of transformation.
	proj_view_inv: Matrix4<N>,
	/// Whether to compute transformation. Default is `true`.
	compute_mat: bool,
	/// Whether to compute inverse transformation. Default is `true`.
	compute_inv: bool,
}

impl<N: Copy + RealField> Image<N> {
	/// Computes initial transformations from frame, scope, and screen's width and height.
	#[must_use]
	pub fn new(frame: &Frame<N>, scope: &Scope<N>, max: Point2<N>) -> Self {
		let mut image = Self {
			pos: Point2::origin(),
			max,
			upp: zero(),
			frame: *frame,
			scope: *scope,
			view_iso: Isometry3::identity(),
			view_mat: zero(),
			proj_mat: zero(),
			proj_view_mat: zero(),
			proj_view_inv: zero(),
			compute_mat: true,
			compute_inv: true,
		};
		image.compute_view(frame);
		image.compute_projection_and_upp(frame.distance(), scope);
		image.compute_transformation();
		image.compute_inverse_transformation();
		image
	}
	/// Recomputes only cached matrices whose parameters have changed, see [`Self::set_compute()`].
	///
	/// Returns `Some(true)` on success, `Some(false)` on failure, and `None` with no changes.
	#[allow(clippy::useless_let_if_seq)]
	pub fn compute(&mut self, frame: Frame<N>, scope: Scope<N>) -> Option<bool> {
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
	pub fn set_compute(&mut self, compute_mat: bool, compute_inv: bool) {
		self.compute_mat = compute_mat;
		self.compute_inv = compute_inv;
	}
	/// Current position in screen space of hovering input or pointing device.
	#[must_use]
	pub const fn pos(&self) -> &Point2<N> {
		&self.pos
	}
	/// Sets current position in screen space of hovering input or pointing device.
	pub fn set_pos(&mut self, pos: Point2<N>) {
		self.pos = pos;
	}
	/// Maximum position in screen space as screen's width and height.
	#[must_use]
	pub const fn max(&self) -> &Point2<N> {
		&self.max
	}
	/// Sets maximum position in screen space as screen's width and height.
	pub fn set_max(&mut self, max: Point2<N>) {
		// Let `Self::compute()` recompute projection matrix by invalidating cached previous scope.
		if self.max != max {
			self.scope.set_fov(N::zero());
		}
		self.max = max;
	}
	/// Cached unit per pixel on focus plane to scale/project positions/vectors onto focus plane.
	#[must_use]
	pub const fn upp(&self) -> N {
		self.upp
	}
	/// Cached view isometry.
	#[must_use]
	pub const fn view_isometry(&self) -> &Isometry3<N> {
		&self.view_iso
	}
	/// Cached view matrix.
	#[must_use]
	pub const fn view(&self) -> &Matrix4<N> {
		&self.view_mat
	}
	/// Computes view isometry and matrix from frame wrt camera eye and target.
	pub fn compute_view(&mut self, frame: &Frame<N>) {
		self.view_iso = frame.view();
		self.view_mat = self.view_iso.to_homogeneous();
	}
	/// Cached projection matrix.
	#[must_use]
	pub const fn projection(&self) -> &Matrix4<N> {
		&self.proj_mat
	}
	/// Computes projection matrix and unit per pixel on focus plane.
	pub fn compute_projection_and_upp(&mut self, zat: N, scope: &Scope<N>) {
		let (mat, upp) = scope.projection_and_upp(zat, &self.max);
		self.upp = upp;
		self.proj_mat = mat;
	}
	/// Cached projection view matrix.
	#[must_use]
	pub const fn transformation(&self) -> &Matrix4<N> {
		&self.proj_view_mat
	}
	/// Computes projection view matrix.
	pub fn compute_transformation(&mut self) {
		self.proj_view_mat = self.proj_mat * self.view_mat;
	}
	/// Cached inverse projection view matrix.
	#[must_use]
	pub const fn inverse_transformation(&self) -> &Matrix4<N> {
		&self.proj_view_inv
	}
	/// Computes inverse of projection view matrix.
	///
	/// Returns `true` on success.
	pub fn compute_inverse_transformation(&mut self) -> bool {
		let inv = self.proj_view_mat.try_inverse();
		if let Some(mat) = inv {
			self.proj_view_inv = mat;
		}
		inv.is_some()
	}
	/// Clamps position in screen space wrt its maximum in screen space.
	#[must_use]
	pub fn clamp_pos_wrt_max(pos: &Point2<N>, max: &Point2<N>) -> Point2<N> {
		Point2::new(pos.x.clamp(N::zero(), max.x), pos.y.clamp(N::zero(), max.y))
	}
	/// Clamps position in screen space.
	#[must_use]
	pub fn clamp_pos(&self, pos: &Point2<N>) -> Point2<N> {
		Self::clamp_pos_wrt_max(pos, &self.max)
	}
	/// Transforms position and its maximum from screen to camera space wrt its maximum.
	#[must_use]
	pub fn transform_pos_and_max_wrt_max(
		pos: &Point2<N>,
		max: &Point2<N>,
	) -> (Point2<N>, Point2<N>) {
		let max = max * convert(0.5);
		(Point2::new(pos.x - max.x, max.y - pos.y), max)
	}
	/// Transforms position from screen to camera space.
	#[must_use]
	pub fn transform_pos(&self, pos: &Point2<N>) -> Point2<N> {
		Self::transform_pos_and_max_wrt_max(pos, &self.max).0
	}
	/// Transforms vector from screen to camera space.
	#[must_use]
	pub fn transform_vec(pos: &Vector2<N>) -> Vector2<N> {
		Vector2::new(pos.x, -pos.y)
	}
	/// Transforms position from screen to camera space and projects it onto focus plane.
	#[must_use]
	pub fn project_pos(&self, pos: &Point2<N>) -> Point3<N> {
		self.transform_pos(pos)
			.coords
			.scale(self.upp)
			.push(N::zero())
			.into()
	}
	/// Transforms vector from screen to camera space and projects it onto focus plane.
	#[must_use]
	pub fn project_vec(&self, vec: &Vector2<N>) -> Vector3<N> {
		Self::transform_vec(vec).scale(self.upp).push(N::zero())
	}
	/// Casts components to another type, e.g., between [`f32`] and [`f64`].
	#[must_use]
	pub fn cast<M: Copy + RealField>(self) -> Image<M>
	where
		N: SubsetOf<M>,
	{
		Image {
			pos: self.pos.cast(),
			max: self.max.cast(),
			upp: self.upp.to_superset(),
			frame: self.frame.cast(),
			scope: self.scope.cast(),
			view_iso: self.view_iso.cast(),
			view_mat: self.view_mat.cast(),
			proj_mat: self.proj_mat.cast(),
			proj_view_mat: self.proj_view_mat.cast(),
			proj_view_inv: self.proj_view_inv.cast(),
			compute_mat: self.compute_mat,
			compute_inv: self.compute_inv,
		}
	}
}

#[cfg(feature = "rkyv")]
impl<N: Copy + RealField> rkyv::Archive for Image<N> {
	type Archived = Self;
	type Resolver = ();

	#[inline]
	#[allow(unsafe_code)]
	unsafe fn resolve(&self, _: usize, (): Self::Resolver, out: *mut Self::Archived) {
		out.write(rkyv::to_archived!(*self as Self));
	}
}

#[cfg(feature = "rkyv")]
impl<Ser: rkyv::Fallible + ?Sized, N: Copy + RealField> rkyv::Serialize<Ser> for Image<N> {
	#[inline]
	fn serialize(&self, _: &mut Ser) -> Result<Self::Resolver, Ser::Error> {
		Ok(())
	}
}

#[cfg(feature = "rkyv")]
impl<De: rkyv::Fallible + ?Sized, N: Copy + RealField> rkyv::Deserialize<Self, De> for Image<N> {
	#[inline]
	fn deserialize(&self, _: &mut De) -> Result<Self, De::Error> {
		Ok(rkyv::from_archived!(*self))
	}
}
