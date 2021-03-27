use crate::{Frame, Scene};
use nalgebra::{convert, zero, Isometry3, Matrix4, Point2, Point3, RealField, Vector2, Vector3};

/// Image as projection of [`Scene`] wrt [`Frame`].
#[derive(Debug, Clone, Copy)]
pub struct Image<N: RealField> {
	/// Current position in screen space of hovering input or pointing device.
	pos: Point2<N>,
	/// Maximum position in screen space as screen's width and height.
	max: Point2<N>,
	/// Cached unit per pixel on focus plane to scale/project positions/vectors onto focus plane.
	upp: N,
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
}

impl<N: RealField> Image<N> {
	/// Computes initial transformations from frame, scene, and screen's width and height.
	pub fn new(frame: &Frame<N>, scene: &Scene<N>, max: &Point2<N>) -> Self {
		let mut image = Self {
			pos: Point2::origin(),
			max: max.clone(),
			upp: zero(),
			view_iso: Isometry3::identity(),
			view_mat: zero(),
			proj_mat: zero(),
			proj_view_mat: zero(),
			proj_view_inv: zero(),
		};
		image.compute_view(frame);
		image.compute_projection(frame, scene);
		image.compute_transformation();
		let _is_ok = image.compute_inverse_transformation();
		image
	}
	/// Current position in screen space of hovering input or pointing device.
	pub fn pos(&self) -> &Point2<N> {
		&self.pos
	}
	/// Sets current position in screen space of hovering input or pointing device.
	pub fn set_pos(&mut self, pos: &Point2<N>) {
		self.pos = pos.clone();
	}
	/// Maximum position in screen space as screen's width and height.
	pub fn max(&self) -> &Point2<N> {
		&self.max
	}
	/// Sets maximum position in screen space as screen's width and height.
	pub fn set_max(&mut self, max: &Point2<N>) {
		self.max = max.clone();
	}
	/// Cached unit per pixel on focus plane to scale/project positions/vectors onto focus plane.
	pub fn upp(&self) -> N {
		self.upp
	}
	/// Cached view isometry.
	pub fn view_isometry(&self) -> &Isometry3<N> {
		&self.view_iso
	}
	/// Cached view matrix.
	pub fn view(&self) -> &Matrix4<N> {
		&self.view_mat
	}
	/// Computes view isometry and matrix from frame wrt camera eye and target.
	pub fn compute_view(&mut self, frame: &Frame<N>) {
		self.view_iso = frame.view();
		self.view_mat = self.view_iso.to_homogeneous();
	}
	/// Cached projection matrix.
	pub fn projection(&self) -> &Matrix4<N> {
		&self.proj_mat
	}
	/// Computes projection matrix and unit per pixel on focus plane.
	pub fn compute_projection(&mut self, frame: &Frame<N>, scene: &Scene<N>) {
		let (mat, upp) = scene.projection(frame.distance(), &self.max);
		self.upp = upp;
		self.proj_mat = mat;
	}
	/// Cached projection view matrix.
	pub fn transformation(&self) -> &Matrix4<N> {
		&self.proj_view_mat
	}
	/// Computes projection view matrix.
	pub fn compute_transformation(&mut self) {
		self.proj_view_mat = self.proj_mat * self.view_mat
	}
	/// Cached inverse projection view matrix.
	pub fn inverse_transformation(&self) -> &Matrix4<N> {
		&self.proj_view_inv
	}
	/// Computes inverse of projection view matrix.
	#[must_use = "return value is `true` on success"]
	pub fn compute_inverse_transformation(&mut self) -> bool {
		self.proj_view_mat.try_inverse_mut()
	}
	/// Clamps position in screen space wrt its maximum in screen space.
	pub fn clamp_pos_wrt_max(pos: &Point2<N>, max: &Point2<N>) -> Point2<N> {
		Point2::new(pos.x.clamp(N::zero(), max.x), pos.y.clamp(N::zero(), max.y))
	}
	/// Clamps position in screen space.
	pub fn clamp_pos(&self, pos: &Point2<N>) -> Point2<N> {
		Self::clamp_pos_wrt_max(pos, &self.max)
	}
	/// Transforms position from screen to camera space wrt its maximum in screen space.
	pub fn transform_pos_wrt_max(pos: &Point2<N>, max: &Point2<N>) -> (Point2<N>, Point2<N>) {
		let max = max * convert(0.5);
		(Point2::new(pos.x - max.x, max.y - pos.y), max)
	}
	/// Transforms position from screen to camera space.
	pub fn transform_pos(&self, pos: &Point2<N>) -> Point2<N> {
		Self::transform_pos_wrt_max(pos, &self.max).0
	}
	/// Transforms vector from screen to camera space.
	pub fn transform_vec(pos: &Vector2<N>) -> Vector2<N> {
		Vector2::new(pos.x, -pos.y)
	}
	/// Transforms position from screen to camera space and projects it onto focus plane.
	pub fn project_pos(&self, pos: &Point2<N>) -> Point3<N> {
		self.transform_pos(pos)
			.coords
			.scale(self.upp)
			.push(N::zero())
			.into()
	}
	/// Transforms vector from screen to camera space and projects it onto focus plane.
	pub fn project_vec(&self, vec: &Vector2<N>) -> Vector3<N> {
		Self::transform_vec(vec).scale(self.upp).push(N::zero())
	}
}
