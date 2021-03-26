use crate::{Frame, Scene};
use nalgebra::{convert, zero, Point2, Point3, RealField, Vector2, Vector3};
use nalgebra::{Isometry3, Matrix4, Orthographic3, Perspective3};

/// Image as projection of [`Scene`] wrt [`Frame`].
#[derive(Debug, Clone, Copy)]
pub struct Image<N: RealField> {
	/// Current position in screen space of hovering input or pointing device.
	pub pos: Point2<N>,
	/// Maximum position in screen space as screen's width and height.
	pub max: Point2<N>,
	/// Unit per pixel on focus plane to scale slide operations and localize orbit/scale operations.
	pub upp: N,
	/// Cached view isometry from world to camera space coinciding with right-handed look-at space.
	pub view_iso: Isometry3<N>,
	/// Cached homogeneous view matrix computed from view isometry.
	pub view_mat: Matrix4<N>,
	/// Cached scale-identical orthographic or perspective projection matrix.
	pub proj_mat: Matrix4<N>,
	/// Cached transformation.
	pub proj_view_mat: Matrix4<N>,
	/// Cached inverse of transformation.
	pub proj_view_inv: Matrix4<N>,
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
		image.view(frame);
		image.projection(frame, scene);
		image.transformation();
		let _is_ok = image.inverse_transformation();
		image
	}
	/// Computes view matrix from frame wrt camera eye and target.
	pub fn view(&mut self, frame: &Frame<N>) {
		self.view_iso = frame.view();
		self.view_mat = self.view_iso.to_homogeneous();
	}
	/// Computes projection matrix and unit per pixel on focus plane.
	pub fn projection(&mut self, frame: &Frame<N>, scene: &Scene<N>) {
		let (znear, zfar) = scene.clip_planes(frame.distance());
		let aspect = self.max.x / self.max.y;
		let two = N::one() + N::one();
		let top = frame.distance() * (scene.fov() / two).tan();
		let right = aspect * top;
		self.upp = right * two / self.max.x;
		self.proj_mat = if scene.ortho() {
			Orthographic3::new(-right, right, -top, top, znear, zfar).into_inner()
		} else {
			Perspective3::new(aspect, scene.fov(), znear, zfar).into_inner()
		};
	}
	/// Computes projection view matrix.
	pub fn transformation(&mut self) {
		self.proj_view_mat = self.proj_mat * self.view_mat
	}
	/// Computes inverse of projection view matrix.
	#[must_use = "return value is `true` on success"]
	pub fn inverse_transformation(&mut self) -> bool {
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
