use crate::{Frame, Rigid, Scope};
use glam::{Mat4, Vec2, Vec3};

/// Image as projection of [`Scope`] wrt [`Frame`].
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
	feature = "rkyv-08",
	derive(rkyv_08::Archive, rkyv_08::Serialize, rkyv_08::Deserialize),
	rkyv(crate = rkyv_08)
)]
pub struct Image {
    /// Current position in screen space of hovering input or pointing device.
    pos: Vec2,
    /// Maximum position in screen space as screen's width and height.
    max: Vec2,
    /// Cached unit per pixel on focus plane to scale/project positions/vectors onto focus plane.
    upp: f32,
    /// Cached previous frame or `None` to recompute view.
    frame: Option<Frame>,
    /// Cached previous scope or `None` to recompute projection.
    scope: Option<Scope>,
    /// Cached view transformation from camera to world space or inversely if passive.
    view_rigid: Rigid,
    /// Cached homogeneous view matrix computed from view transformation.
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
    pub fn new(frame: Frame, scope: Scope, max: Vec2) -> Self {
        let mut image = Self {
            pos: Vec2::ZERO,
            max,
            upp: 0.0,
            frame: None,
            scope: None,
            view_rigid: Rigid::IDENTITY,
            view_mat: Mat4::ZERO,
            proj_mat: Mat4::ZERO,
            proj_view_mat: Mat4::ZERO,
            proj_view_inv: Mat4::ZERO,
            compute_mat: true,
            compute_inv: true,
            use_passive: false,
        };
        if frame.distance() != 0.0 {
            image.compute(frame, scope);
        }
        image
    }
    /// Recomputes only cached matrices whose parameters have changed, see [`Self::set_compute()`].
    ///
    /// Returns `Some(true)` on success, `Some(false)` on failure, and `None` with no changes.
    pub fn compute(&mut self, frame: Frame, scope: Scope) -> Option<bool> {
        let new_view = self.frame != Some(frame);
        let new_projection = self.frame.map(|frame| frame.distance()) != Some(frame.distance())
            || self.scope != Some(scope);
        self.frame = Some(frame);
        self.scope = Some(scope);
        if new_view {
            self.view_rigid = if self.use_passive {
                frame.inverse_view()
            } else {
                frame.view()
            };
            self.view_mat = self.view_rigid.into();
        }
        if new_projection {
            (self.proj_mat, self.upp) = scope.projection_and_upp(frame.distance(), self.max);
        }
        (new_view || new_projection).then(|| {
            if self.compute_mat || self.compute_inv {
                self.proj_view_mat = self.proj_mat * self.view_mat;
            }
            if self.compute_inv {
                let det = self.proj_view_mat.determinant();
                let inv = det != 0.0 && det.is_finite();
                if inv {
                    self.proj_view_inv = self.proj_view_mat.inverse();
                }
                inv
            } else {
                true
            }
        })
    }
    /// Sets whether to compute transformation and inverse transformation with [`Self::compute()`].
    ///
    /// Default is `(true, true)`. Takes effect with next [`Self::compute()`].
    pub const fn set_compute(&mut self, compute_mat: bool, compute_inv: bool) {
        // Let `Self::compute()` recompute transformations by invalidating cached previous scope.
        if self.compute_mat != compute_mat || self.compute_inv != compute_inv {
            self.scope = None;
        }
        self.compute_mat = compute_mat;
        self.compute_inv = compute_inv;
    }
    /// Sets whether to use passive transformations.
    ///
    /// Default is `false`. Takes effect with next [`Self::compute()`].
    pub const fn set_passive(&mut self, use_passive: bool) {
        // Let `Self::compute()` recompute view by invalidating cached previous frame.
        if self.use_passive != use_passive {
            self.frame = None;
        }
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
    ///
    /// Takes effect with next [`Self::compute()`].
    pub fn set_max(&mut self, max: Vec2) {
        // Let `Self::compute()` recompute projection by invalidating cached previous scope.
        if self.max != max {
            self.scope = None;
        }
        self.max = max;
    }
    /// Cached unit per pixel on focus plane to scale/project positions/vectors onto focus plane.
    #[must_use]
    pub const fn upp(&self) -> f32 {
        self.upp
    }
    /// Cached view transformation from camera to world space or inversely if passive.
    ///
    /// See [`Self::set_passive()`].
    #[must_use]
    pub const fn view_rigid(&self) -> Rigid {
        self.view_rigid
    }
    /// Cached view matrix.
    #[must_use]
    pub const fn view(&self) -> Mat4 {
        self.view_mat
    }
    /// Cached projection matrix.
    #[must_use]
    pub const fn projection(&self) -> Mat4 {
        self.proj_mat
    }
    /// Cached projection view matrix.
    #[must_use]
    pub const fn transformation(&self) -> Mat4 {
        self.proj_view_mat
    }
    /// Cached inverse projection view matrix.
    #[must_use]
    pub const fn inverse_transformation(&self) -> Mat4 {
        self.proj_view_inv
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
