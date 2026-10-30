//! The physics world: fixed-step rigid bodies with persistent contacts.
//!
//! Physical state uses [`Real`] (`f32`). Elapsed time stays [`numeric::Scalar`] (`f64`); each
//! substep narrows its `dt` to `Real` once. See docs/numerics.md for the f32 precision envelope.
//!
//! External impulses change velocity once; forces act for the requested duration. Contacts are
//! prepared once per step and reused by its bounded substeps (#378, see
//! docs/fixed-step-approximation.md); each substep accumulates clamped sequential impulses, then
//! integrates the new linear/angular velocity. Contact impulses persist for warm starting. No integer quantization,
//! rational remainder tree, event-restart loop, or whole-world sleep proxy conversion is used.
//!
//! CCD sweeps translation over each substep for fast shapes. Orientations are held fixed during
//! those sweeps and integrated between substeps, so this is NOT analytic rotational CCD.

/// Physical-state scalar. Elapsed time stays [`numeric::Scalar`] (`f64`).
pub type Real = f32;
#[allow(unused_imports)]
use std::f32::consts as real_consts;
/// Bit width of [`Real`] in this build (32 or 64).
pub const REAL_BITS: u32 = (size_of::<Real>() * 8) as u32;

/// Rounding allowance, in units of `Real::EPSILON`, for re-checking a swept impact after
/// conservative narrowing. Zero in the f64 build, where narrowing is the identity.
const NARROWING_ULPS: Real = if REAL_BITS == 64 { 0.0 } else { 16.0 };

/// Rounding-only test tolerance: the f64 reference bound, or `ulps` of `Real::EPSILON`
/// (scaled by the magnitude under test) when that is larger, as in the f32 build.
#[cfg(test)]
fn rounding_tolerance(reference: Real, ulps: Real, scale: Real) -> Real {
    reference.max(ulps * Real::EPSILON * scale)
}

/// Narrows f64 time to the solver scalar once per substep (identity in the f64 build).
#[allow(clippy::unnecessary_cast)]
fn narrow_time(seconds: Scalar) -> Real {
    seconds as Real
}
/// Widens a solver substep exactly into the f64 time domain (identity in the f64 build).
#[allow(clippy::unnecessary_cast)]
fn widen_time(seconds: Real) -> Scalar {
    seconds as Scalar
}
/// Substep length in solver precision. The division happens in f64 before one narrowing.
fn substep_length(dt: Scalar, substeps: u8) -> Real {
    narrow_time(dt / Scalar::from(substeps))
}

mod bookkeeping;
mod contact;
mod mass;
mod position;
mod primitive;
pub use mass::{MassProperties, MassPropertiesError};
mod query;
pub use position::PositionReport;
pub use query::{
    QueryFailure, QueryFailureReason, QueryFilter, QueryHit, QueryPose, QueryStats, RayCast,
    RayFeature, RayHit, ShapeCast,
};
mod convergence;
mod islands;
pub use convergence::{Convergence, ConvergenceStats};
pub use islands::{ConvergenceScope, IslandStats};
mod geometry;
pub use bookkeeping::BookkeepingStats;
use bookkeeping::{Scratch, push, reserve};
use geometry::GeometryCache;
pub use geometry::GeometryStats;
mod ccd;
mod character;
mod terrain;
pub use character::{Autostep, CharacterMove, CharacterMoveResult, CharacterSettings, Embedded};
pub use terrain::{HeightfieldDesc, TerrainId};
mod math;
mod response;
use crate::{BodyId, CollisionLayers3d, numeric};
pub use math::{Quaternion, Vector};
use numeric::Scalar;
use response::PreparedResponse;
use std::collections::BTreeMap;
mod transaction;
pub use transaction::{FailedStepWork, TransactionStats};
#[cfg(feature = "checkpoints")]
mod checkpoint;
#[cfg(feature = "checkpoints")]
pub use checkpoint::{
    Checkpoint, CheckpointContext, CheckpointError, CheckpointLimits, CheckpointStats,
};

mod capabilities;
pub use capabilities::PRIMITIVE_CAPABILITIES_JSON;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Shape {
    Box(Vector),
    Sphere(Real),
    /// Local-Y segment expanded by a spherical radius.
    Capsule {
        half_segment: Real,
        radius: Real,
    },
    /// Right triangular prism inside the local bounding box. The ramp rises toward -X.
    Wedge(Vector),
    /// Static terrain created by [`World::add_heightfield`]; `half_extents` bounds it around
    /// the body position.
    Heightfield {
        terrain: TerrainId,
        half_extents: Vector,
    },
    /// Static solid halfspace below the plane through the body position; the outward normal is
    /// the body's local +Y (an infinite floor with the identity orientation).
    HalfSpace,
}
impl Shape {
    pub const fn capsule(half_segment: Real, radius: Real) -> Self {
        Self::Capsule {
            half_segment,
            radius,
        }
    }
    pub const fn wedge(half_extents: Vector) -> Self {
        Self::Wedge(half_extents)
    }
    /// Terrain and halfspaces are static only and stay out of the bounds structures. They
    /// answer queries and character moves; halfspaces also give dynamic bodies rigid contacts,
    /// heightfields give dynamic boxes rigid contacts.
    pub fn query_only(self) -> bool {
        matches!(self, Self::Heightfield { .. } | Self::HalfSpace)
    }
    pub fn radius(self) -> Real {
        match self {
            Self::Box(h)
            | Self::Wedge(h)
            | Self::Heightfield {
                half_extents: h, ..
            } => h.length(),
            Self::HalfSpace => 0.0,
            Self::Sphere(r) => r,
            Self::Capsule {
                half_segment,
                radius,
            } => half_segment + radius,
        }
    }
    pub fn half_extents(self) -> Vector {
        match self {
            Self::Box(h)
            | Self::Wedge(h)
            | Self::Heightfield {
                half_extents: h, ..
            } => h,
            Self::HalfSpace => Vector::ZERO,
            Self::Sphere(r) => Vector(r, r, r),
            Self::Capsule {
                half_segment,
                radius,
            } => Vector(radius, half_segment + radius, radius),
        }
    }
    fn valid_dimensions(self) -> bool {
        match self {
            Self::Box(h) | Self::Wedge(h) => {
                h.finite() && h.min_component() >= 1e-6 && h.max_component() < 1e12
            }
            Self::Sphere(radius) => radius.is_finite() && (1e-6..1e12).contains(&radius),
            Self::Heightfield {
                half_extents: h, ..
            } => h.finite() && h.max_component() < 1e12,
            Self::HalfSpace => true,
            Self::Capsule {
                half_segment,
                radius,
            } => {
                half_segment.is_finite()
                    && (0.0..1e12).contains(&half_segment)
                    && radius.is_finite()
                    && (1e-6..1e12).contains(&radius)
            }
        }
    }
    /// Evaluated in f64 in every build (the identity for the default build): with f32 state,
    /// `mass * length^2` can overflow for accepted inputs although the inverse inertia is
    /// representable. The narrowed result rounds to nearest.
    #[allow(clippy::unnecessary_cast)]
    fn local_inverse_inertia(self, mass: Real) -> Option<Vector> {
        let mass = mass as f64;
        let narrow = |x: f64, y: f64, z: f64| Some(Vector(x as Real, y as Real, z as Real));
        match self {
            Self::Sphere(r) => {
                let r = r as f64;
                let inverse = 2.5 / (mass * r * r);
                narrow(inverse, inverse, inverse)
            }
            Self::Box(h) => {
                let [h0, h1, h2] = [h.0 as f64, h.1 as f64, h.2 as f64];
                narrow(
                    3.0 / (mass * (h1 * h1 + h2 * h2)),
                    3.0 / (mass * (h0 * h0 + h2 * h2)),
                    3.0 / (mass * (h0 * h0 + h1 * h1)),
                )
            }
            Self::Capsule {
                half_segment: h,
                radius: r,
            } => {
                let (h, r) = (h as f64, r as f64);
                let cylinder_volume = r * r * (2.0 * h);
                let sphere_volume = (4.0 / 3.0) * r * r * r;
                let total_volume = cylinder_volume + sphere_volume;
                let cylinder_mass = mass * cylinder_volume / total_volume;
                let sphere_mass = mass * sphere_volume / total_volume;
                let axial = cylinder_mass * r * r * 0.5 + sphere_mass * r * r * (2.0 / 5.0);
                let radial = cylinder_mass * (3.0 * r * r + 4.0 * h * h) / 12.0
                    + sphere_mass * ((2.0 / 5.0) * r * r + h * h + (3.0 / 4.0) * h * r);
                narrow(1.0 / radial, 1.0 / axial, 1.0 / radial)
            }
            // Solver-owned dynamic wedges are required to be rotation locked until a COM-centered
            // wedge inertia/pose contract is introduced. Terrain is static.
            Self::Wedge(_) | Self::Heightfield { .. } | Self::HalfSpace => None,
        }
    }
}
/// How two bodies' friction or restitution coefficients combine at a contact, as Rapier's
/// `CoefficientCombineRule`. When the two bodies' rules differ, the later variant wins
/// (Average < Min < Multiply < Max), as in Rapier.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum CombineRule {
    /// `(a + b) / 2`, Rapier's default.
    #[default]
    Average,
    Min,
    Multiply,
    Max,
}
impl CombineRule {
    /// The pair coefficient of `a` (with `rule_a`) and `b` (with `rule_b`).
    #[must_use]
    pub fn combine(a: Real, rule_a: Self, b: Real, rule_b: Self) -> Real {
        match rule_a.max(rule_b) {
            Self::Average => (a + b) * 0.5,
            Self::Min => a.min(b),
            Self::Multiply => a * b,
            Self::Max => a.max(b),
        }
    }
    #[cfg(feature = "checkpoints")]
    const fn bits(self) -> u16 {
        self as u16
    }
    #[cfg(feature = "checkpoints")]
    const fn from_bits(bits: u16) -> Self {
        match bits & 3 {
            0 => Self::Average,
            1 => Self::Min,
            2 => Self::Multiply,
            _ => Self::Max,
        }
    }
}
/// A contact material: the [`Body`] fields of the same names, for bodies built by the world
/// (heightfields, [`HeightfieldDesc::material`]). The default is Rapier's collider default.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Material {
    pub friction: Real,
    pub restitution: Real,
    pub friction_combine: CombineRule,
    pub restitution_combine: CombineRule,
}
impl Default for Material {
    fn default() -> Self {
        Self {
            friction: 0.5,
            restitution: 0.0,
            friction_combine: CombineRule::Average,
            restitution_combine: CombineRule::Average,
        }
    }
}
impl Body {
    /// Sets the body's contact material.
    pub fn set_material(&mut self, material: Material) {
        self.friction = material.friction;
        self.restitution = material.restitution;
        self.friction_combine = material.friction_combine;
        self.restitution_combine = material.restitution_combine;
    }
}
/// The contact friction of `a` against `b`.
fn pair_friction(a: &Body, b: &Body) -> Real {
    CombineRule::combine(
        a.friction,
        a.friction_combine,
        b.friction,
        b.friction_combine,
    )
}
/// The contact restitution of `a` against `b`.
fn pair_restitution(a: &Body, b: &Body) -> Real {
    CombineRule::combine(
        a.restitution,
        a.restitution_combine,
        b.restitution,
        b.restitution_combine,
    )
}
#[derive(Clone, Debug, PartialEq)]
pub struct Body {
    /// Assigned by [`World::add_body`]; [`BodyId::NONE`] before.
    pub id: BodyId,
    /// Deterministic traversal and tie-break order, assigned by [`World::add_body`].
    order: Order,
    /// Optional game-supplied stable key (the game's entity id), unique among live bodies.
    pub key: Option<u64>,
    pub position: Vector,
    pub velocity: Vector,
    pub orientation: Quaternion,
    /// Radians per second in world coordinates.
    pub angular_velocity: Vector,
    pub shape: Shape,
    /// Zero is immovable geometry; positive mass is dynamic unless `external` is set.
    pub mass: Real,
    /// Finite coefficient in [0, 10], default 0.5 (Rapier's). Contacts combine the pair's
    /// coefficients by [`Body::friction_combine`] and clamp the accumulated tangential impulse
    /// to a disk of radius mu * normal impulse. Linear-support contacts disable friction. See
    /// docs/contact-materials.md.
    pub friction: Real,
    /// Finite coefficient in [0, 1], default zero. Contacts combine the pair's coefficients by
    /// [`Body::restitution_combine`]. Restitution requires closing speed > 1 m/s (scaled by
    /// `length_unit`), separation <= contact_slop and no retained pair history; speculative
    /// positive separation does not bounce.
    pub restitution: Real,
    /// Default [`CombineRule::Average`], as in Rapier.
    pub friction_combine: CombineRule,
    /// Default [`CombineRule::Average`], as in Rapier.
    pub restitution_combine: CombineRule,
    /// Finite, nonnegative, default zero. After forces, each substep scales the linear
    /// velocity by `1 / (1 + h * linear_damping)`, as Rapier does.
    pub linear_damping: Real,
    /// Finite, nonnegative, default zero. The same scaling for the angular velocity.
    pub angular_damping: Real,
    pub rotation_locked: bool,
    pub layers: CollisionLayers3d,
    pub external: bool,
    pub sensor: bool,
    pub sleep_allowed: bool,
    pub ccd: bool,
    pub retire_on_impact: bool,
    pub linear_support: Option<Vector>,
    sleeping: bool,
    quiet_time: Real,
    force: Vector,
    torque: Vector,
    impulse: Vector,
    angular_impulse: Vector,
    cached_bounds: (Vector, Vector),
}
impl Body {
    /// A body carrying the game's stable `key` (see [`World::body_id`]).
    pub fn keyed(key: u64, shape: Shape, position: Vector, mass: Real) -> Self {
        let mut body = Self::new(shape, position, mass);
        body.key = Some(key);
        body
    }
    /// Test fixture: keyed, with the id it would have as slot `key` of a fresh world.
    #[cfg(test)]
    pub(crate) fn fixture(key: u64, shape: Shape, position: Vector, mass: Real) -> Self {
        let mut body = Self::keyed(key, shape, position, mass);
        body.id = BodyId::new(key as u32, 0);
        body.order = Order::new(Some(key), 0);
        body
    }
    pub fn new(shape: Shape, position: Vector, mass: Real) -> Self {
        Self {
            id: BodyId::NONE,
            order: Order(u128::MAX),
            key: None,
            shape,
            position,
            mass,
            velocity: Vector::ZERO,
            orientation: Quaternion::IDENTITY,
            angular_velocity: Vector::ZERO,
            friction: 0.5,
            restitution: 0.0,
            friction_combine: CombineRule::Average,
            restitution_combine: CombineRule::Average,
            linear_damping: 0.0,
            angular_damping: 0.0,
            rotation_locked: false,
            layers: CollisionLayers3d::ALL,
            external: false,
            sensor: false,
            sleep_allowed: true,
            ccd: false,
            retire_on_impact: false,
            linear_support: None,
            sleeping: false,
            quiet_time: 0.0,
            force: Vector::ZERO,
            torque: Vector::ZERO,
            impulse: Vector::ZERO,
            angular_impulse: Vector::ZERO,
            cached_bounds: (Vector::ZERO, Vector::ZERO),
        }
    }
    /// Diagnostic kinetic energy in mass-units * scene-units squared / second squared.
    /// This observation does not feed back into dynamics or the sleep policy.
    /// Angular motion with unsupported inertia returns NaN rather than invented energy.
    pub fn kinetic_energy(&self) -> Real {
        if self.mass == 0.0 {
            return 0.0;
        }
        let linear = 0.5 * self.mass * self.velocity.dot(self.velocity);
        if self.angular_velocity == Vector::ZERO {
            return linear;
        }
        let Some(inverse) = self.shape.local_inverse_inertia(self.mass) else {
            return Real::NAN;
        };
        let w = self.orientation.inverse_rotate(self.angular_velocity);
        linear + 0.5 * (w.0 * w.0 / inverse.0 + w.1 * w.1 / inverse.1 + w.2 * w.2 / inverse.2)
    }
    pub fn is_sleeping(&self) -> bool {
        self.sleeping
    }
    fn movable(&self) -> bool {
        self.mass > 0.0 && !self.external
    }
    fn inverse_mass(&self) -> Real {
        if self.movable() && !self.sleeping {
            1.0 / self.mass
        } else {
            0.0
        }
    }
    #[cfg(test)]
    fn inverse_inertia(&self, v: Vector) -> Vector {
        if self.inverse_mass() == 0.0 || self.rotation_locked {
            return Vector::ZERO;
        }
        let inverse = self
            .shape
            .local_inverse_inertia(self.mass)
            .unwrap_or(Vector::ZERO);
        self.orientation
            .rotate(self.orientation.inverse_rotate(v).component_mul(inverse))
    }
    fn valid(&self) -> bool {
        self.position.finite()
            && self.velocity.finite()
            && self.angular_velocity.finite()
            && self.angular_velocity.abs().max_component() < 1e9
            && self.orientation.finite()
            && self.shape.valid_dimensions()
            && (!matches!(self.shape, Shape::Wedge(_)) || self.rotation_locked || !self.movable())
            && (!self.shape.query_only() || (self.mass == 0.0 && !self.external))
            && self.mass.is_finite()
            && (self.mass == 0.0 || self.mass >= 1e-6)
            && self.mass < 1e12
            && self.friction.is_finite()
            && (0.0..=10.0).contains(&self.friction)
            && self.restitution.is_finite()
            && (0.0..=1.0).contains(&self.restitution)
            && (0.0..1e6).contains(&self.linear_damping)
            && (0.0..1e6).contains(&self.angular_damping)
            && (!self.rotation_locked || self.angular_velocity == Vector::ZERO)
            && (self.mass > 0.0
                || (self.velocity == Vector::ZERO && self.angular_velocity == Vector::ZERO))
            && self.position.abs().max_component() < 1e12
            && self.velocity.abs().max_component() < 1e12
    }
    /// [`Self::valid`] for a valid body whose motion changed: only the motion terms can fail.
    fn motion_valid(&self) -> bool {
        self.position.finite()
            && self.velocity.finite()
            && self.angular_velocity.finite()
            && self.angular_velocity.abs().max_component() < 1e9
            && self.orientation.finite()
            && (!self.rotation_locked || self.angular_velocity == Vector::ZERO)
            && (self.mass > 0.0
                || (self.velocity == Vector::ZERO && self.angular_velocity == Vector::ZERO))
            && self.position.abs().max_component() < 1e12
            && self.velocity.abs().max_component() < 1e12
    }
}
/// How a world stores its bodies; fixed when the world is created. Both modes give identical
/// results for the same calls (every traversal and tie follows body order); they differ only
/// in what adding and removing bodies costs. See docs/world-api-contract.md.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum BodyStorage {
    /// Bodies stay in body order in one array. Adding a body after every existing one and
    /// removing any body are amortized O(1) (removal leaves a tombstone that the next step,
    /// or a later removal once half the records are tombstones, compacts away); adding a body
    /// in the middle of the order moves the later records, O(n).
    #[default]
    Ordered,
    /// Bodies are appended to an unordered arena with a separate order index: adding and
    /// removing any body is O(log n). The next step (or the next call that wakes an island)
    /// sorts the arena back into body order once, O(n + k log k) for k out-of-order adds.
    Indexed,
}

/// Eligible pairs in the bounded post-integration position stage.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum PositionCorrection {
    /// Preserve the original fixed-collider-only comparison policy.
    #[default]
    FixedColliders,
    /// Also correct residual overlap between awake dynamic pairs admitted by CCD/contact
    /// generation in this substep. This does not discover contacts or replace swept admission.
    AdmittedContacts,
}

#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// m/s², default (0, −9.81, 0) as in Rapier; scaled by `length_unit`.
    pub gravity: Vector,
    pub substeps: u8,
    pub velocity_iterations: u8,
    /// Shared post-integration position-pass budget (historical field name).
    /// Zero disables correction; fixed and admitted dynamic pairs share each selected pass.
    pub fixed_position_iterations: u8,
    pub position_correction: PositionCorrection,
    /// Metres, default 0.005 (Rapier's allowed linear error); scaled by `length_unit`.
    pub contact_slop: Real,
    /// m/s, default 0.05 (Rapier's sleep threshold); scaled by `length_unit`.
    pub sleep_speed: Real,
    /// Seconds below `sleep_speed` before a body sleeps, default 0.5 as in Rapier.
    pub sleep_seconds: Real,
    pub warm_start: bool,
    /// None retains the fixed-pass reference. Some permits a checked early exit.
    pub convergence: Option<Convergence>,
    /// Contact-island-local stopping, or the original whole-world convergence reference.
    /// With `convergence: None`, both scopes use the unchanged fixed-pass solver.
    pub convergence_scope: ConvergenceScope,
    /// Scene units per metre (100 for a centimetre game), as Rapier's `length_unit`. The
    /// dynamics fields above (`gravity`, `contact_slop`, `sleep_speed`) are in metres and
    /// scaled by it once in [`World::new`], as are built-in tolerances and character defaults
    /// ([`World::character_settings`]); body and shape values stay in scene units.
    pub length_unit: Real,
    /// Body storage layout, fixed for the world's lifetime.
    pub body_storage: BodyStorage,
}
impl Default for Config {
    fn default() -> Self {
        Self {
            gravity: Vector(0.0, -9.81, 0.0),
            substeps: 4,
            velocity_iterations: 8,
            fixed_position_iterations: 0,
            position_correction: PositionCorrection::FixedColliders,
            contact_slop: 0.005,
            sleep_speed: 0.05,
            sleep_seconds: 0.5,
            warm_start: true,
            convergence: Some(Convergence::default()),
            convergence_scope: ConvergenceScope::ContactIslands,
            length_unit: 1.0,
            // The test-only `indexed-storage` feature runs every suite on `Indexed` storage.
            body_storage: if cfg!(feature = "indexed-storage") {
                BodyStorage::Indexed
            } else {
                BodyStorage::Ordered
            },
        }
    }
}
#[cfg(test)]
impl Config {
    /// The pre-#318 scene-unit dynamics settings that legacy solver tests were tuned with.
    pub(crate) fn legacy() -> Self {
        Self {
            gravity: Vector(0.0, -3600.0, 0.0),
            contact_slop: 0.02,
            sleep_speed: 1.0,
            ..Self::default()
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    InvalidInput,
    /// Another live body already has this game key.
    DuplicateKey(u64),
    MissingBody(BodyId),
    NonFiniteState(BodyId),
    /// A failed search cannot establish collision absence. The attempted step is rolled back.
    CollisionSearchFailed {
        bodies: [BodyId; 2],
        reason: SweepFailure,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SweepFailure {
    InvalidGeometryInput,
    NonFiniteComputation,
    IterationLimit,
}
impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "fixed-step approximation: {self:?}")
    }
}
impl std::error::Error for Error {}
#[derive(Clone, Debug, Default)]
pub struct Report {
    pub position: PositionReport,
    pub substeps: u32,
    pub pair_tests: u64,
    pub narrow_tests: u64,
    pub contact_points: u64,
    /// Equivalent solver rounds per substep (maximum over islands), not the sum over islands.
    /// Use `convergence.constraint_visits` for actual row work and `islands` for grouping costs.
    pub impulse_iterations: u64,
    pub integrated_bodies: u64,
    pub woken_bodies: u64,
    pub swept_contacts: u64,
    pub retired: Vec<BodyId>,
    pub max_penetration: Real,
    pub bookkeeping: BookkeepingStats,
    pub geometry: GeometryStats,
    pub convergence: ConvergenceStats,
    pub islands: IslandStats,
    /// Active response bodies prepared, including newly awakened bodies in the same substep.
    pub response_preparations: u64,
    /// Shape/mass inertia coefficients calculated; independent of contact iteration count.
    pub inertia_preparations: u64,
    /// Inverse-inertia vector evaluations (preparation is reused for these).
    pub inertia_applications: u64,
}
#[derive(Clone, Debug)]
#[cfg_attr(test, derive(PartialEq))]
struct CachedPoint {
    a: Vector,
    b: Vector,
    normal: Vector,
    impulse: Real,
    tangent: Vector,
}
#[derive(Clone, Debug)]
struct Constraint {
    a: usize,
    b: usize,
    n: Vector,
    ra: Vector,
    rb: Vector,
    t1: Vector,
    t2: Vector,
    normal_mass: Real,
    tangent_mass: [Real; 2],
    bias: Real,
    friction: Real,
    normal_impulse: Real,
    tangent_impulse: [Real; 2],
    sep: Real,
    swept: bool,
    response: [bool; 2],
    spin: bool,
    /// Centre offset `b - a` when `sep` was last measured: later substeps of the step advance
    /// `sep` by the bodies' relative motion along `n` instead of regenerating the contact
    /// (#378).
    offset: Vector,
    /// Restitution coefficient while the pair has no cached contact yet; zero afterwards.
    restitution: Real,
}
/// Internal body order: keyed bodies first by key, then keyless bodies by creation serial.
/// Every traversal, pair orientation and tie uses it, so results depend on game keys and the
/// world's contents, not on slot reuse or insertion history of keyed bodies.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub(crate) struct Order(u128);
impl Ord for Order {
    #[inline]
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        crate::work::add(crate::work::Counter::OrderComparisons, 1);
        self.0.cmp(&other.0)
    }
}
impl PartialOrd for Order {
    #[inline]
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
impl Order {
    pub(crate) fn new(key: Option<u64>, serial: u64) -> Self {
        Self(match key {
            Some(key) => u128::from(key),
            None => (1 << 64) | u128::from(serial),
        })
    }
}

const FREE_SLOT: u32 = u32::MAX;
/// Steps of motion a new broad-phase box (`bookkeeping::BroadBox`) leaves room for (#412).
const BOX_STEPS: Real = 4.0;
/// One id slot: its current generation and the body's index while live.
#[derive(Clone, Copy, Debug)]
struct Slot {
    generation: u32,
    index: u32,
}

#[derive(Clone, Debug)]
pub struct World {
    /// The configuration in scene units: metre-valued fields scaled by `length_unit`.
    scene: Config,
    /// The configuration as given, returned by [`World::config`] and checkpointed.
    config: Config,
    /// Closing speed above which restitution applies: 1 m/s in scene units.
    restitution_speed: Real,
    bodies: Vec<Body>,
    cache: BTreeMap<(Order, Order), Vec<CachedPoint>>,
    /// Prior substep length. Time composition and its checkpoint value stay f64.
    last_h: Scalar,
    // Derived substep scratch, indexed like bodies. Refresh contents; reuse allocated capacity.
    responses: Vec<PreparedResponse>,
    bookkeeping: Scratch,
    geometry: GeometryCache,
    island_scratch: islands::Scratch,
    constraints: Vec<Constraint>,
    manifold_scratch: Vec<(usize, usize, contact::Manifold)>,
    pub last_report: Report,
    transaction: transaction::Journal,
    last_transaction: TransactionStats,
    query_tree: query::QueryTree,
    terrains: Vec<terrain::Terrain>,
    /// Halfspace bodies, which every query tests (they have no finite bounds).
    halfspaces: Vec<BodyId>,
    slots: Vec<Slot>,
    /// Free slots, descending: `pop` returns the lowest.
    free: Vec<u32>,
    keys: BTreeMap<u64, BodyId>,
    /// Creation serial of the next keyless body (its order after every keyed body).
    next_serial: u64,
    characters: character::Characters,
    failed_work: Option<FailedStepWork>,
    elapsed: Scalar,
    /// Removed bodies still occupying a storage record (`id == BodyId::NONE`).
    tombstones: usize,
    /// Leading records in body order; `Indexed` appends out-of-order bodies after them.
    sorted_len: usize,
    /// `Indexed` only: every live body in body order.
    order_index: BTreeMap<Order, BodyId>,
    /// Reused merge buffer for sorting `Indexed` storage.
    settle_scratch: Vec<Body>,
}
impl World {
    pub fn new(config: Config) -> Result<Self, Error> {
        if !config.gravity.finite()
            || config.substeps == 0
            || config.substeps > 32
            || config.velocity_iterations == 0
            || config.velocity_iterations > 64
            || config.fixed_position_iterations > 8
            || config.convergence.is_some_and(|c| !c.valid())
            || !config.contact_slop.is_finite()
            || config.contact_slop <= 0.0
            || !config.length_unit.is_finite()
            || !(1e-6..=1e6).contains(&config.length_unit)
            || !config.sleep_speed.is_finite()
            || config.sleep_speed <= 0.0
            || !config.sleep_seconds.is_finite()
            || config.sleep_seconds <= 0.0
        {
            return Err(Error::InvalidInput);
        }
        let settings = config;
        let unit = config.length_unit;
        let config = Config {
            gravity: config.gravity * unit,
            contact_slop: config.contact_slop * unit,
            sleep_speed: config.sleep_speed * unit,
            ..config
        };
        if !config.gravity.finite()
            || !config.contact_slop.is_normal()
            || !config.sleep_speed.is_normal()
        {
            return Err(Error::InvalidInput);
        }
        Ok(Self {
            scene: config,
            config: settings,
            restitution_speed: unit,
            bodies: Vec::new(),
            cache: BTreeMap::new(),
            last_h: 0.0,
            responses: Vec::new(),
            bookkeeping: Scratch::default(),
            geometry: GeometryCache::default(),
            island_scratch: islands::Scratch::default(),
            constraints: Vec::new(),
            manifold_scratch: Vec::new(),
            last_report: Report::default(),
            transaction: transaction::Journal::default(),
            last_transaction: TransactionStats::default(),
            query_tree: query::QueryTree::default(),
            terrains: Vec::new(),
            halfspaces: Vec::new(),
            slots: Vec::new(),
            free: Vec::new(),
            keys: BTreeMap::new(),
            next_serial: 0,
            characters: character::Characters::default(),
            failed_work: None,
            elapsed: 0.0,
            tombstones: 0,
            sorted_len: 0,
            order_index: BTreeMap::new(),
            settle_scratch: Vec::new(),
        })
    }
    /// Every live body, in body order (keyed bodies by key, then keyless ones by creation).
    pub fn bodies(&self) -> impl Iterator<Item = &Body> {
        let indexed = self.scene.body_storage == BodyStorage::Indexed;
        let ordered = (!indexed).then(|| self.bodies.iter().filter(|b| b.id != BodyId::NONE));
        let index = indexed.then(|| {
            self.order_index
                .values()
                .map(|&id| &self.bodies[self.slot_index(id.slot())])
        });
        ordered
            .into_iter()
            .flatten()
            .chain(index.into_iter().flatten())
    }
    /// Number of live bodies.
    pub fn body_count(&self) -> usize {
        self.bodies.len() - self.tombstones
    }
    /// Storage position of the live body in id slot `slot` (query-tree leaves are slots).
    fn slot_index(&self, slot: u32) -> usize {
        self.slots[slot as usize].index as usize
    }
    /// Storage positions follow body order (always for `Ordered`; for `Indexed` until a body
    /// is added out of order). Tombstones may remain.
    fn storage_sorted(&self) -> bool {
        self.sorted_len == self.bodies.len()
    }
    /// Positions of every live body, in body order.
    fn ordered_positions(&self) -> Vec<usize> {
        if self.storage_sorted() {
            (0..self.bodies.len())
                .filter(|&i| self.bodies[i].id != BodyId::NONE)
                .collect()
        } else {
            self.order_index
                .values()
                .map(|&id| self.slot_index(id.slot()))
                .collect()
        }
    }
    /// Removes tombstones and sorts storage into body order, so positions are body ranks;
    /// O(1) when already settled. The step and island traversals need settled storage.
    fn settle_storage(&mut self) {
        if self.tombstones == 0 && self.storage_sorted() {
            return;
        }
        let records = self.bodies.len();
        let sorted_live = self.bodies[..self.sorted_len]
            .iter()
            .filter(|b| b.id != BodyId::NONE)
            .count();
        self.bodies.retain(|b| b.id != BodyId::NONE);
        if sorted_live < self.bodies.len() {
            // Indexed: merge the sorted prefix with the sorted out-of-order tail.
            self.bodies[sorted_live..].sort_unstable_by_key(|b| b.order);
            if sorted_live > 0
                && self.bodies[sorted_live - 1].order > self.bodies[sorted_live].order
            {
                let scratch = &mut self.settle_scratch;
                scratch.clear();
                scratch.reserve(self.bodies.len());
                let (head, tail) = self.bodies.split_at(sorted_live);
                let (mut a, mut b) = (0, 0);
                while a < head.len() && b < tail.len() {
                    if head[a].order < tail[b].order {
                        scratch.push(head[a].clone());
                        a += 1;
                    } else {
                        scratch.push(tail[b].clone());
                        b += 1;
                    }
                }
                scratch.extend_from_slice(&head[a..]);
                scratch.extend_from_slice(&tail[b..]);
                std::mem::swap(&mut self.bodies, scratch);
            }
        }
        crate::work::add(crate::work::Counter::StorageShifts, records as u64);
        self.tombstones = 0;
        self.sorted_len = self.bodies.len();
        for (i, b) in self.bodies.iter().enumerate() {
            self.slots[b.id.slot() as usize].index = i as u32;
        }
        // Positions moved; the bodies and the tree's slot-keyed leaves did not.
        let from = self.bookkeeping.generation;
        self.bookkeeping.layout_changed();
        self.geometry.positions_moved();
        self.query_tree.retag(from, self.bookkeeping.generation);
    }
    /// Island traversals binary-search storage by body order: sort `Indexed` storage first.
    fn sort_storage(&mut self) {
        if !self.storage_sorted() {
            self.settle_storage();
        }
    }
    pub fn body(&self, id: BodyId) -> Option<&Body> {
        self.index(id).ok().map(|i| &self.bodies[i])
    }
    /// Work from the most recent step attempt, including a numerical failure.
    /// A call that does not enter the solver journals nothing. Capacities describe
    /// vector storage observed during the attempt, excluding allocator/tree overhead.
    /// This observation never substitutes for a success report.
    pub fn last_step_transaction(&self) -> TransactionStats {
        self.last_transaction
    }
    /// Discarded work if the most recent attempt entered the solver and failed.
    /// Early input rejection and successful attempts return None. Actual time,
    /// body state, events and the successful report remain the prior boundary.
    pub fn last_failed_step_work(&self) -> Option<&FailedStepWork> {
        self.failed_work.as_ref()
    }
    /// Releases disposable rollback storage at a call boundary. Physical contact
    /// history, pending input and the previous successful report are retained.
    pub fn release_transaction_scratch(&mut self) {
        self.transaction = transaction::Journal::default();
        self.last_transaction = TransactionStats::default();
        self.failed_work = None;
    }
    pub fn elapsed_seconds(&self) -> Scalar {
        self.elapsed
    }
    pub fn is_quiescent(&self) -> bool {
        self.bookkeeping.activity.count == 0
    }
    /// The configuration as given to [`World::new`], metre-valued fields unscaled.
    pub fn config(&self) -> Config {
        self.config
    }
    /// O(1) slot lookup; a stale generation never matches.
    fn index(&self, id: BodyId) -> Result<usize, Error> {
        self.slots
            .get(id.slot() as usize)
            .filter(|s| s.generation == id.generation() && s.index != FREE_SLOT)
            .map(|s| s.index as usize)
            .filter(|&i| self.bodies.get(i).is_some_and(|b| b.id == id))
            // Mid-step layout changes (retirement) refresh slots only at the step's end.
            .or_else(|| self.bodies.iter().position(|b| b.id == id))
            .ok_or(Error::MissingBody(id))
    }
    /// The live body with game key `key`.
    pub fn body_id(&self, key: u64) -> Option<BodyId> {
        self.keys.get(&key).copied()
    }
    /// Recomputes slot indices, free slots and keys from `bodies` after internal layout
    /// changes (retirement, rollback, restore). Generations are kept.
    fn reindex_slots(&mut self) {
        for slot in &mut self.slots {
            slot.index = FREE_SLOT;
        }
        self.keys.clear();
        for (i, b) in self.bodies.iter().enumerate() {
            if b.id == BodyId::NONE {
                continue;
            }
            self.slots[b.id.slot() as usize].index = i as u32;
            if let Some(key) = b.key {
                self.keys.insert(key, b.id);
            }
        }
        self.free = (0..self.slots.len() as u32)
            .rev()
            .filter(|&s| self.slots[s as usize].index == FREE_SLOT)
            .collect();
        // Only ever called on settled storage (step-end retirement, rollback, restore).
        debug_assert_eq!(self.tombstones, 0);
        self.sorted_len = self.bodies.len();
        if self.scene.body_storage == BodyStorage::Indexed {
            self.order_index = self.bodies.iter().map(|b| (b.order, b.id)).collect();
        }
    }
    fn shift_slots(&mut self, from: usize) {
        for (i, b) in self.bodies.iter().enumerate().skip(from) {
            if b.id != BodyId::NONE {
                self.slots[b.id.slot() as usize].index = i as u32;
            }
        }
    }
    /// Adds a body and returns its new id. `body.key`, when set, must be unique among live
    /// bodies; `body.id` is ignored.
    pub fn add_body(&mut self, body: Body) -> Result<BodyId, Error> {
        if matches!(body.shape, Shape::Heightfield { .. }) {
            return Err(Error::InvalidInput);
        }
        self.insert_body(body)
    }
    fn insert_body(&mut self, mut body: Body) -> Result<BodyId, Error> {
        body.orientation = body.orientation.normalized();
        if !body.valid() || self.slots.len() >= u32::MAX as usize - 1 {
            return Err(Error::InvalidInput);
        }
        if let Some(key) = body.key
            && self.keys.contains_key(&key)
        {
            return Err(Error::DuplicateKey(key));
        }
        // Reuse the lowest free slot with a new generation, else a fresh slot.
        body.id = match self.free.pop() {
            Some(slot) => {
                let s = &mut self.slots[slot as usize];
                s.generation = s.generation.wrapping_add(1);
                BodyId::new(slot, s.generation)
            }
            None => {
                self.slots.push(Slot {
                    generation: 0,
                    index: FREE_SLOT,
                });
                BodyId::new(self.slots.len() as u32 - 1, 0)
            }
        };
        body.order = Order::new(body.key, self.next_serial);
        self.next_serial += 1;
        body.cached_bounds = contact::bounds(&body);
        // Local topology change: only a genuine new fixed overlap can disturb a sleeper.
        if body.mass == 0.0 && !body.sensor && !body.shape.query_only() {
            let wake = self.sleepers_touching(&body);
            if !wake.is_empty() {
                self.sort_storage();
                for order in wake {
                    self.wake_island(order);
                }
            }
        }
        self.bookkeeping.activity.count += usize::from(body.mass > 0.0 && !body.sleeping);
        let synced = self.query_tree.current(self.bookkeeping.generation);
        let (id, key, order) = (body.id, body.key, body.order);
        if body.shape == Shape::HalfSpace {
            self.halfspaces.push(id);
        }
        let len = self.bodies.len();
        let at = match self.scene.body_storage {
            BodyStorage::Ordered => {
                let i = self.bodies.partition_point(|b| b.order < order);
                // A tombstone next to the insertion point keeps the order sorted when reused.
                let reuse = [i, i.wrapping_sub(1)]
                    .into_iter()
                    .find(|&j| j < len && self.bodies[j].id == BodyId::NONE);
                if let Some(j) = reuse {
                    self.bodies[j] = body;
                    self.tombstones -= 1;
                    j
                } else {
                    self.bodies.insert(i, body);
                    crate::work::add(crate::work::Counter::StorageShifts, (len - i) as u64);
                    self.shift_slots(i + 1);
                    self.sorted_len += 1;
                    i
                }
            }
            BodyStorage::Indexed => {
                // While storage is sorted, the records between the live neighbours in body order
                // are tombstones: reusing one keeps it sorted (re-adding a removed door, say).
                let gap = self.storage_sorted().then(|| {
                    let position = |(_, id): (&Order, &BodyId)| self.slot_index(id.slot());
                    let lo = self
                        .order_index
                        .range(..order)
                        .next_back()
                        .map_or(0, |p| position(p) + 1);
                    let hi = self.order_index.range(order..).next().map_or(len, position);
                    (lo, hi)
                });
                self.order_index.insert(order, id);
                match gap {
                    Some((lo, hi)) if lo < hi => {
                        self.bodies[lo] = body;
                        self.tombstones -= 1;
                        lo
                    }
                    _ => {
                        // Appended: still sorted only after every live body and tombstone.
                        let in_order = gap.is_some()
                            && self.bodies.last().is_none_or(|last| last.order < order);
                        self.bodies.push(body);
                        self.sorted_len += usize::from(in_order);
                        len
                    }
                }
            }
        };
        self.slots[id.slot() as usize].index = at as u32;
        if let Some(key) = key {
            self.keys.insert(key, id);
        }
        self.bookkeeping.layout_changed();
        if synced {
            self.query_tree
                .inserted(&self.bodies[at], self.bookkeeping.generation);
        } else {
            self.query_tree
                .sync(&self.bodies, self.bookkeeping.generation);
            self.bookkeeping.forget_boxes();
        }
        // The slot's previous body's box is gone; boxes near a new resting body lack its leaf
        // (an awake one makes its own box). The broad phase sizes `broad` for new slots.
        self.bookkeeping.drop_box(id.slot());
        let b = &self.bodies[at];
        if !(b.mass > 0.0 && !b.sleeping) {
            let bounds = b.cached_bounds;
            self.resting_changed(at, bounds);
        }
        Ok(id)
    }
    /// Orders of sleeping bodies that `body` (not yet stored) touches, in body order.
    fn sleepers_touching(&self, body: &Body) -> Vec<Order> {
        let touches = |b: &Body| {
            b.id != BodyId::NONE
                && b.sleeping
                && b.layers.collides_with(body.layers)
                && contact::current(b, body, 0.0).is_some()
        };
        let mut wake = Vec::new();
        if self.query_tree.current(self.bookkeeping.generation) {
            let tree = &self.query_tree;
            let (lo, hi) = body.cached_bounds;
            let reach = tree
                .magnitude
                .max(lo.abs().max_component())
                .max(hi.abs().max_component());
            let pad = 64.0 * Real::EPSILON * (1.0 + reach);
            tree.tree().visit_aabb(
                spatial_kernels::Aabb {
                    min: [lo.0 - pad, lo.1 - pad, lo.2 - pad],
                    max: [hi.0 + pad, hi.1 + pad, hi.2 + pad],
                },
                |leaf| {
                    let b = &self.bodies[self.slot_index(leaf.id)];
                    if touches(b) {
                        wake.push(b.order);
                    }
                },
            );
            wake.sort_unstable();
        } else {
            for i in self.ordered_positions() {
                if touches(&self.bodies[i]) {
                    wake.push(self.bodies[i].order);
                }
            }
        }
        wake
    }
    pub fn remove_body(&mut self, id: BodyId) -> Option<Body> {
        let order = self.bodies[self.index(id).ok()?].order;
        self.characters.remove(id);
        self.halfspaces.retain(|&h| h != id);
        // Bodies in contact with the removed one wake (its contact pairs, in pair order).
        let neighbors = self
            .cache
            .keys()
            .filter_map(|&(a, b)| {
                (a == order)
                    .then_some(b)
                    .or_else(|| (b == order).then_some(a))
            })
            .collect::<Vec<_>>();
        if !neighbors.is_empty() {
            self.sort_storage();
            for n in neighbors {
                self.wake_island(n);
            }
        }
        // O(contact pairs), independent of the body count.
        self.geometry.remove(order);
        self.cache.retain(|&(a, b), _| a != order && b != order);
        let i = self.index(id).ok()?;
        let synced = self.query_tree.current(self.bookkeeping.generation);
        let removed = self.bodies[i].clone();
        self.bodies[i].id = BodyId::NONE;
        self.tombstones += 1;
        if self.scene.body_storage == BodyStorage::Indexed {
            self.order_index.remove(&order);
        }
        let slot = id.slot();
        self.slots[slot as usize].index = FREE_SLOT;
        self.bookkeeping.drop_box(slot);
        // Descending, so `pop` reuses the lowest free slot.
        let at = self.free.partition_point(|&f| f > slot);
        self.free.insert(at, slot);
        if let Some(key) = removed.key {
            self.keys.remove(&key);
        }
        self.bookkeeping.activity.count -= usize::from(removed.mass > 0.0 && !removed.sleeping);
        self.bookkeeping.layout_changed();
        if synced {
            self.query_tree.removed(slot, self.bookkeeping.generation);
        } else {
            self.query_tree
                .sync(&self.bodies, self.bookkeeping.generation);
            self.bookkeeping.forget_boxes();
        }
        // Tombstones are compacted by the next step, or here once they are half the records:
        // amortized O(1) per removal.
        if 2 * self.tombstones > self.bodies.len() {
            self.settle_storage();
        }
        Some(removed)
    }
    /// Places an external (kinematic) body at `position` without resolving contact.
    pub fn set_kinematic_position(&mut self, id: BodyId, position: Vector) -> Result<(), Error> {
        // Writing back the current position changes nothing a character relies on.
        let unchanged = self.body(id).is_some_and(|b| b.position == position);
        self.place_kinematic(id, position)?;
        if !unchanged {
            self.body_placed(id);
        }
        Ok(())
    }
    /// Moves a static body (mass 0, not external) to `position` in place, as Rapier moves a
    /// fixed collider: the layout and every other body's caches stay valid. Sleeping bodies
    /// it was in contact with or now touches wake; characters it now reaches check for
    /// overlap again. Halfspaces and heightfields stay where they were added.
    pub fn set_static_position(&mut self, id: BodyId, position: Vector) -> Result<(), Error> {
        let i = self.index(id)?;
        let body = &self.bodies[i];
        if body.mass != 0.0
            || body.external
            || matches!(body.shape, Shape::HalfSpace | Shape::Heightfield { .. })
            || !position.finite()
            || position.abs().max_component() >= 1e12
        {
            return Err(Error::InvalidInput);
        }
        if body.position == position {
            return Ok(());
        }
        let order = body.order;
        let mut wake = self
            .cache
            .keys()
            .filter_map(|&(a, b)| {
                (a == order)
                    .then_some(b)
                    .or_else(|| (b == order).then_some(a))
            })
            .collect::<Vec<_>>();
        let body = &mut self.bodies[i];
        let before = body.cached_bounds;
        body.position = position;
        body.cached_bounds = contact::bounds(body);
        self.bookkeeping.poses_changed();
        if self.query_tree.current(self.bookkeeping.generation) {
            self.query_tree.moved(&self.bodies[i]);
        }
        self.resting_changed(i, before);
        if !self.bodies[i].sensor && !self.bodies[i].shape.query_only() {
            wake.extend(self.sleepers_touching(&self.bodies[i]));
        }
        if !wake.is_empty() {
            wake.sort_unstable();
            wake.dedup();
            self.sort_storage();
            for order in wake {
                self.wake_island(order);
            }
        }
        self.body_placed(id);
        Ok(())
    }
    fn place_kinematic(&mut self, id: BodyId, position: Vector) -> Result<(), Error> {
        let i = self.index(id)?;
        if !self.bodies[i].external || !position.finite() || position.abs().max_component() >= 1e12
        {
            return Err(Error::InvalidInput);
        }
        let body = &mut self.bodies[i];
        let before = body.cached_bounds;
        body.position = position;
        body.cached_bounds = contact::bounds(body);
        self.bookkeeping.poses_changed();
        if self.query_tree.current(self.bookkeeping.generation) {
            self.query_tree.moved(&self.bodies[i]);
        }
        // An awake body's leaf no longer holds its box; a resting one changed under others.
        let b = &self.bodies[i];
        if b.mass > 0.0 && !b.sleeping {
            self.bookkeeping.drop_box(b.id.slot());
        } else {
            self.resting_changed(i, before);
        }
        Ok(())
    }
    pub fn set_velocity(&mut self, id: BodyId, v: Vector) -> Result<(), Error> {
        if !v.finite() || v.abs().max_component() >= 1e12 {
            return Err(Error::InvalidInput);
        }
        self.sort_storage();
        let i = self.index(id)?;
        if self.bodies[i].mass == 0.0 {
            return if v == Vector::ZERO {
                Ok(())
            } else {
                Err(Error::InvalidInput)
            };
        }
        if self.bodies[i].velocity != v {
            self.wake_island(self.bodies[i].order);
            self.bodies[i].velocity = v;
        }
        Ok(())
    }
    pub fn apply_impulse(
        &mut self,
        id: BodyId,
        j: Vector,
        world_point: Vector,
    ) -> Result<(), Error> {
        if !j.finite() || !world_point.finite() {
            return Err(Error::InvalidInput);
        }
        self.sort_storage();
        let i = self.index(id)?;
        if !self.bodies[i].movable() {
            return Err(Error::InvalidInput);
        }
        let angular = (world_point - self.bodies[i].position).cross(j);
        let pending = self.bodies[i].impulse + j;
        let spin = self.bodies[i].angular_impulse + angular;
        if !pending.finite() || !spin.finite() {
            return Err(Error::InvalidInput);
        }
        if j != Vector::ZERO {
            self.wake_island(self.bodies[i].order);
        }
        self.bodies[i].impulse = pending;
        self.bodies[i].angular_impulse = spin;
        Ok(())
    }
    pub fn add_force(&mut self, id: BodyId, force: Vector) -> Result<(), Error> {
        if !force.finite() {
            return Err(Error::InvalidInput);
        }
        self.sort_storage();
        let i = self.index(id)?;
        if !self.bodies[i].movable() {
            return Err(Error::InvalidInput);
        }
        let total = self.bodies[i].force + force;
        if !total.finite() {
            return Err(Error::InvalidInput);
        }
        if force != Vector::ZERO {
            self.wake_island(self.bodies[i].order);
        }
        self.bodies[i].force = total;
        Ok(())
    }
    /// Index of the body with internal order `order` (bodies are sorted by it).
    fn index_of_order(&self, order: Order) -> Option<usize> {
        self.bodies.binary_search_by_key(&order, |b| b.order).ok()
    }
    fn wake_island(&mut self, root: Order) -> u64 {
        debug_assert!(
            self.storage_sorted(),
            "island traversal over unsorted storage"
        );
        let Some(root) = self.index_of_order(root) else {
            return 0;
        };
        let s = &mut self.bookkeeping;
        s.ensure_graph(&self.bodies, &self.cache);
        s.traversal.begin(self.bodies.len(), &mut s.work);
        s.traversal
            .collect(root, &self.bodies, &s.graph, &mut s.work);
        // Retain only newly awakened indices for same-substep force preparation.
        // The next traversal resets this scratch; no persistent wake-state copy is added.
        s.traversal.island.retain(|&i| {
            let b = &mut self.bodies[i];
            if !b.sleeping {
                return false;
            }
            self.transaction.body(i, b);
            b.sleeping = false;
            b.quiet_time = 0.0;
            true
        });
        let count = s.traversal.island.len();
        s.activity.count += count;
        for &i in &s.traversal.island {
            s.activity.woke(i, &mut s.work);
        }
        count as u64
    }
    fn wake_contact_island<const PREPARED: bool>(
        &mut self,
        root: Order,
        h: Real,
        report: &mut Report,
    ) -> u64 {
        let woke = self.wake_island(root);
        for n in 0..self.bookkeeping.traversal.island.len() {
            let i = self.bookkeeping.traversal.island[n];
            let s = &mut self.bookkeeping;
            push(&mut s.stepped, i, &mut s.work);
            if PREPARED {
                self.responses[i] = PreparedResponse::new(&self.bodies[i], report);
            }
            // These bodies skipped the ordinary force phase while parked.
            self.apply_substep_forces::<PREPARED>(i, h, report);
        }
        woke
    }
    pub fn has_support(&self, id: BodyId) -> bool {
        let Ok(i) = self.index(id) else {
            return false;
        };
        let id = self.bodies[i].order;
        let up = (-self.scene.gravity).unit();
        self.cache.iter().any(|(&(a, b), p)| {
            p.iter().any(|p| {
                (b == id && p.normal.dot(up) > 0.5) || (a == id && p.normal.dot(up) < -0.5)
            })
        })
    }
    /// Advance up to 0.1 seconds in a bounded number of substeps. No dropped remainder.
    /// A returned error preserves physical state/history, pending inputs, time
    /// and the previous successful report. Panics are outside this guarantee.
    pub fn step(&mut self, dt: Scalar) -> Result<Report, Error> {
        self.step_with_preparation::<true>(dt)
    }

    // The false specialization is only instantiated by tests, as an unprepared replay oracle.
    fn step_with_preparation<const PREPARED: bool>(&mut self, dt: Scalar) -> Result<Report, Error> {
        self.step_with_geometry::<PREPARED, true>(dt)
    }
    fn step_with_geometry<const PREPARED: bool, const CACHED: bool>(
        &mut self,
        dt: Scalar,
    ) -> Result<Report, Error> {
        self.failed_work = None;
        self.last_transaction = TransactionStats {
            vector_capacity_bytes: self.transaction.capacity_bytes(),
            ..TransactionStats::default()
        };
        if !dt.is_finite() || !(0.0..=0.1).contains(&dt) {
            return Err(Error::InvalidInput);
        }
        if dt == 0.0 {
            self.last_report = Report::default();
            self.last_report.islands.scratch_retained_bytes = self.island_scratch.retained_bytes();
            self.last_report.geometry.retained_bytes = self.geometry.retained_bytes() as u64;
            self.last_report.position.scratch_retained_bytes =
                self.bookkeeping.position.retained_bytes();
            return Ok(self.last_report.clone());
        }
        if substep_length(dt, self.scene.substeps) == 0.0 {
            return Err(Error::InvalidInput);
        }
        let mut report = Report::default();
        if self.is_quiescent() {
            self.elapsed += dt;
            self.finish_bookkeeping(&mut report);
            self.last_report = report.clone();
            return Ok(report);
        }
        // The solver iterates positions as body ranks.
        self.settle_storage();
        let layout = self.bookkeeping.generation;
        self.transaction.begin(self.bodies.len());
        let prior_h = self.last_h;
        let prior_activity = self.bookkeeping.activity.count;
        let result = self.advance_substeps::<PREPARED, CACHED>(dt);
        self.last_transaction = self.transaction.stats(result.is_err());
        if let Some(work) = &mut self.failed_work {
            work.transaction = self.last_transaction;
        }
        let result = match result {
            Ok(report) => {
                self.transaction.commit();
                self.last_report = report.clone();
                Ok(report)
            }
            Err(error) => {
                let mut journal = std::mem::take(&mut self.transaction);
                journal.rollback(self);
                self.transaction = journal;
                self.last_h = prior_h;
                self.bookkeeping.activity.count = prior_activity;
                Err(error)
            }
        };
        let generation = self.bookkeeping.generation;
        let moved = if self.bookkeeping.generation != layout
            || !self.query_tree.current(self.bookkeeping.generation)
        {
            // Every leaf returns to its body's bounds.
            if self.bookkeeping.generation != layout {
                self.reindex_slots();
            }
            self.bookkeeping.forget_boxes();
            None
        } else {
            // Bodies awake at some point of the step, sorted for a canonical refit order. An
            // awake body whose box still holds its bounds keeps the box in its leaf for the
            // next step; any other leaf returns to its body's bounds.
            let s = &mut self.bookkeeping;
            let epoch = s.box_epoch();
            s.stepped.sort_unstable();
            s.stepped.dedup();
            let (bodies, broad) = (&self.bodies, &mut s.broad);
            s.stepped.retain(|&i| {
                let b = &bodies[i];
                let Some(e) = broad.get_mut(b.id.slot() as usize) else {
                    return true;
                };
                let (lo, hi) = b.cached_bounds;
                let keep = b.mass > 0.0
                    && !b.sleeping
                    && e.epoch == epoch
                    && e.lo.0 <= lo.0
                    && e.lo.1 <= lo.1
                    && e.lo.2 <= lo.2
                    && hi.0 <= e.hi.0
                    && hi.1 <= e.hi.1
                    && hi.2 <= e.hi.2;
                if !keep {
                    e.epoch = 0;
                }
                !keep
            });
            Some(s.stepped.as_slice())
        };
        self.query_tree
            .sync_step(&self.bodies, moved, generation, dt as Real);
        result
    }
    fn advance_substeps<const PREPARED: bool, const CACHED: bool>(
        &mut self,
        dt: Scalar,
    ) -> Result<Report, Error> {
        let mut report = Report::default();
        let h = substep_length(dt, self.scene.substeps);
        let layout = self.bookkeeping.generation;
        // Every per-body loop below runs over the awake bodies only (#412).
        {
            let s = &mut self.bookkeeping;
            s.activity.refresh(&self.bodies, &mut s.work);
            s.stepped.clear();
            reserve(&mut s.stepped, s.activity.indices.len(), &mut s.work);
            s.stepped.extend_from_slice(&s.activity.indices);
            // Replaced boxes leave their leaves behind in `broad_slots`: keep only the awake
            // bodies' boxes once that garbage outgrows them.
            let epoch = s.box_epoch();
            let live: usize = s
                .activity
                .indices
                .iter()
                .map(|&i| {
                    let e = s.broad.get(self.bodies[i].id.slot() as usize);
                    e.filter(|e| e.epoch == epoch).map_or(0, |e| e.len as usize)
                })
                .sum();
            if s.broad_slots.len() > 1024.max(2 * live) {
                let mut spare = std::mem::take(&mut s.broad_spare);
                spare.clear();
                reserve(&mut spare, live, &mut s.work);
                for &i in &s.activity.indices {
                    let Some(e) = s.broad.get_mut(self.bodies[i].id.slot() as usize) else {
                        continue;
                    };
                    if e.epoch == epoch {
                        let start = spare.len();
                        spare.extend_from_slice(
                            &s.broad_slots[e.start as usize..(e.start + e.len) as usize],
                        );
                        e.start = start as u32;
                    }
                }
                // Resting bodies hold no box (they dropped it on falling asleep).
                s.broad_spare = std::mem::replace(&mut s.broad_slots, spare);
            }
        }
        // Contacts are generated once per step and their constraints reused by later substeps,
        // as Rapier does (#378). A substep regenerates only after something the reused rows
        // cannot represent: a sweep or CCD advance, a wake, a sleep or a retirement.
        let mut reuse = false;
        for substep in 0..self.scene.substeps {
            let lookahead = h * Real::from(self.scene.substeps - 1 - substep);
            report.substeps += 1;
            self.bookkeeping.ccd.clear();
            self.bookkeeping.ccd_offsets.clear();
            self.bookkeeping.ccd_start.clear();
            self.bookkeeping.ccd_retired.clear();
            self.rest_per_body();
            let awake = self.bookkeeping.activity.indices.len();
            // Responses are prepared with the contacts: reused rows keep the step's response,
            // as their masses do (#378). Resting bodies keep the default response.
            if !reuse {
                crate::work::add(crate::work::Counter::BodyVisits, awake as u64);
                for &i in &self.bookkeeping.activity.indices {
                    self.responses[i] = if PREPARED {
                        PreparedResponse::new(&self.bodies[i], &mut report)
                    } else {
                        PreparedResponse::default()
                    };
                }
            }
            // External impulses still apply exactly once, before the first force integration.
            // Only awake bodies hold them: `apply_impulse` wakes its body.
            if substep == 0 {
                crate::work::add(crate::work::Counter::BodyVisits, awake as u64);
                for &i in &self.bookkeeping.activity.indices {
                    let b = &mut self.bodies[i];
                    if b.movable() || b.impulse != Vector::ZERO || b.angular_impulse != Vector::ZERO
                    {
                        self.transaction.body(i, b);
                    }
                    if b.movable() {
                        let prepared = self.responses[i].select::<PREPARED>(b, &mut report);
                        b.velocity += b.impulse * prepared.inverse_mass;
                        b.angular_velocity += prepared.inertia(b, b.angular_impulse, &mut report);
                    }
                    b.impulse = Vector::ZERO;
                    b.angular_impulse = Vector::ZERO;
                }
            }
            crate::work::add(crate::work::Counter::BodyVisits, awake as u64);
            for q in 0..awake {
                let i = self.bookkeeping.activity.indices[q];
                if self.bodies[i].movable() {
                    self.apply_substep_forces::<PREPARED>(i, h, &mut report);
                }
            }
            let mut pairs = std::mem::take(&mut self.manifold_scratch);
            let mut reuse_next = reuse;
            if !reuse {
                let regenerate =
                    match self.manifolds::<CACHED>(h, lookahead, &mut report, &mut pairs) {
                        Ok(regenerate) => regenerate,
                        Err(error) => {
                            self.manifold_scratch = pairs;
                            self.failed_work = Some(FailedStepWork::capture(
                                &report,
                                BookkeepingStats {
                                    scratch_retained_bytes: self.bookkeeping.retained_bytes()
                                        as u64,
                                    ..self.bookkeeping.work
                                },
                            ));
                            return Err(error);
                        }
                    };
                let mut roots = std::mem::take(&mut self.bookkeeping.roots);
                roots.clear();
                reserve(&mut roots, pairs.len() * 2, &mut self.bookkeeping.work);
                let up = (-self.scene.gravity).unit();
                roots.extend(
                    pairs
                        .iter()
                        // Only a sleeping side can be a root.
                        .filter(|(i, j, _)| self.bodies[*i].sleeping || self.bodies[*j].sleeping)
                        .flat_map(|(i, j, m)| {
                            let a = &self.bodies[*i];
                            let b = &self.bodies[*j];
                            let approach = -(b.velocity - a.velocity).dot(m.normal);
                            // A separating command alone is not wake evidence. An admitted
                            // previously load-bearing support moving away is a dependency loss.
                            let departing_support = approach < 0.0
                                && ((a.external && b.sleeping && m.normal.dot(up) > 0.5)
                                    || (b.external && a.sleeping && m.normal.dot(up) < -0.5))
                                && self.cache.get(&(a.order, b.order)).is_some_and(|points| {
                                    points
                                        .iter()
                                        .any(|p| p.impulse > 0.0 && p.normal.dot(m.normal) > 0.99)
                                });
                            let driven_contact_motion = (a.sleeping || b.sleeping)
                                && (a.external || b.external)
                                && (&m.points).into_iter().any(|point| {
                                    let spin =
                                        a.linear_support.is_none() && b.linear_support.is_none();
                                    let relative = contact_velocity(b, point.rb, spin)
                                        - contact_velocity(a, point.ra, spin);
                                    relative != Vector::ZERO && relative.dot(m.normal) <= 0.0
                                });
                            let disruptive = m.swept
                                || approach > self.scene.sleep_speed
                                || ((a.external || b.external) && approach > 0.0)
                                || departing_support
                                || driven_contact_motion;
                            [
                                if disruptive && a.sleeping && b.mass > 0.0 {
                                    Some(a.order)
                                } else {
                                    None
                                },
                                if disruptive && b.sleeping && a.mass > 0.0 {
                                    Some(b.order)
                                } else {
                                    None
                                },
                            ]
                        })
                        .flatten(),
                );
                let mut woke = 0;
                for root in roots.drain(..) {
                    woke += self.wake_contact_island::<PREPARED>(root, h, &mut report);
                }
                self.bookkeeping.roots = roots;
                report.woken_bodies += woke;
                if woke > 0 {
                    // Real response and forces are ready before the regenerated contact impulse.
                    if let Err(error) =
                        self.manifolds::<CACHED>(h, lookahead, &mut report, &mut pairs)
                    {
                        self.manifold_scratch = pairs;
                        self.failed_work = Some(FailedStepWork::capture(
                            &report,
                            BookkeepingStats {
                                scratch_retained_bytes: self.bookkeeping.retained_bytes() as u64,
                                ..self.bookkeeping.work
                            },
                        ));
                        return Err(error);
                    }
                }
                // CCD remaining-time impacts respond before the contact solve, so a struck body's
                // current contacts (e.g. its floor) still resist the impulse in this substep.
                {
                    let s = &mut self.bookkeeping;
                    for (i, j, m) in &pairs {
                        if m.swept && ccd::deferred(&self.bodies[*i], &self.bodies[*j]) {
                            for k in [*i, *j] {
                                if ccd::advances(&self.bodies[k]) {
                                    push(&mut s.ccd, k, &mut s.work);
                                }
                            }
                        }
                    }
                }
                if !self.bookkeeping.ccd.is_empty()
                    && let Err(error) = self.advance_ccd_bodies::<PREPARED>(h, &mut report)
                {
                    self.manifold_scratch = pairs;
                    self.failed_work = Some(FailedStepWork::capture(
                        &report,
                        BookkeepingStats {
                            scratch_retained_bytes: self.bookkeeping.retained_bytes() as u64,
                            ..self.bookkeeping.work
                        },
                    ));
                    return Err(error);
                }
                reuse_next = lookahead > 0.0
                    && !regenerate
                    && woke == 0
                    && self.bookkeeping.ccd.is_empty()
                    && !pairs.iter().any(|&(i, j, ref m)| {
                        // A tumbling corner can reach a feature this manifold lacks.
                        let tumbles = |b: &Body| {
                            !matches!(b.shape, Shape::Sphere(_))
                                && b.angular_velocity.length() * b.shape.radius() * lookahead
                                    > self.scene.contact_slop
                        };
                        m.swept || tumbles(&self.bodies[i]) || tumbles(&self.bodies[j])
                    });
            }
            let mut constraints = std::mem::take(&mut self.constraints);
            if reuse {
                self.advance_constraints(&mut constraints, h, &mut report);
            } else {
                constraints.clear();
                reserve(
                    &mut constraints,
                    pairs.iter().map(|(_, _, m)| m.points.len()).sum(),
                    &mut self.bookkeeping.work,
                );
                // Pairs and cache entries are both in pair order: a cursor finds each pair's history.
                let mut history = self.cache.iter().peekable();
                let mut last_history = None;
                for (i, j, m) in pairs.drain(..) {
                    // Remaining-time advancement already resolved every contact of an advanced CCD
                    // body along its actual path; start-pose constraints would act on its response
                    // velocity a second time.
                    if (m.swept && ccd::deferred(&self.bodies[i], &self.bodies[j]))
                        || self.bookkeeping.ccd.binary_search(&i).is_ok()
                        || self.bookkeeping.ccd.binary_search(&j).is_ok()
                    {
                        continue;
                    }
                    let a = &self.bodies[i];
                    let b = &self.bodies[j];
                    let n = m.normal;
                    let linear = a.linear_support.is_some() || b.linear_support.is_some();
                    let mut response = [true, true];
                    if let Some(s) = a.linear_support
                        && n.dot(s.unit()) > 0.5
                    {
                        response[1] = false;
                    }
                    if let Some(s) = b.linear_support
                        && (-n).dot(s.unit()) > 0.5
                    {
                        response[0] = false;
                    }
                    // A side without prepared inverse mass takes no impulse; skipping it changes no value.
                    let prepared = [
                        self.responses[i].select::<PREPARED>(a, &mut report),
                        self.responses[j].select::<PREPARED>(b, &mut report),
                    ];
                    response[0] &= prepared[0].inverse_mass > 0.0;
                    response[1] &= prepared[1].inverse_mass > 0.0;
                    // Without rotational response every direction has the same effective mass.
                    let spins = !linear
                        && ((response[0] && prepared[0].spins())
                            || (response[1] && prepared[1].spins()));
                    // Rows read and write angular velocity only when a side can spin in response or
                    // already spins (#375): upright crates on a fixed floor skip the cross products,
                    // whose zero terms would change at most the sign of a zero.
                    let row_spin = !linear
                        && (prepared[0].spins()
                            || prepared[1].spins()
                            || a.angular_velocity != Vector::ZERO
                            || b.angular_velocity != Vector::ZERO);
                    let t1 = if n.0.abs() < 0.7 {
                        n.cross(Vector::X).unit()
                    } else {
                        n.cross(Vector::Y).unit()
                    };
                    let t2 = n.cross(t1);
                    let key = (a.order, b.order);
                    // A pair with several manifolds (a box on heightfield triangles) shares
                    // one cache entry: its later manifolds read the same history.
                    let cached = match last_history {
                        Some((last, points)) if last == key => points,
                        _ => {
                            while history.next_if(|(k, _)| **k < key).is_some() {}
                            history
                                .next_if(|(k, _)| **k == key)
                                .map(|(_, points)| points)
                        }
                    };
                    last_history = Some((key, cached));
                    // A cached point warm-starts only within this reach. A half-space has zero
                    // radius, so its contacts never match and skip the search.
                    let warm_reach = 0.1 * a.shape.radius().min(b.shape.radius());
                    let used = &mut self.bookkeeping.used;
                    used.clear();
                    reserve(used, m.points.len(), &mut self.bookkeeping.work);
                    for point in &m.points {
                        let mut point = *point;
                        if !m.swept
                            && !linear
                            && a.movable()
                            && b.movable()
                            && !a.sleeping
                            && !b.sleeping
                            && response == [true, true]
                        {
                            // Opposite impulses at separate witnesses create an internal
                            // friction couple. Reciprocal dynamic current contacts share
                            // the midpoint along the contact normal. Projecting the witness
                            // gap avoids feeding tangential reconstruction roundoff into
                            // resting friction rows; separation still owns the target.
                            let delta = (b.position - a.position) + point.rb - point.ra;
                            let gap =
                                if matches!((a.shape, b.shape), (Shape::Box(_), Shape::Box(_))) {
                                    // Box clipping projects witnesses along this normal;
                                    // generic primitive support vertices need the full gap.
                                    n * delta.dot(n)
                                } else {
                                    delta
                                };
                            let shift = gap * 0.5;
                            point.ra += shift;
                            point.rb -= shift;
                        }
                        let response_bodies = [(a, &self.responses[i]), (b, &self.responses[j])];
                        let arms = [point.ra, point.rb];
                        let k = effective_mass::<PREPARED>(
                            response_bodies,
                            arms,
                            n,
                            response,
                            !linear,
                            &mut report,
                        );
                        if k <= 1e-14 {
                            continue;
                        }
                        let rel = contact_velocity(b, point.rb, !linear)
                            - contact_velocity(a, point.ra, !linear);
                        let restitution = if cached.is_none() {
                            pair_restitution(a, b)
                        } else {
                            0.0
                        };
                        let bias = contact_bias(
                            point.separation,
                            restitution,
                            rel.dot(n),
                            h,
                            self.scene.contact_slop,
                            self.restitution_speed,
                        );
                        let mut c = Constraint {
                            a: i,
                            b: j,
                            n,
                            ra: point.ra,
                            rb: point.rb,
                            t1,
                            t2,
                            normal_mass: 1.0 / k,
                            tangent_mass: if spins {
                                [
                                    effective_mass::<PREPARED>(
                                        response_bodies,
                                        arms,
                                        t1,
                                        response,
                                        !linear,
                                        &mut report,
                                    )
                                    .recip(),
                                    effective_mass::<PREPARED>(
                                        response_bodies,
                                        arms,
                                        t2,
                                        response,
                                        !linear,
                                        &mut report,
                                    )
                                    .recip(),
                                ]
                            } else {
                                [k.recip(); 2]
                            },
                            bias,
                            // Additional corners can limit approach while still separated at the
                            // admitted contact time. They have no frictional load yet.
                            friction: if linear
                                || (contact::fixed_box_interval(a, b)
                                    && point.separation
                                        + (b.velocity - a.velocity).dot(n) * h * m.time
                                        > self.scene.contact_slop)
                            {
                                0.0
                            } else {
                                pair_friction(a, b)
                            },
                            normal_impulse: 0.0,
                            tangent_impulse: [0.0; 2],
                            sep: point.separation,
                            swept: m.swept,
                            response,
                            spin: row_spin,
                            offset: b.position - a.position,
                            restitution,
                        };
                        if self.scene.warm_start
                            && warm_reach > 0.0
                            && point.separation <= self.scene.contact_slop
                            && self.last_h > 0.0
                            && let Some(old) = cached
                        {
                            let la = a.orientation.inverse_rotate(c.ra);
                            let lb = b.orientation.inverse_rotate(c.rb);
                            // The nearest unused cached point with this normal; ties keep the first.
                            let mut nearest: Option<(usize, &CachedPoint, Real)> = None;
                            for (idx, p) in old.iter().enumerate() {
                                if used.contains(&idx) || p.normal.dot(n) <= 0.99 {
                                    continue;
                                }
                                let d = (p.a - la).dot(p.a - la) + (p.b - lb).dot(p.b - lb);
                                if nearest.is_none_or(|(_, _, best)| d.total_cmp(&best).is_lt()) {
                                    nearest = Some((idx, p, d));
                                }
                            }
                            if let Some((idx, p, _)) = nearest
                                && (p.a - la).length() + (p.b - lb).length() < warm_reach
                            {
                                used.push(idx);
                                let scale = (h / narrow_time(self.last_h)).clamp(0.0, 2.0);
                                c.normal_impulse = p.impulse * scale;
                                c.tangent_impulse =
                                    [p.tangent.dot(t1) * scale, p.tangent.dot(t2) * scale];
                            }
                        }
                        report.max_penetration =
                            report.max_penetration.max((-point.separation).max(0.0));
                        constraints.push(c);
                    }
                }
            }
            self.manifold_scratch = pairs;
            report.contact_points += constraints.len() as u64;
            for c in &constraints {
                self.transaction.body(c.a, &self.bodies[c.a]);
                self.transaction.body(c.b, &self.bodies[c.b]);
                apply::<PREPARED>(
                    &mut self.bodies,
                    &self.responses,
                    c,
                    c.n * c.normal_impulse
                        + c.t1 * c.tangent_impulse[0]
                        + c.t2 * c.tangent_impulse[1],
                    &mut report,
                );
            }
            match self.scene.convergence {
                Some(tolerances)
                    if self.scene.convergence_scope == ConvergenceScope::ContactIslands =>
                {
                    islands::solve::<PREPARED>(
                        &mut self.bodies,
                        &self.responses,
                        &mut constraints,
                        self.scene.velocity_iterations,
                        tolerances,
                        &mut self.island_scratch,
                        &mut report,
                    )
                }
                Some(tolerances) => convergence::solve::<PREPARED, true>(
                    &mut self.bodies,
                    &self.responses,
                    &mut constraints,
                    self.scene.velocity_iterations,
                    tolerances,
                    &mut report,
                ),
                None => convergence::solve::<PREPARED, false>(
                    &mut self.bodies,
                    &self.responses,
                    &mut constraints,
                    self.scene.velocity_iterations,
                    Convergence::default(),
                    &mut report,
                ),
            }
            let scratch = &mut self.bookkeeping;
            scratch.retired.clear();
            scratch.support_edges.clear();
            // Immovable and sleeping bodies stay supported (`Scratch::per_body`); awake ones
            // start from whether they are kinematic at rest.
            scratch.activity.refresh(&self.bodies, &mut scratch.work);
            crate::work::add(
                crate::work::Counter::BodyVisits,
                scratch.activity.indices.len() as u64,
            );
            for &i in &scratch.activity.indices {
                let b = &self.bodies[i];
                scratch.supported[i] =
                    b.external && b.velocity == Vector::ZERO && b.angular_velocity == Vector::ZERO;
            }
            // Reuse point allocations when the pair survives. Warm starting has already read the
            // old impulses; only adjacency-key changes invalidate the graph, not updated impulses.
            // Cache entries and a pair's contiguous constraint run are both in pair order: walk
            // them together, so neither side searches the other (#375). New pairs are inserted
            // after the walk.
            let up = (-self.scene.gravity).unit();
            debug_assert!(
                constraints
                    .is_sorted_by_key(|c| { (self.bodies[c.a].order, self.bodies[c.b].order) })
            );
            let mut runs = constraints
                .chunk_by(|x, y| x.a == y.a && x.b == y.b)
                .peekable();
            let mut entries = self.cache.iter_mut().peekable();
            let mut fresh = Vec::new();
            loop {
                let run_key = runs
                    .peek()
                    .map(|run| (self.bodies[run[0].a].order, self.bodies[run[0].b].order));
                let entry_key = entries.peek().map(|(key, _)| **key);
                let (run, entry) = match (run_key, entry_key) {
                    (None, None) => break,
                    (Some(r), Some(e)) if r == e => (runs.next(), entries.next()),
                    (Some(r), e) if e.is_none_or(|e| r < e) => (runs.next(), None),
                    _ => (None, entries.next()),
                };
                let entry = entry.map(|(&key, points)| {
                    let (a, b) = match run {
                        Some(run) => (run[0].a, run[0].b),
                        None => (
                            self.bodies
                                .binary_search_by_key(&key.0, |b| b.order)
                                .expect("live contact"),
                            self.bodies
                                .binary_search_by_key(&key.1, |b| b.order)
                                .expect("live contact"),
                        ),
                    };
                    if !scratch.supported[a] || !scratch.supported[b] {
                        if !points.is_empty() {
                            self.transaction.pair(key, Some(points));
                        }
                        points.clear();
                    }
                    points
                });
                let Some(run) = run else {
                    continue;
                };
                let a = &self.bodies[run[0].a];
                let b = &self.bodies[run[0].b];
                let key = (a.order, b.order);
                let cached = run.iter().any(|c| c.sep <= self.scene.contact_slop * 2.0);
                let mut new = None;
                let mut points = match entry {
                    _ if !cached => None,
                    Some(points) => {
                        self.transaction.pair(key, Some(points));
                        Some(points)
                    }
                    None => {
                        self.transaction.pair(key, None);
                        scratch.graph.dirty = true;
                        Some(new.insert(Vec::new()))
                    }
                };
                for c in run {
                    if c.normal_impulse > 0.0 {
                        if a.retire_on_impact {
                            push(&mut scratch.retired, a.order, &mut scratch.work);
                        }
                        if b.retire_on_impact {
                            push(&mut scratch.retired, b.order, &mut scratch.work);
                        }
                        if c.swept {
                            report.swept_contacts += 1;
                        }
                    }
                    if c.sep <= self.scene.contact_slop * 2.0
                        && let Some(points) = points.as_deref_mut()
                    {
                        points.push(CachedPoint {
                            a: a.orientation.inverse_rotate(c.ra),
                            b: b.orientation.inverse_rotate(c.rb),
                            normal: c.n,
                            impulse: c.normal_impulse,
                            tangent: c.t1 * c.tangent_impulse[0] + c.t2 * c.tangent_impulse[1],
                        });
                        if c.n.dot(up) > 0.5 {
                            push(&mut scratch.support_edges, (c.a, c.b), &mut scratch.work);
                        }
                        if c.n.dot(up) < -0.5 {
                            push(&mut scratch.support_edges, (c.b, c.a), &mut scratch.work);
                        }
                    }
                }
                if let Some(points) = new {
                    fresh.push((key, points));
                }
            }
            self.cache.extend(fresh);
            self.cache.retain(|&key, points| {
                if points.is_empty() {
                    self.transaction.pair(key, Some(points));
                    scratch.graph.dirty = true;
                    false
                } else {
                    true
                }
            });
            let scratch = &mut self.bookkeeping;
            scratch.support.propagate(
                &mut scratch.supported,
                &scratch.support_edges,
                &mut scratch.work,
            );
            scratch.activity.refresh(&self.bodies, &mut scratch.work);
            crate::work::add(
                crate::work::Counter::BodyVisits,
                scratch.activity.indices.len() as u64,
            );
            for &i in &scratch.activity.indices {
                let b = &mut self.bodies[i];
                self.transaction.body(i, b);
                report.integrated_bodies += 1;
                let (movement, spin, correction_quiet) = (b.velocity, b.angular_velocity, true);
                b.position += movement * h;
                if let Some(offset) = scratch.ccd_offsets.get(i) {
                    b.position += *offset;
                }
                if !b.rotation_locked {
                    b.orientation = b.orientation.integrate(spin, h);
                }
                debug_assert_eq!(b.motion_valid(), b.valid());
                if !b.motion_valid() {
                    let error = Error::NonFiniteState(b.id);
                    self.failed_work = Some(FailedStepWork::capture(
                        &report,
                        BookkeepingStats {
                            scratch_retained_bytes: scratch.retained_bytes() as u64,
                            ..scratch.work
                        },
                    ));
                    return Err(error);
                }
                if b.movable()
                    && b.sleep_allowed
                    && scratch.supported[i]
                    && correction_quiet
                    && b.velocity.length() + b.angular_velocity.length() * b.shape.radius()
                        < self.scene.sleep_speed
                {
                    b.quiet_time += h;
                } else {
                    b.quiet_time = 0.0;
                }
                b.cached_bounds = contact::bounds(b);
            }
            scratch.ccd_offsets.clear();
            self.constraints = constraints;
            if let Err(error) = self.correct_positions(h, &mut report.position) {
                self.failed_work = Some(FailedStepWork::capture(
                    &report,
                    BookkeepingStats {
                        scratch_retained_bytes: self.bookkeeping.retained_bytes() as u64,
                        ..self.bookkeeping.work
                    },
                ));
                return Err(error);
            }
            let active = self.bookkeeping.activity.count;
            self.sleep_quiet_islands();
            reuse_next &= self.bookkeeping.activity.count == active;
            // Retirement is local; remove all affected cached edges before indexed graph reuse.
            let s = &mut self.bookkeeping;
            for n in 0..s.ccd_retired.len() {
                push(&mut s.retired, s.ccd_retired[n], &mut s.work);
            }
            s.ccd_retired.clear();
            self.bookkeeping.retired.sort_unstable();
            self.bookkeeping.retired.dedup();
            for n in 0..self.bookkeeping.retired.len() {
                let id = self.bookkeeping.retired[n];
                if let Some(i) = self.index_of_order(id) {
                    let b = self.bodies.remove(i);
                    let handle = b.id;
                    self.bookkeeping.activity.count -= usize::from(b.mass > 0.0 && !b.sleeping);
                    self.geometry.remove(id);
                    self.cache.retain(|&(a, b), points| {
                        if a == id || b == id {
                            self.transaction.pair((a, b), Some(points));
                            false
                        } else {
                            true
                        }
                    });
                    self.transaction.remove(i, b);
                    // Later substeps look bodies up by query-tree slot.
                    self.shift_slots(i);
                    let synced = self.query_tree.current(self.bookkeeping.generation);
                    self.bookkeeping.layout_changed();
                    if synced {
                        self.query_tree
                            .removed(handle.slot(), self.bookkeeping.generation);
                    } else {
                        self.bookkeeping.forget_boxes();
                    }
                    self.bookkeeping.drop_box(handle.slot());
                    report.retired.push(handle);
                }
            }
            // Reused rows hold storage indices: a retirement shifts them.
            reuse = reuse_next && self.bookkeeping.retired.is_empty();
            self.last_h = widen_time(h);
        }
        // Only bodies awake at the step's start hold forces: `add_force` wakes its body.
        let s = &mut self.bookkeeping;
        let all = s.generation != layout;
        let count = if all {
            self.bodies.len()
        } else {
            s.stepped.len()
        };
        crate::work::add(crate::work::Counter::BodyVisits, count as u64);
        for q in 0..count {
            let i = if all { q } else { s.stepped[q] };
            let b = &mut self.bodies[i];
            if b.force != Vector::ZERO || b.torque != Vector::ZERO {
                self.transaction.body(i, b);
            }
            b.force = Vector::ZERO;
            b.torque = Vector::ZERO;
        }
        self.elapsed += dt;
        self.finish_bookkeeping(&mut report);
        Ok(report)
    }
    /// Carries the step's constraint rows into the next substep (#378). Normals, arms, masses and
    /// friction stay as generated; each row's separation advances by the bodies' relative
    /// motion along the normal and the position target follows it. Translation is measured
    /// (it includes position correction); rotation is linearised at the fixed arms, so a
    /// rolling sphere's support stays exact. The accumulated impulses stay on the rows and
    /// warm-start this substep.
    fn advance_constraints(&self, constraints: &mut [Constraint], h: Real, report: &mut Report) {
        let slop = self.scene.contact_slop;
        for run in constraints.chunk_by_mut(|x, y| x.a == y.a && x.b == y.b) {
            // A pair stops bouncing once it has a cached contact, as a regenerated row would.
            let cached = run.iter().any(|c| c.sep <= slop * 2.0);
            for c in run {
                let (a, b) = (&self.bodies[c.a], &self.bodies[c.b]);
                let offset = b.position - a.position;
                c.sep += (offset - c.offset).dot(c.n);
                if c.spin {
                    c.sep += (b.angular_velocity.cross(c.rb) - a.angular_velocity.cross(c.ra))
                        .dot(c.n)
                        * h;
                }
                c.offset = offset;
                if cached {
                    c.restitution = 0.0;
                }
                let approach = if c.restitution > 0.0 {
                    (contact_velocity(b, c.rb, c.spin) - contact_velocity(a, c.ra, c.spin)).dot(c.n)
                } else {
                    0.0
                };
                c.bias = contact_bias(
                    c.sep,
                    c.restitution,
                    approach,
                    h,
                    slop,
                    self.restitution_speed,
                );
                report.max_penetration = report.max_penetration.max((-c.sep).max(0.0));
            }
        }
    }
    fn apply_substep_forces<const PREPARED: bool>(
        &mut self,
        index: usize,
        h: Real,
        report: &mut Report,
    ) {
        let b = &mut self.bodies[index];
        self.transaction.body(index, b);
        let prepared = self.responses[index].select::<PREPARED>(b, report);
        b.velocity += (self.scene.gravity + b.force / b.mass) * h;
        b.angular_velocity += prepared.inertia(b, b.torque, report) * h;
        if b.linear_damping > 0.0 {
            b.velocity = b.velocity * (1.0 / (1.0 + h * b.linear_damping));
        }
        if b.angular_damping > 0.0 {
            b.angular_velocity = b.angular_velocity * (1.0 / (1.0 + h * b.angular_damping));
        }
    }
    fn finish_bookkeeping(&mut self, report: &mut Report) {
        report.islands.scratch_retained_bytes = self.island_scratch.retained_bytes();
        report.position.scratch_retained_bytes = self.bookkeeping.position.retained_bytes();
        report.geometry.retained_bytes = self.geometry.retained_bytes() as u64;
        self.bookkeeping.work.scratch_retained_bytes = (self.bookkeeping.retained_bytes()
            + bookkeeping::bytes(&self.constraints)
            + bookkeeping::bytes(&self.manifold_scratch))
            as u64;
        report.bookkeeping = std::mem::take(&mut self.bookkeeping.work);
    }
    fn sleep_quiet_islands(&mut self) {
        let s = &mut self.bookkeeping;
        s.activity.refresh(&self.bodies, &mut s.work);
        s.ensure_graph(&self.bodies, &self.cache);
        s.traversal.begin(self.bodies.len(), &mut s.work);
        let mut slept = 0;
        // Fully sleeping components need no rediscovery. An awake root still reaches every
        // dynamic neighbor, including sleeping members, so support and wake semantics are unchanged.
        for &root in &s.activity.indices {
            s.traversal
                .collect(root, &self.bodies, &s.graph, &mut s.work);
            if s.traversal.island.iter().all(|&i| {
                let b = &self.bodies[i];
                b.sleeping || (b.sleep_allowed && b.quiet_time >= self.scene.sleep_seconds)
            }) {
                for &i in &s.traversal.island {
                    let b = &mut self.bodies[i];
                    if b.sleeping {
                        continue;
                    }
                    self.transaction.body(i, b);
                    slept += 1;
                    b.sleeping = true;
                    b.velocity = Vector::ZERO;
                    b.angular_velocity = Vector::ZERO;
                    // Resting values (`Scratch::per_body`), and a leaf at the body's bounds:
                    // later substeps find it as a resting body. Boxes that overlapped its box
                    // may not hold its leaf (it may have held theirs): they query again.
                    self.responses[i] = PreparedResponse::default();
                    s.supported[i] = true;
                    let slot = b.id.slot();
                    if self.query_tree.current(s.generation) {
                        let e = s.broad.get(slot as usize).copied().unwrap_or_default();
                        let near = if e.epoch == s.broad_epoch + 1 {
                            (e.lo, e.hi)
                        } else {
                            b.cached_bounds
                        };
                        drop_boxes_near(
                            &self.query_tree,
                            &mut s.broad,
                            near,
                            self.scene.contact_slop,
                        );
                        self.query_tree.moved(b);
                    } else {
                        s.broad_epoch += 1;
                        s.boxes_live = false;
                    }
                    if let Some(e) = s.broad.get_mut(slot as usize) {
                        e.epoch = 0;
                    }
                }
            }
        }
        s.activity.count -= slept;
        if slept > 0 {
            s.activity.slept(&self.bodies);
        }
    }
    /// Gives every resting body its resting per-body values (`Scratch::per_body`) after a
    /// layout change or rollback; O(1) otherwise.
    fn rest_per_body(&mut self) {
        let s = &mut self.bookkeeping;
        s.activity.refresh(&self.bodies, &mut s.work);
        let n = self.bodies.len();
        if s.per_body == Some(s.generation)
            && self.responses.len() == n
            && s.supported.len() == n
            && s.earliest.len() == n
        {
            return;
        }
        crate::work::add(crate::work::Counter::BodyVisits, n as u64);
        self.responses.clear();
        reserve(&mut self.responses, n, &mut s.work);
        self.responses.resize(n, PreparedResponse::default());
        s.supported.clear();
        reserve(&mut s.supported, n, &mut s.work);
        s.supported.resize(n, true);
        s.earliest.clear();
        reserve(&mut s.earliest, n, &mut s.work);
        s.earliest.resize(n, Real::INFINITY);
        s.earliest_set.clear();
        s.per_body = Some(s.generation);
    }

    /// Contacts for this substep plus `lookahead`, the rest of the step: pairs within reach
    /// before the step ends are admitted as speculative contacts, so later substeps can reuse
    /// the constraints (#378). Returns whether the next substep must regenerate instead: a pair
    /// needed a swept test, which covers only this substep, or two dynamic bodies can close.
    fn manifolds<const CACHED: bool>(
        &mut self,
        h: Real,
        lookahead: Real,
        report: &mut Report,
        out: &mut Vec<(usize, usize, contact::Manifold)>,
    ) -> Result<bool, Error> {
        let mut regenerate = false;
        let interval = h + lookahead;
        // Displacement per unit of constant acceleration over the later substeps: velocity
        // gains a·h per substep after this one, so the step ends a·L·(L + h)/2 further on.
        let gain = lookahead * interval * 0.5;
        let step = h * Real::from(self.scene.substeps);
        self.rest_per_body();
        let in_order = self.storage_sorted();
        let s = &mut self.bookkeeping;
        if CACHED {
            self.geometry.begin(self.bodies.len());
        }
        out.clear();
        // Broad phase (#412), as Rapier's: the query tree holds every body, so pairs come only
        // from awake bodies and a static, sleeping or resting kinematic body is visited only when
        // an awake one reaches it. Pairs of two resting bodies produce no contact. An awake
        // body's leaf holds its box (`BroadBox`, a fat AABB with room for a few steps of motion);
        // the body queries the tree only when its swept bounds leave the box, and keeps the
        // leaves it found until then. Of two awake bodies whose bounds overlap, the one whose box
        // queried last found the other. A resting body that changes drops the boxes near it.
        if !self.query_tree.current(s.generation) {
            crate::work::add(crate::work::Counter::BodyVisits, self.bodies.len() as u64);
            self.query_tree.sync(&self.bodies, s.generation);
            s.forget_boxes();
        }
        s.activity.refresh(&self.bodies, &mut s.work);
        s.bounds.clear();
        reserve(&mut s.bounds, s.activity.indices.len(), &mut s.work);
        crate::work::add(
            crate::work::Counter::BodyVisits,
            s.activity.indices.len() as u64,
        );
        // A body's candidate bounds over the rest of the step; a resting body's are its bounds
        // padded by the contact slop.
        let swept = |b: &Body| {
            let (lo, hi) = b.cached_bounds;
            let delta = if b.sleeping {
                Vector::ZERO
            } else {
                b.velocity * interval
            };
            let angular = b.angular_velocity.length() * b.shape.radius() * interval;
            // Gravity and forces keep accelerating the body through the later substeps.
            let drift = step_drift(b, self.scene.gravity, gain).abs();
            let pad = Vector(angular, angular, angular)
                + drift
                + Vector(1.0, 1.0, 1.0)
                    * (self.scene.contact_slop
                        // Endpoint addition can round at the displacement's scale even
                        // when the original pose and collider are small.
                        + 8.0 * Real::EPSILON * delta.abs().max_component());
            (lo.min(lo + delta) - pad, hi.max(hi + delta) + pad)
        };
        let finite = |(lo, hi): (Vector, Vector)| lo.finite() && hi.finite();
        let epoch = s.box_epoch();
        if s.broad.len() < self.slots.len() {
            reserve(&mut s.broad, self.slots.len(), &mut s.work);
            s.broad
                .resize(self.slots.len(), bookkeeping::BroadBox::default());
        }
        // Room a new box leaves around the swept bounds, besides the motion it predicts.
        let room = Vector(1.0, 1.0, 1.0) * (20.0 * self.scene.contact_slop);
        let ahead = step * BOX_STEPS;
        let mut made = 0;
        for &i in &s.activity.indices {
            let b = &self.bodies[i];
            let (lo, hi) = swept(b);
            let e = &mut s.broad[b.id.slot() as usize];
            e.row = s.bounds.len() as u32;
            push(&mut s.bounds, (i, lo, hi), &mut s.work);
            s.work.bounds_updates += 1;
            // Non-finite bounds (an overflowing velocity) cannot enter the tree: that body
            // tests every body below, and its narrow phase reports the failure.
            if !finite((lo, hi)) {
                e.epoch = 0;
                continue;
            }
            if e.epoch == epoch
                && e.lo.0 <= lo.0
                && e.lo.1 <= lo.1
                && e.lo.2 <= lo.2
                && hi.0 <= e.hi.0
                && hi.1 <= e.hi.1
                && hi.2 <= e.hi.2
            {
                continue;
            }
            let d = b.velocity * ahead;
            let (bl, bh) = (lo.min(lo + d) - room, hi.max(hi + d) + room);
            if !finite((bl, bh)) {
                e.epoch = 0;
                continue;
            }
            (e.epoch, e.lo, e.hi) = (epoch, bl, bh);
            s.boxes_live = true;
            // Marks the box for a query below.
            e.len = u32::MAX;
            self.query_tree.swept(b, (bl, bh), step);
            made += 1;
        }
        // New boxes query the tree once every awake leaf holds its box. The leaves a box
        // overlaps within the contact slop are kept with it.
        let slop = Vector(1.0, 1.0, 1.0) * self.scene.contact_slop;
        let mut nodes = 0;
        if made > 0 {
            let needed = s.broad_slots.len() + made * 4;
            reserve(&mut s.broad_slots, needed, &mut s.work);
        }
        for &i in &s.activity.indices {
            let own = self.bodies[i].id.slot();
            let e = s.broad[own as usize];
            if e.epoch != epoch || e.len != u32::MAX {
                continue;
            }
            let start = s.broad_slots.len();
            nodes += self.query_tree.tree().visit_aabb(
                spatial_kernels::Aabb {
                    min: [e.lo.0 - slop.0, e.lo.1 - slop.1, e.lo.2 - slop.2],
                    max: [e.hi.0 + slop.0, e.hi.1 + slop.1, e.hi.2 + slop.2],
                },
                |leaf| {
                    if leaf.id != own {
                        push(&mut s.broad_slots, leaf.id, &mut s.work);
                    }
                },
            );
            s.queries += 1;
            let e = &mut s.broad[own as usize];
            e.query = s.queries;
            e.start = start as u32;
            e.len = (s.broad_slots.len() - start) as u32;
        }
        crate::work::add(crate::work::Counter::TreeNodes, nodes as u64);
        let mut candidates = std::mem::take(&mut s.candidates);
        candidates.clear();
        let (rows, broad) = (&s.bounds, &s.broad);
        let has_box = |b: &Body| broad[b.id.slot() as usize].epoch == epoch;
        // The candidate pair of awake `i` (bounds `lo`, `hi`) and any body `j`, reported once:
        // a pair of awake bodies by the one whose box queried last (it holds the other's leaf),
        // a pair with a body without a box by that body's scan.
        let mut consider = |i: usize, (lo, hi): (Vector, Vector), j: usize| {
            let Some(b) = self.bodies.get(j) else {
                // The slot of a body removed since the box queried.
                return;
            };
            if j == i || b.id == BodyId::NONE {
                return;
            }
            let (bl, bh) = if b.mass > 0.0 && !b.sleeping {
                let e = broad[b.id.slot() as usize];
                let (_, bl, bh) = rows[e.row as usize];
                match (has_box(&self.bodies[i]), has_box(b)) {
                    (true, true) if broad[self.bodies[i].id.slot() as usize].query < e.query => {
                        return;
                    }
                    (true, false) => return,
                    (false, false) if j < i => return,
                    _ => {}
                }
                (bl, bh)
            } else if b.velocity == Vector::ZERO && b.angular_velocity == Vector::ZERO {
                // `swept` of a body at rest.
                let (bl, bh) = b.cached_bounds;
                (bl - slop, bh + slop)
            } else {
                swept(b)
            };
            report.pair_tests += 1;
            if bl.0 > hi.0
                || bh.0 < lo.0
                || bl.1 > hi.1
                || bh.1 < lo.1
                || bl.2 > hi.2
                || bh.2 < lo.2
            {
                return;
            }
            let pair = if self.bodies[i].order < b.order {
                (i, j)
            } else {
                (j, i)
            };
            push(&mut candidates, pair, &mut s.work);
        };
        for &(i, lo, hi) in rows {
            let e = broad[self.bodies[i].id.slot() as usize];
            if e.epoch != epoch {
                crate::work::add(crate::work::Counter::BodyVisits, self.bodies.len() as u64);
                for j in 0..self.bodies.len() {
                    consider(i, (lo, hi), j);
                }
                continue;
            }
            for &slot in &s.broad_slots[e.start as usize..(e.start + e.len) as usize] {
                consider(i, (lo, hi), self.slots[slot as usize].index as usize);
            }
        }
        // Narrow phase in pair order, so the first failure and every cache visit are canonical.
        // Settled storage is in body order.
        if in_order {
            candidates.sort_unstable();
        } else {
            candidates.sort_unstable_by_key(|&(i, j)| (self.bodies[i].order, self.bodies[j].order));
        }
        debug_assert!(candidates.windows(2).all(|w| w[0] != w[1]));
        for &(i, j) in &candidates {
            {
                let a = &self.bodies[i];
                let b = &self.bodies[j];
                let driven_contact_candidate = (a.movable()
                    && b.external
                    && (b.velocity != Vector::ZERO || b.angular_velocity != Vector::ZERO))
                    || (b.movable()
                        && a.external
                        && (a.velocity != Vector::ZERO || a.angular_velocity != Vector::ZERO));
                let driven_wake_candidate = driven_contact_candidate && (a.sleeping || b.sleeping);
                // Boxes take rigid contacts with heightfield terrain, one manifold per triangle.
                // As with halfspaces, the approach over the step is the speculative margin.
                if let (Shape::Heightfield { terrain, .. }, body)
                | (body, Shape::Heightfield { terrain, .. }) = (a.shape, b.shape)
                    && let Shape::Box(_) = body
                {
                    let moving = if matches!(a.shape, Shape::Box(_)) {
                        a
                    } else {
                        b
                    };
                    if moving.inverse_mass() == 0.0
                        || moving.sensor
                        || moving.sleeping
                        || !a.layers.collides_with(b.layers)
                    {
                        continue;
                    }
                    report.narrow_tests += 1;
                    let approach = moving.velocity.length() * interval
                        + step_drift(moving, self.scene.gravity, gain).length()
                        + moving.angular_velocity.length() * moving.shape.radius() * interval;
                    contact::heightfield_box(
                        a,
                        b,
                        &self.terrains[terrain.0 as usize].prepared,
                        self.scene.contact_slop + approach,
                        interval,
                        |m| push(out, (i, j, m), &mut s.work),
                    );
                    continue;
                }
                if a.sensor
                    || b.sensor
                    // Terrain answers queries and character moves; halfspace contacts with
                    // dynamic bodies are generated after this sweep.
                    || a.shape.query_only()
                    || b.shape.query_only()
                    || !a.layers.collides_with(b.layers)
                    || (a.inverse_mass() == 0.0
                        && b.inverse_mass() == 0.0
                        && !driven_wake_candidate)
                {
                    continue;
                }
                let speed = (b.velocity - a.velocity).length();
                let travel = speed * h;
                // Retain full-interval discovery after response preparation wakes the
                // body; the second geometry pass must not discard an admitted sweep.
                // A fixed boundary may be crossed below the shape-sized fast threshold.
                // The current margin only rules out travel no larger than that margin.
                let fast = driven_contact_candidate
                    || a.ccd
                    || b.ccd
                    || ((a.mass == 0.0 || b.mass == 0.0) && travel > self.scene.contact_slop)
                    || travel
                        > 0.5
                            * a.shape
                                .half_extents()
                                .min_component()
                                .min(b.shape.half_extents().min_component());
                // A tolerance-only manifold is not wake evidence. Probe the actual surface
                // and full interval before replacing the parked body's zero response.
                let margin = if driven_wake_candidate {
                    0.0
                } else {
                    self.scene.contact_slop
                };
                // A separated pair slow enough to skip the sweep is admitted speculatively
                // when it can close before the step ends; a fast pair keeps this substep's
                // sweep instead.
                let reach = if fast || lookahead == 0.0 {
                    0.0
                } else {
                    (speed
                        + a.angular_velocity.length() * a.shape.radius()
                        + b.angular_velocity.length() * b.shape.radius())
                        * lookahead
                        + (step_drift(b, self.scene.gravity, gain)
                            - step_drift(a, self.scene.gravity, gain))
                        .length()
                };
                report.narrow_tests += 1;
                // GeometryCache shares read-only frames even for rotating pairs; it retains
                // complete pair results only when both orientation dependencies are stable.
                let current = if CACHED {
                    self.geometry
                        .query_interval([i, j], [a, b], h, margin, &mut report.geometry)
                } else {
                    contact::current_interval_counted(a, b, h, margin, &mut report.geometry)
                };
                let m = if current.is_some() {
                    current
                } else if fast {
                    regenerate = true;
                    let swept = if CACHED {
                        self.geometry
                            .swept([i, j], [a, b], h, margin, &mut report.geometry)
                    } else {
                        contact::swept(a, b, h, margin, &mut report.geometry)
                    };
                    swept.map_err(|reason| Error::CollisionSearchFailed {
                        bodies: [a.id, b.id],
                        reason,
                    })?
                } else if reach > 0.0 {
                    let margin = margin + reach;
                    let speculative = if CACHED {
                        self.geometry.query_interval(
                            [i, j],
                            [a, b],
                            h,
                            margin,
                            &mut report.geometry,
                        )
                    } else {
                        contact::current_interval_counted(a, b, h, margin, &mut report.geometry)
                    };
                    if speculative.is_some() && a.mass > 0.0 && b.mass > 0.0 {
                        // Speculative rows between two dynamic bodies make stacks rock: such
                        // a pair is discovered by the next substep's regeneration instead.
                        regenerate = true;
                        None
                    } else {
                        speculative
                    }
                } else {
                    None
                };
                if let Some(m) = m {
                    push(out, (i, j, m), &mut s.work);
                }
            }
        }
        s.candidates = candidates;
        if CACHED {
            self.geometry.finish(&mut report.geometry);
        }
        // Pairs are solved in pair order. The broad phase's pairs were tested in pair order, a
        // pair's several manifolds (a box on heightfield triangles) in generation order; each
        // halfspace's pairs below come in pair order too.
        let sorted = self.storage_sorted();
        debug_assert!(out.is_sorted_by(|x, y| pair_order(&self.bodies, sorted)(x, y).is_le()));
        let swept_pairs = out.len();
        // Halfspaces are unbounded and stay out of the sweep: each one is tested against every
        // awake dynamic body. The approach over the substep is admitted as a speculative margin,
        // so a fast body closes exactly to the plane.
        for p in 0..self.halfspaces.len() {
            let Ok(plane) = self.index(self.halfspaces[p]) else {
                continue;
            };
            let n = self.bodies[plane].orientation.rotate(Vector::Y);
            let awake = self.bookkeeping.activity.indices.len();
            crate::work::add(crate::work::Counter::BodyVisits, awake as u64);
            // Only awake dynamic bodies take halfspace contacts.
            for q in 0..awake {
                let k = self.bookkeeping.activity.indices[q];
                let (a, b) = (&self.bodies[plane], &self.bodies[k]);
                if b.id == BodyId::NONE
                    || b.inverse_mass() == 0.0
                    || b.sensor
                    || b.shape.query_only()
                    || !a.layers.collides_with(b.layers)
                {
                    continue;
                }
                report.narrow_tests += 1;
                let approach = (-b.velocity.dot(n)).max(0.0) * interval
                    + (-step_drift(b, self.scene.gravity, gain).dot(n)).max(0.0)
                    + b.angular_velocity.length() * b.shape.radius() * interval;
                let (i, j) = if a.order < b.order {
                    (plane, k)
                } else {
                    (k, plane)
                };
                if let Some(m) = contact::halfspace(
                    &self.bodies[i],
                    &self.bodies[j],
                    self.scene.contact_slop + approach,
                    interval,
                ) {
                    push(out, (i, j, m), &mut self.bookkeeping.work);
                }
            }
        }
        // A floor added before every dynamic body precedes all of their pairs: then the
        // halfspace pairs only rotate to the front.
        let order = pair_order(&self.bodies, sorted);
        let (sweep, planes) = out.split_at(swept_pairs);
        if planes.is_sorted_by(|x, y| order(x, y).is_le())
            && (sweep.is_empty()
                || planes.is_empty()
                || order(&planes[planes.len() - 1], &sweep[0]).is_lt())
        {
            out.rotate_left(swept_pairs);
        } else {
            out.sort_unstable_by(order);
        }
        let s = &mut self.bookkeeping;
        // A fast projectile's original trajectory must not activate bodies behind its first hit.
        // Keep all equal-time contacts, but defer later speculative contacts to the next substep,
        // where the already-updated velocity becomes authoritative. No event-restart loop is added.
        // `earliest` is infinity outside this pass (see `Scratch::per_body`).
        for (i, j, m) in out.iter() {
            for index in [*i, *j] {
                if self.bodies[index].ccd {
                    if s.earliest[index] == Real::INFINITY {
                        push(&mut s.earliest_set, index, &mut s.work);
                    }
                    s.earliest[index] = s.earliest[index].min(m.time);
                }
            }
        }
        let bodies = &self.bodies;
        out.retain(|(i, j, m)| {
            let keep = [i, j]
                .into_iter()
                .all(|index| m.time <= s.earliest[*index] + 1e-7);
            // A deferred later sweep is not wake evidence, but its CCD body still advances
            // through the whole substep and re-sweeps every candidate.
            if !keep && m.swept && ccd::deferred(&bodies[*i], &bodies[*j]) {
                for k in [*i, *j] {
                    if ccd::advances(&bodies[k]) {
                        push(&mut s.ccd, k, &mut s.work);
                    }
                }
            }
            keep
        });
        for &index in &s.earliest_set {
            s.earliest[index] = Real::INFINITY;
        }
        s.earliest_set.clear();
        Ok(regenerate)
    }
    /// A resting body's bounds change between `before` and its current bounds (it moved, was
    /// added or is about to be removed): broad-phase boxes near either may lack its leaf.
    fn resting_changed(&mut self, i: usize, before: (Vector, Vector)) {
        let s = &mut self.bookkeeping;
        if !s.boxes_live {
            return;
        }
        if !self.query_tree.current(s.generation) {
            s.forget_boxes();
            return;
        }
        let slop = self.scene.contact_slop;
        drop_boxes_near(&self.query_tree, &mut s.broad, before, slop);
        let now = self.bodies[i].cached_bounds;
        if now != before {
            drop_boxes_near(&self.query_tree, &mut s.broad, now, slop);
        }
    }
}
/// Drops the broad-phase boxes whose leaves overlap `(lo, hi)` padded by `slop` (#412).
fn drop_boxes_near(
    tree: &query::QueryTree,
    broad: &mut [bookkeeping::BroadBox],
    (lo, hi): (Vector, Vector),
    slop: Real,
) {
    if broad.is_empty() || !(lo.finite() && hi.finite()) {
        return;
    }
    let pad = slop
        + 64.0 * Real::EPSILON * (1.0 + lo.abs().max_component().max(hi.abs().max_component()));
    tree.tree().visit_aabb(
        spatial_kernels::Aabb {
            min: [lo.0 - pad, lo.1 - pad, lo.2 - pad],
            max: [hi.0 + pad, hi.1 + pad, hi.2 + pad],
        },
        |leaf| {
            if let Some(e) = broad.get_mut(leaf.id as usize) {
                e.epoch = 0;
            }
        },
    );
}
/// Pair order. Settled storage (`sorted`) is in body order, so storage positions compare
/// pairs as their body orders do.
fn pair_order(
    bodies: &[Body],
    sorted: bool,
) -> impl Fn(
    &(usize, usize, contact::Manifold),
    &(usize, usize, contact::Manifold),
) -> std::cmp::Ordering
+ '_ {
    move |&(a, b, _), &(c, d, _)| {
        if sorted {
            (a, b).cmp(&(c, d))
        } else {
            (bodies[a].order, bodies[b].order).cmp(&(bodies[c].order, bodies[d].order))
        }
    }
}
/// The normal row's velocity target. A separated (speculative) row permits closing exactly up
/// to its surface. Restitution (`restitution` is the pair's coefficient, `approach` the
/// relative normal velocity) and softness beyond the slop keep the hard response; within the
/// slop the positional target is zero, so a support keeps its load-bearing impulse.
fn contact_bias(
    sep: Real,
    restitution: Real,
    approach: Real,
    h: Real,
    slop: Real,
    restitution_speed: Real,
) -> Real {
    if sep > 0.0 {
        return -sep / h;
    }
    let bounce = if approach < -restitution_speed && sep <= slop {
        -restitution * approach
    } else {
        0.0
    };
    (0.2 * (-sep - slop).max(0.0) / h).min(60.0).max(bounce)
}
/// Extra displacement from gravity and accumulated force, which keep acting on an awake
/// dynamic body through the rest of the step; `gain` is zero on the step's last substep.
fn step_drift(b: &Body, gravity: Vector, gain: Real) -> Vector {
    if gain > 0.0 && b.movable() && !b.sleeping {
        (gravity + b.force / b.mass) * gain
    } else {
        Vector::ZERO
    }
}
#[inline]
fn contact_velocity(b: &Body, r: Vector, spin: bool) -> Vector {
    b.velocity
        + if spin {
            b.angular_velocity.cross(r)
        } else {
            Vector::ZERO
        }
}
#[inline]
fn effective_mass<const PREPARED: bool>(
    bodies: [(&Body, &PreparedResponse); 2],
    arms: [Vector; 2],
    n: Vector,
    response: [bool; 2],
    spin: bool,
    report: &mut Report,
) -> Real {
    let mut k = 0.0;
    for (((body, cached), r), yes) in bodies.into_iter().zip(arms).zip(response) {
        if yes {
            let prepared = cached.select::<PREPARED>(body, report);
            k += prepared.inverse_mass;
            if spin {
                k += prepared.inertia(body, r.cross(n), report).cross(r).dot(n);
            }
        }
    }
    k
}
#[inline]
fn apply<const PREPARED: bool>(
    bodies: &mut [Body],
    responses: &[PreparedResponse],
    c: &Constraint,
    j: Vector,
    report: &mut Report,
) {
    for (i, r, j, yes) in [
        (c.a, c.ra, -j, c.response[0]),
        (c.b, c.rb, j, c.response[1]),
    ] {
        if yes {
            let b = &mut bodies[i];
            let prepared = responses[i].select::<PREPARED>(b, report);
            b.velocity += j * prepared.inverse_mass;
            if c.spin {
                b.angular_velocity += prepared.inertia(b, r.cross(j), report);
            }
        }
    }
}
