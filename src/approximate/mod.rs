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
use bookkeeping::{Scratch, reserve};
use geometry::GeometryCache;
pub use geometry::GeometryStats;
mod ccd;
mod character;
mod terrain;
pub use character::{Autostep, CharacterMove, CharacterMoveResult, CharacterSettings, Embedded};
pub use terrain::{HeightfieldDesc, TerrainId};
mod math;
mod response;
mod step;
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
