//! Explicit floating fixed-step approximation, alongside (not silently replacing) the event solver.
//!
//! Physical state uses [`Real`]: `f64` by default, `f32` with the whole-build `f32-physics`
//! feature. Elapsed time stays [`numeric::Scalar`] (`f64`) in every build; each substep narrows
//! its `dt` to `Real` once. See docs/numerics.md for the f32 precision envelope.
//!
//! External impulses change velocity once; forces act for the requested duration. Each bounded
//! substep prepares contacts, accumulates clamped sequential impulses, then integrates the new
//! linear/angular velocity. Contact impulses persist for warm starting. No integer quantization,
//! rational remainder tree, event-restart loop, or whole-world sleep proxy conversion is used.
//!
//! CCD sweeps translation over each substep for fast shapes. Orientations are held fixed during
//! those sweeps and integrated between substeps, so this is NOT analytic rotational CCD.
#[cfg(all(feature = "f32-physics", feature = "exact-reference"))]
compile_error!("`f32-physics` and the diagnostic `exact-reference` backend are mutually exclusive");

/// Floating-solver physical scalar: `f32` with the `f32-physics` feature, else `f64`.
///
/// The feature is a whole-build choice. Legacy worlds and time composition keep
/// [`numeric::Scalar`] (`f64`) in every build.
#[cfg(feature = "f32-physics")]
pub type Real = f32;
/// Floating-solver physical scalar: `f32` with the `f32-physics` feature, else `f64`.
///
/// The feature is a whole-build choice. Legacy worlds and time composition keep
/// [`numeric::Scalar`] (`f64`) in every build.
#[cfg(not(feature = "f32-physics"))]
pub type Real = f64;
#[cfg(feature = "f32-physics")]
#[allow(unused_imports)]
use std::f32::consts as real_consts;
#[cfg(not(feature = "f32-physics"))]
#[allow(unused_imports)]
use std::f64::consts as real_consts;
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
#[cfg(feature = "experimental-soft-contact")]
mod correction;
#[cfg(feature = "experimental-soft-contact")]
pub use correction::{CorrectionStats, SoftContact};
mod islands;
pub use convergence::{Convergence, ConvergenceStats};
pub use islands::{ConvergenceScope, IslandStats};
mod geometry;
pub use bookkeeping::BookkeepingStats;
use bookkeeping::{Scratch, SweepRow, push, reserve};
use geometry::GeometryCache;
pub use geometry::GeometryStats;
mod math;
mod response;
use crate::{
    BodyId, BodyKind, CollisionLayers3d, ContactMode3d, MotionAuthority3d, RigidBox3d, SleepMode3d,
    SolverParticipation3d, numeric,
};
pub use math::{Quaternion, Vector};
use numeric::Scalar;
use response::PreparedResponse;
use std::collections::BTreeMap;
mod transaction;
pub use transaction::{FailedStepWork, TransactionStats};
mod checkpoint;
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
    pub fn radius(self) -> Real {
        match self {
            Self::Box(h) | Self::Wedge(h) => h.length(),
            Self::Sphere(r) => r,
            Self::Capsule {
                half_segment,
                radius,
            } => half_segment + radius,
        }
    }
    pub fn half_extents(self) -> Vector {
        match self {
            Self::Box(h) | Self::Wedge(h) => h,
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
    fn local_inverse_inertia(self, mass: Real) -> Option<Vector> {
        match self {
            Self::Sphere(r) => {
                let inverse = 2.5 / (mass * r * r);
                Some(Vector(inverse, inverse, inverse))
            }
            Self::Box(h) => Some(Vector(
                3.0 / (mass * (h.1 * h.1 + h.2 * h.2)),
                3.0 / (mass * (h.0 * h.0 + h.2 * h.2)),
                3.0 / (mass * (h.0 * h.0 + h.1 * h.1)),
            )),
            Self::Capsule {
                half_segment: h,
                radius: r,
            } => {
                let cylinder_volume = r * r * (2.0 * h);
                let sphere_volume = (4.0 / 3.0) * r * r * r;
                let total_volume = cylinder_volume + sphere_volume;
                let cylinder_mass = mass * cylinder_volume / total_volume;
                let sphere_mass = mass * sphere_volume / total_volume;
                let axial = cylinder_mass * r * r * 0.5 + sphere_mass * r * r * (2.0 / 5.0);
                let radial = cylinder_mass * (3.0 * r * r + 4.0 * h * h) / 12.0
                    + sphere_mass * ((2.0 / 5.0) * r * r + h * h + (3.0 / 4.0) * h * r);
                Some(Vector(1.0 / radial, 1.0 / axial, 1.0 / radial))
            }
            // Solver-owned dynamic wedges are required to be rotation locked until a COM-centered
            // wedge inertia/pose contract is introduced.
            Self::Wedge(_) => None,
        }
    }
}
#[derive(Clone, Debug, PartialEq)]
pub struct Body {
    pub id: BodyId,
    pub position: Vector,
    pub velocity: Vector,
    pub orientation: Quaternion,
    /// Radians per second in world coordinates.
    pub angular_velocity: Vector,
    pub shape: Shape,
    /// Zero is immovable geometry; positive mass is dynamic unless `external` is set.
    pub mass: Real,
    /// Finite coefficient in [0, 10], default 0.6. Contacts use the larger coefficient
    /// and clamp the accumulated tangential impulse to a disk of radius mu * normal impulse.
    /// Linear-support contacts disable friction. See docs/contact-materials.md.
    pub friction: Real,
    /// Finite coefficient in [0, 1], default zero. Contacts use the smaller coefficient.
    /// Restitution requires closing speed > 1 scene unit/s, separation <= contact_slop
    /// and no retained pair history; speculative positive separation does not bounce.
    pub restitution: Real,
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
    pub fn new(id: BodyId, shape: Shape, position: Vector, mass: Real) -> Self {
        Self {
            id,
            shape,
            position,
            mass,
            velocity: Vector::ZERO,
            orientation: Quaternion::IDENTITY,
            angular_velocity: Vector::ZERO,
            friction: 0.6,
            restitution: 0.0,
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
    /// One-way compatibility import at scene construction. Never performed during integration.
    pub fn from_legacy(body: &RigidBox3d) -> Self {
        let b = body.body();
        let a = body.angular();
        let q = a.orientation;
        let qs = crate::ORIENTATION_SCALE as Real;
        let mut out = Self::new(
            b.id(),
            Shape::Box(b.half_extents().into()),
            b.position().into(),
            if b.kind() == BodyKind::Fixed {
                0.0
            } else {
                b.mass_units() as Real
            },
        );
        out.velocity = b.velocity().into();
        out.orientation = Quaternion(
            q.x as Real / qs,
            q.y as Real / qs,
            q.z as Real / qs,
            q.w as Real / qs,
        )
        .normalized();
        let w = a.angular_velocity;
        let ws = crate::ANGULAR_VELOCITY_SCALE as Real;
        out.angular_velocity = Vector(w.x as Real / ws, w.y as Real / ws, w.z as Real / ws);
        out.friction = b.material().friction_milli() as Real / 1000.0;
        out.restitution = b.material().restitution_milli() as Real / 1000.0;
        out.rotation_locked = body.rotation_locked();
        out.layers = body.collision_layers();
        out.external = body.motion_authority() == MotionAuthority3d::External;
        out.sensor = body.solver_participation() == SolverParticipation3d::OverlapOnly;
        out.sleep_allowed = body.sleep_mode() != SleepMode3d::Never;
        if let ContactMode3d::LinearPush { support_direction } = body.contact_mode() {
            out.linear_support = Some(support_direction.into());
        }
        out
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
            && self.mass.is_finite()
            && (self.mass == 0.0 || self.mass >= 1e-6)
            && self.mass < 1e12
            && self.friction.is_finite()
            && (0.0..=10.0).contains(&self.friction)
            && self.restitution.is_finite()
            && (0.0..=1.0).contains(&self.restitution)
            && (!self.rotation_locked || self.angular_velocity == Vector::ZERO)
            && (self.mass > 0.0
                || (self.velocity == Vector::ZERO && self.angular_velocity == Vector::ZERO))
            && self.position.abs().max_component() < 1e12
            && self.velocity.abs().max_component() < 1e12
    }
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
    pub gravity: Vector,
    pub substeps: u8,
    pub velocity_iterations: u8,
    /// Shared post-integration position-pass budget (historical field name).
    /// Zero disables correction; fixed and admitted dynamic pairs share each selected pass.
    pub fixed_position_iterations: u8,
    pub position_correction: PositionCorrection,
    /// Scene-space length, default 0.02. Chosen explicitly for the legacy 36-unit crates.
    pub contact_slop: Real,
    pub sleep_speed: Real,
    pub sleep_seconds: Real,
    pub warm_start: bool,
    /// None retains the fixed-pass reference. Some permits a checked early exit.
    pub convergence: Option<Convergence>,
    /// Contact-island-local stopping, or the original whole-world convergence reference.
    /// With `convergence: None`, both scopes use the unchanged fixed-pass solver.
    pub convergence_scope: ConvergenceScope,
    /// Explicit diagnostic policy; absent from ordinary library and Pages builds.
    #[cfg(feature = "experimental-soft-contact")]
    pub soft_contact: Option<SoftContact>,
}
impl Default for Config {
    fn default() -> Self {
        Self {
            gravity: Vector(0.0, -3600.0, 0.0),
            substeps: 4,
            velocity_iterations: 8,
            fixed_position_iterations: 0,
            position_correction: PositionCorrection::FixedColliders,
            contact_slop: 0.02,
            sleep_speed: 1.0,
            sleep_seconds: 0.5,
            warm_start: true,
            convergence: Some(Convergence::default()),
            convergence_scope: ConvergenceScope::ContactIslands,
            #[cfg(feature = "experimental-soft-contact")]
            soft_contact: None,
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    InvalidInput,
    DuplicateBody(BodyId),
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
    #[cfg(feature = "experimental-soft-contact")]
    pub correction: CorrectionStats,
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
    #[cfg(feature = "experimental-soft-contact")]
    hard_normal: bool,
    #[cfg(feature = "experimental-soft-contact")]
    relaxing_normal: bool,
    #[cfg(feature = "experimental-soft-contact")]
    normal_coefficients: correction::Coefficients,
    friction: Real,
    normal_impulse: Real,
    tangent_impulse: [Real; 2],
    sep: Real,
    swept: bool,
    response: [bool; 2],
    spin: bool,
}
#[derive(Clone, Debug)]
pub struct World {
    config: Config,
    bodies: Vec<Body>,
    cache: BTreeMap<(BodyId, BodyId), Vec<CachedPoint>>,
    /// Prior substep length. Time composition and its checkpoint value stay f64.
    last_h: Scalar,
    // Derived substep scratch, indexed like bodies. Refresh contents; reuse allocated capacity.
    responses: Vec<PreparedResponse>,
    bookkeeping: Scratch,
    geometry: GeometryCache,
    island_scratch: islands::Scratch,
    constraints: Vec<Constraint>,
    manifold_scratch: Vec<(usize, usize, contact::Manifold)>,
    #[cfg(feature = "experimental-soft-contact")]
    relaxation_motion: Vec<correction::Motion>,
    #[cfg(feature = "experimental-soft-contact")]
    relaxation_bias: Vec<Real>,
    pub last_report: Report,
    transaction: transaction::Journal,
    last_transaction: TransactionStats,
    failed_work: Option<FailedStepWork>,
    elapsed: Scalar,
}
impl World {
    pub fn new(config: Config) -> Result<Self, Error> {
        #[cfg(feature = "experimental-soft-contact")]
        if config.soft_contact.is_some_and(|c| !c.valid()) {
            return Err(Error::InvalidInput);
        }
        if !config.gravity.finite()
            || config.substeps == 0
            || config.substeps > 32
            || config.velocity_iterations == 0
            || config.velocity_iterations > 64
            || config.fixed_position_iterations > 8
            || config.convergence.is_some_and(|c| !c.valid())
            || !config.contact_slop.is_finite()
            || config.contact_slop <= 0.0
            || !config.sleep_speed.is_finite()
            || config.sleep_speed <= 0.0
            || !config.sleep_seconds.is_finite()
            || config.sleep_seconds <= 0.0
        {
            return Err(Error::InvalidInput);
        }
        Ok(Self {
            config,
            bodies: Vec::new(),
            cache: BTreeMap::new(),
            last_h: 0.0,
            responses: Vec::new(),
            bookkeeping: Scratch::default(),
            geometry: GeometryCache::default(),
            island_scratch: islands::Scratch::default(),
            constraints: Vec::new(),
            manifold_scratch: Vec::new(),
            #[cfg(feature = "experimental-soft-contact")]
            relaxation_motion: Vec::new(),
            #[cfg(feature = "experimental-soft-contact")]
            relaxation_bias: Vec::new(),
            last_report: Report::default(),
            transaction: transaction::Journal::default(),
            last_transaction: TransactionStats::default(),
            failed_work: None,
            elapsed: 0.0,
        })
    }
    pub fn bodies(&self) -> impl Iterator<Item = &Body> {
        self.bodies.iter()
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
    pub fn config(&self) -> Config {
        self.config
    }
    fn index(&self, id: BodyId) -> Result<usize, Error> {
        self.bodies
            .binary_search_by_key(&id, |b| b.id)
            .map_err(|_| Error::MissingBody(id))
    }
    pub fn add_body(&mut self, mut body: Body) -> Result<(), Error> {
        body.orientation = body.orientation.normalized();
        if !body.valid() {
            return Err(Error::InvalidInput);
        }
        let i = match self.bodies.binary_search_by_key(&body.id, |b| b.id) {
            Ok(_) => return Err(Error::DuplicateBody(body.id)),
            Err(i) => i,
        };
        // Local topology change: only a genuine new fixed overlap can disturb a sleeper.
        let wake = if body.mass == 0.0 && !body.sensor {
            self.bodies
                .iter()
                .filter(|b| {
                    b.sleeping
                        && b.layers.collides_with(body.layers)
                        && contact::current(b, &body, 0.0).is_some()
                })
                .map(|b| b.id)
                .collect::<Vec<_>>()
        } else {
            Vec::new()
        };
        for id in wake {
            self.wake_island(id);
        }
        body.cached_bounds = contact::bounds(&body);
        self.bookkeeping.activity.count += usize::from(body.mass > 0.0 && !body.sleeping);
        self.bodies.insert(i, body);
        self.bookkeeping.layout_changed();
        Ok(())
    }
    pub fn remove_body(&mut self, id: BodyId) -> Option<Body> {
        let i = self.index(id).ok()?;
        self.bookkeeping.ensure_graph(&self.bodies, &self.cache);
        let neighbors = self
            .bookkeeping
            .graph
            .neighbors(i)
            .iter()
            .map(|&j| self.bodies[j].id)
            .collect::<Vec<_>>();
        for n in neighbors {
            self.wake_island(n);
        }
        self.geometry.remove(id);
        self.cache.retain(|&(a, b), _| a != id && b != id);
        let removed = self.bodies.remove(i);
        self.bookkeeping.activity.count -= usize::from(removed.mass > 0.0 && !removed.sleeping);
        self.bookkeeping.layout_changed();
        Some(removed)
    }
    pub fn set_velocity(&mut self, id: BodyId, v: Vector) -> Result<(), Error> {
        if !v.finite() || v.abs().max_component() >= 1e12 {
            return Err(Error::InvalidInput);
        }
        let i = self.index(id)?;
        if self.bodies[i].mass == 0.0 {
            return if v == Vector::ZERO {
                Ok(())
            } else {
                Err(Error::InvalidInput)
            };
        }
        if self.bodies[i].velocity != v {
            self.wake_island(id);
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
            self.wake_island(id);
        }
        self.bodies[i].impulse = pending;
        self.bodies[i].angular_impulse = spin;
        Ok(())
    }
    pub fn add_force(&mut self, id: BodyId, force: Vector) -> Result<(), Error> {
        if !force.finite() {
            return Err(Error::InvalidInput);
        }
        let i = self.index(id)?;
        if !self.bodies[i].movable() {
            return Err(Error::InvalidInput);
        }
        let total = self.bodies[i].force + force;
        if !total.finite() {
            return Err(Error::InvalidInput);
        }
        if force != Vector::ZERO {
            self.wake_island(id);
        }
        self.bodies[i].force = total;
        Ok(())
    }
    fn wake_island(&mut self, root: BodyId) -> u64 {
        let Ok(root) = self.index(root) else {
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
        s.activity.dirty |= count > 0;
        count as u64
    }
    fn wake_contact_island<const PREPARED: bool>(
        &mut self,
        root: BodyId,
        h: Real,
        report: &mut Report,
    ) -> u64 {
        let woke = self.wake_island(root);
        for n in 0..self.bookkeeping.traversal.island.len() {
            let i = self.bookkeeping.traversal.island[n];
            if PREPARED {
                self.responses[i] = PreparedResponse::new(&self.bodies[i], report);
            }
            // These bodies skipped the ordinary force phase while parked.
            self.apply_substep_forces::<PREPARED>(i, h, report);
        }
        woke
    }
    pub fn has_support(&self, id: BodyId) -> bool {
        let up = (-self.config.gravity).unit();
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
            #[cfg(feature = "experimental-soft-contact")]
            {
                self.last_report.correction.relaxation_motion_bytes =
                    (bookkeeping::bytes(&self.relaxation_motion)
                        + bookkeeping::bytes(&self.relaxation_bias)) as u64;
            }
            return Ok(self.last_report.clone());
        }
        if substep_length(dt, self.config.substeps) == 0.0 {
            return Err(Error::InvalidInput);
        }
        #[cfg(feature = "experimental-soft-contact")]
        let softness = self
            .config
            .soft_contact
            .map(|c| {
                c.prepare(substep_length(dt, self.config.substeps))
                    .ok_or(Error::InvalidInput)
            })
            .transpose()?;
        let mut report = Report::default();
        if self.is_quiescent() {
            self.elapsed += dt;
            self.finish_bookkeeping(&mut report);
            self.last_report = report.clone();
            return Ok(report);
        }
        self.transaction.begin(self.bodies.len());
        let prior_h = self.last_h;
        let prior_activity = self.bookkeeping.activity.count;
        let result = self.advance_substeps::<PREPARED, CACHED>(
            dt,
            #[cfg(feature = "experimental-soft-contact")]
            softness,
        );
        self.last_transaction = self.transaction.stats(result.is_err());
        if let Some(work) = &mut self.failed_work {
            work.transaction = self.last_transaction;
        }
        match result {
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
        }
    }
    fn advance_substeps<const PREPARED: bool, const CACHED: bool>(
        &mut self,
        dt: Scalar,
        #[cfg(feature = "experimental-soft-contact")] softness: Option<correction::Coefficients>,
    ) -> Result<Report, Error> {
        let mut report = Report::default();
        let h = substep_length(dt, self.config.substeps);
        for substep in 0..self.config.substeps {
            report.substeps += 1;
            self.responses
                .resize(self.bodies.len(), PreparedResponse::default());
            for (b, prepared) in self.bodies.iter().zip(&mut self.responses) {
                *prepared = if PREPARED {
                    PreparedResponse::new(b, &mut report)
                } else {
                    PreparedResponse::default()
                };
            }
            // External impulses still apply exactly once, before the first force integration.
            if substep == 0 {
                for (i, (b, cached)) in self.bodies.iter_mut().zip(&self.responses).enumerate() {
                    if (b.movable() && !b.sleeping)
                        || b.impulse != Vector::ZERO
                        || b.angular_impulse != Vector::ZERO
                    {
                        self.transaction.body(i, b);
                    }
                    if b.movable() {
                        let prepared = cached.select::<PREPARED>(b, &mut report);
                        b.velocity += b.impulse * prepared.inverse_mass;
                        b.angular_velocity += prepared.inertia(b, b.angular_impulse, &mut report);
                    }
                    b.impulse = Vector::ZERO;
                    b.angular_impulse = Vector::ZERO;
                }
            }
            for i in 0..self.bodies.len() {
                if self.bodies[i].movable() && !self.bodies[i].sleeping {
                    self.apply_substep_forces::<PREPARED>(i, h, &mut report);
                }
            }
            let mut pairs = std::mem::take(&mut self.manifold_scratch);
            if let Err(error) = self.manifolds::<CACHED>(h, &mut report, &mut pairs) {
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
            let mut roots = std::mem::take(&mut self.bookkeeping.roots);
            roots.clear();
            reserve(&mut roots, pairs.len() * 2, &mut self.bookkeeping.work);
            let up = (-self.config.gravity).unit();
            roots.extend(
                pairs
                    .iter()
                    .flat_map(|(i, j, m)| {
                        let a = &self.bodies[*i];
                        let b = &self.bodies[*j];
                        let approach = -(b.velocity - a.velocity).dot(m.normal);
                        // A separating command alone is not wake evidence. An admitted
                        // previously load-bearing support moving away is a dependency loss.
                        let departing_support = approach < 0.0
                            && ((a.external && b.sleeping && m.normal.dot(up) > 0.5)
                                || (b.external && a.sleeping && m.normal.dot(up) < -0.5))
                            && self.cache.get(&(a.id, b.id)).is_some_and(|points| {
                                points
                                    .iter()
                                    .any(|p| p.impulse > 0.0 && p.normal.dot(m.normal) > 0.99)
                            });
                        let driven_contact_motion = (a.sleeping || b.sleeping)
                            && (a.external || b.external)
                            && (&m.points).into_iter().any(|point| {
                                let spin = a.linear_support.is_none() && b.linear_support.is_none();
                                let relative = contact_velocity(b, point.rb, spin)
                                    - contact_velocity(a, point.ra, spin);
                                relative != Vector::ZERO && relative.dot(m.normal) <= 0.0
                            });
                        let disruptive = m.swept
                            || approach > self.config.sleep_speed
                            || ((a.external || b.external) && approach > 0.0)
                            || departing_support
                            || driven_contact_motion;
                        [
                            if disruptive && a.sleeping && b.mass > 0.0 {
                                Some(a.id)
                            } else {
                                None
                            },
                            if disruptive && b.sleeping && a.mass > 0.0 {
                                Some(b.id)
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
                if let Err(error) = self.manifolds::<CACHED>(h, &mut report, &mut pairs) {
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
            let mut constraints = std::mem::take(&mut self.constraints);
            constraints.clear();
            #[cfg(feature = "experimental-soft-contact")]
            let relax_enabled = self
                .config
                .soft_contact
                .is_some_and(|c| c.relaxation_iterations > 0);
            #[cfg(feature = "experimental-soft-contact")]
            if relax_enabled {
                self.relaxation_bias.clear();
            }
            reserve(
                &mut constraints,
                pairs.iter().map(|(_, _, m)| m.points.len()).sum(),
                &mut self.bookkeeping.work,
            );
            for (i, j, m) in pairs.drain(..) {
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
                let t1 = if n.0.abs() < 0.7 {
                    n.cross(Vector::X).unit()
                } else {
                    n.cross(Vector::Y).unit()
                };
                let t2 = n.cross(t1);
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
                        let gap = if matches!((a.shape, b.shape), (Shape::Box(_), Shape::Box(_))) {
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
                    let restitution = if rel.dot(n) < -1.0
                        && point.separation <= self.config.contact_slop
                        && !self.cache.contains_key(&(a.id, b.id))
                    {
                        -a.restitution.min(b.restitution) * rel.dot(n)
                    } else {
                        0.0
                    };
                    #[cfg(not(feature = "experimental-soft-contact"))]
                    let bias = if point.separation > 0.0 {
                        -point.separation / h
                    } else {
                        (0.2 * (-point.separation - self.config.contact_slop).max(0.0) / h)
                            .min(60.0)
                    }
                    .max(restitution);
                    // Speculative and restitutive impacts retain the hard response. Softness
                    // applies only beyond the existing slop, on non-bouncing normal constraints.
                    // Within slop the positional target is zero: keep the support hard rather
                    // than repeatedly reducing its load-bearing accumulated impulse.
                    #[cfg(feature = "experimental-soft-contact")]
                    let soft = softness.filter(|_| {
                        point.separation < -self.config.contact_slop && restitution == 0.0
                    });
                    #[cfg(feature = "experimental-soft-contact")]
                    let correction = (-point.separation - self.config.contact_slop).max(0.0);
                    #[cfg(feature = "experimental-soft-contact")]
                    let bias = if point.separation > 0.0 {
                        -point.separation / h
                    } else if let Some(soft) = soft {
                        (soft.bias_rate * correction).min(60.0)
                    } else {
                        (0.2 * correction / h).min(60.0)
                    }
                    .max(restitution);
                    // A separated speculative constraint must still permit closing up to its surface.
                    let bias = if point.separation > 0.0 {
                        -point.separation / h
                    } else {
                        bias
                    };
                    let mut c = Constraint {
                        a: i,
                        b: j,
                        n,
                        ra: point.ra,
                        rb: point.rb,
                        t1,
                        t2,
                        normal_mass: 1.0 / k,
                        tangent_mass: [
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
                        ],
                        bias,
                        #[cfg(feature = "experimental-soft-contact")]
                        hard_normal: soft.is_none() || a.mass == 0.0 || b.mass == 0.0,
                        #[cfg(feature = "experimental-soft-contact")]
                        relaxing_normal: false,
                        #[cfg(feature = "experimental-soft-contact")]
                        normal_coefficients: soft.unwrap_or(correction::Coefficients::RIGID),
                        // Additional corners can limit approach while still separated at the
                        // admitted contact time. They have no frictional load yet.
                        friction: if linear
                            || (contact::fixed_box_interval(a, b)
                                && point.separation + (b.velocity - a.velocity).dot(n) * h * m.time
                                    > self.config.contact_slop)
                        {
                            0.0
                        } else {
                            a.friction.max(b.friction)
                        },
                        normal_impulse: 0.0,
                        tangent_impulse: [0.0; 2],
                        sep: point.separation,
                        swept: m.swept,
                        response,
                        spin: !linear,
                    };
                    if self.config.warm_start
                        && point.separation <= self.config.contact_slop
                        && self.last_h > 0.0
                        && let Some(old) = self.cache.get(&(a.id, b.id))
                    {
                        let la = a.orientation.inverse_rotate(c.ra);
                        let lb = b.orientation.inverse_rotate(c.rb);
                        if let Some((idx, p)) = old
                            .iter()
                            .enumerate()
                            .filter(|(idx, p)| !used.contains(idx) && p.normal.dot(n) > 0.99)
                            .min_by(|(_, x), (_, y)| {
                                ((x.a - la).dot(x.a - la) + (x.b - lb).dot(x.b - lb)).total_cmp(
                                    &((y.a - la).dot(y.a - la) + (y.b - lb).dot(y.b - lb)),
                                )
                            })
                            && (p.a - la).length() + (p.b - lb).length()
                                < 0.1 * a.shape.radius().min(b.shape.radius())
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
                    #[cfg(feature = "experimental-soft-contact")]
                    {
                        report.correction.softened_points += u64::from(!c.hard_normal);
                        report.correction.hard_support_points += u64::from(
                            softness.is_some()
                                && point.separation <= 0.0
                                && restitution == 0.0
                                && (a.mass == 0.0 || b.mass == 0.0),
                        );
                        if relax_enabled {
                            self.relaxation_bias.push(if point.separation > 0.0 {
                                -point.separation / h
                            } else {
                                restitution
                            });
                        }
                    }
                    constraints.push(c);
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
            #[cfg(feature = "experimental-soft-contact")]
            let converged_before = report.convergence.converged_substeps;
            match self.config.convergence {
                Some(tolerances)
                    if self.config.convergence_scope == ConvergenceScope::ContactIslands =>
                {
                    islands::solve::<PREPARED>(
                        &mut self.bodies,
                        &self.responses,
                        &mut constraints,
                        self.config.velocity_iterations,
                        tolerances,
                        &mut self.island_scratch,
                        &mut report,
                    )
                }
                Some(tolerances) => convergence::solve::<PREPARED, true>(
                    &mut self.bodies,
                    &self.responses,
                    &mut constraints,
                    self.config.velocity_iterations,
                    tolerances,
                    &mut report,
                ),
                None => convergence::solve::<PREPARED, false>(
                    &mut self.bodies,
                    &self.responses,
                    &mut constraints,
                    self.config.velocity_iterations,
                    Convergence::default(),
                    &mut report,
                ),
            }
            #[cfg(feature = "experimental-soft-contact")]
            let relaxing = {
                // Admission/retirement belongs to the biased physical solve. A later relaxation
                // correction cannot erase an already admitted projectile impact.
                self.bookkeeping.retired.clear();
                for c in &constraints {
                    if c.normal_impulse > 0.0 {
                        for index in [c.a, c.b] {
                            let b = &self.bodies[index];
                            if b.retire_on_impact {
                                push(
                                    &mut self.bookkeeping.retired,
                                    b.id,
                                    &mut self.bookkeeping.work,
                                );
                            }
                        }
                        if c.swept {
                            report.swept_contacts += 1;
                        }
                    }
                }
                let primary_converged = report.convergence.converged_substeps > converged_before;
                self.relax_contacts::<PREPARED>(&mut constraints, primary_converged, &mut report)
            };
            let scratch = &mut self.bookkeeping;
            #[cfg(not(feature = "experimental-soft-contact"))]
            scratch.retired.clear();
            scratch.support_edges.clear();
            reserve(&mut scratch.supported, self.bodies.len(), &mut scratch.work);
            scratch.supported.clear();
            scratch.supported.extend(self.bodies.iter().map(|b| {
                b.mass == 0.0
                    || b.sleeping
                    || (b.external
                        && b.velocity == Vector::ZERO
                        && b.angular_velocity == Vector::ZERO)
            }));
            // Reuse point allocations when the pair survives. Warm starting has already read the
            // old impulses; only adjacency-key changes invalidate the graph, not updated impulses.
            for (&(a, b), points) in &mut self.cache {
                let key = (a, b);
                let a = self
                    .bodies
                    .binary_search_by_key(&a, |b| b.id)
                    .expect("live contact");
                let b = self
                    .bodies
                    .binary_search_by_key(&b, |b| b.id)
                    .expect("live contact");
                if !scratch.supported[a] || !scratch.supported[b] {
                    if !points.is_empty() {
                        self.transaction.pair(key, Some(points));
                    }
                    points.clear();
                }
            }
            for c in &constraints {
                let a = &self.bodies[c.a];
                let b = &self.bodies[c.b];
                #[cfg(not(feature = "experimental-soft-contact"))]
                if c.normal_impulse > 0.0 {
                    if a.retire_on_impact {
                        push(&mut scratch.retired, a.id, &mut scratch.work);
                    }
                    if b.retire_on_impact {
                        push(&mut scratch.retired, b.id, &mut scratch.work);
                    }
                    if c.swept {
                        report.swept_contacts += 1;
                    }
                }
                if c.sep <= self.config.contact_slop * 2.0 {
                    self.transaction
                        .pair((a.id, b.id), self.cache.get(&(a.id, b.id)));
                    let points = match self.cache.entry((a.id, b.id)) {
                        std::collections::btree_map::Entry::Occupied(entry) => entry.into_mut(),
                        std::collections::btree_map::Entry::Vacant(entry) => {
                            scratch.graph.dirty = true;
                            entry.insert(Vec::new())
                        }
                    };
                    points.push(CachedPoint {
                        a: a.orientation.inverse_rotate(c.ra),
                        b: b.orientation.inverse_rotate(c.rb),
                        normal: c.n,
                        impulse: c.normal_impulse,
                        tangent: c.t1 * c.tangent_impulse[0] + c.t2 * c.tangent_impulse[1],
                    });
                    if c.n.dot((-self.config.gravity).unit()) > 0.5 {
                        push(&mut scratch.support_edges, (c.a, c.b), &mut scratch.work);
                    }
                    if c.n.dot((-self.config.gravity).unit()) < -0.5 {
                        push(&mut scratch.support_edges, (c.b, c.a), &mut scratch.work);
                    }
                }
            }
            self.cache.retain(|&key, points| {
                if points.is_empty() {
                    self.transaction.pair(key, Some(points));
                    scratch.graph.dirty = true;
                    false
                } else {
                    true
                }
            });
            scratch.support.propagate(
                &mut scratch.supported,
                &scratch.support_edges,
                &mut scratch.work,
            );
            scratch.activity.refresh(&self.bodies, &mut scratch.work);
            for &i in &scratch.activity.indices {
                let b = &mut self.bodies[i];
                self.transaction.body(i, b);
                report.integrated_bodies += 1;
                #[cfg(not(feature = "experimental-soft-contact"))]
                let (movement, spin, correction_quiet) = (b.velocity, b.angular_velocity, true);
                #[cfg(feature = "experimental-soft-contact")]
                let (movement, spin, correction_quiet) = {
                    let motion = if relaxing {
                        self.relaxation_motion[i]
                    } else {
                        correction::Motion {
                            velocity: b.velocity,
                            angular: b.angular_velocity,
                        }
                    };
                    (
                        motion.velocity,
                        motion.angular,
                        !relaxing
                            || motion.velocity.length()
                                + motion.angular.length() * b.shape.radius()
                                < self.config.sleep_speed,
                    )
                };
                b.position += movement * h;
                if !b.rotation_locked {
                    b.orientation = b.orientation.integrate(spin, h);
                }
                if !b.valid() {
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
                        < self.config.sleep_speed
                {
                    b.quiet_time += h;
                } else {
                    b.quiet_time = 0.0;
                }
                b.cached_bounds = contact::bounds(b);
            }
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
            self.sleep_quiet_islands();
            // Retirement is local; remove all affected cached edges before indexed graph reuse.
            self.bookkeeping.retired.sort_unstable();
            self.bookkeeping.retired.dedup();
            for n in 0..self.bookkeeping.retired.len() {
                let id = self.bookkeeping.retired[n];
                if let Ok(i) = self.index(id) {
                    let b = self.bodies.remove(i);
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
                    self.bookkeeping.layout_changed();
                    report.retired.push(id);
                }
            }
            self.last_h = widen_time(h);
        }
        for (i, b) in self.bodies.iter_mut().enumerate() {
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
    fn apply_substep_forces<const PREPARED: bool>(
        &mut self,
        index: usize,
        h: Real,
        report: &mut Report,
    ) {
        let b = &mut self.bodies[index];
        self.transaction.body(index, b);
        let prepared = self.responses[index].select::<PREPARED>(b, report);
        b.velocity += (self.config.gravity + b.force / b.mass) * h;
        b.angular_velocity += prepared.inertia(b, b.torque, report) * h;
    }
    fn finish_bookkeeping(&mut self, report: &mut Report) {
        #[cfg(feature = "experimental-soft-contact")]
        {
            report.correction.relaxation_motion_bytes =
                (bookkeeping::bytes(&self.relaxation_motion)
                    + bookkeeping::bytes(&self.relaxation_bias)) as u64;
        }
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
                b.sleeping || (b.sleep_allowed && b.quiet_time >= self.config.sleep_seconds)
            }) {
                for &i in &s.traversal.island {
                    let b = &mut self.bodies[i];
                    self.transaction.body(i, b);
                    slept += usize::from(!b.sleeping);
                    b.sleeping = true;
                    b.velocity = Vector::ZERO;
                    b.angular_velocity = Vector::ZERO;
                }
            }
        }
        s.activity.count -= slept;
        s.activity.dirty |= slept > 0;
    }

    fn manifolds<const CACHED: bool>(
        &mut self,
        h: Real,
        report: &mut Report,
        out: &mut Vec<(usize, usize, contact::Manifold)>,
    ) -> Result<(), Error> {
        let s = &mut self.bookkeeping;
        if CACHED {
            self.geometry.begin(self.bodies.len());
        }
        out.clear();
        if s.bounds.len() != self.bodies.len() {
            reserve(&mut s.bounds, self.bodies.len(), &mut s.work);
            s.bounds.clear();
            s.bounds
                .extend((0..self.bodies.len()).map(|index| SweepRow {
                    index,
                    lo: Vector::ZERO,
                    hi: Vector::ZERO,
                    active: true,
                }));
        }
        for row in &mut s.bounds {
            let b = &self.bodies[row.index];
            let active = b.mass > 0.0 && !b.sleeping;
            if !active && !row.active {
                continue;
            }
            let (lo, hi) = b.cached_bounds;
            let delta = if b.sleeping {
                Vector::ZERO
            } else {
                b.velocity * h
            };
            let angular = b.angular_velocity.length() * b.shape.radius() * h;
            let pad = Vector(angular, angular, angular)
                + Vector(1.0, 1.0, 1.0)
                    * (self.config.contact_slop
                        // Endpoint addition can round at the displacement's scale even
                        // when the original pose and collider are small.
                        + 8.0 * Real::EPSILON * delta.abs().max_component());
            row.lo = lo.min(lo + delta) - pad;
            row.hi = hi.max(hi + delta) + pad;
            row.active = active;
            s.work.bounds_updates += 1;
        }
        // The key is total and unique (BodyId), so unstable sort preserves canonical pair order
        // while avoiding stable-sort allocation. Stationary rows retain their valid padded bounds.
        let compare = |a: &SweepRow, b: &SweepRow| {
            a.lo.0
                .total_cmp(&b.lo.0)
                .then_with(|| self.bodies[a.index].id.cmp(&self.bodies[b.index].id))
        };
        let ordered = s.bounds.windows(2).all(|rows| {
            s.work.bounds_order_checks += 1;
            !compare(&rows[0], &rows[1]).is_gt()
        });
        if !ordered {
            s.bounds.sort_unstable_by(compare);
            s.work.bound_rows_sorted += s.bounds.len() as u64;
        }
        let bounds = &s.bounds;
        for p in 0..bounds.len() {
            let SweepRow {
                index: i, lo, hi, ..
            } = bounds[p];
            for &SweepRow {
                index: j,
                lo: bl,
                hi: bh,
                ..
            } in bounds.iter().skip(p + 1)
            {
                if bl.0 > hi.0 {
                    break;
                }
                report.pair_tests += 1;
                if bl.1 > hi.1 || bh.1 < lo.1 || bl.2 > hi.2 || bh.2 < lo.2 {
                    continue;
                }
                let (i, j) = if self.bodies[i].id < self.bodies[j].id {
                    (i, j)
                } else {
                    (j, i)
                };
                let a = &self.bodies[i];
                let b = &self.bodies[j];
                let driven_contact_candidate = (a.movable()
                    && b.external
                    && (b.velocity != Vector::ZERO || b.angular_velocity != Vector::ZERO))
                    || (b.movable()
                        && a.external
                        && (a.velocity != Vector::ZERO || a.angular_velocity != Vector::ZERO));
                let driven_wake_candidate = driven_contact_candidate && (a.sleeping || b.sleeping);
                if a.sensor
                    || b.sensor
                    || !a.layers.collides_with(b.layers)
                    || (a.inverse_mass() == 0.0
                        && b.inverse_mass() == 0.0
                        && !driven_wake_candidate)
                {
                    continue;
                }
                // A tolerance-only manifold is not wake evidence. Probe the actual surface
                // and full interval before replacing the parked body's zero response.
                let margin = if driven_wake_candidate {
                    0.0
                } else {
                    self.config.contact_slop
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
                } else {
                    let travel = (b.velocity - a.velocity).length() * h;
                    // Retain full-interval discovery after response preparation wakes the
                    // body; the second geometry pass must not discard an admitted sweep.
                    // A fixed boundary may be crossed below the shape-sized fast threshold.
                    // The current margin only rules out travel no larger than that margin.
                    if driven_contact_candidate
                        || a.ccd
                        || b.ccd
                        || ((a.mass == 0.0 || b.mass == 0.0) && travel > self.config.contact_slop)
                        || travel
                            > 0.5
                                * a.shape
                                    .half_extents()
                                    .min_component()
                                    .min(b.shape.half_extents().min_component())
                    {
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
                    } else {
                        None
                    }
                };
                if let Some(m) = m {
                    push(out, (i, j, m), &mut s.work);
                }
            }
        }
        if CACHED {
            self.geometry.finish(&mut report.geometry);
        }
        // A fast projectile's original trajectory must not activate bodies behind its first hit.
        // Keep all equal-time contacts, but defer later speculative contacts to the next substep,
        // where the already-updated velocity becomes authoritative. No event-restart loop is added.
        reserve(&mut s.earliest, self.bodies.len(), &mut s.work);
        s.earliest.clear();
        s.earliest.resize(self.bodies.len(), Real::INFINITY);
        for (i, j, m) in out.iter() {
            for index in [*i, *j] {
                if self.bodies[index].ccd {
                    s.earliest[index] = s.earliest[index].min(m.time);
                }
            }
        }
        out.retain(|(i, j, m)| {
            [i, j]
                .into_iter()
                .all(|index| m.time <= s.earliest[*index] + 1e-7)
        });
        out.sort_unstable_by_key(|(i, j, _)| (self.bodies[*i].id, self.bodies[*j].id));
        Ok(())
    }
}
fn contact_velocity(b: &Body, r: Vector, spin: bool) -> Vector {
    b.velocity
        + if spin {
            b.angular_velocity.cross(r)
        } else {
            Vector::ZERO
        }
}
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
