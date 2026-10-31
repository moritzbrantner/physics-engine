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
use crate::{BodyId, numeric};
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
mod body;
mod config;
mod control;
mod error;
mod lifecycle;
mod material;
mod report;
mod shape;
mod storage;
pub use body::Body;
pub(crate) use body::Order;
pub use config::{BodyStorage, Config, PositionCorrection};
use control::drop_boxes_near;
pub use error::{Error, SweepFailure};
pub use material::{CombineRule, Material};
use material::{pair_friction, pair_restitution};
pub use report::Report;
pub use shape::Shape;

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
    /// Per side (a, b): the world inverse inertia applied to `r × d` for d = n, t1, t2, the
    /// angular velocity change per unit impulse along `d` (zero where the side does not
    /// spin). Prepared once per row, as Rapier's solver contacts do (#424).
    angular: [[Vector; 3]; 2],
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
}
