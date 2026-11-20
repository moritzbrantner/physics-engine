//! The per-step work report.

use super::Order;
use super::{
    BookkeepingStats, ConvergenceStats, GeometryStats, IslandStats, PositionReport, Real, Vector,
};
use crate::BodyId;

#[derive(Clone, Debug, Default)]
pub struct Report {
    /// Contact enter/exit transitions of this step, in pair order (see [`ContactEvent`]).
    pub contact_events: Vec<ContactEvent>,
    /// With [`Config::report_contact_impulses`](super::Config::report_contact_impulses): the
    /// normal impulse each contact took over this step, in pair order (see [`ContactImpulse`]).
    /// Empty otherwise.
    pub contact_impulses: Vec<ContactImpulse>,
    pub position: PositionReport,
    pub substeps: u32,
    pub pair_tests: u64,
    pub narrow_tests: u64,
    pub contact_points: u64,
    /// Joint rows over the step's substeps, one per joint with an awake dynamic side (#453).
    pub joint_rows: u64,
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

/// A transition of physical contact between two bodies, reported by the step that made it.
///
/// A pair is in contact while the persistent contact cache holds it (the solver found a point
/// within twice the contact slop); an impulse on a pair the cache does not hold (a swept or CCD
/// impact, a projectile retired on impact) is a contact too. Only pairs the solver handles take
/// part: a dynamic body against a dynamic, static, kinematic, halfspace or terrain body whose
/// layers collide. Sensors, query-only shapes and character moves never produce events.
/// Sleeping pairs stay in contact.
///
/// Per step and pair there is at most one `Started` and one `Stopped`, `Started` first: a pair
/// that touched and separated within the step reports both, and a contact that broke and
/// re-formed within the step reports neither. An uncached impact's `Stopped` is held for one
/// step and dropped if the pair is in contact by then. Events are ordered by pair (`a` precedes
/// `b` in body order, then by `b`). Removing a body, or its retirement on impact, stops its
/// contacts; [`World::remove_body`](super::World::remove_body) reports them in the next
/// successful step. See docs/world-api-contract.md#contact-events.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ContactEvent {
    Started(BodyId, BodyId),
    Stopped(BodyId, BodyId),
}
impl ContactEvent {
    /// The pair, `a` before `b` in body order.
    pub fn bodies(self) -> (BodyId, BodyId) {
        match self {
            Self::Started(a, b) | Self::Stopped(a, b) => (a, b),
        }
    }
    pub fn started(self) -> bool {
        matches!(self, Self::Started(..))
    }
}

/// The normal impulse one contact applied over a step (#575), reported with
/// [`Config::report_contact_impulses`](super::Config::report_contact_impulses).
///
/// One entry per pair of touching shapes that took a positive normal impulse: per body pair
/// and, for a compound body, per child (`child_a`, `child_b`: the child's index in the
/// authored list, `None` for a primitive side). Rows from several manifolds or points of one
/// such contact, and from every substep, are summed. `normal_impulse` (mass × length / s, in
/// scene units) is the sum of the solver's normal impulses: the contact rows' accumulated
/// impulse per substep, the end-of-step restitution, and remaining-time CCD impacts.
/// Friction is not included. A resting contact reports its share of the weight times the
/// step's length; an impact reports its impulse on top.
///
/// `normal` is the unit normal from `a` toward `b`: the impulse pushes `b` along it and `a`
/// against it. Both it and `point`, a world point midway between the two surfaces, are
/// averages weighted by impulse. Entries are ordered as [`ContactEvent`]s (`a` before `b`
/// in body order, then by `b`), then by `child_a` and `child_b`. A body retired on impact
/// keeps its id here. Sensors, query-only shapes, character moves and joints contribute
/// nothing; neither does a sleeping pair.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactImpulse {
    pub a: BodyId,
    pub b: BodyId,
    pub child_a: Option<u32>,
    pub child_b: Option<u32>,
    pub normal: Vector,
    pub point: Vector,
    pub normal_impulse: Real,
}

/// One recorded contribution to a [`ContactImpulse`]: the impulse and its impulse-weighted
/// normal and point sums, keyed by pair order and children.
#[derive(Clone, Copy, Debug)]
pub(super) struct ImpulseRecord {
    pub key: (Order, Order),
    pub ids: (BodyId, BodyId),
    pub children: [u32; 2],
    pub impulse: Real,
    pub normal: Vector,
    pub point: Vector,
}
