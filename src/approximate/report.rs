//! The per-step work report.

use super::{BookkeepingStats, ConvergenceStats, GeometryStats, IslandStats, PositionReport, Real};
use crate::BodyId;

#[derive(Clone, Debug, Default)]
pub struct Report {
    /// Contact enter/exit transitions of this step, in pair order (see [`ContactEvent`]).
    pub contact_events: Vec<ContactEvent>,
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
