//! The per-step work report.

use super::{BookkeepingStats, ConvergenceStats, GeometryStats, IslandStats, PositionReport, Real};
use crate::BodyId;

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
