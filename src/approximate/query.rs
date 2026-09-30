//! Read-only queries against current authoritative poses; reusable geometry remains upstream.
use super::{
    Body, BodyId, Quaternion, Scalar, Shape, SweepFailure, Vector as V, World, contact, primitive,
};
use crate::CollisionLayers3d;
use geometry_kernels::primitive3::PrimitiveWork3;
use std::{error::Error, fmt, mem::size_of};

/// A query shape in world units. Orientation must already be a unit quaternion.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QueryPose {
    pub shape: Shape,
    pub position: V,
    pub orientation: Quaternion,
}
impl QueryPose {
    pub const fn new(shape: Shape, position: V) -> Self {
        Self {
            shape,
            position,
            orientation: Quaternion::IDENTITY,
        }
    }
    fn body(self) -> Option<Body> {
        if !self.shape.valid_dimensions()
            || !self.position.finite()
            || self.position.abs().max_component() >= 1e12
            || !unit_orientation(self.orientation)
        {
            return None;
        }
        let mut body = Body::new(BodyId(0), self.shape, self.position, 0.0);
        body.orientation = self.orientation;
        Some(body)
    }
}

/// Translation only, across the complete displacement, with both orientations held fixed.
/// Target poses stay at their current snapshot, irrespective of their physical velocity.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ShapeCast {
    pub pose: QueryPose,
    pub displacement: V,
    /// Per-target conservative-advance/SAT budget, in 1..=128. Exhaustion is an error.
    pub max_iterations: u32,
}
impl ShapeCast {
    pub const fn new(pose: QueryPose, displacement: V) -> Self {
        Self {
            pose,
            displacement,
            max_iterations: 128,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct QueryFilter {
    pub layers: CollisionLayers3d,
    pub exclude: Option<BodyId>,
    pub include_sensors: bool,
}
impl Default for QueryFilter {
    fn default() -> Self {
        Self {
            layers: CollisionLayers3d::ALL,
            exclude: None,
            include_sensors: false,
        }
    }
}
impl QueryFilter {
    fn admits(self, body: &Body) -> bool {
        self.exclude != Some(body.id)
            && (self.include_sensors || !body.sensor)
            && self.layers.collides_with(body.layers)
    }
}

/// Contact witnesses use the upstream primitive kernel's separation/normal contract.
/// Stable feature IDs and a clipped manifold are not supplied by this query surface.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QueryHit {
    pub body: BodyId,
    /// Fraction of the requested displacement, in [0, 1]; overlaps/touching start at zero.
    pub fraction: Scalar,
    pub distance: Scalar,
    /// Outward from the target toward the query; deterministic at degenerate contacts.
    pub normal: V,
    pub query_point: V,
    pub target_point: V,
    /// Signed separation along the selected normal; negative denotes penetration.
    pub separation: Scalar,
    /// Strict geometric penetration at the initial pose, before sweeping.
    pub starts_overlapping: bool,
    /// Current target velocity at its witness point, including angular velocity.
    pub support_velocity: V,
    pub sensor: bool,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct QueryStats {
    pub bodies_visited: u64,
    pub filtered_bodies: u64,
    pub bound_tests: u64,
    pub exact_candidates: u64,
    pub query_bound_preparations: u64,
    /// Kernel body-frame adapters, including preparation on a failed search.
    pub geometry_preparations: u64,
    /// Explicit initial/witness contact queries, excluding queries inside conservative advance.
    pub current_queries: u64,
    pub primitive_queries: u64,
    pub sweep_iterations: u64,
    pub axes_tested: u64,
    pub support_evaluations: u64,
    pub vertex_tests: u64,
    /// Includes admitted hits subsequently discarded on error.
    pub hits_admitted: u64,
    pub output_capacity_growths: u64,
    /// Caller output vector payload only; excludes allocator overhead. No query state is retained.
    pub output_capacity_bytes: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QueryFailureReason {
    InvalidInput,
    Search { body: BodyId, reason: SweepFailure },
}
/// Output is empty on error; diagnostics include all work and hits discarded before failure.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct QueryFailure {
    pub reason: QueryFailureReason,
    pub stats: QueryStats,
}
impl fmt::Display for QueryFailure {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "floating shape query failed: {:?}", self.reason)
    }
}
impl Error for QueryFailure {}

impl World {
    /// All current overlaps, including exact touching, in stable BodyId order.
    /// Reuses caller output capacity, clearing its previous contents on success or error.
    /// This does not step, wake bodies, consume input, or alter solver/contact history.
    pub fn overlap_shape(
        &self,
        pose: QueryPose,
        filter: QueryFilter,
        hits: &mut Vec<QueryHit>,
    ) -> Result<QueryStats, Box<QueryFailure>> {
        self.shape_query(pose, None, filter, hits, true)
    }
    /// All translation-only hits ordered by fraction then BodyId. Initial overlap/touching
    /// is included even for separating/tangential movement; response policy belongs to callers.
    /// Positive residual separation within the shared scale-aware sweep tolerance may be returned.
    pub fn cast_shape(
        &self,
        cast: ShapeCast,
        filter: QueryFilter,
        hits: &mut Vec<QueryHit>,
    ) -> Result<QueryStats, Box<QueryFailure>> {
        self.shape_query(cast.pose, Some(cast), filter, hits, true)
    }
    fn shape_query(
        &self,
        pose: QueryPose,
        cast: Option<ShapeCast>,
        filter: QueryFilter,
        hits: &mut Vec<QueryHit>,
        prune: bool,
    ) -> Result<QueryStats, Box<QueryFailure>> {
        hits.clear();
        let mut stats = QueryStats::default();
        let result = self.collect_shape_hits(pose, cast, filter, hits, prune, &mut stats);
        if result.is_err() {
            hits.clear();
        }
        stats.output_capacity_bytes = (hits.capacity() * size_of::<QueryHit>()) as u64;
        match result {
            Ok(()) => {
                hits.sort_unstable_by(|a, b| {
                    a.fraction.total_cmp(&b.fraction).then(a.body.cmp(&b.body))
                });
                Ok(stats)
            }
            Err(reason) => Err(Box::new(QueryFailure { reason, stats })),
        }
    }
    fn collect_shape_hits(
        &self,
        pose: QueryPose,
        cast: Option<ShapeCast>,
        filter: QueryFilter,
        hits: &mut Vec<QueryHit>,
        prune: bool,
        stats: &mut QueryStats,
    ) -> Result<(), QueryFailureReason> {
        let mut query = pose.body().ok_or(QueryFailureReason::InvalidInput)?;
        let displacement = cast.map_or(V::ZERO, |cast| cast.displacement);
        if !displacement.finite()
            || !(pose.position + displacement).finite()
            || (pose.position + displacement).abs().max_component() >= 1e12
            || cast.is_some_and(|cast| !(1..=128).contains(&cast.max_iterations))
        {
            return Err(QueryFailureReason::InvalidInput);
        }
        let distance = displacement.length();
        let (min, max) = contact::bounds(&query);
        stats.query_bound_preparations += 1;
        let sweep_bounds = (min.min(min + displacement), max.max(max + displacement));
        for target in &self.bodies {
            stats.bodies_visited += 1;
            if !filter.admits(target) {
                stats.filtered_bodies += 1;
                continue;
            }
            let failure = |reason| QueryFailureReason::Search {
                body: target.id,
                reason,
            };
            if !unit_orientation(target.orientation) {
                return Err(failure(SweepFailure::InvalidGeometryInput));
            }
            if prune {
                stats.bound_tests += 1;
                // Cover floating-point addition rounding and the shared sweep admission tolerance.
                let magnitude = pose
                    .position
                    .abs()
                    .max_component()
                    .max(target.position.abs().max_component())
                    .max(displacement.abs().max_component());
                let radii = pose.shape.radius() + target.shape.radius();
                let pad = 8.0 * Scalar::EPSILON * (1.0 + magnitude + radii)
                    + if cast.is_some() {
                        1e-9 * (1.0 + radii)
                    } else {
                        0.0
                    };
                let (target_min, target_max) = target.cached_bounds;
                if (0..3).any(|axis| {
                    sweep_bounds.1.at(axis) + pad < target_min.at(axis)
                        || target_max.at(axis) + pad < sweep_bounds.0.at(axis)
                }) {
                    continue;
                }
            }
            stats.exact_candidates += 1;
            query.position = pose.position;
            let initial = current(&query, target, stats).map_err(failure)?;
            let fraction = if initial.separation <= 0.0 {
                0.0
            } else if let Some(cast) = cast {
                let mut work = PrimitiveWork3::default();
                stats.geometry_preparations += 2;
                let result = primitive::snapshot_sweep(
                    &query,
                    target,
                    displacement,
                    cast.max_iterations,
                    &mut work,
                );
                accumulate(stats, work);
                match result.map_err(failure)? {
                    Some(fraction) => fraction,
                    None => continue,
                }
            } else {
                continue;
            };
            if !fraction.is_finite() || !(0.0..=1.0).contains(&fraction) {
                return Err(failure(SweepFailure::NonFiniteComputation));
            }
            let contact = if fraction == 0.0 {
                initial
            } else {
                query.position = pose.position + displacement * fraction;
                current(&query, target, stats).map_err(failure)?
            };
            let support_velocity = target.velocity
                + target
                    .angular_velocity
                    .cross(contact.point_b - target.position);
            if !support_velocity.finite() {
                return Err(failure(SweepFailure::NonFiniteComputation));
            }
            let capacity = hits.capacity();
            hits.push(QueryHit {
                body: target.id,
                fraction,
                distance: distance * fraction,
                normal: -contact.normal,
                query_point: contact.point_a,
                target_point: contact.point_b,
                separation: contact.separation,
                starts_overlapping: initial.separation < 0.0,
                support_velocity,
                sensor: target.sensor,
            });
            stats.output_capacity_growths += u64::from(hits.capacity() > capacity);
            stats.hits_admitted += 1;
        }
        Ok(())
    }
}

fn unit_orientation(q: Quaternion) -> bool {
    q.finite() && (q.0 * q.0 + q.1 * q.1 + q.2 * q.2 + q.3 * q.3 - 1.0).abs() <= 1e-9
}
fn current(
    query: &Body,
    target: &Body,
    stats: &mut QueryStats,
) -> Result<primitive::PrimitiveContact, SweepFailure> {
    let mut work = PrimitiveWork3::default();
    stats.geometry_preparations += 2;
    let contact = primitive::snapshot_query(query, target, &mut work);
    stats.current_queries += 1;
    accumulate(stats, work);
    if !contact.separation.is_finite()
        || !contact.normal.finite()
        || !contact.point_a.finite()
        || !contact.point_b.finite()
        || (contact.normal.dot(contact.normal) - 1.0).abs() > 1e-6
    {
        Err(SweepFailure::NonFiniteComputation)
    } else {
        Ok(contact)
    }
}
fn accumulate(stats: &mut QueryStats, work: PrimitiveWork3) {
    stats.primitive_queries += work.pair_dispatches.into_iter().sum::<u64>();
    stats.sweep_iterations += work.sweep_iterations;
    stats.axes_tested += work.axes_tested;
    stats.support_evaluations += work.support_evaluations;
    stats.vertex_tests += work.vertex_tests;
}

#[cfg(test)]
mod tests;
