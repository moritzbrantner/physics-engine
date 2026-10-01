use super::*;
pub use geometry_kernels::primitive3::PrimitiveRayFeature3 as RayFeature;
use geometry_kernels::primitive3::PrimitiveRayWork3;

/// Current-pose ray across a complete finite displacement; zero displacement is invalid.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayCast {
    pub origin: V,
    pub displacement: V,
}
impl RayCast {
    pub const fn new(origin: V, displacement: V) -> Self {
        Self {
            origin,
            displacement,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayHit {
    pub body: BodyId,
    pub fraction: Real,
    pub distance: Real,
    /// Surface entry point, or the original interior point for an initial overlap.
    pub point: V,
    /// Outward at the surface; opposite displacement for an interior origin.
    pub normal: V,
    pub feature: RayFeature,
    /// Strict interior beyond the kernel rounding band; touching is false.
    pub starts_overlapping: bool,
    pub support_velocity: V,
    pub sensor: bool,
}

impl World {
    /// All current-pose ray entries, ordered by fraction then BodyId. Initial touching
    /// and overlap are included even for separating movement. Reuses caller output;
    /// errors clear it and report all work, including discarded hits. Never steps/wakes.
    pub fn cast_ray(
        &self,
        cast: RayCast,
        filter: QueryFilter,
        hits: &mut Vec<RayHit>,
    ) -> Result<QueryStats, Box<QueryFailure>> {
        self.ray_query(cast, filter, hits, true)
    }

    fn ray_query(
        &self,
        cast: RayCast,
        filter: QueryFilter,
        hits: &mut Vec<RayHit>,
        prune: bool,
    ) -> Result<QueryStats, Box<QueryFailure>> {
        hits.clear();
        let mut stats = QueryStats::default();
        let result = self.collect_ray_hits(cast, filter, hits, prune, &mut stats);
        if result.is_err() {
            hits.clear();
        }
        stats.output_capacity_bytes = (hits.capacity() * size_of::<RayHit>()) as u64;
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

    fn collect_ray_hits(
        &self,
        cast: RayCast,
        filter: QueryFilter,
        hits: &mut Vec<RayHit>,
        prune: bool,
        stats: &mut QueryStats,
    ) -> Result<(), QueryFailureReason> {
        let end = cast.origin + cast.displacement;
        if !cast.origin.finite()
            || !cast.displacement.finite()
            || cast.displacement == V::ZERO
            || !end.finite()
            || cast.origin.abs().max_component() >= 1e12
            || end.abs().max_component() >= 1e12
        {
            return Err(QueryFailureReason::InvalidInput);
        }
        let bounds = (cast.origin.min(end), cast.origin.max(end));
        stats.query_bound_preparations += 1;
        let distance = cast.displacement.length();
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
                let magnitude = cast
                    .origin
                    .abs()
                    .max_component()
                    .max(end.abs().max_component())
                    .max(target.position.abs().max_component())
                    .max(target.shape.radius());
                // Conservative rounding padding only; snapshot rays have no collision skin.
                let pad = 64.0 * Real::EPSILON * (1.0 + magnitude);
                let (min, max) = target.cached_bounds;
                if (0..3)
                    .any(|i| bounds.1.at(i) + pad < min.at(i) || max.at(i) + pad < bounds.0.at(i))
                {
                    continue;
                }
            }
            stats.exact_candidates += 1;
            stats.geometry_preparations += 1;
            let mut work = PrimitiveRayWork3::default();
            let result = primitive::snapshot_ray(target, cast.origin, cast.displacement, &mut work);
            stats.ray_queries += work.queries;
            stats.ray_planes_tested += work.planes_tested;
            stats.ray_quadratic_tests += work.quadratic_tests;
            let Some(hit) = result.map_err(failure)? else {
                continue;
            };
            let point = V(
                primitive::nearest(hit.point[0]),
                primitive::nearest(hit.point[1]),
                primitive::nearest(hit.point[2]),
            );
            // An earlier fraction is the conservative narrowing of the kernel hit.
            let fraction = primitive::toward_zero(hit.fraction);
            let support_velocity =
                target.velocity + target.angular_velocity.cross(point - target.position);
            if !support_velocity.finite() {
                return Err(failure(SweepFailure::NonFiniteComputation));
            }
            let capacity = hits.capacity();
            hits.push(RayHit {
                body: target.id,
                fraction,
                distance: distance * fraction,
                point,
                normal: V(
                    primitive::nearest(hit.normal[0]),
                    primitive::nearest(hit.normal[1]),
                    primitive::nearest(hit.normal[2]),
                ),
                feature: hit.feature,
                starts_overlapping: hit.starts_inside,
                support_velocity,
                sensor: target.sensor,
            });
            stats.output_capacity_growths += u64::from(hits.capacity() > capacity);
            stats.hits_admitted += 1;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests;
