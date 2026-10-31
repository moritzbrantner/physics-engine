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

/// Ray query output: every hit, or only the best first hit (no allocation).
enum RayOut<'a> {
    All(&'a mut Vec<RayHit>),
    First(Option<RayHit>),
}
impl RayOut<'_> {
    /// The current best hit of a first-hit query; `None` when collecting every hit.
    fn best(&self) -> Option<&RayHit> {
        match self {
            Self::All(_) => None,
            Self::First(best) => best.as_ref(),
        }
    }
    /// Stores a hit; returns whether the output vector grew.
    fn push(&mut self, hit: RayHit) -> bool {
        match self {
            Self::All(hits) => {
                let capacity = hits.capacity();
                hits.push(hit);
                hits.capacity() > capacity
            }
            Self::First(best) => {
                *best = Some(hit);
                false
            }
        }
    }
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

    /// The earliest current-pose ray entry, ties broken by the lower BodyId: the same hit as
    /// `cast_ray(..)[0]`, without collecting the others. Targets whose bounds the segment can
    /// only enter after the best hit so far are skipped. Never steps/wakes.
    pub fn cast_ray_first(
        &self,
        cast: RayCast,
        filter: QueryFilter,
    ) -> Result<Option<RayHit>, Box<QueryFailure>> {
        let mut stats = QueryStats::default();
        let mut out = RayOut::First(None);
        match self.collect_ray_hits(cast, filter, &mut out, true, &mut stats) {
            Ok(()) => Ok(match out {
                RayOut::First(best) => best,
                RayOut::All(_) => None,
            }),
            Err(reason) => Err(Box::new(QueryFailure { reason, stats })),
        }
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
        let result = self.collect_ray_hits(cast, filter, &mut RayOut::All(hits), prune, &mut stats);
        if result.is_err() {
            hits.clear();
        }
        stats.output_capacity_bytes = (hits.capacity() * size_of::<RayHit>()) as u64;
        match result {
            Ok(()) => {
                let order = |id| self.index(id).map(|i| self.bodies[i].order).ok();
                hits.sort_unstable_by(|a, b| {
                    a.fraction
                        .total_cmp(&b.fraction)
                        .then_with(|| order(a.body).cmp(&order(b.body)))
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
        hits: &mut RayOut<'_>,
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
        let ray_magnitude = cast
            .origin
            .abs()
            .max_component()
            .max(end.abs().max_component());
        // Entry parameter of the segment into `min..max` padded for rounding, or `None` when
        // the segment misses it. Conservative: never rejects a box the exact test could hit.
        let entry = |min: V, max: V| -> Option<Real> {
            // Twice the bounds' largest coordinate covers any contained position and radius.
            let magnitude =
                ray_magnitude.max(2.0 * min.abs().max_component().max(max.abs().max_component()));
            let pad = 64.0 * Real::EPSILON * (1.0 + magnitude);
            if (0..3).any(|i| bounds.1.at(i) + pad < min.at(i) || max.at(i) + pad < bounds.0.at(i))
            {
                return None;
            }
            let (mut enter, mut exit): (Real, Real) = (0.0, 1.0);
            for i in 0..3 {
                let d = cast.displacement.at(i);
                if d != 0.0 {
                    let o = cast.origin.at(i);
                    let a = (min.at(i) - pad - o) / d;
                    let b = (max.at(i) + pad - o) / d;
                    enter = enter.max(a.min(b));
                    exit = exit.min(a.max(b));
                }
            }
            let slack = 64.0 * Real::EPSILON * (1.0 + enter.abs().max(exit.abs()));
            (enter <= exit + slack).then_some(enter - slack)
        };
        let first = matches!(hits, RayOut::First(_));
        // `bounded`: the tree already entered the target's padded bounds (exact for fixed
        // bodies, conservative for moving ones) no later than the best hit, so the per-target
        // slab test below would repeat it.
        let mut test = |index: usize,
                        bounded: bool,
                        hits: &mut RayOut<'_>|
         -> Result<(), QueryFailureReason> {
            let target = &self.bodies[index];
            stats.bodies_visited += 1;
            if bounded {
                stats.bound_tests += 1;
                crate::work::add(crate::work::Counter::BoundTests, 1);
            }
            if !filter.admits(target) {
                stats.filtered_bodies += 1;
                return Ok(());
            }
            let failure = |reason| QueryFailureReason::Search {
                body: target.id,
                reason,
            };
            if prune && !bounded && target.shape != Shape::HalfSpace {
                stats.bound_tests += 1;
                crate::work::add(crate::work::Counter::BoundTests, 1);
                let (min, max) = target.cached_bounds;
                let Some(enter) = entry(min, max) else {
                    return Ok(());
                };
                // A later entry cannot beat the best first hit so far.
                if hits.best().is_some_and(|best| enter > best.fraction) {
                    return Ok(());
                }
            }
            if !unit_orientation(target.orientation) {
                return Err(failure(SweepFailure::InvalidGeometryInput));
            }
            stats.exact_candidates += 1;
            crate::work::add(crate::work::Counter::ExactTests, 1);
            stats.geometry_preparations += 1;
            let mut work = PrimitiveRayWork3::default();
            let result = if target.shape == Shape::HalfSpace {
                let (point, normal) = super::super::terrain::plane_of(target);
                let start = [cast.origin.0, cast.origin.1, cast.origin.2].map(f64::from);
                let end = [end.0, end.1, end.2].map(f64::from);
                work.queries += 1;
                Ok(
                    geometry_kernels::halfspace::segment_halfspace(start, end, point, normal).map(
                        |fraction| geometry_kernels::primitive3::PrimitiveRayHit3 {
                            fraction,
                            point: std::array::from_fn(|k| {
                                start[k] + (end[k] - start[k]) * fraction
                            }),
                            normal,
                            feature: RayFeature::Interior,
                            starts_inside: fraction == 0.0,
                        },
                    ),
                )
            } else if let Shape::Heightfield { terrain, .. } = target.shape {
                let start = [cast.origin.0, cast.origin.1, cast.origin.2].map(f64::from);
                let end = [end.0, end.1, end.2].map(f64::from);
                work.queries += 1;
                let mut terrain_work = geometry_kernels::heightfield::HeightfieldWork3::default();
                let hit = self
                    .terrain(terrain)
                    .cast_segment_counted(start, end, &mut terrain_work);
                crate::work::add(
                    crate::work::Counter::TerrainCells,
                    terrain_work.cells_visited,
                );
                crate::work::add(
                    crate::work::Counter::TerrainTriangles,
                    terrain_work.triangles_tested,
                );
                Ok(
                    hit.map(|hit| geometry_kernels::primitive3::PrimitiveRayHit3 {
                        fraction: hit.fraction,
                        point: hit.point,
                        normal: hit.normal,
                        feature: RayFeature::HeightfieldTriangle(hit.triangle),
                        starts_inside: false,
                    }),
                )
            } else {
                primitive::snapshot_ray(target, cast.origin, cast.displacement, &mut work)
            };
            stats.ray_queries += work.queries;
            stats.ray_planes_tested += work.planes_tested;
            stats.ray_quadratic_tests += work.quadratic_tests;
            let Some(hit) = result.map_err(failure)? else {
                return Ok(());
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
            // First hit: earliest fraction wins; equal fractions keep the lower BodyId.
            if hits.best().is_some_and(|best| {
                best.fraction < fraction
                    || (best.fraction == fraction
                        && self
                            .index(best.body)
                            .is_ok_and(|i| self.bodies[i].order < target.order))
            }) {
                return Ok(());
            }
            let grew = hits.push(RayHit {
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
            stats.output_capacity_growths += u64::from(grew);
            stats.hits_admitted += 1;
            Ok(())
        };
        if prune && self.query_tree.current(self.bookkeeping.generation) {
            let tree = &self.query_tree;
            // Covers every per-target pad above: leaf coordinates are bounded by `magnitude`.
            let pad = 64.0 * Real::EPSILON * (1.0 + ray_magnitude.max(2.0 * tree.magnitude));
            let mut failure = None;
            let nodes = tree.visit_segment(
                [cast.origin.0, cast.origin.1, cast.origin.2],
                [
                    cast.displacement.0,
                    cast.displacement.1,
                    cast.displacement.2,
                ],
                1.0,
                pad,
                |leaf, _| {
                    let index = self.slot_index(leaf.id);
                    if failure.is_none()
                        && self.bodies[index].shape != Shape::HalfSpace
                        && let Err(reason) = test(index, true, hits)
                    {
                        failure = Some(reason);
                    }
                    match (failure, first, hits.best()) {
                        (Some(_), ..) => -1.0,
                        // Keep boxes entered at the best fraction: a lower BodyId may tie.
                        (None, true, Some(best)) => {
                            best.fraction + 64.0 * Real::EPSILON * (1.0 + best.fraction)
                        }
                        _ => 1.0,
                    }
                },
            );
            crate::work::add(crate::work::Counter::TreeNodes, nodes as u64);
            if let Some(reason) = failure {
                return Err(reason);
            }
            // Halfspaces are unbounded: every ray tests them (their tree leaves are points).
            for &id in &self.halfspaces {
                if let Ok(index) = self.index(id) {
                    test(index, false, hits)?;
                }
            }
            Ok(())
        } else {
            self.ordered_positions()
                .into_iter()
                .try_for_each(|i| test(i, false, hits))
        }
    }
}

#[cfg(test)]
mod tests;
