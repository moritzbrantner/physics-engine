//! Read-only queries against current authoritative poses; reusable geometry remains upstream.
use super::{
    Body, BodyId, Quaternion, Real, Shape, SweepFailure, Vector as V, World, contact, primitive,
};
use crate::CollisionLayers3d;
use geometry_kernels::{halfspace, primitive3::PrimitiveWork3};
use std::{error::Error, fmt, mem::size_of};

mod ray;
mod tree;
pub use ray::{RayCast, RayFeature, RayHit};
pub(super) use tree::QueryTree;

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
        let mut body = Body::new(self.shape, self.position, 0.0);
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
    pub fraction: Real,
    pub distance: Real,
    /// Outward from the target toward the query; deterministic at degenerate contacts.
    pub normal: V,
    pub query_point: V,
    pub target_point: V,
    /// Signed separation along the selected normal; negative denotes penetration.
    pub separation: Real,
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
    /// Entered closest-segment problems, including work discarded on query failure.
    pub segment_distance_evaluations: u64,
    /// Segment endpoint regions, box slabs/breakpoints and distance candidates visited.
    pub segment_feature_tests: u64,
    pub ray_queries: u64,
    pub ray_planes_tested: u64,
    pub ray_quadratic_tests: u64,
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
        write!(f, "floating geometry query failed: {:?}", self.reason)
    }
}
impl Error for QueryFailure {}

/// Tree candidates: inline for the usual handful, spilling to the heap beyond that.
#[derive(Default)]
struct Candidates {
    inline: [u32; 16],
    len: usize,
    spill: Vec<u32>,
}
impl Candidates {
    fn push(&mut self, index: u32) {
        if self.len < self.inline.len() {
            self.inline[self.len] = index;
        } else {
            if self.spill.is_empty() {
                self.spill.extend_from_slice(&self.inline);
            }
            self.spill.push(index);
        }
        self.len += 1;
    }
    fn sort(&mut self) {
        if self.spill.is_empty() {
            self.inline[..self.len].sort_unstable();
        } else {
            self.spill.sort_unstable();
        }
    }
    /// Sorts by body order when storage positions are not in body order (`Indexed` storage
    /// between steps), so candidates are tested in the same order in every storage mode.
    fn sort_by_order(&mut self, bodies: &[Body]) {
        let key = |&i: &u32| bodies[i as usize].order;
        if self.spill.is_empty() {
            self.inline[..self.len].sort_unstable_by_key(key);
        } else {
            self.spill.sort_unstable_by_key(key);
        }
    }
    fn len(&self) -> usize {
        self.len
    }
    fn get(&self, n: usize) -> u32 {
        if self.spill.is_empty() {
            self.inline[n]
        } else {
            self.spill[n]
        }
    }
}

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
        self.shape_query(pose, None, filter, None, hits, true)
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
        self.shape_query(cast.pose, Some(cast), filter, None, hits, true)
    }
    /// A shape query whose filter also leaves out body `also` (a character move's
    /// [`CharacterMove::exclude`](super::CharacterMove::exclude) beside the character itself).
    pub(super) fn shape_query(
        &self,
        pose: QueryPose,
        cast: Option<ShapeCast>,
        filter: QueryFilter,
        also: Option<BodyId>,
        hits: &mut Vec<QueryHit>,
        prune: bool,
    ) -> Result<QueryStats, Box<QueryFailure>> {
        hits.clear();
        let mut stats = QueryStats::default();
        let result = self.collect_shape_hits(pose, cast, (filter, also), hits, prune, &mut stats);
        if result.is_err() {
            hits.clear();
        }
        stats.output_capacity_bytes = (hits.capacity() * size_of::<QueryHit>()) as u64;
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
    fn collect_shape_hits(
        &self,
        pose: QueryPose,
        cast: Option<ShapeCast>,
        // The query's filter and one more body it leaves out.
        (filter, also): (QueryFilter, Option<BodyId>),
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
        // Tree preselection, in BodyId order so results and failures match a full scan.
        let candidates =
            (prune && self.query_tree.current(self.bookkeeping.generation)).then(|| {
                let tree = &self.query_tree;
                let m = tree.magnitude;
                let magnitude = pose
                    .position
                    .abs()
                    .max_component()
                    .max(m)
                    .max(displacement.abs().max_component());
                // Every target radius is at most twice the leaf coordinate bound.
                let radii = pose.shape.radius() + 2.0 * m;
                let pad = 8.0 * Real::EPSILON * (1.0 + magnitude + radii)
                    + if cast.is_some() {
                        1e-9 * (1.0 + radii)
                    } else {
                        0.0
                    };
                let (lo, hi) = (
                    sweep_bounds.0 - V(pad, pad, pad),
                    sweep_bounds.1 + V(pad, pad, pad),
                );
                let mut found = Candidates::default();
                let nodes = tree.tree().visit_aabb(
                    spatial_kernels::Aabb {
                        min: [lo.0, lo.1, lo.2],
                        max: [hi.0, hi.1, hi.2],
                    },
                    |leaf| {
                        let index = self.slot_index(leaf.id);
                        if self.bodies[index].shape != Shape::HalfSpace {
                            found.push(index as u32);
                        }
                    },
                );
                crate::work::add(crate::work::Counter::TreeNodes, nodes as u64);
                // Halfspaces are unbounded: every query tests them.
                for &id in &self.halfspaces {
                    if let Ok(index) = self.index(id) {
                        found.push(index as u32);
                    }
                }
                if self.storage_sorted() {
                    found.sort();
                } else {
                    found.sort_by_order(&self.bodies);
                }
                found
            });
        // Without the tree: every live body, in body order.
        let scan = candidates.is_none().then(|| self.ordered_positions());
        let count = candidates
            .as_ref()
            .map_or(scan.as_ref().map_or(0, Vec::len), Candidates::len);
        for n in 0..count {
            let target = &self.bodies[match (&candidates, &scan) {
                (Some(found), _) => found.get(n) as usize,
                (None, Some(scan)) => scan[n],
                (None, None) => unreachable!(),
            }];
            stats.bodies_visited += 1;
            if !filter.admits(target) || also == Some(target.id) {
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
            if prune && target.shape != Shape::HalfSpace {
                stats.bound_tests += 1;
                // Cover floating-point addition rounding and the shared sweep admission tolerance.
                let magnitude = pose
                    .position
                    .abs()
                    .max_component()
                    .max(target.position.abs().max_component())
                    .max(displacement.abs().max_component());
                let radii = pose.shape.radius() + target.shape.radius();
                let pad = 8.0 * Real::EPSILON * (1.0 + magnitude + radii)
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
            crate::work::add(crate::work::Counter::ExactTests, 1);
            query.position = pose.position;
            if target.shape == Shape::HalfSpace {
                let (point, normal) = super::terrain::plane_of(target);
                // A box or wedge meets the plane first at its deepest point, which translation
                // keeps deepest: a capsule with a collapsed segment and no radius there.
                let polytope = || {
                    matches!(query.shape, Shape::Box(_) | Shape::Wedge(_)).then(|| {
                        let low = primitive::deepest_along(&query, normal);
                        (low, low, 0.0)
                    })
                };
                let Some((p, q, r)) = super::terrain::capsule_of(&query).or_else(polytope) else {
                    return Err(failure(SweepFailure::InvalidGeometryInput));
                };
                let initial = halfspace::capsule_halfspace_contact(p, q, r, point, normal);
                let outcome = if initial.separation <= 0.0 {
                    Some((0.0, initial))
                } else if cast.is_some() {
                    let d = [displacement.0, displacement.1, displacement.2].map(f64::from);
                    halfspace::capsule_halfspace_sweep(p, q, r, d, point, normal)
                        .map(|(t, c)| (primitive::toward_zero(t), c))
                } else {
                    None
                };
                let Some((fraction, contact)) = outcome else {
                    continue;
                };
                let n = V(
                    primitive::nearest(normal[0]),
                    primitive::nearest(normal[1]),
                    primitive::nearest(normal[2]),
                );
                let point = V(
                    primitive::nearest(contact.point[0]),
                    primitive::nearest(contact.point[1]),
                    primitive::nearest(contact.point[2]),
                );
                let separation = primitive::round_down(contact.separation);
                let capacity = hits.capacity();
                hits.push(QueryHit {
                    body: target.id,
                    fraction,
                    distance: distance * fraction,
                    normal: n,
                    query_point: point + n * separation,
                    target_point: point,
                    separation,
                    starts_overlapping: initial.separation < 0.0,
                    support_velocity: V::ZERO,
                    sensor: target.sensor,
                });
                stats.output_capacity_growths += u64::from(hits.capacity() > capacity);
                stats.hits_admitted += 1;
                continue;
            }
            if let Shape::Heightfield { terrain, .. } = target.shape {
                let field = self.terrain(terrain);
                let mut terrain_work = geometry_kernels::heightfield::HeightfieldWork3::default();
                let capsule = super::terrain::capsule_of(&query);
                let initial = match (capsule, cast) {
                    (Some((p, q, r)), _) => {
                        field.capsule_contact_counted(p, q, r, 0.0, &mut terrain_work)
                    }
                    // Boxes and wedges: overlap only (door probes); terrain casts take
                    // spheres and capsules.
                    (None, None) => {
                        let Some(polytope) = primitive::polytope_of(&query) else {
                            return Err(failure(SweepFailure::InvalidGeometryInput));
                        };
                        field.polytope_contact_counted(
                            geometry_kernels::heightfield::ConvexPolytope3 {
                                vertices: &polytope.vertices,
                                face_normals: &polytope.face_normals,
                                edges: &polytope.edges,
                            },
                            &mut terrain_work,
                        )
                    }
                    (None, Some(_)) => {
                        return Err(failure(SweepFailure::InvalidGeometryInput));
                    }
                };
                // A terrain is one body, so a cast reports one contact with it: the triangle the
                // motion runs into first (most head-on among those it starts touching), never
                // one it starts touching and leaves. A capsule at a cliff's foot or in a corner
                // touches several faces; the deepest of them can be one the motion leaves while
                // it runs into another (#396). Only a cast that runs into nothing reports the
                // initial contact, at fraction 0, as for any body it starts touching.
                let outcome = match (initial, cast, capsule) {
                    (initial, Some(cast), Some((p, q, r))) => {
                        let d = [displacement.0, displacement.1, displacement.2].map(f64::from);
                        stats.sweep_iterations += 1;
                        field
                            .capsule_sweep_counted(
                                p,
                                q,
                                r,
                                d,
                                cast.max_iterations.max(64),
                                &mut terrain_work,
                            )
                            .map_err(|_| failure(SweepFailure::NonFiniteComputation))?
                            .map(|hit| (primitive::toward_zero(hit.fraction), hit.contact))
                            .or(initial.map(|c| (0.0, c)))
                    }
                    (Some(c), ..) => Some((0.0, c)),
                    _ => None,
                };
                crate::work::add(
                    crate::work::Counter::TerrainCells,
                    terrain_work.cells_visited,
                );
                crate::work::add(
                    crate::work::Counter::TerrainTriangles,
                    terrain_work.triangles_tested,
                );
                let Some((fraction, contact)) = outcome else {
                    continue;
                };
                let (separation, normal, point) = super::terrain::narrow(contact);
                let capacity = hits.capacity();
                hits.push(QueryHit {
                    body: target.id,
                    fraction,
                    distance: distance * fraction,
                    normal,
                    query_point: point + normal * separation,
                    target_point: point,
                    separation,
                    starts_overlapping: fraction == 0.0
                        && initial.is_some_and(|c| c.separation < 0.0),
                    support_velocity: V::ZERO,
                    sensor: target.sensor,
                });
                stats.output_capacity_growths += u64::from(hits.capacity() > capacity);
                stats.hits_admitted += 1;
                continue;
            }
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

/// Unit-length admission for query poses: 1e-9 in the f64 build, widened to the
/// rounding of one `Real` normalization (64 ulps of 1) when `Real` is f32.
const UNIT_ORIENTATION_TOLERANCE: Real = if 64.0 * Real::EPSILON > 1e-9 {
    64.0 * Real::EPSILON
} else {
    1e-9
};
fn unit_orientation(q: Quaternion) -> bool {
    q.finite()
        && (q.0 * q.0 + q.1 * q.1 + q.2 * q.2 + q.3 * q.3 - 1.0).abs() <= UNIT_ORIENTATION_TOLERANCE
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
    stats.segment_distance_evaluations += work.segment_distance_evaluations;
    stats.segment_feature_tests += work.segment_feature_tests;
}

#[cfg(test)]
mod tests;
