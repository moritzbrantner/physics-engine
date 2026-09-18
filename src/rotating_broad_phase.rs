use std::{
    collections::{BTreeMap, BTreeSet},
    error::Error,
    fmt,
};

use crate::{
    BodyId, BodyKind, CollisionLayers3d, ContactPersistence3d, MotionAuthority3d, OrientedBox3d,
    RigidBox3d, RigidBoxFreeFlightConfig3d, RigidBoxFreeFlightError3d, RotationalSweepBounds3d,
    SolverParticipation3d, rigid_box_free_flight_sweep_bounds,
};

#[path = "rotating_broad_phase_tree.rs"]
mod tree;
use tree::IndexedBvh3d;

/// Canonically ordered broad-phase candidate pair for rotating rigid boxes.
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct RotationalSweepPair3d {
    pub left: BodyId,
    pub right: BodyId,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub(crate) struct RotatingBroadPhaseStats3d {
    pub queries: u64,
    pub rebuilds: u64,
    pub reuses: u64,
    pub incremental_updates: u64,
    pub reinserts: u64,
    pub rotations: u64,
    pub partial_queries: u64,
    pub partial_body_updates: u64,
    /// Geometrically eligible solid pairs rejected because neither participant can receive solver mutation.
    pub response_authority_pair_rejections: u64,
    pub fixed_bound_reuses: u64,
    pub fixed_bound_recomputations: u64,
    pub dynamic_bound_recomputations: u64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RotatingBroadPhaseError3d {
    DuplicateBodyId(BodyId),
    IncrementalQueryUnsynchronized(BodyId),
    FreeFlight(RigidBoxFreeFlightError3d),
}

impl fmt::Display for RotatingBroadPhaseError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DuplicateBodyId(id) => write!(
                formatter,
                "rotational broad phase received duplicate body id {}",
                id.0
            ),
            Self::IncrementalQueryUnsynchronized(id) => write!(
                formatter,
                "incremental rotational broad-phase query is not synchronized for body {}",
                id.0
            ),
            Self::FreeFlight(error) => {
                write!(formatter, "rotational broad-phase sweep failed: {error}")
            }
        }
    }
}

impl Error for RotatingBroadPhaseError3d {}

impl From<RigidBoxFreeFlightError3d> for RotatingBroadPhaseError3d {
    fn from(value: RigidBoxFreeFlightError3d) -> Self {
        Self::FreeFlight(value)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct BoundedBody3d {
    id: BodyId,
    kind: BodyKind,
    collision_layers: CollisionLayers3d,
    solver_participation: SolverParticipation3d,
    receives_solver_response: bool,
    bounds: RotationalSweepBounds3d,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct FixedBoundCacheEntry3d {
    shape: OrientedBox3d,
    collision_layers: CollisionLayers3d,
    solver_participation: SolverParticipation3d,
    bounded: BoundedBody3d,
}

/// Persistent deterministic broad phase for rotating rigid boxes.
///
/// The retained fat-AABB tree uses indexed arena nodes, parent links, and a `BodyId -> leaf` index.
/// Exact sweep bounds can therefore escape and reinsert without scanning unrelated subtrees. Small and
/// medium escape batches are updated incrementally; when more than half of the tree escapes in one query,
/// a balanced rebuild is cheaper and is selected deterministically. Membership or body-kind changes also
/// rebuild. Fat candidates are streamed directly through exact-bound filtering so only surviving pairs
/// are materialized. Tree persistence and balancing therefore affect pruning cost only and cannot change
/// collision truth.
#[derive(Clone, Debug, Default)]
pub(crate) struct RotatingBroadPhase3d {
    tree: IndexedBvh3d,
    exact: BTreeMap<BodyId, BoundedBody3d>,
    fixed_bounds: BTreeMap<BodyId, FixedBoundCacheEntry3d>,
    stats: RotatingBroadPhaseStats3d,
}

impl RotatingBroadPhase3d {
    pub fn candidate_pairs(
        &mut self,
        boxes: &[RigidBox3d],
        config: RigidBoxFreeFlightConfig3d,
    ) -> Result<Vec<RotationalSweepPair3d>, RotatingBroadPhaseError3d> {
        self.candidate_pairs_with_admission(boxes, config, false)
    }

    /// Returns only pairs that can mutate at least one physics-owned dynamic body.
    ///
    /// Geometric overlap/query callers must use `candidate_pairs` so externally authoritative contacts
    /// remain visible even when no physical response is possible.
    pub(crate) fn response_candidate_pairs(
        &mut self,
        boxes: &[RigidBox3d],
        config: RigidBoxFreeFlightConfig3d,
    ) -> Result<Vec<RotationalSweepPair3d>, RotatingBroadPhaseError3d> {
        self.candidate_pairs_with_admission(boxes, config, true)
    }

    fn candidate_pairs_with_admission(
        &mut self,
        boxes: &[RigidBox3d],
        config: RigidBoxFreeFlightConfig3d,
        require_response_authority: bool,
    ) -> Result<Vec<RotationalSweepPair3d>, RotatingBroadPhaseError3d> {
        self.stats.queries = self.stats.queries.saturating_add(1);
        let exact = self.bounded_bodies(boxes, config)?;

        if !self.membership_matches(&exact) {
            self.stats.rebuilds = self.stats.rebuilds.saturating_add(1);
            self.rebuild(exact);
        } else {
            let mut escaped = exact
                .iter()
                .copied()
                .filter(|body| {
                    !self
                        .tree
                        .leaf_bounds(body.id)
                        .is_some_and(|fat| contains_bounds(fat, body.bounds))
                })
                .collect::<Vec<_>>();
            escaped.sort_by_key(|body| body.id);
            self.exact = exact.into_iter().map(|body| (body.id, body)).collect();

            if escaped.is_empty() {
                self.stats.reuses = self.stats.reuses.saturating_add(1);
            } else if escaped.len() > self.exact.len() / 2 {
                self.stats.rebuilds = self.stats.rebuilds.saturating_add(1);
                self.rebuild(self.exact.values().copied().collect());
            } else {
                self.stats.incremental_updates = self.stats.incremental_updates.saturating_add(1);
                let mut rotations = 0_u64;
                let mut incremental_ok = true;
                for mut body in escaped {
                    body.bounds = fatten_bounds(body.bounds);
                    if self.tree.reinsert(body, &mut rotations) {
                        self.stats.reinserts = self.stats.reinserts.saturating_add(1);
                    } else {
                        incremental_ok = false;
                        break;
                    }
                }
                self.stats.rotations = self.stats.rotations.saturating_add(rotations);
                if !incremental_ok {
                    self.stats.rebuilds = self.stats.rebuilds.saturating_add(1);
                    self.rebuild(self.exact.values().copied().collect());
                }
            }
        }

        let exact = &self.exact;
        let mut pairs = Vec::new();
        let mut response_authority_rejections = 0_u64;
        self.tree.for_each_candidate_pair(|left, right| {
            let left_body = exact
                .get(&left)
                .expect("indexed broad phase keeps every leaf exact bound");
            let right_body = exact
                .get(&right)
                .expect("indexed broad phase keeps every leaf exact bound");
            if left_body.solver_participation != SolverParticipation3d::Solid
                || right_body.solver_participation != SolverParticipation3d::Solid
                || !bounds_overlap(left_body.bounds, right_body.bounds)
                || !left_body
                    .collision_layers
                    .collides_with(right_body.collision_layers)
            {
                return;
            }
            if require_response_authority
                && !left_body.receives_solver_response
                && !right_body.receives_solver_response
            {
                response_authority_rejections = response_authority_rejections.saturating_add(1);
                return;
            }
            pairs.push(RotationalSweepPair3d { left, right });
        });
        self.stats.response_authority_pair_rejections = self
            .stats
            .response_authority_pair_rejections
            .saturating_add(response_authority_rejections);
        pairs.sort_unstable();
        Ok(pairs)
    }

    /// Updates current-position bounds only for the supplied bodies and returns only candidate
    /// pairs touching one of those bodies. The full broad phase must already be synchronized to the
    /// same current world state; this is the precise API for persistent contact-island propagation
    /// after a local solver response, where unchanged bodies cannot create a new current overlap by
    /// themselves. Transient changed bodies still participate in swept collision discovery, but their
    /// already-resolved impact pairs are deliberately omitted from this resting-contact query.
    pub(crate) fn candidate_pairs_for_changed_current_bodies<'a>(
        &mut self,
        changed_boxes: impl IntoIterator<Item = &'a RigidBox3d>,
    ) -> Result<Vec<RotationalSweepPair3d>, RotatingBroadPhaseError3d> {
        self.candidate_pairs_for_changed_current_bodies_internal(changed_boxes, true)
    }

    /// Query-facing partial update that preserves transient contacts.
    ///
    /// Resting stabilization deliberately omits transient impact pairs after response; observable contact
    /// queries must retain them so lifecycle consumers can see the impact before removing the body.
    pub(crate) fn candidate_pairs_for_changed_current_query_bodies<'a>(
        &mut self,
        changed_boxes: impl IntoIterator<Item = &'a RigidBox3d>,
    ) -> Result<Vec<RotationalSweepPair3d>, RotatingBroadPhaseError3d> {
        self.candidate_pairs_for_changed_current_bodies_internal(changed_boxes, false)
    }

    fn candidate_pairs_for_changed_current_bodies_internal<'a>(
        &mut self,
        changed_boxes: impl IntoIterator<Item = &'a RigidBox3d>,
        omit_transient_contacts: bool,
    ) -> Result<Vec<RotationalSweepPair3d>, RotatingBroadPhaseError3d> {
        self.stats.queries = self.stats.queries.saturating_add(1);
        self.stats.partial_queries = self.stats.partial_queries.saturating_add(1);
        let current = RigidBoxFreeFlightConfig3d::new(crate::Vec3i::ZERO, 0, 1);
        let mut changed_ids = BTreeSet::new();
        let mut transient_changed_ids = BTreeSet::new();
        let mut escaped = Vec::new();

        for rigid_box in changed_boxes {
            let body = bounded_body(rigid_box, current)?;
            if !changed_ids.insert(body.id) {
                return Err(RotatingBroadPhaseError3d::DuplicateBodyId(body.id));
            }
            if omit_transient_contacts
                && rigid_box.contact_persistence() == ContactPersistence3d::Transient
            {
                transient_changed_ids.insert(body.id);
            }
            let Some(previous) = self.exact.get(&body.id) else {
                return Err(RotatingBroadPhaseError3d::IncrementalQueryUnsynchronized(
                    body.id,
                ));
            };
            if previous.kind != body.kind
                || previous.collision_layers != body.collision_layers
                || previous.solver_participation != body.solver_participation
                || previous.receives_solver_response != body.receives_solver_response
            {
                return Err(RotatingBroadPhaseError3d::IncrementalQueryUnsynchronized(
                    body.id,
                ));
            }
            self.exact.insert(body.id, body);
            let Some(fat) = self.tree.leaf_bounds(body.id) else {
                return Err(RotatingBroadPhaseError3d::IncrementalQueryUnsynchronized(
                    body.id,
                ));
            };
            if !contains_bounds(fat, body.bounds) {
                let mut fat_body = body;
                fat_body.bounds = fatten_bounds(body.bounds);
                escaped.push(fat_body);
            }
        }

        self.stats.partial_body_updates = self
            .stats
            .partial_body_updates
            .saturating_add(u64::try_from(changed_ids.len()).unwrap_or(u64::MAX));

        if escaped.is_empty() {
            self.stats.reuses = self.stats.reuses.saturating_add(1);
        } else {
            self.stats.incremental_updates = self.stats.incremental_updates.saturating_add(1);
            let mut rotations = 0_u64;
            let mut incremental_ok = true;
            for body in escaped {
                if self.tree.reinsert(body, &mut rotations) {
                    self.stats.reinserts = self.stats.reinserts.saturating_add(1);
                } else {
                    incremental_ok = false;
                    break;
                }
            }
            self.stats.rotations = self.stats.rotations.saturating_add(rotations);
            if !incremental_ok {
                self.stats.rebuilds = self.stats.rebuilds.saturating_add(1);
                let exact = self.exact.values().copied().collect();
                self.rebuild(exact);
            }
        }

        let exact = &self.exact;
        let mut pairs = BTreeSet::new();
        let mut rejected_pairs = BTreeSet::new();
        for id in changed_ids {
            self.tree
                .for_each_candidate_pair_for_body(id, |left, right| {
                    let left_body = exact
                        .get(&left)
                        .expect("incremental broad phase keeps every leaf exact bound");
                    let right_body = exact
                        .get(&right)
                        .expect("incremental broad phase keeps every leaf exact bound");
                    if transient_changed_ids.contains(&left)
                        || transient_changed_ids.contains(&right)
                        || left_body.solver_participation != SolverParticipation3d::Solid
                        || right_body.solver_participation != SolverParticipation3d::Solid
                        || !bounds_overlap(left_body.bounds, right_body.bounds)
                        || !left_body
                            .collision_layers
                            .collides_with(right_body.collision_layers)
                    {
                        return;
                    }
                    let pair = RotationalSweepPair3d { left, right };
                    if !left_body.receives_solver_response && !right_body.receives_solver_response {
                        rejected_pairs.insert(pair);
                        return;
                    }
                    pairs.insert(pair);
                });
        }
        self.stats.response_authority_pair_rejections = self
            .stats
            .response_authority_pair_rejections
            .saturating_add(u64::try_from(rejected_pairs.len()).unwrap_or(u64::MAX));
        Ok(pairs.into_iter().collect())
    }

    #[must_use]
    pub const fn stats(&self) -> RotatingBroadPhaseStats3d {
        self.stats
    }

    fn bounded_bodies(
        &mut self,
        boxes: &[RigidBox3d],
        config: RigidBoxFreeFlightConfig3d,
    ) -> Result<Vec<BoundedBody3d>, RotatingBroadPhaseError3d> {
        config.exact_timestep()?;
        let mut ids = BTreeSet::new();
        let mut fixed_ids = BTreeSet::new();
        let mut bounded = Vec::with_capacity(boxes.len());

        for rigid_box in boxes {
            let id = rigid_box.body().id();
            if !ids.insert(id) {
                return Err(RotatingBroadPhaseError3d::DuplicateBodyId(id));
            }
            if rigid_box.body().kind() == BodyKind::Fixed {
                fixed_ids.insert(id);
                let shape = rigid_box.oriented_box();
                if let Some(cached) = self.fixed_bounds.get(&id)
                    && cached.shape == shape
                    && cached.collision_layers == rigid_box.collision_layers()
                    && cached.solver_participation == rigid_box.solver_participation()
                {
                    self.stats.fixed_bound_reuses = self.stats.fixed_bound_reuses.saturating_add(1);
                    bounded.push(cached.bounded);
                    continue;
                }
                let body = bounded_body(rigid_box, config)?;
                self.fixed_bounds.insert(
                    id,
                    FixedBoundCacheEntry3d {
                        shape,
                        collision_layers: rigid_box.collision_layers(),
                        solver_participation: rigid_box.solver_participation(),
                        bounded: body,
                    },
                );
                self.stats.fixed_bound_recomputations =
                    self.stats.fixed_bound_recomputations.saturating_add(1);
                bounded.push(body);
            } else {
                self.fixed_bounds.remove(&id);
                self.stats.dynamic_bound_recomputations =
                    self.stats.dynamic_bound_recomputations.saturating_add(1);
                bounded.push(bounded_body(rigid_box, config)?);
            }
        }

        self.fixed_bounds.retain(|id, _| fixed_ids.contains(id));
        Ok(bounded)
    }

    fn membership_matches(&self, exact: &[BoundedBody3d]) -> bool {
        if exact.len() != self.exact.len() || exact.len() != self.tree.len() {
            return false;
        }
        exact.iter().all(|body| {
            self.exact.get(&body.id).is_some_and(|previous| {
                previous.kind == body.kind
                    && previous.solver_participation == body.solver_participation
                    && previous.receives_solver_response == body.receives_solver_response
            }) && self.tree.has_leaf(body.id)
        })
    }

    fn rebuild(&mut self, exact: Vec<BoundedBody3d>) {
        self.exact = exact.iter().copied().map(|body| (body.id, body)).collect();
        let mut fat = exact
            .into_iter()
            .map(|mut body| {
                body.bounds = fatten_bounds(body.bounds);
                body
            })
            .collect::<Vec<_>>();
        self.tree.rebuild(&mut fat);
    }
}

/// Retained stationary-bounds index for wrapper-level spatial queries such as parked-body wake discovery.
///
/// Membership changes rebuild the deterministic tree once, while ordinary frame queries traverse the retained
/// topology and exact stationary bounds without reconstructing every parked body's envelope.
#[derive(Clone, Debug, Default)]
pub(crate) struct RotatingBoundsIndex3d {
    tree: IndexedBvh3d,
    exact: BTreeMap<BodyId, RotationalSweepBounds3d>,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub(crate) struct RotatingBoundsQuery3d {
    pub body_ids: Vec<BodyId>,
    pub visited_nodes: usize,
}

impl RotatingBoundsIndex3d {
    pub(crate) fn rebuild_stationary<'a>(
        &mut self,
        boxes: impl IntoIterator<Item = &'a RigidBox3d>,
    ) -> Result<(), RotatingBroadPhaseError3d> {
        let current = RigidBoxFreeFlightConfig3d::new(crate::Vec3i::ZERO, 0, 1);
        let mut ids = BTreeSet::new();
        let mut exact = BTreeMap::new();
        let mut fat = Vec::new();

        for rigid_box in boxes {
            let id = rigid_box.body().id();
            if !ids.insert(id) {
                return Err(RotatingBroadPhaseError3d::DuplicateBodyId(id));
            }
            if rigid_box.solver_participation() != SolverParticipation3d::Solid {
                continue;
            }
            let mut body = bounded_body(rigid_box, current)?;
            exact.insert(body.id, body.bounds);
            body.bounds = fatten_bounds(body.bounds);
            fat.push(body);
        }

        self.exact = exact;
        self.tree.rebuild(&mut fat);
        Ok(())
    }

    #[must_use]
    pub(crate) fn overlapping_ids(&self, bounds: RotationalSweepBounds3d) -> RotatingBoundsQuery3d {
        let mut body_ids = Vec::new();
        let visited_nodes = self.tree.for_each_body_overlapping_bounds(bounds, |id| {
            if self
                .exact
                .get(&id)
                .is_some_and(|exact| bounds_overlap(bounds, *exact))
            {
                body_ids.push(id);
            }
        });
        body_ids.sort_unstable();
        RotatingBoundsQuery3d {
            body_ids,
            visited_nodes,
        }
    }
}

/// Produces a deterministic conservative candidate set for rotating-box contact search.
///
/// Every body is first enclosed by [`rigid_box_free_flight_sweep_bounds`], so acceleration, velocity
/// reversal, and arbitrary orientation change are preserved as broad-phase possibilities. A one-shot
/// call uses the same indexed AABB BVH implementation as [`RotatingBroadPhase3d`], while world stepping
/// keeps a persistent instance so repeated search/frontier queries can reuse or incrementally update its
/// fat-tree topology.
///
/// Fixed/fixed pairs are omitted because neither body can respond. Input ordering does not affect output.
/// This function proves only that emitted pairs *may* contact and that pairs whose conservative envelopes
/// overlap are retained. It does not evaluate OBB geometry or claim a time of impact.
///
/// # Errors
///
/// Returns [`RotatingBroadPhaseError3d`] for duplicate body IDs or malformed/overflowing free-flight
/// sweep inputs.
pub fn rotational_sweep_candidate_pairs(
    boxes: &[RigidBox3d],
    config: RigidBoxFreeFlightConfig3d,
) -> Result<Vec<RotationalSweepPair3d>, RotatingBroadPhaseError3d> {
    RotatingBroadPhase3d::default().candidate_pairs(boxes, config)
}

fn bounded_body(
    rigid_box: &RigidBox3d,
    config: RigidBoxFreeFlightConfig3d,
) -> Result<BoundedBody3d, RotatingBroadPhaseError3d> {
    Ok(BoundedBody3d {
        id: rigid_box.body().id(),
        kind: rigid_box.body().kind(),
        collision_layers: rigid_box.collision_layers(),
        solver_participation: rigid_box.solver_participation(),
        receives_solver_response: rigid_box.body().kind() == BodyKind::Dynamic
            && rigid_box.motion_authority() == MotionAuthority3d::Physics,
        bounds: rigid_box_free_flight_sweep_bounds(rigid_box, config)?,
    })
}

fn fatten_bounds(bounds: RotationalSweepBounds3d) -> RotationalSweepBounds3d {
    let mut minimum = bounds.minimum;
    let mut maximum = bounds.maximum;
    for axis in 0..3 {
        let extent = (i128::from(bounds.maximum[axis]) - i128::from(bounds.minimum[axis])).max(1);
        minimum[axis] = clamp_i128_to_i64(i128::from(bounds.minimum[axis]) - extent);
        maximum[axis] = clamp_i128_to_i64(i128::from(bounds.maximum[axis]) + extent);
    }
    RotationalSweepBounds3d { minimum, maximum }
}

fn clamp_i128_to_i64(value: i128) -> i64 {
    value
        .clamp(i128::from(i64::MIN), i128::from(i64::MAX))
        .try_into()
        .expect("clamped value fits i64")
}

fn contains_bounds(outer: RotationalSweepBounds3d, inner: RotationalSweepBounds3d) -> bool {
    (0..3).all(|axis| {
        outer.minimum[axis] <= inner.minimum[axis] && outer.maximum[axis] >= inner.maximum[axis]
    })
}

fn union_bounds(
    left: RotationalSweepBounds3d,
    right: RotationalSweepBounds3d,
) -> RotationalSweepBounds3d {
    RotationalSweepBounds3d {
        minimum: [
            left.minimum[0].min(right.minimum[0]),
            left.minimum[1].min(right.minimum[1]),
            left.minimum[2].min(right.minimum[2]),
        ],
        maximum: [
            left.maximum[0].max(right.maximum[0]),
            left.maximum[1].max(right.maximum[1]),
            left.maximum[2].max(right.maximum[2]),
        ],
    }
}

fn widest_axis(bounds: RotationalSweepBounds3d) -> usize {
    let extents = [
        i128::from(bounds.maximum[0]) - i128::from(bounds.minimum[0]),
        i128::from(bounds.maximum[1]) - i128::from(bounds.minimum[1]),
        i128::from(bounds.maximum[2]) - i128::from(bounds.minimum[2]),
    ];
    if extents[1] > extents[0] && extents[1] >= extents[2] {
        1
    } else if extents[2] > extents[0] && extents[2] > extents[1] {
        2
    } else {
        0
    }
}

fn center_twice(bounds: RotationalSweepBounds3d, axis: usize) -> i128 {
    i128::from(bounds.minimum[axis]) + i128::from(bounds.maximum[axis])
}

fn bounds_overlap(left: RotationalSweepBounds3d, right: RotationalSweepBounds3d) -> bool {
    (0..3).all(|axis| overlaps_on_axis(left, right, axis))
}

fn overlaps_on_axis(
    left: RotationalSweepBounds3d,
    right: RotationalSweepBounds3d,
    axis: usize,
) -> bool {
    left.minimum[axis] <= right.maximum[axis] && right.minimum[axis] <= left.maximum[axis]
}

#[cfg(test)]
mod tests {
    use std::{hint::black_box, time::Instant};

    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, BodyKind, MotionAuthority3d, Orientation3d,
        RigidBody, RigidBox3d, RigidBoxFreeFlightConfig3d, RotationalSweepBounds3d,
        SolverParticipation3d, Vec3i, rigid_box_free_flight_sweep_bounds,
    };

    use super::{
        BoundedBody3d, RotatingBoundsIndex3d, RotatingBroadPhase3d, RotatingBroadPhaseError3d,
        RotationalSweepPair3d, bounds_overlap, rotational_sweep_candidate_pairs,
        tree::IndexedBvh3d,
    };

    fn dynamic(id: u64, position: Vec3i, velocity: Vec3i) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::dynamic(BodyId(id), position, velocity, Vec3i::new(1, 1, 1)),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid dynamic box")
    }

    fn fixed(id: u64, position: Vec3i) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::fixed(BodyId(id), position, Vec3i::new(1, 1, 1)),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid fixed box")
    }

    fn external(id: u64, position: Vec3i, velocity: Vec3i) -> RigidBox3d {
        dynamic(id, position, velocity).with_external_motion()
    }

    fn brute_force_candidate_pairs(
        boxes: &[RigidBox3d],
        config: RigidBoxFreeFlightConfig3d,
    ) -> Vec<RotationalSweepPair3d> {
        let bounded = boxes
            .iter()
            .map(|rigid_box| BoundedBody3d {
                id: rigid_box.body().id(),
                kind: rigid_box.body().kind(),
                collision_layers: rigid_box.collision_layers(),
                solver_participation: rigid_box.solver_participation(),
                receives_solver_response: rigid_box.body().kind() == BodyKind::Dynamic
                    && rigid_box.motion_authority() == MotionAuthority3d::Physics,
                bounds: rigid_box_free_flight_sweep_bounds(rigid_box, config)
                    .expect("valid brute-force sweep bounds"),
            })
            .collect::<Vec<_>>();
        let mut pairs = Vec::new();
        for left_index in 0..bounded.len() {
            let left = bounded[left_index];
            for right in bounded.iter().copied().skip(left_index + 1) {
                if left.kind == BodyKind::Fixed && right.kind == BodyKind::Fixed {
                    continue;
                }
                if left.solver_participation != SolverParticipation3d::Solid
                    || right.solver_participation != SolverParticipation3d::Solid
                {
                    continue;
                }
                if !bounds_overlap(left.bounds, right.bounds) {
                    continue;
                }
                let (pair_left, pair_right) = if left.id < right.id {
                    (left.id, right.id)
                } else {
                    (right.id, left.id)
                };
                pairs.push(RotationalSweepPair3d {
                    left: pair_left,
                    right: pair_right,
                });
            }
        }
        pairs.sort_unstable();
        pairs
    }

    #[test]
    fn moving_sweeps_produce_stable_pair_independent_of_input_order() {
        let left = dynamic(9, Vec3i::new(-20, 0, 0), Vec3i::new(20, 0, 0));
        let right = dynamic(2, Vec3i::new(20, 0, 0), Vec3i::new(-20, 0, 0));
        let config = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1);
        let expected = vec![RotationalSweepPair3d {
            left: BodyId(2),
            right: BodyId(9),
        }];

        assert_eq!(
            rotational_sweep_candidate_pairs(&[left.clone(), right.clone()], config)
                .expect("valid candidates"),
            expected
        );
        assert_eq!(
            rotational_sweep_candidate_pairs(&[right, left], config).expect("valid candidates"),
            expected
        );
    }

    #[test]
    fn separated_and_fixed_fixed_pairs_are_omitted() {
        let config = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1);
        assert!(
            rotational_sweep_candidate_pairs(
                &[
                    dynamic(1, Vec3i::new(-100, 0, 0), Vec3i::ZERO),
                    dynamic(2, Vec3i::new(100, 0, 0), Vec3i::ZERO),
                ],
                config,
            )
            .expect("valid separated candidates")
            .is_empty()
        );
        assert!(
            rotational_sweep_candidate_pairs(
                &[fixed(3, Vec3i::ZERO), fixed(4, Vec3i::new(1, 0, 0))],
                config,
            )
            .expect("valid fixed candidates")
            .is_empty()
        );
    }

    #[test]
    fn no_response_authority_pairs_are_rejected_before_contact_search() {
        let config = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1);
        let mut broad_phase = RotatingBroadPhase3d::default();
        let no_authority = [
            external(20, Vec3i::ZERO, Vec3i::new(1, 0, 0)),
            external(21, Vec3i::ZERO, Vec3i::new(-1, 0, 0)),
            fixed(22, Vec3i::ZERO),
        ];

        assert_eq!(
            broad_phase
                .candidate_pairs(&no_authority, config)
                .expect("valid geometric query")
                .len(),
            3,
            "geometric queries must retain externally authoritative overlaps"
        );
        assert!(
            broad_phase
                .response_candidate_pairs(&no_authority, config)
                .expect("valid response-admitted query")
                .is_empty()
        );
        assert!(
            broad_phase.stats().response_authority_pair_rejections >= 3,
            "every overlapping external/external or external/fixed pair should be rejected"
        );

        assert_eq!(
            rotational_sweep_candidate_pairs(
                &[
                    dynamic(30, Vec3i::ZERO, Vec3i::ZERO),
                    external(31, Vec3i::ZERO, Vec3i::ZERO),
                ],
                config,
            )
            .expect("dynamic/external candidate"),
            vec![RotationalSweepPair3d {
                left: BodyId(30),
                right: BodyId(31),
            }]
        );
    }

    #[test]
    fn overlap_only_bodies_are_not_solver_candidates() {
        let sensor = dynamic(7, Vec3i::ZERO, Vec3i::ZERO).with_overlap_only();
        let solid = dynamic(8, Vec3i::ZERO, Vec3i::ZERO);
        let config = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1);

        assert_eq!(
            sensor.solver_participation(),
            SolverParticipation3d::OverlapOnly
        );
        assert!(
            rotational_sweep_candidate_pairs(&[sensor, solid], config)
                .expect("valid sensor candidate query")
                .is_empty()
        );
    }

    #[test]
    fn acceleration_reversal_keeps_interior_candidate() {
        let moving = dynamic(5, Vec3i::ZERO, Vec3i::new(100, 0, 0));
        let obstacle = fixed(6, Vec3i::new(13, 0, 0));
        let config = RigidBoxFreeFlightConfig3d::new(Vec3i::new(-200, 0, 0), 1, 1);

        assert_eq!(
            rotational_sweep_candidate_pairs(&[moving, obstacle], config)
                .expect("valid reversal candidates"),
            vec![RotationalSweepPair3d {
                left: BodyId(5),
                right: BodyId(6),
            }]
        );
    }

    #[test]
    fn indexed_bvh_matches_brute_force_and_input_permutation() {
        let mut boxes = (0..48_u64)
            .map(|id| {
                let position = Vec3i::new(
                    i32::try_from(id % 8).expect("small x") * 5,
                    i32::try_from(id / 8).expect("small y") * 4,
                    i32::try_from(id % 3).expect("small z") * 3,
                );
                if id % 7 == 0 {
                    fixed(id + 1, position)
                } else {
                    dynamic(
                        id + 1,
                        position,
                        Vec3i::new(
                            if id % 2 == 0 { 4 } else { -3 },
                            i32::try_from(id % 3).expect("small velocity") - 1,
                            0,
                        ),
                    )
                }
            })
            .collect::<Vec<_>>();
        let config = RigidBoxFreeFlightConfig3d::new(Vec3i::new(0, -2, 0), 2, 1);
        let expected = brute_force_candidate_pairs(&boxes, config);

        assert_eq!(
            rotational_sweep_candidate_pairs(&boxes, config).expect("valid BVH candidates"),
            expected
        );
        boxes.reverse();
        assert_eq!(
            rotational_sweep_candidate_pairs(&boxes, config)
                .expect("valid permuted BVH candidates"),
            expected
        );
    }

    #[test]
    fn persistent_tree_reuses_fat_topology_without_changing_exact_pairs() {
        let config = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1);
        let initial = vec![
            dynamic(1, Vec3i::new(-6, 0, 0), Vec3i::new(2, 0, 0)),
            dynamic(2, Vec3i::new(0, 0, 0), Vec3i::ZERO),
            fixed(3, Vec3i::new(8, 0, 0)),
        ];
        let moved = vec![
            dynamic(1, Vec3i::new(-5, 0, 0), Vec3i::new(2, 0, 0)),
            dynamic(2, Vec3i::new(1, 0, 0), Vec3i::ZERO),
            fixed(3, Vec3i::new(8, 0, 0)),
        ];
        let mut broad_phase = RotatingBroadPhase3d::default();

        broad_phase
            .candidate_pairs(&initial, config)
            .expect("initial persistent query");
        let actual = broad_phase
            .candidate_pairs(&moved, config)
            .expect("moved persistent query");

        assert_eq!(actual, brute_force_candidate_pairs(&moved, config));
        assert_eq!(broad_phase.stats().queries, 2);
        assert_eq!(broad_phase.stats().rebuilds, 1);
        assert_eq!(broad_phase.stats().reuses, 1);
        assert_eq!(broad_phase.stats().incremental_updates, 0);
        broad_phase
            .tree
            .validate_structure()
            .expect("valid reused indexed tree");
    }

    #[test]
    fn escaping_a_fat_leaf_reinserts_without_full_rebuild() {
        let config = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1);
        let initial = [
            dynamic(1, Vec3i::ZERO, Vec3i::ZERO),
            dynamic(2, Vec3i::new(10, 0, 0), Vec3i::ZERO),
            fixed(3, Vec3i::new(20, 0, 0)),
        ];
        let escaped = [
            dynamic(1, Vec3i::new(1_000, 0, 0), Vec3i::ZERO),
            dynamic(2, Vec3i::new(10, 0, 0), Vec3i::ZERO),
            fixed(3, Vec3i::new(20, 0, 0)),
        ];
        let mut broad_phase = RotatingBroadPhase3d::default();

        broad_phase
            .candidate_pairs(&initial, config)
            .expect("initial persistent query");
        let actual = broad_phase
            .candidate_pairs(&escaped, config)
            .expect("escaped persistent query");

        assert_eq!(actual, brute_force_candidate_pairs(&escaped, config));
        assert_eq!(broad_phase.stats().rebuilds, 1);
        assert_eq!(broad_phase.stats().incremental_updates, 1);
        assert_eq!(broad_phase.stats().reinserts, 1);
        broad_phase
            .tree
            .validate_structure()
            .expect("valid single-reinsert indexed tree");
    }

    #[test]
    fn more_than_eight_escaped_leaves_update_incrementally() {
        let config = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1);
        let initial = (0..64_u64)
            .map(|id| {
                dynamic(
                    id + 1,
                    Vec3i::new(i32::try_from(id).expect("small position") * 8, 0, 0),
                    Vec3i::ZERO,
                )
            })
            .collect::<Vec<_>>();
        let mut escaped = initial.clone();
        for (index, rigid_box) in escaped.iter_mut().take(16).enumerate() {
            *rigid_box = dynamic(
                u64::try_from(index).expect("small id") + 1,
                Vec3i::new(
                    20_000 + i32::try_from(index).expect("small position") * 16,
                    0,
                    0,
                ),
                Vec3i::ZERO,
            );
        }
        let mut broad_phase = RotatingBroadPhase3d::default();
        broad_phase
            .candidate_pairs(&initial, config)
            .expect("initial indexed query");
        let actual = broad_phase
            .candidate_pairs(&escaped, config)
            .expect("multi-leaf indexed query");

        assert_eq!(actual, brute_force_candidate_pairs(&escaped, config));
        assert_eq!(broad_phase.stats().rebuilds, 1);
        assert_eq!(broad_phase.stats().incremental_updates, 1);
        assert_eq!(broad_phase.stats().reinserts, 16);
        broad_phase
            .tree
            .validate_structure()
            .expect("valid multi-reinsert indexed tree");
    }

    #[test]
    fn majority_escape_uses_balanced_rebuild() {
        let config = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1);
        let initial = (0..64_u64)
            .map(|id| {
                dynamic(
                    id + 1,
                    Vec3i::new(i32::try_from(id).expect("small position") * 8, 0, 0),
                    Vec3i::ZERO,
                )
            })
            .collect::<Vec<_>>();
        let mut escaped = initial.clone();
        for (index, rigid_box) in escaped.iter_mut().take(40).enumerate() {
            *rigid_box = dynamic(
                u64::try_from(index).expect("small id") + 1,
                Vec3i::new(
                    50_000 + i32::try_from(index).expect("small position") * 16,
                    0,
                    0,
                ),
                Vec3i::ZERO,
            );
        }
        let mut broad_phase = RotatingBroadPhase3d::default();
        broad_phase
            .candidate_pairs(&initial, config)
            .expect("initial majority query");
        let actual = broad_phase
            .candidate_pairs(&escaped, config)
            .expect("majority escape query");

        assert_eq!(actual, brute_force_candidate_pairs(&escaped, config));
        assert_eq!(broad_phase.stats().rebuilds, 2);
        assert_eq!(broad_phase.stats().incremental_updates, 0);
        assert_eq!(broad_phase.stats().reinserts, 0);
        broad_phase
            .tree
            .validate_structure()
            .expect("valid majority-rebuild indexed tree");
    }

    #[test]
    fn membership_change_still_rebuilds_the_tree() {
        let config = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1);
        let initial = [dynamic(1, Vec3i::ZERO, Vec3i::ZERO)];
        let added = [
            dynamic(1, Vec3i::ZERO, Vec3i::ZERO),
            fixed(2, Vec3i::new(10, 0, 0)),
        ];
        let mut broad_phase = RotatingBroadPhase3d::default();

        broad_phase
            .candidate_pairs(&initial, config)
            .expect("initial persistent query");
        broad_phase
            .candidate_pairs(&added, config)
            .expect("membership-changing query");

        assert_eq!(broad_phase.stats().rebuilds, 2);
        assert_eq!(broad_phase.stats().incremental_updates, 0);
        assert_eq!(broad_phase.stats().reinserts, 0);
    }

    #[test]
    fn repeated_leaf_churn_stays_balanced_and_matches_brute_force() {
        let config = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1);
        let mut boxes = (0..127_u64)
            .map(|id| {
                dynamic(
                    id + 1,
                    Vec3i::new(i32::try_from(id).expect("small position") * 8, 0, 0),
                    Vec3i::ZERO,
                )
            })
            .collect::<Vec<_>>();
        let mut broad_phase = RotatingBroadPhase3d::default();
        broad_phase
            .candidate_pairs(&boxes, config)
            .expect("initial churn query");

        for iteration in 0..64_i32 {
            boxes[0] = dynamic(1, Vec3i::new(10_000 + iteration * 100, 0, 0), Vec3i::ZERO);
            let actual = broad_phase
                .candidate_pairs(&boxes, config)
                .expect("incremental churn query");
            assert_eq!(actual, brute_force_candidate_pairs(&boxes, config));
            assert!(
                broad_phase.tree.height() <= 12,
                "tree height grew to {}",
                broad_phase.tree.height()
            );
            broad_phase
                .tree
                .validate_structure()
                .expect("valid indexed tree during churn");
        }

        assert_eq!(broad_phase.stats().rebuilds, 1);
        assert!(broad_phase.stats().incremental_updates > 0);
        assert!(broad_phase.stats().reinserts > 0);
    }

    #[test]
    fn monotonic_input_builds_a_balanced_indexed_tree() {
        let mut bodies = (0..127_u64)
            .map(|id| {
                let coordinate = i64::try_from(id).expect("small coordinate") * 4;
                BoundedBody3d {
                    id: BodyId(id + 1),
                    kind: BodyKind::Dynamic,
                    collision_layers: crate::CollisionLayers3d::ALL,
                    solver_participation: SolverParticipation3d::Solid,
                    receives_solver_response: true,
                    bounds: RotationalSweepBounds3d {
                        minimum: [coordinate, 0, 0],
                        maximum: [coordinate + 2, 2, 2],
                    },
                }
            })
            .collect::<Vec<_>>();
        let mut tree = IndexedBvh3d::default();
        tree.rebuild(&mut bodies);

        assert!(tree.height() <= 8);
        tree.validate_structure()
            .expect("valid balanced indexed tree");
    }

    #[test]
    #[ignore = "microbenchmark; run explicitly in release mode"]
    fn persistent_broad_phase_benchmark() {
        let boxes = (0..256_u64)
            .map(|id| {
                dynamic(
                    id + 1,
                    Vec3i::new(
                        i32::try_from(id % 16).expect("small x") * 4,
                        i32::try_from(id / 16).expect("small y") * 4,
                        0,
                    ),
                    Vec3i::new(i32::try_from(id % 3).expect("small velocity") - 1, 0, 0),
                )
            })
            .collect::<Vec<_>>();
        let config = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 60);
        let iterations = 100_u32;

        let rebuild_start = Instant::now();
        for _ in 0..iterations {
            black_box(
                rotational_sweep_candidate_pairs(black_box(&boxes), config)
                    .expect("stateless benchmark query"),
            );
        }
        let rebuild_elapsed = rebuild_start.elapsed();

        let mut persistent = RotatingBroadPhase3d::default();
        let persistent_start = Instant::now();
        for _ in 0..iterations {
            black_box(
                persistent
                    .candidate_pairs(black_box(&boxes), config)
                    .expect("persistent benchmark query"),
            );
        }
        let persistent_elapsed = persistent_start.elapsed();

        eprintln!(
            "broad-phase 256 bodies × {iterations}: rebuild={rebuild_elapsed:?}, indexed={persistent_elapsed:?}, stats={:?}",
            persistent.stats()
        );
        assert_eq!(persistent.stats().rebuilds, 1);
        assert_eq!(persistent.stats().reuses, u64::from(iterations - 1));
    }

    #[test]
    #[ignore = "microbenchmark; run explicitly in release mode"]
    fn indexed_multi_leaf_churn_benchmark() {
        let base = (0..256_u64)
            .map(|id| {
                dynamic(
                    id + 1,
                    Vec3i::new(
                        i32::try_from(id % 16).expect("small x") * 4,
                        i32::try_from(id / 16).expect("small y") * 4,
                        0,
                    ),
                    Vec3i::ZERO,
                )
            })
            .collect::<Vec<_>>();
        let snapshots = (0..100_i32)
            .map(|iteration| {
                let mut boxes = base.clone();
                for (index, rigid_box) in boxes.iter_mut().enumerate().take(16) {
                    *rigid_box = dynamic(
                        u64::try_from(index).expect("small id") + 1,
                        Vec3i::new(
                            10_000
                                + iteration * 100
                                + i32::try_from(index).expect("small index") * 8,
                            0,
                            0,
                        ),
                        Vec3i::ZERO,
                    );
                }
                boxes
            })
            .collect::<Vec<_>>();
        let config = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 60);

        let rebuild_start = Instant::now();
        for boxes in &snapshots {
            black_box(
                rotational_sweep_candidate_pairs(black_box(boxes), config)
                    .expect("stateless churn benchmark query"),
            );
        }
        let rebuild_elapsed = rebuild_start.elapsed();

        let mut persistent = RotatingBroadPhase3d::default();
        let indexed_start = Instant::now();
        for boxes in &snapshots {
            black_box(
                persistent
                    .candidate_pairs(black_box(boxes), config)
                    .expect("indexed churn benchmark query"),
            );
        }
        let indexed_elapsed = indexed_start.elapsed();

        eprintln!(
            "broad-phase 256-body 16-leaf churn × {}: rebuild={rebuild_elapsed:?}, indexed={indexed_elapsed:?}, stats={:?}",
            snapshots.len(),
            persistent.stats()
        );
        assert_eq!(persistent.stats().rebuilds, 1);
        assert!(persistent.stats().incremental_updates > 0);
        assert!(persistent.stats().reinserts > 8);
    }

    #[test]
    fn retained_bounds_index_prunes_sparse_stationary_queries() {
        let boxes = (0..256_u64)
            .map(|index| {
                dynamic(
                    index + 1,
                    Vec3i::new(i32::try_from(index).expect("small index") * 16, 0, 0),
                    Vec3i::ZERO,
                )
            })
            .collect::<Vec<_>>();
        let mut index = RotatingBoundsIndex3d::default();
        index
            .rebuild_stationary(boxes.iter())
            .expect("stationary bounds index");

        let query_body = dynamic(10_000, Vec3i::new(100 * 16, 0, 0), Vec3i::ZERO);
        let query_bounds = rigid_box_free_flight_sweep_bounds(
            &query_body,
            RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 0, 1),
        )
        .expect("query bounds");
        let query = index.overlapping_ids(query_bounds);

        let expected = boxes
            .iter()
            .filter_map(|rigid_box| {
                let bounds = rigid_box_free_flight_sweep_bounds(
                    rigid_box,
                    RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 0, 1),
                )
                .expect("stationary bounds");
                bounds_overlap(query_bounds, bounds).then_some(rigid_box.body().id())
            })
            .collect::<Vec<_>>();

        assert_eq!(query.body_ids, expected);
        assert!(
            query.visited_nodes < boxes.len(),
            "sparse query should prune retained-tree work: visited={} bodies={}",
            query.visited_nodes,
            boxes.len()
        );
    }

    #[test]
    fn repeated_queries_reuse_fixed_bounds_but_recompute_dynamic_sweeps() {
        let mut boxes = (0..128_u64)
            .map(|index| {
                fixed(
                    index + 10,
                    Vec3i::new(i32::try_from(index).expect("small index") * 8, -10, 0),
                )
            })
            .collect::<Vec<_>>();
        boxes.push(dynamic(1, Vec3i::ZERO, Vec3i::new(1, 0, 0)));
        let config = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 60);
        let mut broad_phase = RotatingBroadPhase3d::default();

        broad_phase
            .candidate_pairs(&boxes, config)
            .expect("first query");
        let first = broad_phase.stats();
        broad_phase
            .candidate_pairs(&boxes, config)
            .expect("second query");
        let second = broad_phase.stats();

        assert_eq!(first.fixed_bound_recomputations, 128);
        assert_eq!(first.fixed_bound_reuses, 0);
        assert_eq!(first.dynamic_bound_recomputations, 1);
        assert_eq!(second.fixed_bound_recomputations, 128);
        assert_eq!(second.fixed_bound_reuses, 128);
        assert_eq!(second.dynamic_bound_recomputations, 2);
    }

    #[test]
    fn duplicate_ids_fail_closed() {
        let config = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1);
        assert_eq!(
            rotational_sweep_candidate_pairs(
                &[
                    dynamic(7, Vec3i::ZERO, Vec3i::ZERO),
                    dynamic(7, Vec3i::new(10, 0, 0), Vec3i::ZERO),
                ],
                config,
            ),
            Err(RotatingBroadPhaseError3d::DuplicateBodyId(BodyId(7)))
        );
    }
}

#[cfg(test)]
mod necessary_work_tests {
    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d,
        RigidBoxFreeFlightConfig3d, Vec3i,
    };

    use super::RotatingBroadPhase3d;

    fn box3d(body: RigidBody) -> RigidBox3d {
        RigidBox3d::new(
            body,
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid box")
    }

    #[test]
    fn partial_current_query_updates_only_supplied_bodies_and_matches_full_truth() {
        let mut boxes = vec![
            box3d(RigidBody::dynamic(
                BodyId(1),
                Vec3i::new(0, 0, 0),
                Vec3i::ZERO,
                Vec3i::new(5, 5, 5),
            )),
            box3d(RigidBody::dynamic(
                BodyId(2),
                Vec3i::new(30, 0, 0),
                Vec3i::ZERO,
                Vec3i::new(5, 5, 5),
            )),
            box3d(RigidBody::fixed(
                BodyId(3),
                Vec3i::new(60, 0, 0),
                Vec3i::new(5, 5, 5),
            )),
        ];
        let current = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 0, 1);
        let mut partial = RotatingBroadPhase3d::default();
        partial
            .candidate_pairs(&boxes, current)
            .expect("initial sync");

        boxes[0].body.position = Vec3i::new(25, 0, 0);
        let partial_pairs = partial
            .candidate_pairs_for_changed_current_bodies([&boxes[0]])
            .expect("partial query");

        let mut full = RotatingBroadPhase3d::default();
        let full_pairs = full.candidate_pairs(&boxes, current).expect("full query");
        let expected = full_pairs
            .into_iter()
            .filter(|pair| pair.left == BodyId(1) || pair.right == BodyId(1))
            .collect::<Vec<_>>();
        assert_eq!(partial_pairs, expected);
        assert_eq!(partial.stats().partial_queries, 1);
        assert_eq!(partial.stats().partial_body_updates, 1);
    }

    #[test]
    fn transient_changed_body_is_not_reintroduced_as_a_resting_contact() {
        let boxes = vec![
            box3d(RigidBody::dynamic(
                BodyId(1),
                Vec3i::ZERO,
                Vec3i::new(10, 0, 0),
                Vec3i::new(5, 5, 5),
            ))
            .with_transient_contacts(),
            box3d(RigidBody::fixed(
                BodyId(2),
                Vec3i::new(9, 0, 0),
                Vec3i::new(5, 5, 5),
            )),
        ];
        let current = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 0, 1);
        let mut broad_phase = RotatingBroadPhase3d::default();
        assert_eq!(
            broad_phase
                .candidate_pairs(&boxes, current)
                .expect("ordinary current query still discovers the impact pair")
                .len(),
            1
        );
        assert!(
            broad_phase
                .candidate_pairs_for_changed_current_bodies([&boxes[0]])
                .expect("stabilization query")
                .is_empty(),
            "transient impact must not become a persistent stabilization constraint"
        );
    }
}
