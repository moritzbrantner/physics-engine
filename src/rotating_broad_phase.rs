use std::{
    collections::{BTreeMap, BTreeSet},
    error::Error,
    fmt,
};

use crate::{
    BodyId, BodyKind, RigidBox3d, RigidBoxFreeFlightConfig3d, RigidBoxFreeFlightError3d,
    RotationalSweepBounds3d, rigid_box_free_flight_sweep_bounds,
};

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
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RotatingBroadPhaseError3d {
    DuplicateBodyId(BodyId),
    FreeFlight(RigidBoxFreeFlightError3d),
}

impl fmt::Display for RotatingBroadPhaseError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DuplicateBodyId(id) => {
                write!(
                    formatter,
                    "rotational broad phase received duplicate body id {}",
                    id.0
                )
            }
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
    bounds: RotationalSweepBounds3d,
}

#[derive(Clone, Debug)]
struct BroadPhaseBvhNode3d {
    bounds: RotationalSweepBounds3d,
    has_dynamic: bool,
    kind: BroadPhaseBvhNodeKind3d,
}

#[derive(Clone, Debug)]
enum BroadPhaseBvhNodeKind3d {
    Leaf(BoundedBody3d),
    Branch {
        left: Box<BroadPhaseBvhNode3d>,
        right: Box<BroadPhaseBvhNode3d>,
    },
}

impl BroadPhaseBvhNode3d {
    fn leaf(body: BoundedBody3d) -> Self {
        Self {
            bounds: body.bounds,
            has_dynamic: body.kind == BodyKind::Dynamic,
            kind: BroadPhaseBvhNodeKind3d::Leaf(body),
        }
    }

    fn branch(left: Self, right: Self) -> Self {
        Self {
            bounds: union_bounds(left.bounds, right.bounds),
            has_dynamic: left.has_dynamic || right.has_dynamic,
            kind: BroadPhaseBvhNodeKind3d::Branch {
                left: Box::new(left),
                right: Box::new(right),
            },
        }
    }
}

/// Persistent deterministic broad phase for rotating rigid boxes.
///
/// The tree stores fat conservative envelopes so repeated queries can retain the same balanced topology
/// while exact sweep bounds move within those envelopes. Candidate output is still filtered against the
/// exact current sweep bounds, so persistence changes pruning cost only; it cannot add or remove a pair
/// relative to a fresh conservative broad-phase build.
#[derive(Clone, Debug, Default)]
pub(crate) struct RotatingBroadPhase3d {
    tree: Option<BroadPhaseBvhNode3d>,
    exact: BTreeMap<BodyId, BoundedBody3d>,
    fat_bounds: BTreeMap<BodyId, RotationalSweepBounds3d>,
    stats: RotatingBroadPhaseStats3d,
}

impl RotatingBroadPhase3d {
    pub fn candidate_pairs(
        &mut self,
        boxes: &[RigidBox3d],
        config: RigidBoxFreeFlightConfig3d,
    ) -> Result<Vec<RotationalSweepPair3d>, RotatingBroadPhaseError3d> {
        self.stats.queries = self.stats.queries.saturating_add(1);
        let exact = bounded_bodies(boxes, config)?;

        if self.can_reuse(&exact) {
            self.stats.reuses = self.stats.reuses.saturating_add(1);
            self.exact = exact.into_iter().map(|body| (body.id, body)).collect();
        } else {
            self.stats.rebuilds = self.stats.rebuilds.saturating_add(1);
            self.rebuild(exact);
        }

        let Some(tree) = &self.tree else {
            return Ok(Vec::new());
        };
        let mut pairs = Vec::new();
        collect_exact_pairs_within(tree, &self.exact, &mut pairs);
        pairs.sort_unstable();
        Ok(pairs)
    }

    #[must_use]
    pub const fn stats(&self) -> RotatingBroadPhaseStats3d {
        self.stats
    }

    fn can_reuse(&self, exact: &[BoundedBody3d]) -> bool {
        if self.tree.is_none()
            || exact.len() != self.exact.len()
            || exact.len() != self.fat_bounds.len()
        {
            return false;
        }

        exact.iter().all(|body| {
            self.exact
                .get(&body.id)
                .is_some_and(|previous| previous.kind == body.kind)
                && self
                    .fat_bounds
                    .get(&body.id)
                    .is_some_and(|fat| contains_bounds(*fat, body.bounds))
        })
    }

    fn rebuild(&mut self, exact: Vec<BoundedBody3d>) {
        self.exact = exact.iter().copied().map(|body| (body.id, body)).collect();
        self.fat_bounds.clear();
        let mut fat = exact
            .into_iter()
            .map(|mut body| {
                body.bounds = fatten_bounds(body.bounds);
                self.fat_bounds.insert(body.id, body.bounds);
                body
            })
            .collect::<Vec<_>>();
        self.tree = build_balanced_bvh(&mut fat);
    }
}

/// Produces a deterministic conservative candidate set for rotating-box contact search.
///
/// Every body is first enclosed by [`rigid_box_free_flight_sweep_bounds`], so acceleration, velocity
/// reversal, and arbitrary orientation change are preserved as broad-phase possibilities. A one-shot
/// call uses the same balanced AABB BVH implementation as [`RotatingBroadPhase3d`], while world stepping
/// keeps a persistent instance so repeated search/frontier queries can reuse fat-tree topology.
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

fn bounded_bodies(
    boxes: &[RigidBox3d],
    config: RigidBoxFreeFlightConfig3d,
) -> Result<Vec<BoundedBody3d>, RotatingBroadPhaseError3d> {
    let mut ids = BTreeSet::new();
    let mut bounded = Vec::with_capacity(boxes.len());
    for rigid_box in boxes {
        let id = rigid_box.body().id();
        if !ids.insert(id) {
            return Err(RotatingBroadPhaseError3d::DuplicateBodyId(id));
        }
        bounded.push(BoundedBody3d {
            id,
            kind: rigid_box.body().kind(),
            bounds: rigid_box_free_flight_sweep_bounds(rigid_box, config)?,
        });
    }
    Ok(bounded)
}

fn build_balanced_bvh(bodies: &mut [BoundedBody3d]) -> Option<BroadPhaseBvhNode3d> {
    if bodies.is_empty() {
        None
    } else {
        Some(build_bvh_node(bodies))
    }
}

fn build_bvh_node(bodies: &mut [BoundedBody3d]) -> BroadPhaseBvhNode3d {
    if let [body] = bodies {
        return BroadPhaseBvhNode3d::leaf(*body);
    }

    let aggregate = bodies
        .iter()
        .skip(1)
        .fold(bodies[0].bounds, |bounds, body| {
            union_bounds(bounds, body.bounds)
        });
    let axis = widest_axis(aggregate);
    bodies.sort_by_key(|body| (center_twice(body.bounds, axis), body.id));

    let middle = bodies.len() / 2;
    let (left_bodies, right_bodies) = bodies.split_at_mut(middle);
    BroadPhaseBvhNode3d::branch(build_bvh_node(left_bodies), build_bvh_node(right_bodies))
}

fn collect_exact_pairs_within(
    node: &BroadPhaseBvhNode3d,
    exact: &BTreeMap<BodyId, BoundedBody3d>,
    pairs: &mut Vec<RotationalSweepPair3d>,
) {
    let BroadPhaseBvhNodeKind3d::Branch { left, right } = &node.kind else {
        return;
    };

    collect_exact_pairs_within(left, exact, pairs);
    collect_exact_pairs_within(right, exact, pairs);
    collect_exact_pairs_between(left, right, exact, pairs);
}

fn collect_exact_pairs_between(
    left: &BroadPhaseBvhNode3d,
    right: &BroadPhaseBvhNode3d,
    exact: &BTreeMap<BodyId, BoundedBody3d>,
    pairs: &mut Vec<RotationalSweepPair3d>,
) {
    if (!left.has_dynamic && !right.has_dynamic) || !bounds_overlap(left.bounds, right.bounds) {
        return;
    }

    match (&left.kind, &right.kind) {
        (BroadPhaseBvhNodeKind3d::Leaf(left_fat), BroadPhaseBvhNodeKind3d::Leaf(right_fat)) => {
            let left_body = exact
                .get(&left_fat.id)
                .expect("persistent broad phase keeps every leaf exact bound");
            let right_body = exact
                .get(&right_fat.id)
                .expect("persistent broad phase keeps every leaf exact bound");
            if left_body.kind == BodyKind::Fixed && right_body.kind == BodyKind::Fixed {
                return;
            }
            if !bounds_overlap(left_body.bounds, right_body.bounds) {
                return;
            }
            let (pair_left, pair_right) = if left_body.id < right_body.id {
                (left_body.id, right_body.id)
            } else {
                (right_body.id, left_body.id)
            };
            pairs.push(RotationalSweepPair3d {
                left: pair_left,
                right: pair_right,
            });
        }
        (
            BroadPhaseBvhNodeKind3d::Branch {
                left: left_left,
                right: left_right,
            },
            BroadPhaseBvhNodeKind3d::Leaf(_),
        ) => {
            collect_exact_pairs_between(left_left, right, exact, pairs);
            collect_exact_pairs_between(left_right, right, exact, pairs);
        }
        (
            BroadPhaseBvhNodeKind3d::Leaf(_),
            BroadPhaseBvhNodeKind3d::Branch {
                left: right_left,
                right: right_right,
            },
        ) => {
            collect_exact_pairs_between(left, right_left, exact, pairs);
            collect_exact_pairs_between(left, right_right, exact, pairs);
        }
        (
            BroadPhaseBvhNodeKind3d::Branch {
                left: left_left,
                right: left_right,
            },
            BroadPhaseBvhNodeKind3d::Branch {
                left: right_left,
                right: right_right,
            },
        ) => {
            collect_exact_pairs_between(left_left, right_left, exact, pairs);
            collect_exact_pairs_between(left_left, right_right, exact, pairs);
            collect_exact_pairs_between(left_right, right_left, exact, pairs);
            collect_exact_pairs_between(left_right, right_right, exact, pairs);
        }
    }
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
        AngularState3d, AngularVelocity3d, BodyId, BodyKind, Orientation3d, RigidBody, RigidBox3d,
        RigidBoxFreeFlightConfig3d, RotationalSweepBounds3d, Vec3i,
        rigid_box_free_flight_sweep_bounds,
    };

    use super::{
        BoundedBody3d, BroadPhaseBvhNode3d, BroadPhaseBvhNodeKind3d, RotatingBroadPhase3d,
        RotatingBroadPhaseError3d, RotationalSweepPair3d, bounds_overlap, build_balanced_bvh,
        rotational_sweep_candidate_pairs,
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

    fn brute_force_candidate_pairs(
        boxes: &[RigidBox3d],
        config: RigidBoxFreeFlightConfig3d,
    ) -> Vec<RotationalSweepPair3d> {
        let bounded = boxes
            .iter()
            .map(|rigid_box| BoundedBody3d {
                id: rigid_box.body().id(),
                kind: rigid_box.body().kind(),
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

    fn bvh_height(node: &BroadPhaseBvhNode3d) -> usize {
        match &node.kind {
            BroadPhaseBvhNodeKind3d::Leaf(_) => 1,
            BroadPhaseBvhNodeKind3d::Branch { left, right } => {
                1 + bvh_height(left).max(bvh_height(right))
            }
        }
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
    fn balanced_bvh_matches_brute_force_and_input_permutation() {
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
                    let x_velocity = if id % 2 == 0 { 4 } else { -3 };
                    dynamic(
                        id + 1,
                        position,
                        Vec3i::new(
                            x_velocity,
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
    }

    #[test]
    fn escaping_a_fat_leaf_rebuilds_deterministically() {
        let config = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1);
        let initial = [
            dynamic(1, Vec3i::ZERO, Vec3i::ZERO),
            fixed(2, Vec3i::new(10, 0, 0)),
        ];
        let escaped = [
            dynamic(1, Vec3i::new(1_000, 0, 0), Vec3i::ZERO),
            fixed(2, Vec3i::new(10, 0, 0)),
        ];
        let mut broad_phase = RotatingBroadPhase3d::default();

        broad_phase
            .candidate_pairs(&initial, config)
            .expect("initial persistent query");
        broad_phase
            .candidate_pairs(&escaped, config)
            .expect("escaped persistent query");

        assert_eq!(broad_phase.stats().rebuilds, 2);
        assert_eq!(broad_phase.stats().reuses, 0);
    }

    #[test]
    fn monotonic_input_builds_a_balanced_tree() {
        let mut bodies = (0..127_u64)
            .map(|id| {
                let coordinate = i64::try_from(id).expect("small coordinate") * 4;
                BoundedBody3d {
                    id: BodyId(id + 1),
                    kind: BodyKind::Dynamic,
                    bounds: RotationalSweepBounds3d {
                        minimum: [coordinate, 0, 0],
                        maximum: [coordinate + 2, 2, 2],
                    },
                }
            })
            .collect::<Vec<_>>();
        let tree = build_balanced_bvh(&mut bodies).expect("non-empty BVH");

        assert!(bvh_height(&tree) <= 8);
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
            "broad-phase 256 bodies × {iterations}: rebuild={rebuild_elapsed:?}, persistent={persistent_elapsed:?}, stats={:?}",
            persistent.stats()
        );
        assert_eq!(persistent.stats().rebuilds, 1);
        assert_eq!(persistent.stats().reuses, u64::from(iterations - 1));
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
