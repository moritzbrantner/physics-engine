use std::{collections::BTreeSet, error::Error, fmt};

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

#[derive(Debug)]
struct BroadPhaseBvhNode3d {
    bounds: RotationalSweepBounds3d,
    has_dynamic: bool,
    kind: BroadPhaseBvhNodeKind3d,
}

#[derive(Debug)]
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

/// Produces a deterministic conservative candidate set for rotating-box contact search.
///
/// Every body is first enclosed by [`rigid_box_free_flight_sweep_bounds`], so acceleration, velocity
/// reversal, and arbitrary orientation change are preserved as broad-phase possibilities. Those
/// conservative envelopes are arranged into a deterministic balanced AABB BVH. Each branch splits on
/// its widest aggregate axis at the median body center, with stable [`BodyId`] tie breaking. Branches
/// containing only fixed bodies are pruned when paired with another fixed-only branch.
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

    let Some(tree) = build_balanced_bvh(&mut bounded) else {
        return Ok(Vec::new());
    };

    let mut pairs = Vec::new();
    collect_pairs_within(&tree, &mut pairs);
    pairs.sort_unstable();
    Ok(pairs)
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

fn collect_pairs_within(node: &BroadPhaseBvhNode3d, pairs: &mut Vec<RotationalSweepPair3d>) {
    let BroadPhaseBvhNodeKind3d::Branch { left, right } = &node.kind else {
        return;
    };

    collect_pairs_within(left, pairs);
    collect_pairs_within(right, pairs);
    collect_pairs_between(left, right, pairs);
}

fn collect_pairs_between(
    left: &BroadPhaseBvhNode3d,
    right: &BroadPhaseBvhNode3d,
    pairs: &mut Vec<RotationalSweepPair3d>,
) {
    if (!left.has_dynamic && !right.has_dynamic) || !bounds_overlap(left.bounds, right.bounds) {
        return;
    }

    match (&left.kind, &right.kind) {
        (BroadPhaseBvhNodeKind3d::Leaf(left_body), BroadPhaseBvhNodeKind3d::Leaf(right_body)) => {
            if left_body.kind == BodyKind::Fixed && right_body.kind == BodyKind::Fixed {
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
            collect_pairs_between(left_left, right, pairs);
            collect_pairs_between(left_right, right, pairs);
        }
        (
            BroadPhaseBvhNodeKind3d::Leaf(_),
            BroadPhaseBvhNodeKind3d::Branch {
                left: right_left,
                right: right_right,
            },
        ) => {
            collect_pairs_between(left, right_left, pairs);
            collect_pairs_between(left, right_right, pairs);
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
            collect_pairs_between(left_left, right_left, pairs);
            collect_pairs_between(left_left, right_right, pairs);
            collect_pairs_between(left_right, right_left, pairs);
            collect_pairs_between(left_right, right_right, pairs);
        }
    }
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
    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, BodyKind, Orientation3d, RigidBody, RigidBox3d,
        RigidBoxFreeFlightConfig3d, RotationalSweepBounds3d, Vec3i,
        rigid_box_free_flight_sweep_bounds,
    };

    use super::{
        BoundedBody3d, BroadPhaseBvhNode3d, BroadPhaseBvhNodeKind3d, RotatingBroadPhaseError3d,
        RotationalSweepPair3d, bounds_overlap, build_balanced_bvh,
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
