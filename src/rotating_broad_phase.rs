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

/// Produces a deterministic conservative candidate set for rotating-box contact search.
///
/// Every body is first enclosed by [`rigid_box_free_flight_sweep_bounds`], so acceleration, velocity
/// reversal, and arbitrary orientation change are preserved as broad-phase possibilities. The sweep is
/// ordered by minimum X and stable [`BodyId`], with Y/Z interval rejection before emitting a pair.
/// Fixed/fixed pairs are omitted because neither body can respond. Input ordering does not affect output.
///
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

    bounded.sort_by_key(|body| (body.bounds.minimum[0], body.id));
    let mut pairs = Vec::new();
    for left_index in 0..bounded.len() {
        let left = bounded[left_index];
        for right in bounded.iter().copied().skip(left_index + 1) {
            if right.bounds.minimum[0] > left.bounds.maximum[0] {
                break;
            }
            if left.kind == BodyKind::Fixed && right.kind == BodyKind::Fixed {
                continue;
            }
            if !overlaps_on_axis(left.bounds, right.bounds, 1)
                || !overlaps_on_axis(left.bounds, right.bounds, 2)
            {
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
    Ok(pairs)
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
        AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d,
        RigidBoxFreeFlightConfig3d, Vec3i,
    };

    use super::{
        RotatingBroadPhaseError3d, RotationalSweepPair3d, rotational_sweep_candidate_pairs,
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
                &[fixed(3, Vec3i::ZERO), fixed(4, Vec3i::new(1, 0, 0)),],
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
