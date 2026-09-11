use std::{cmp::Ordering, collections::BTreeMap};

use crate::{
    BodyId, ObbContactSeed3d, RigidBox3d, RotatingContactSearchConfig3d,
    RotatingContactSearchError3d, RotatingContactSearchHit3d, RotationalSweepPair3d,
    SampledContactTime3d, obb_contact_seed, rotational_sweep_candidate_pairs,
    sample_rigid_box_free_flight, sampled_rotating_contact_search,
};

/// Finds the globally earliest strictly-positive sampled rotating contact.
///
/// Pairs that start clear reuse the ordinary sampled rotating-contact search. Pairs that start in
/// contact are treated as existing constraints until the configured coarse grid first observes a clear
/// sample; only a later clear-to-contact transition is admitted as a new event. That transition is then
/// refined with the same deterministic bisection policy as ordinary sampled search.
///
/// This is the positive-progress search needed by repeated collision-event stepping: persistent resting
/// contacts cannot be rediscovered forever at time zero, while sampled clear-then-recontact transitions
/// remain visible. It is still sampled rotational collision handling, not analytic rotational CCD; a
/// separation/contact island that exists wholly between coarse samples can still be missed.
///
/// # Errors
///
/// Returns [`RotatingContactSearchError3d`] for invalid search resolution, broad-phase failure, missing
/// candidate bodies, free-flight sampling failure, or OBB contact-query failure.
pub fn sampled_new_rotating_contact_search(
    boxes: &[RigidBox3d],
    config: RotatingContactSearchConfig3d,
) -> Result<Option<RotatingContactSearchHit3d>, RotatingContactSearchError3d> {
    validate_resolution(config)?;
    let candidates = rotational_sweep_candidate_pairs(boxes, config.free_flight)?;
    let by_id = boxes
        .iter()
        .map(|rigid_box| (rigid_box.body.id, rigid_box))
        .collect::<BTreeMap<_, _>>();
    let mut earliest: Option<RotatingContactSearchHit3d> = None;

    for pair in candidates {
        let left =
            by_id
                .get(&pair.left)
                .ok_or(RotatingContactSearchError3d::MissingCandidateBody(
                    pair.left,
                ))?;
        let right =
            by_id
                .get(&pair.right)
                .ok_or(RotatingContactSearchError3d::MissingCandidateBody(
                    pair.right,
                ))?;
        let start_contact = obb_contact_seed(left.oriented_box(), right.oriented_box())?;
        let candidate = if start_contact.is_some() {
            sampled_recontact_after_start(left, right, pair, config)?
        } else {
            sampled_rotating_contact_search(&[(*left).clone(), (*right).clone()], config)?
                .filter(|hit| hit.time.numerator > 0)
        };

        if let Some(candidate) = candidate {
            let replace = earliest.is_none_or(|current| compare_hits(candidate, current).is_lt());
            if replace {
                earliest = Some(candidate);
            }
        }
    }

    Ok(earliest)
}

fn sampled_recontact_after_start(
    left: &RigidBox3d,
    right: &RigidBox3d,
    pair: RotationalSweepPair3d,
    config: RotatingContactSearchConfig3d,
) -> Result<Option<RotatingContactSearchHit3d>, RotatingContactSearchError3d> {
    let coarse_denominator = u32::from(config.sample_count);
    let mut last_clear = None;
    let mut bracket = None;

    for sample in 1..=config.sample_count {
        let numerator = u32::from(sample);
        match sampled_contact(left, right, config, numerator, coarse_denominator)? {
            Some(contact) => {
                if let Some(clear_numerator) = last_clear {
                    bracket = Some((clear_numerator, numerator, coarse_denominator, contact));
                    break;
                }
            }
            None => last_clear = Some(numerator),
        }
    }

    let Some((mut lower, mut upper, mut denominator, mut contact)) = bracket else {
        return Ok(None);
    };

    for _ in 0..config.refinement_steps {
        lower = lower
            .checked_mul(2)
            .ok_or(RotatingContactSearchError3d::ResolutionTooFine {
                sample_count: config.sample_count,
                refinement_steps: config.refinement_steps,
            })?;
        upper = upper
            .checked_mul(2)
            .ok_or(RotatingContactSearchError3d::ResolutionTooFine {
                sample_count: config.sample_count,
                refinement_steps: config.refinement_steps,
            })?;
        denominator =
            denominator
                .checked_mul(2)
                .ok_or(RotatingContactSearchError3d::ResolutionTooFine {
                    sample_count: config.sample_count,
                    refinement_steps: config.refinement_steps,
                })?;
        let midpoint = lower + (upper - lower) / 2;
        if let Some(midpoint_contact) = sampled_contact(left, right, config, midpoint, denominator)?
        {
            upper = midpoint;
            contact = midpoint_contact;
        } else {
            lower = midpoint;
        }
    }

    Ok(Some(RotatingContactSearchHit3d {
        time: canonical_time(upper, denominator),
        pair,
        contact,
    }))
}

fn sampled_contact(
    left: &RigidBox3d,
    right: &RigidBox3d,
    config: RotatingContactSearchConfig3d,
    numerator: u32,
    denominator: u32,
) -> Result<Option<ObbContactSeed3d>, RotatingContactSearchError3d> {
    let left = sample_rigid_box_free_flight(left, config.free_flight, numerator, denominator)?;
    let right = sample_rigid_box_free_flight(right, config.free_flight, numerator, denominator)?;
    Ok(obb_contact_seed(left.oriented_box(), right.oriented_box())?)
}

fn validate_resolution(
    config: RotatingContactSearchConfig3d,
) -> Result<(), RotatingContactSearchError3d> {
    if config.sample_count == 0 {
        return Err(RotatingContactSearchError3d::ZeroSampleCount);
    }
    let scale = 1_u64
        .checked_shl(u32::from(config.refinement_steps))
        .ok_or(RotatingContactSearchError3d::ResolutionTooFine {
            sample_count: config.sample_count,
            refinement_steps: config.refinement_steps,
        })?;
    let final_denominator = u64::from(config.sample_count)
        .checked_mul(scale)
        .ok_or(RotatingContactSearchError3d::ResolutionTooFine {
            sample_count: config.sample_count,
            refinement_steps: config.refinement_steps,
        })?;
    if final_denominator > u64::from(u32::MAX) {
        return Err(RotatingContactSearchError3d::ResolutionTooFine {
            sample_count: config.sample_count,
            refinement_steps: config.refinement_steps,
        });
    }
    Ok(())
}

fn compare_hits(left: RotatingContactSearchHit3d, right: RotatingContactSearchHit3d) -> Ordering {
    compare_times(left.time, right.time)
        .then_with(|| left.pair.left.cmp(&right.pair.left))
        .then_with(|| left.pair.right.cmp(&right.pair.right))
}

fn compare_times(left: SampledContactTime3d, right: SampledContactTime3d) -> Ordering {
    let left_scaled = u64::from(left.numerator) * u64::from(right.denominator);
    let right_scaled = u64::from(right.numerator) * u64::from(left.denominator);
    left_scaled.cmp(&right_scaled)
}

fn canonical_time(numerator: u32, denominator: u32) -> SampledContactTime3d {
    let divisor = gcd_u32(numerator, denominator);
    SampledContactTime3d {
        numerator: numerator / divisor,
        denominator: denominator / divisor,
    }
}

fn gcd_u32(mut left: u32, mut right: u32) -> u32 {
    while right != 0 {
        let remainder = left % right;
        left = right;
        right = remainder;
    }
    left.max(1)
}

#[cfg(test)]
mod tests {
    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d,
        RigidBoxFreeFlightConfig3d, RotatingContactSearchConfig3d, RotatingContactSearchError3d,
        Vec3i,
    };

    use super::sampled_new_rotating_contact_search;

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

    fn config(gravity: Vec3i) -> RotatingContactSearchConfig3d {
        RotatingContactSearchConfig3d::new(RigidBoxFreeFlightConfig3d::new(gravity, 1, 1), 4, 3)
    }

    #[test]
    fn persistent_start_contact_is_not_a_new_event() {
        let boxes = [
            dynamic(1, Vec3i::ZERO, Vec3i::ZERO),
            fixed(2, Vec3i::new(2, 0, 0)),
        ];

        assert_eq!(
            sampled_new_rotating_contact_search(&boxes, config(Vec3i::ZERO))
                .expect("valid positive-progress search"),
            None
        );
    }

    #[test]
    fn start_contact_can_clear_and_recontact_at_positive_sampled_time() {
        let boxes = [
            dynamic(1, Vec3i::ZERO, Vec3i::new(8, 0, 0)),
            fixed(2, Vec3i::new(-2, 0, 0)),
        ];
        let hit = sampled_new_rotating_contact_search(&boxes, config(Vec3i::new(-16, 0, 0)))
            .expect("valid positive-progress search")
            .expect("body should clear and return to the initial contact");

        assert!(hit.time.numerator > 0);
        assert!(hit.time.numerator <= hit.time.denominator);
        assert_eq!(hit.pair.left, BodyId(1));
        assert_eq!(hit.pair.right, BodyId(2));
    }

    #[test]
    fn pair_that_starts_clear_keeps_later_positive_impact() {
        let boxes = [
            dynamic(1, Vec3i::new(-10, 0, 0), Vec3i::new(20, 0, 0)),
            fixed(2, Vec3i::ZERO),
        ];
        let hit = sampled_new_rotating_contact_search(&boxes, config(Vec3i::ZERO))
            .expect("valid positive-progress search")
            .expect("moving body should reach the obstacle");

        assert!(hit.time.numerator > 0);
        assert!(hit.time.numerator < hit.time.denominator);
    }

    #[test]
    fn overflowing_refined_denominator_fails_before_pair_search() {
        let boxes = [
            dynamic(1, Vec3i::ZERO, Vec3i::ZERO),
            fixed(2, Vec3i::new(2, 0, 0)),
        ];
        let config = RotatingContactSearchConfig3d::new(
            RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1),
            32_768,
            17,
        );

        assert_eq!(
            sampled_new_rotating_contact_search(&boxes, config),
            Err(RotatingContactSearchError3d::ResolutionTooFine {
                sample_count: 32_768,
                refinement_steps: 17,
            })
        );
    }

    #[test]
    fn positive_progress_search_is_repeatable() {
        let boxes = [
            dynamic(1, Vec3i::new(-10, 0, 0), Vec3i::new(20, 0, 0)),
            fixed(2, Vec3i::ZERO),
        ];
        let search = || sampled_new_rotating_contact_search(&boxes, config(Vec3i::ZERO));

        assert_eq!(
            search().expect("first search"),
            search().expect("second search")
        );
    }
}
