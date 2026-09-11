use std::{cmp::Ordering, collections::BTreeMap};

use crate::{
    BodyId, ObbContactSeed3d, RigidBox3d, RotatingContactSearchConfig3d,
    RotatingContactSearchError3d, RotatingContactSearchHit3d, RotationalSweepPair3d,
    SampledContactTime3d, obb_contact_seed, rotational_sweep_candidate_pairs,
    sample_rigid_box_free_flight,
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct ContactBracket3d {
    clear_numerator: u32,
    contact_numerator: u32,
    denominator: u32,
    contact: ObbContactSeed3d,
}

/// Finds the earliest strictly-positive sampled rotating contact after an observed clear sample.
///
/// This is the re-contact counterpart to [`crate::sampled_rotating_contact_search`]. A pair that is
/// already touching at the interval start is not reported again merely because it remains touching.
/// Such a pair becomes eligible only after the configured coarse grid observes it clear and then later
/// observes contact again. Pairs that start clear retain the ordinary clear-to-contact search behavior.
///
/// This explicit state transition is required by repeated-event stepping: a resolved/resting time-zero
/// contact must not hide a later impact or be rediscovered forever as the next event. As with the existing
/// rotating search, this remains a bounded **sampled approximation**, not analytic rotational CCD. A clear
/// interval or contact island that exists entirely between adjacent coarse samples can still be missed.
/// Refinement only sharpens an already observed clear/contact bracket.
///
/// # Errors
///
/// Returns [`RotatingContactSearchError3d`] for invalid search resolution, duplicate/missing body
/// identity, malformed free-flight state, broad-phase failure, or OBB contact arithmetic failure.
pub fn sampled_rotating_recontact_search(
    boxes: &[RigidBox3d],
    config: RotatingContactSearchConfig3d,
) -> Result<Option<RotatingContactSearchHit3d>, RotatingContactSearchError3d> {
    validate_resolution(config)?;
    let pairs = rotational_sweep_candidate_pairs(boxes, config.free_flight)?;
    if pairs.is_empty() {
        return Ok(None);
    }

    let by_id = boxes
        .iter()
        .map(|rigid_box| (rigid_box.body().id(), rigid_box))
        .collect::<BTreeMap<BodyId, &RigidBox3d>>();
    let mut best = None;
    for pair in pairs {
        let left = by_id.get(&pair.left).copied().ok_or(
            RotatingContactSearchError3d::MissingCandidateBody(pair.left),
        )?;
        let right = by_id.get(&pair.right).copied().ok_or(
            RotatingContactSearchError3d::MissingCandidateBody(pair.right),
        )?;
        let Some(hit) = search_pair(left, right, pair, config)? else {
            continue;
        };
        if best.is_none_or(|current| compare_hits(hit, current) == Ordering::Less) {
            best = Some(hit);
        }
    }
    Ok(best)
}

fn validate_resolution(
    config: RotatingContactSearchConfig3d,
) -> Result<(), RotatingContactSearchError3d> {
    if config.sample_count == 0 {
        return Err(RotatingContactSearchError3d::ZeroSampleCount);
    }
    let final_denominator = u64::from(config.sample_count)
        .checked_shl(u32::from(config.refinement_steps))
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

fn search_pair(
    left: &RigidBox3d,
    right: &RigidBox3d,
    pair: RotationalSweepPair3d,
    config: RotatingContactSearchConfig3d,
) -> Result<Option<RotatingContactSearchHit3d>, RotatingContactSearchError3d> {
    let initially_contacting = obb_contact_seed(left.oriented_box(), right.oriented_box())?.is_some();
    let denominator = u32::from(config.sample_count);
    let mut last_clear = if initially_contacting { None } else { Some(0) };

    for numerator in 1..=denominator {
        let (sampled_left, sampled_right) =
            sample_pair(left, right, config, numerator, denominator)?;
        match obb_contact_seed(sampled_left.oriented_box(), sampled_right.oriented_box())? {
            Some(contact) => {
                let Some(clear_numerator) = last_clear else {
                    continue;
                };
                return Ok(Some(refine_contact_bracket(
                    left,
                    right,
                    pair,
                    config,
                    ContactBracket3d {
                        clear_numerator,
                        contact_numerator: numerator,
                        denominator,
                        contact,
                    },
                )?));
            }
            None => last_clear = Some(numerator),
        }
    }
    Ok(None)
}

fn refine_contact_bracket(
    left: &RigidBox3d,
    right: &RigidBox3d,
    pair: RotationalSweepPair3d,
    config: RotatingContactSearchConfig3d,
    mut bracket: ContactBracket3d,
) -> Result<RotatingContactSearchHit3d, RotatingContactSearchError3d> {
    for _ in 0..config.refinement_steps {
        let midpoint_numerator = bracket
            .clear_numerator
            .checked_add(bracket.contact_numerator)
            .ok_or(resolution_too_fine(config))?;
        bracket.denominator = bracket
            .denominator
            .checked_mul(2)
            .ok_or(resolution_too_fine(config))?;
        bracket.clear_numerator = bracket
            .clear_numerator
            .checked_mul(2)
            .ok_or(resolution_too_fine(config))?;
        bracket.contact_numerator = bracket
            .contact_numerator
            .checked_mul(2)
            .ok_or(resolution_too_fine(config))?;

        let (sampled_left, sampled_right) = sample_pair(
            left,
            right,
            config,
            midpoint_numerator,
            bracket.denominator,
        )?;
        if let Some(contact) =
            obb_contact_seed(sampled_left.oriented_box(), sampled_right.oriented_box())?
        {
            bracket.contact_numerator = midpoint_numerator;
            bracket.contact = contact;
        } else {
            bracket.clear_numerator = midpoint_numerator;
        }
    }

    Ok(RotatingContactSearchHit3d {
        time: canonical_time(bracket.contact_numerator, bracket.denominator),
        pair,
        contact: bracket.contact,
    })
}

fn sample_pair(
    left: &RigidBox3d,
    right: &RigidBox3d,
    config: RotatingContactSearchConfig3d,
    numerator: u32,
    denominator: u32,
) -> Result<(RigidBox3d, RigidBox3d), RotatingContactSearchError3d> {
    Ok((
        sample_rigid_box_free_flight(left, config.free_flight, numerator, denominator)?,
        sample_rigid_box_free_flight(right, config.free_flight, numerator, denominator)?,
    ))
}

fn resolution_too_fine(config: RotatingContactSearchConfig3d) -> RotatingContactSearchError3d {
    RotatingContactSearchError3d::ResolutionTooFine {
        sample_count: config.sample_count,
        refinement_steps: config.refinement_steps,
    }
}

fn canonical_time(numerator: u32, denominator: u32) -> SampledContactTime3d {
    let divisor = greatest_common_divisor(u64::from(numerator), u64::from(denominator));
    let divisor = u32::try_from(divisor).expect("gcd of u32 values fits u32");
    SampledContactTime3d {
        numerator: numerator / divisor,
        denominator: denominator / divisor,
    }
}

fn compare_hits(left: RotatingContactSearchHit3d, right: RotatingContactSearchHit3d) -> Ordering {
    compare_times(left.time, right.time).then_with(|| left.pair.cmp(&right.pair))
}

fn compare_times(left: SampledContactTime3d, right: SampledContactTime3d) -> Ordering {
    let left_scaled = u64::from(left.numerator) * u64::from(right.denominator);
    let right_scaled = u64::from(right.numerator) * u64::from(left.denominator);
    left_scaled.cmp(&right_scaled)
}

fn greatest_common_divisor(mut left: u64, mut right: u64) -> u64 {
    while right != 0 {
        let remainder = left % right;
        left = right;
        right = remainder;
    }
    left
}

#[cfg(test)]
mod tests {
    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d,
        RigidBoxFreeFlightConfig3d, RotatingContactSearchConfig3d, RotatingContactSearchError3d,
        Vec3i, obb_contact_seed, sample_rigid_box_free_flight,
    };

    use super::sampled_rotating_recontact_search;

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

    fn config(
        gravity: Vec3i,
        sample_count: u16,
        refinement_steps: u8,
    ) -> RotatingContactSearchConfig3d {
        RotatingContactSearchConfig3d::new(
            RigidBoxFreeFlightConfig3d::new(gravity, 1, 1),
            sample_count,
            refinement_steps,
        )
    }

    #[test]
    fn persistent_time_zero_contact_is_not_rediscovered() {
        let boxes = [dynamic(1, Vec3i::new(-2, 0, 0), Vec3i::ZERO), fixed(2, Vec3i::ZERO)];

        assert_eq!(
            sampled_rotating_recontact_search(&boxes, config(Vec3i::ZERO, 8, 3))
                .expect("valid persistent-contact search"),
            None
        );
    }

    #[test]
    fn touching_pair_must_clear_before_later_recontact_is_admitted() {
        let moving = dynamic(1, Vec3i::new(-2, 0, 0), Vec3i::new(-8, 0, 0));
        let obstacle = fixed(2, Vec3i::ZERO);
        let search = config(Vec3i::new(16, 0, 0), 8, 3);
        let hit = sampled_rotating_recontact_search(&[moving.clone(), obstacle.clone()], search)
            .expect("valid recontact search")
            .expect("clear then recontact should be found");

        assert!(hit.time.numerator > 0);
        let sampled = sample_rigid_box_free_flight(
            &moving,
            search.free_flight,
            hit.time.numerator,
            hit.time.denominator,
        )
        .expect("valid returned-time sample");
        assert!(
            obb_contact_seed(sampled.oriented_box(), obstacle.oriented_box())
                .expect("valid returned OBBs")
                .is_some()
        );
    }

    #[test]
    fn pair_that_starts_clear_keeps_first_contact_behavior() {
        let moving = dynamic(1, Vec3i::new(-10, 0, 0), Vec3i::new(20, 0, 0));
        let obstacle = fixed(2, Vec3i::ZERO);
        let hit = sampled_rotating_recontact_search(
            &[moving, obstacle],
            config(Vec3i::ZERO, 4, 3),
        )
        .expect("valid initial-clear search")
        .expect("first contact should be found");

        assert!(hit.time.numerator > 0);
    }

    #[test]
    fn result_is_stable_across_input_order() {
        let moving = dynamic(11, Vec3i::new(-10, 0, 0), Vec3i::new(20, 0, 0));
        let obstacle = fixed(3, Vec3i::ZERO);
        let search = config(Vec3i::ZERO, 8, 2);
        let forward = sampled_rotating_recontact_search(&[moving.clone(), obstacle.clone()], search)
            .expect("valid forward search");
        let reverse = sampled_rotating_recontact_search(&[obstacle, moving], search)
            .expect("valid reverse search");

        assert_eq!(forward, reverse);
    }

    #[test]
    fn duplicate_ids_fail_closed_through_the_shared_broad_phase() {
        let boxes = [
            dynamic(7, Vec3i::ZERO, Vec3i::ZERO),
            dynamic(7, Vec3i::new(10, 0, 0), Vec3i::ZERO),
        ];
        assert!(matches!(
            sampled_rotating_recontact_search(&boxes, config(Vec3i::ZERO, 4, 2)),
            Err(RotatingContactSearchError3d::BroadPhase(_))
        ));
    }

    #[test]
    fn invalid_resolution_matches_the_existing_search_contract() {
        let boxes = [dynamic(1, Vec3i::ZERO, Vec3i::ZERO), fixed(2, Vec3i::new(2, 0, 0))];
        assert_eq!(
            sampled_rotating_recontact_search(&boxes, config(Vec3i::ZERO, 0, 0)),
            Err(RotatingContactSearchError3d::ZeroSampleCount)
        );
    }
}