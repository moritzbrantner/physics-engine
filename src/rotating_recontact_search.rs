use std::{
    cmp::Ordering,
    collections::{BTreeMap, BTreeSet},
};

use crate::{
    BodyId, ObbContactSeed3d, OrientedBox3d, RigidBox3d, RotatingContactSearchConfig3d,
    RotatingContactSearchError3d, RotatingContactSearchHit3d, RotationalSweepPair3d,
    SampledContactTime3d, sample_rigid_box_free_flight,
};
use crate::{
    oriented_box::{PreparedObb3d, obb_contact_seed_prepared},
    rotating_broad_phase::RotatingBroadPhase3d,
    rotating_contact_search::coarse_sample_limit,
};

const MAX_CACHED_COARSE_SAMPLES: usize = 4_096;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct ContactBracket3d {
    clear_numerator: u32,
    contact_numerator: u32,
    denominator: u32,
    contact: ObbContactSeed3d,
}

#[derive(Debug)]
struct CoarseSampleCache3d {
    samples_by_body: Vec<Vec<PreparedObb3d>>,
    latest_geometry: Vec<Option<PreparedObb3d>>,
    cached_entries: usize,
    denominator: u32,
    #[cfg(test)]
    preparations: usize,
}

impl CoarseSampleCache3d {
    fn new(body_count: usize, denominator: u32) -> Self {
        Self {
            samples_by_body: vec![Vec::new(); body_count],
            latest_geometry: vec![None; body_count],
            cached_entries: 0,
            denominator,
            #[cfg(test)]
            preparations: 0,
        }
    }

    fn prepare_geometry(&mut self, body_index: usize, shape: OrientedBox3d) -> PreparedObb3d {
        if let Some(prepared) = self.latest_geometry[body_index]
            && prepared.shape == shape
        {
            return prepared;
        }
        let prepared = PreparedObb3d::new(shape);
        self.latest_geometry[body_index] = Some(prepared);
        #[cfg(test)]
        {
            self.preparations += 1;
        }
        prepared
    }

    fn sample_geometry(
        &mut self,
        body_index: usize,
        rigid_box: &RigidBox3d,
        config: RotatingContactSearchConfig3d,
        numerator: u32,
    ) -> Result<PreparedObb3d, RotatingContactSearchError3d> {
        let sample_index = usize::try_from(numerator - 1).expect("coarse numerator fits usize");
        if let Some(sampled) = self.samples_by_body[body_index].get(sample_index).copied() {
            return Ok(sampled);
        }

        // Keep canonical sampling and its checked exact-time arithmetic authoritative, even for
        // fixed/unchanged geometry. Only geometry preparation is reused after the sample succeeds.
        let shape = sample_rigid_box_free_flight(
            rigid_box,
            config.free_flight,
            numerator,
            self.denominator,
        )?
        .oriented_box();
        let sampled = self.prepare_geometry(body_index, shape);
        if self.len() < MAX_CACHED_COARSE_SAMPLES
            && sample_index == self.samples_by_body[body_index].len()
        {
            self.samples_by_body[body_index].push(sampled);
            self.cached_entries += 1;
        }
        Ok(sampled)
    }

    fn len(&self) -> usize {
        self.cached_entries
    }
}

#[derive(Clone, Copy, Debug)]
struct CachedPairContact3d {
    left: OrientedBox3d,
    right: OrientedBox3d,
    contact: Option<ObbContactSeed3d>,
}

/// A single exact result cache for one pair traversal. Contact history is owned by the caller.
#[derive(Debug, Default)]
struct PairContactCache3d {
    previous: Option<CachedPairContact3d>,
    #[cfg(test)]
    evaluations: usize,
}

impl PairContactCache3d {
    fn contact(
        &mut self,
        left: &PreparedObb3d,
        right: &PreparedObb3d,
    ) -> Result<Option<ObbContactSeed3d>, RotatingContactSearchError3d> {
        if let Some(previous) = self.previous
            && previous.left == left.shape
            && previous.right == right.shape
        {
            return Ok(previous.contact);
        }
        let contact = obb_contact_seed_prepared(left, right)?;
        self.previous = Some(CachedPairContact3d {
            left: left.shape,
            right: right.shape,
            contact,
        });
        #[cfg(test)]
        {
            self.evaluations += 1;
        }
        Ok(contact)
    }
}

/// Finds the earliest strictly-positive sampled rotating contact after an observed clear sample.
///
/// This is the re-contact counterpart to [`crate::sampled_rotating_contact_search`]. A pair that is
/// already touching at the interval start is not reported again merely because it remains touching.
/// Such a pair becomes eligible only after the configured coarse grid observes it clear and then later
/// observes contact again. Pairs that start clear retain the ordinary clear-to-contact search behavior.
/// Coarse free-flight samples and prepared SAT geometry are cached by body index and coarse-grid numerator
/// under the search's fixed denominator, so candidate pairs that share a body reuse identical work without
/// an ordered-map lookup. Preparation is also reused when a body's full quantized shape is unchanged
/// between samples. The coarse cache is bounded; at capacity, uncached canonical sampling remains the
/// fallback. One additional preparation per input body and one exact pair-result slot avoid repeated work
/// for stationary geometry. These caches live only within this search, never across collision response or
/// world mutation. Refinement and every configured clear/contact observation remain unchanged. Once a best
/// hit exists, later pairs inspect only coarse samples that can still match or precede its exact rational
/// time, including the containing known-contact sample.
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
    let mut broad_phase = RotatingBroadPhase3d::default();
    sampled_rotating_recontact_search_with_broad_phase(boxes, config, &mut broad_phase)
}

pub(crate) fn sampled_rotating_recontact_search_with_broad_phase(
    boxes: &[RigidBox3d],
    config: RotatingContactSearchConfig3d,
    broad_phase: &mut RotatingBroadPhase3d,
) -> Result<Option<RotatingContactSearchHit3d>, RotatingContactSearchError3d> {
    let mut persistent_pairs = BTreeSet::new();
    sampled_rotating_recontact_search_with_persistent_pairs_and_broad_phase(
        boxes,
        config,
        &mut persistent_pairs,
        broad_phase,
    )
}

pub(crate) fn sampled_rotating_recontact_search_with_persistent_pairs_and_broad_phase(
    boxes: &[RigidBox3d],
    config: RotatingContactSearchConfig3d,
    persistent_pairs: &mut BTreeSet<RotationalSweepPair3d>,
    broad_phase: &mut RotatingBroadPhase3d,
) -> Result<Option<RotatingContactSearchHit3d>, RotatingContactSearchError3d> {
    validate_resolution(config)?;
    let pairs = broad_phase.candidate_pairs(boxes, config.free_flight)?;
    if pairs.is_empty() {
        return Ok(None);
    }

    let by_id = boxes
        .iter()
        .enumerate()
        .map(|(index, rigid_box)| (rigid_box.body().id(), index))
        .collect::<BTreeMap<BodyId, usize>>();
    let denominator = u32::from(config.sample_count);
    let mut coarse_samples = CoarseSampleCache3d::new(boxes.len(), denominator);
    let mut best = None;
    let mut observed_clears = Vec::new();
    for pair in pairs {
        let left_index = *by_id
            .get(&pair.left)
            .ok_or(RotatingContactSearchError3d::MissingCandidateBody(pair.left))?;
        let right_index = *by_id
            .get(&pair.right)
            .ok_or(RotatingContactSearchError3d::MissingCandidateBody(pair.right))?;
        let left = &boxes[left_index];
        let right = &boxes[right_index];
        let coarse_limit = best.map_or(denominator, |hit: RotatingContactSearchHit3d| {
            coarse_sample_limit(hit.time, config.sample_count)
        });
        let (hit, first_positive_clear) = search_pair(
            (left_index, right_index),
            left,
            right,
            pair,
            config,
            coarse_limit,
            persistent_pairs.contains(&pair),
            &mut coarse_samples,
        )?;
        if let Some(clear_time) = first_positive_clear {
            observed_clears.push((pair, clear_time));
        }
        let Some(hit) = hit else {
            continue;
        };
        if best.is_none_or(|current| compare_hits(hit, current) == Ordering::Less) {
            best = Some(hit);
        }
    }

    for (pair, clear_time) in observed_clears {
        if best.is_none_or(|hit| compare_times(clear_time, hit.time) != Ordering::Greater) {
            persistent_pairs.remove(&pair);
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
    body_indices: (usize, usize),
    left: &RigidBox3d,
    right: &RigidBox3d,
    pair: RotationalSweepPair3d,
    config: RotatingContactSearchConfig3d,
    coarse_numerator_limit: u32,
    historically_contacting: bool,
    coarse_samples: &mut CoarseSampleCache3d,
) -> Result<
    (
        Option<RotatingContactSearchHit3d>,
        Option<SampledContactTime3d>,
    ),
    RotatingContactSearchError3d,
> {
    let (left_index, right_index) = body_indices;
    let mut contacts = PairContactCache3d::default();
    let prepared_left = coarse_samples.prepare_geometry(left_index, left.oriented_box());
    let prepared_right = coarse_samples.prepare_geometry(right_index, right.oriented_box());
    let initially_contacting = historically_contacting
        || contacts.contact(&prepared_left, &prepared_right)?.is_some();
    let denominator = u32::from(config.sample_count);
    let mut last_clear = if initially_contacting { None } else { Some(0) };
    let mut first_positive_clear = None;

    for numerator in 1..=coarse_numerator_limit {
        let sampled_left = coarse_samples.sample_geometry(left_index, left, config, numerator)?;
        let sampled_right = coarse_samples.sample_geometry(right_index, right, config, numerator)?;
        match contacts.contact(&sampled_left, &sampled_right)? {
            Some(contact) => {
                let Some(clear_numerator) = last_clear else {
                    continue;
                };
                return Ok((
                    Some(refine_contact_bracket(
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
                    )?),
                    first_positive_clear,
                ));
            }
            None => {
                last_clear = Some(numerator);
                if historically_contacting && first_positive_clear.is_none() {
                    first_positive_clear = Some(canonical_time(numerator, denominator));
                }
            }
        }
    }
    Ok((None, first_positive_clear))
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

        let (sampled_left, sampled_right) =
            sample_pair(left, right, config, midpoint_numerator, bracket.denominator)?;
        if let Some(contact) =
            crate::obb_contact_seed(sampled_left.oriented_box(), sampled_right.oriented_box())?
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
    use std::collections::BTreeSet;

    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d,
        RigidBoxFreeFlightConfig3d, RotatingContactSearchConfig3d, RotatingContactSearchError3d,
        RotationalSweepPair3d, Vec3i, obb_contact_seed, sample_rigid_box_free_flight,
    };

    use super::{
        CoarseSampleCache3d, MAX_CACHED_COARSE_SAMPLES, sampled_rotating_recontact_search,
        sampled_rotating_recontact_search_with_persistent_pairs_and_broad_phase,
    };
    use crate::rotating_broad_phase::RotatingBroadPhase3d;

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
        let boxes = [
            dynamic(1, Vec3i::new(-2, 0, 0), Vec3i::ZERO),
            fixed(2, Vec3i::ZERO),
        ];

        assert_eq!(
            sampled_rotating_recontact_search(&boxes, config(Vec3i::ZERO, 8, 3))
                .expect("valid persistent-contact search"),
            None
        );
    }

    #[test]
    fn historical_contact_does_not_treat_interval_start_separation_as_clear() {
        let moving = dynamic(1, Vec3i::new(-3, 0, 0), Vec3i::new(4, 0, 0));
        let obstacle = fixed(2, Vec3i::ZERO);
        let pair = RotationalSweepPair3d {
            left: BodyId(1),
            right: BodyId(2),
        };
        let mut persistent_pairs = BTreeSet::from([pair]);
        let mut broad_phase = RotatingBroadPhase3d::default();

        assert_eq!(
            sampled_rotating_recontact_search_with_persistent_pairs_and_broad_phase(
                &[moving, obstacle],
                config(Vec3i::ZERO, 4, 0),
                &mut persistent_pairs,
                &mut broad_phase,
            )
            .expect("valid history-aware recontact search"),
            None
        );
        assert!(persistent_pairs.contains(&pair));
    }

    #[test]
    fn historical_contact_recontacts_after_positive_clear_sample() {
        let moving = dynamic(1, Vec3i::new(-3, 0, 0), Vec3i::new(1, 0, 0));
        let obstacle = fixed(2, Vec3i::ZERO);
        let pair = RotationalSweepPair3d {
            left: BodyId(1),
            right: BodyId(2),
        };
        let mut persistent_pairs = BTreeSet::from([pair]);
        let mut broad_phase = RotatingBroadPhase3d::default();
        let hit = sampled_rotating_recontact_search_with_persistent_pairs_and_broad_phase(
            &[moving, obstacle],
            config(Vec3i::ZERO, 4, 0),
            &mut persistent_pairs,
            &mut broad_phase,
        )
        .expect("valid history-aware recontact search")
        .expect("positive clear sample should admit later recontact");

        assert_eq!(hit.time.numerator, 1);
        assert_eq!(hit.time.denominator, 2);
        assert!(!persistent_pairs.contains(&pair));
    }

    #[test]
    fn positive_clear_before_unrelated_frontier_releases_contact_history() {
        let historical = RotationalSweepPair3d {
            left: BodyId(1),
            right: BodyId(2),
        };
        let selector = RotationalSweepPair3d {
            left: BodyId(3),
            right: BodyId(4),
        };
        let boxes = [
            dynamic(1, Vec3i::new(-5, 0, 0), Vec3i::new(8, 0, 0)),
            fixed(2, Vec3i::ZERO),
            dynamic(3, Vec3i::new(-4, 10, 0), Vec3i::new(8, 0, 0)),
            fixed(4, Vec3i::new(0, 10, 0)),
        ];
        let mut persistent_pairs = BTreeSet::from([historical]);
        let mut broad_phase = RotatingBroadPhase3d::default();
        let hit = sampled_rotating_recontact_search_with_persistent_pairs_and_broad_phase(
            &boxes,
            config(Vec3i::ZERO, 8, 0),
            &mut persistent_pairs,
            &mut broad_phase,
        )
        .expect("valid history-aware recontact search")
        .expect("unrelated pair should select the earlier frontier");

        assert_eq!(hit.pair, selector);
        assert_eq!(
            hit.time,
            crate::SampledContactTime3d {
                numerator: 1,
                denominator: 4,
            }
        );
        assert!(!persistent_pairs.contains(&historical));
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
        let hit = sampled_rotating_recontact_search(&[moving, obstacle], config(Vec3i::ZERO, 4, 3))
            .expect("valid initial-clear search")
            .expect("first contact should be found");

        assert!(hit.time.numerator > 0);
    }

    #[test]
    fn shared_body_pair_ties_keep_stable_pair_order() {
        let moving = dynamic(11, Vec3i::new(-10, 0, 0), Vec3i::new(20, 0, 0));
        let first = fixed(3, Vec3i::ZERO);
        let second = fixed(8, Vec3i::ZERO);
        let hit =
            sampled_rotating_recontact_search(&[second, moving, first], config(Vec3i::ZERO, 8, 2))
                .expect("valid shared-body recontact search")
                .expect("shared body should contact both obstacles");

        assert_eq!(hit.pair.left, BodyId(3));
        assert_eq!(hit.pair.right, BodyId(11));
    }

    #[test]
    fn coarse_sample_cache_is_bounded_and_reuses_geometry() {
        let rigid_box = fixed(1, Vec3i::ZERO);
        let search = config(Vec3i::ZERO, 32, 0);
        let mut cache = CoarseSampleCache3d::new(1, 32);

        let first = cache
            .sample_geometry(0, &rigid_box, search, 1)
            .expect("first coarse sample");
        let repeated = cache
            .sample_geometry(0, &rigid_box, search, 1)
            .expect("repeated coarse sample");
        assert_eq!(first, repeated);
        assert_eq!(cache.len(), 1);

        let denominator = u32::try_from(MAX_CACHED_COARSE_SAMPLES + 2).expect("small cache bound");
        let mut bounded_cache = CoarseSampleCache3d::new(1, denominator);
        for numerator in 1..=denominator {
            bounded_cache
                .sample_geometry(0, &rigid_box, search, numerator)
                .expect("bounded coarse sample");
        }
        assert_eq!(bounded_cache.len(), MAX_CACHED_COARSE_SAMPLES);
        assert!(bounded_cache.latest_geometry[0].is_some());
        assert_eq!(bounded_cache.preparations, 1);
    }

    #[test]
    fn result_is_stable_across_input_order() {
        let moving = dynamic(11, Vec3i::new(-10, 0, 0), Vec3i::new(20, 0, 0));
        let obstacle = fixed(3, Vec3i::ZERO);
        let search = config(Vec3i::ZERO, 8, 2);
        let forward = sampled_rotating_recontact_search(
            &[moving.clone(), obstacle.clone()],
            search,
        )
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
        let boxes = [
            dynamic(1, Vec3i::ZERO, Vec3i::ZERO),
            fixed(2, Vec3i::new(2, 0, 0)),
        ];
        assert_eq!(
            sampled_rotating_recontact_search(&boxes, config(Vec3i::ZERO, 0, 0)),
            Err(RotatingContactSearchError3d::ZeroSampleCount)
        );
    }
}

#[cfg(test)]
#[path = "rotating_recontact_prepared_tests.rs"]
mod prepared_tests;
