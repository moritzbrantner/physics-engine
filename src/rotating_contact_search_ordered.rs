use std::{cmp::Ordering, collections::BTreeMap};

use crate::rotating_broad_phase::RotatingBroadPhase3d;
use crate::{
    BodyId, ObbContactSeed3d, OrientedBox3d, RigidBox3d, RigidBoxFreeFlightConfig3d,
    RotatingContactSearchConfig3d, RotatingContactSearchError3d, RotatingContactSearchHit3d,
    RotationalSweepPair3d, SampledContactTime3d, obb_contact_seed, sample_rigid_box_free_flight,
};

const MAX_CACHED_COARSE_SAMPLES: usize = 4_096;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub(crate) struct ContactSearchWork3d {
    pub candidate_pairs: u64,
    pub initial_pair_evaluations: u64,
    pub coarse_rows: u64,
    pub coarse_pair_evaluations: u64,
    pub exact_contact_evaluations: u64,
    pub refinement_evaluations: u64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct ContactBracket3d {
    lower_numerator: u32,
    upper_numerator: u32,
    denominator: u32,
    upper_contact: ObbContactSeed3d,
}

#[derive(Debug)]
struct CoarseSampleCache3d {
    samples_by_body: Vec<Vec<OrientedBox3d>>,
    cached_entries: usize,
    denominator: u32,
}

impl CoarseSampleCache3d {
    fn new(body_count: usize, denominator: u32) -> Self {
        Self {
            samples_by_body: vec![Vec::new(); body_count],
            cached_entries: 0,
            denominator,
        }
    }

    fn sample_oriented_box(
        &mut self,
        body_index: usize,
        rigid_box: &RigidBox3d,
        config: RigidBoxFreeFlightConfig3d,
        numerator: u32,
    ) -> Result<OrientedBox3d, RotatingContactSearchError3d> {
        let sample_index = usize::try_from(numerator - 1).expect("coarse numerator fits usize");
        if let Some(sampled) = self.samples_by_body[body_index].get(sample_index).copied() {
            return Ok(sampled);
        }

        let sampled = sample_rigid_box_free_flight(rigid_box, config, numerator, self.denominator)?
            .oriented_box();
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
struct PairState3d {
    pair: RotationalSweepPair3d,
    left_index: usize,
    right_index: usize,
}

/// Searches candidate rotating-box pairs in global coarse-time order and refines the earliest observed
/// contact row.
///
/// The previous implementation traverses one pair at a time and bounds later pairs after finding a best
/// hit. This implementation preserves the same conservative broad-phase candidates, fixed coarse grid,
/// binary refinement, exact OBB contact query, and stable `(time, pair)` tie ordering, but evaluates every
/// candidate at coarse sample 1 before sample 2, and so on. If a complete coarse row contains contact, no
/// later row can refine to an earlier sampled time because every candidate was observed clear on each
/// preceding row. All contacts in that first hit row are refined before the stable minimum is selected.
///
/// No sampled state survives this immutable search. This is still sampled rotational collision handling,
/// not analytic CCD, and all checked arithmetic remains fail-closed.
pub fn sampled_rotating_contact_search(
    boxes: &[RigidBox3d],
    config: RotatingContactSearchConfig3d,
) -> Result<Option<RotatingContactSearchHit3d>, RotatingContactSearchError3d> {
    let mut broad_phase = RotatingBroadPhase3d::default();
    sampled_rotating_contact_search_with_broad_phase(boxes, config, &mut broad_phase)
}

pub(crate) fn sampled_rotating_contact_search_with_broad_phase(
    boxes: &[RigidBox3d],
    config: RotatingContactSearchConfig3d,
    broad_phase: &mut RotatingBroadPhase3d,
) -> Result<Option<RotatingContactSearchHit3d>, RotatingContactSearchError3d> {
    sampled_rotating_contact_search_profiled(boxes, config, broad_phase).map(|(hit, _)| hit)
}

pub(crate) fn sampled_rotating_contact_search_profiled(
    boxes: &[RigidBox3d],
    config: RotatingContactSearchConfig3d,
    broad_phase: &mut RotatingBroadPhase3d,
) -> Result<(Option<RotatingContactSearchHit3d>, ContactSearchWork3d), RotatingContactSearchError3d>
{
    validate_resolution(config)?;
    let pairs = broad_phase.candidate_pairs(boxes, config.free_flight)?;
    let mut work = ContactSearchWork3d {
        candidate_pairs: u64::try_from(pairs.len()).unwrap_or(u64::MAX),
        ..ContactSearchWork3d::default()
    };
    if pairs.is_empty() {
        return Ok((None, work));
    }

    let by_id = boxes
        .iter()
        .enumerate()
        .map(|(index, rigid_box)| (rigid_box.body().id(), index))
        .collect::<BTreeMap<BodyId, usize>>();
    let mut states = Vec::with_capacity(pairs.len());
    let mut initial_best: Option<RotatingContactSearchHit3d> = None;
    for pair in pairs {
        let left_index =
            *by_id
                .get(&pair.left)
                .ok_or(RotatingContactSearchError3d::MissingCandidateBody(
                    pair.left,
                ))?;
        let right_index =
            *by_id
                .get(&pair.right)
                .ok_or(RotatingContactSearchError3d::MissingCandidateBody(
                    pair.right,
                ))?;
        work.initial_pair_evaluations = work.initial_pair_evaluations.saturating_add(1);
        work.exact_contact_evaluations = work.exact_contact_evaluations.saturating_add(1);
        if let Some(contact) = obb_contact_seed(
            boxes[left_index].oriented_box(),
            boxes[right_index].oriented_box(),
        )? {
            let hit = RotatingContactSearchHit3d {
                time: SampledContactTime3d::ZERO,
                pair,
                contact,
            };
            if initial_best.is_none_or(|current| compare_hits(hit, current) == Ordering::Less) {
                initial_best = Some(hit);
            }
        }
        states.push(PairState3d {
            pair,
            left_index,
            right_index,
        });
    }
    if initial_best.is_some() {
        return Ok((initial_best, work));
    }

    let denominator = u32::from(config.sample_count);
    let mut coarse_samples = CoarseSampleCache3d::new(boxes.len(), denominator);
    for numerator in 1..=denominator {
        work.coarse_rows = work.coarse_rows.saturating_add(1);
        let mut row_best: Option<RotatingContactSearchHit3d> = None;
        for state in &states {
            work.coarse_pair_evaluations = work.coarse_pair_evaluations.saturating_add(1);
            let sampled_left = coarse_samples.sample_oriented_box(
                state.left_index,
                &boxes[state.left_index],
                config.free_flight,
                numerator,
            )?;
            let sampled_right = coarse_samples.sample_oriented_box(
                state.right_index,
                &boxes[state.right_index],
                config.free_flight,
                numerator,
            )?;
            work.exact_contact_evaluations = work.exact_contact_evaluations.saturating_add(1);
            let Some(contact) = obb_contact_seed(sampled_left, sampled_right)? else {
                continue;
            };
            let hit = refine_contact_bracket(
                &boxes[state.left_index],
                &boxes[state.right_index],
                state.pair,
                config,
                ContactBracket3d {
                    lower_numerator: numerator - 1,
                    upper_numerator: numerator,
                    denominator,
                    upper_contact: contact,
                },
                &mut work,
            )?;
            if row_best.is_none_or(|current| compare_hits(hit, current) == Ordering::Less) {
                row_best = Some(hit);
            }
        }
        if row_best.is_some() {
            return Ok((row_best, work));
        }
    }

    Ok((None, work))
}

fn validate_resolution(
    config: RotatingContactSearchConfig3d,
) -> Result<(), RotatingContactSearchError3d> {
    if config.sample_count == 0 {
        return Err(RotatingContactSearchError3d::ZeroSampleCount);
    }
    let final_denominator = u64::from(config.sample_count)
        .checked_shl(u32::from(config.refinement_steps))
        .ok_or_else(|| resolution_too_fine(config))?;
    if final_denominator > u64::from(u32::MAX) {
        return Err(resolution_too_fine(config));
    }
    Ok(())
}

fn refine_contact_bracket(
    left: &RigidBox3d,
    right: &RigidBox3d,
    pair: RotationalSweepPair3d,
    config: RotatingContactSearchConfig3d,
    mut bracket: ContactBracket3d,
    work: &mut ContactSearchWork3d,
) -> Result<RotatingContactSearchHit3d, RotatingContactSearchError3d> {
    for _ in 0..config.refinement_steps {
        let midpoint_numerator = bracket
            .lower_numerator
            .checked_add(bracket.upper_numerator)
            .ok_or_else(|| resolution_too_fine(config))?;
        bracket.denominator = bracket
            .denominator
            .checked_mul(2)
            .ok_or_else(|| resolution_too_fine(config))?;
        bracket.lower_numerator = bracket
            .lower_numerator
            .checked_mul(2)
            .ok_or_else(|| resolution_too_fine(config))?;
        bracket.upper_numerator = bracket
            .upper_numerator
            .checked_mul(2)
            .ok_or_else(|| resolution_too_fine(config))?;

        let sampled_left = sample_rigid_box_free_flight(
            left,
            config.free_flight,
            midpoint_numerator,
            bracket.denominator,
        )?;
        let sampled_right = sample_rigid_box_free_flight(
            right,
            config.free_flight,
            midpoint_numerator,
            bracket.denominator,
        )?;
        work.refinement_evaluations = work.refinement_evaluations.saturating_add(1);
        work.exact_contact_evaluations = work.exact_contact_evaluations.saturating_add(1);
        if let Some(contact) =
            obb_contact_seed(sampled_left.oriented_box(), sampled_right.oriented_box())?
        {
            bracket.upper_numerator = midpoint_numerator;
            bracket.upper_contact = contact;
        } else {
            bracket.lower_numerator = midpoint_numerator;
        }
    }

    Ok(RotatingContactSearchHit3d {
        time: canonical_time(bracket.upper_numerator, bracket.denominator),
        pair,
        contact: bracket.upper_contact,
    })
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
        RigidBoxFreeFlightConfig3d, RotatingContactSearchConfig3d, Vec3i,
        rotating_broad_phase::RotatingBroadPhase3d,
        rotating_contact_search_reference::sampled_rotating_contact_search_with_broad_phase as reference_search,
    };

    use super::sampled_rotating_contact_search_profiled;

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

    fn config(samples: u16) -> RotatingContactSearchConfig3d {
        RotatingContactSearchConfig3d::new(
            RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1),
            samples,
            4,
        )
    }

    fn assert_matches_reference(boxes: &[RigidBox3d], config: RotatingContactSearchConfig3d) {
        let mut reference_broad_phase = RotatingBroadPhase3d::default();
        let expected = reference_search(boxes, config, &mut reference_broad_phase)
            .expect("reference first-contact search");
        let mut ordered_broad_phase = RotatingBroadPhase3d::default();
        let (actual, _) =
            sampled_rotating_contact_search_profiled(boxes, config, &mut ordered_broad_phase)
                .expect("ordered first-contact search");
        assert_eq!(actual, expected);
    }

    #[test]
    fn ordered_search_matches_reference_across_deterministic_matrix() {
        for offset in [-4, -2, 0, 2, 4] {
            for velocity in [8, 16, 24, 32] {
                let boxes = [
                    dynamic(11, Vec3i::new(-12, offset, 0), Vec3i::new(velocity, 0, 0)),
                    fixed(3, Vec3i::new(-2, 0, 0)),
                    fixed(7, Vec3i::new(2, 2, 0)),
                    dynamic(13, Vec3i::new(8, -offset, 0), Vec3i::new(-velocity, 0, 0)),
                ];
                assert_matches_reference(&boxes, config(8));
            }
        }
    }

    #[test]
    fn ordered_search_matches_reference_for_initial_contact_ties() {
        let boxes = [
            dynamic(11, Vec3i::ZERO, Vec3i::ZERO),
            fixed(3, Vec3i::new(2, 0, 0)),
            fixed(7, Vec3i::new(-2, 0, 0)),
        ];
        assert_matches_reference(&boxes, config(16));
    }

    #[test]
    fn thirty_six_body_front_loaded_fixture_stops_on_first_hit_row() {
        let mut boxes = Vec::new();
        boxes.push(dynamic(1, Vec3i::new(-40, 0, 0), Vec3i::new(120, 0, 0)));
        for id in 2..36_u64 {
            let offset = i32::try_from(id - 2).expect("small id");
            boxes.push(fixed(id, Vec3i::new(10 + offset * 2, 0, 0)));
        }
        boxes.push(fixed(36, Vec3i::new(-20, 0, 0)));

        let mut broad_phase = RotatingBroadPhase3d::default();
        let (hit, work) =
            sampled_rotating_contact_search_profiled(&boxes, config(64), &mut broad_phase)
                .expect("ordered stress search");
        let hit = hit.expect("front-loaded fixture has a contact");
        assert_eq!(hit.pair.right, BodyId(36));
        assert!(
            work.candidate_pairs >= 30,
            "large candidate set expected: {work:?}"
        );
        assert!(
            work.coarse_rows < 32,
            "global row bound should stop early: {work:?}"
        );
        assert!(
            work.coarse_pair_evaluations < work.candidate_pairs.saturating_mul(32),
            "sample-major traversal should stop all pairs together: {work:?}"
        );
        assert_matches_reference(&boxes, config(64));
    }
}
