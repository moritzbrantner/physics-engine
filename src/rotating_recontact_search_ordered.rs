use std::{cmp::Ordering, collections::BTreeMap};

use crate::{
    BodyId, ObbContactSeed3d, OrientedBox3d, RigidBox3d, RotatingContactSearchConfig3d,
    RotatingContactSearchError3d, RotatingContactSearchHit3d, RotationalSweepPair3d,
    SampledContactTime3d, sample_rigid_box_free_flight,
};
use crate::{
    oriented_box::{PreparedObb3d, obb_contact_seed_prepared},
    rotating_broad_phase::RotatingBroadPhase3d,
};

const MAX_CACHED_COARSE_SAMPLES: usize = 4_096;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub(crate) struct RecontactSearchWork3d {
    pub candidate_pairs: u64,
    pub initial_pair_evaluations: u64,
    pub coarse_rows: u64,
    pub coarse_pair_evaluations: u64,
    pub exact_contact_evaluations: u64,
    pub refinement_evaluations: u64,
}

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
}

impl CoarseSampleCache3d {
    fn new(body_count: usize, denominator: u32) -> Self {
        Self {
            samples_by_body: vec![Vec::new(); body_count],
            latest_geometry: vec![None; body_count],
            cached_entries: 0,
            denominator,
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

#[derive(Debug, Default)]
struct PairContactCache3d {
    previous: Option<CachedPairContact3d>,
}

impl PairContactCache3d {
    fn contact(
        &mut self,
        left: &PreparedObb3d,
        right: &PreparedObb3d,
        work: &mut RecontactSearchWork3d,
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
        work.exact_contact_evaluations = work.exact_contact_evaluations.saturating_add(1);
        Ok(contact)
    }
}

#[derive(Debug)]
struct PairState3d {
    pair: RotationalSweepPair3d,
    left_index: usize,
    right_index: usize,
    last_clear: Option<u32>,
    contacts: PairContactCache3d,
}

/// Finds the earliest strictly-positive sampled rotating re-contact using a globally time-ordered coarse
/// traversal.
///
/// The reference implementation traverses one candidate pair at a time. This implementation keeps the
/// exact same coarse grid, clear-to-contact eligibility rule, binary refinement and `(time, pair)` tie
/// ordering, but advances every candidate pair through coarse row 1 before row 2, and so on. As soon as a
/// complete coarse row contains at least one eligible contact, no later row can refine to an earlier time:
/// every still-active pair was observed clear (or remained ineligible) at the completed row. The search can
/// therefore stop globally after refining all eligible contacts in that row.
///
/// Pair-local state retains only the previous exact contact result and the latest clear coarse numerator.
/// No state survives a search or collision response, so repeated-event semantics and fail-closed behavior
/// remain unchanged. The optimization changes work ordering only; it does not reuse sampled times across a
/// response or reinterpret the sampled approximation as analytic CCD.
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
    sampled_rotating_recontact_search_profiled(boxes, config, broad_phase).map(|(hit, _)| hit)
}

pub(crate) fn sampled_rotating_recontact_search_profiled(
    boxes: &[RigidBox3d],
    config: RotatingContactSearchConfig3d,
    broad_phase: &mut RotatingBroadPhase3d,
) -> Result<(Option<RotatingContactSearchHit3d>, RecontactSearchWork3d), RotatingContactSearchError3d>
{
    validate_resolution(config)?;
    let pairs = broad_phase.candidate_pairs(boxes, config.free_flight)?;
    let mut work = RecontactSearchWork3d {
        candidate_pairs: u64::try_from(pairs.len()).unwrap_or(u64::MAX),
        ..RecontactSearchWork3d::default()
    };
    if pairs.is_empty() {
        return Ok((None, work));
    }

    let by_id = boxes
        .iter()
        .enumerate()
        .map(|(index, rigid_box)| (rigid_box.body().id(), index))
        .collect::<BTreeMap<BodyId, usize>>();
    let denominator = u32::from(config.sample_count);
    let mut coarse_samples = CoarseSampleCache3d::new(boxes.len(), denominator);
    let mut states = Vec::with_capacity(pairs.len());

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
        let mut contacts = PairContactCache3d::default();
        let prepared_left =
            coarse_samples.prepare_geometry(left_index, boxes[left_index].oriented_box());
        let prepared_right =
            coarse_samples.prepare_geometry(right_index, boxes[right_index].oriented_box());
        work.initial_pair_evaluations = work.initial_pair_evaluations.saturating_add(1);
        let initially_contacting = contacts
            .contact(&prepared_left, &prepared_right, &mut work)?
            .is_some();
        states.push(PairState3d {
            pair,
            left_index,
            right_index,
            last_clear: (!initially_contacting).then_some(0),
            contacts,
        });
    }

    for numerator in 1..=denominator {
        work.coarse_rows = work.coarse_rows.saturating_add(1);
        let mut row_best: Option<RotatingContactSearchHit3d> = None;
        for state in &mut states {
            work.coarse_pair_evaluations = work.coarse_pair_evaluations.saturating_add(1);
            let sampled_left = coarse_samples.sample_geometry(
                state.left_index,
                &boxes[state.left_index],
                config,
                numerator,
            )?;
            let sampled_right = coarse_samples.sample_geometry(
                state.right_index,
                &boxes[state.right_index],
                config,
                numerator,
            )?;
            match state
                .contacts
                .contact(&sampled_left, &sampled_right, &mut work)?
            {
                Some(contact) => {
                    let Some(clear_numerator) = state.last_clear else {
                        continue;
                    };
                    let hit = refine_contact_bracket(
                        &boxes[state.left_index],
                        &boxes[state.right_index],
                        state.pair,
                        config,
                        ContactBracket3d {
                            clear_numerator,
                            contact_numerator: numerator,
                            denominator,
                            contact,
                        },
                        &mut work,
                    )?;
                    if row_best.is_none_or(|current| compare_hits(hit, current) == Ordering::Less) {
                        row_best = Some(hit);
                    }
                }
                None => state.last_clear = Some(numerator),
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
    work: &mut RecontactSearchWork3d,
) -> Result<RotatingContactSearchHit3d, RotatingContactSearchError3d> {
    for _ in 0..config.refinement_steps {
        let midpoint_numerator = bracket
            .clear_numerator
            .checked_add(bracket.contact_numerator)
            .ok_or_else(|| resolution_too_fine(config))?;
        bracket.denominator = bracket
            .denominator
            .checked_mul(2)
            .ok_or_else(|| resolution_too_fine(config))?;
        bracket.clear_numerator = bracket
            .clear_numerator
            .checked_mul(2)
            .ok_or_else(|| resolution_too_fine(config))?;
        bracket.contact_numerator = bracket
            .contact_numerator
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
        rotating_recontact_search_reference::sampled_rotating_recontact_search_with_broad_phase as reference_search,
    };

    use super::sampled_rotating_recontact_search_profiled;

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
            .expect("reference re-contact search");
        let mut ordered_broad_phase = RotatingBroadPhase3d::default();
        let (actual, _) =
            sampled_rotating_recontact_search_profiled(boxes, config, &mut ordered_broad_phase)
                .expect("ordered re-contact search");
        assert_eq!(actual, expected);
    }

    #[test]
    fn ordered_search_matches_reference_for_clear_and_persistent_pairs() {
        let boxes = [
            dynamic(1, Vec3i::new(-2, 0, 0), Vec3i::ZERO),
            fixed(2, Vec3i::ZERO),
            dynamic(3, Vec3i::new(-12, 6, 0), Vec3i::new(24, 0, 0)),
            fixed(4, Vec3i::new(0, 6, 0)),
            fixed(5, Vec3i::new(6, 6, 0)),
        ];
        assert_matches_reference(&boxes, config(16));
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
    fn thirty_six_body_front_loaded_fixture_stops_on_first_hit_row() {
        let mut boxes = Vec::new();
        boxes.push(dynamic(1, Vec3i::new(-24, 0, 0), Vec3i::new(64, 0, 0)));
        for id in 2..=35_u64 {
            let x = 10 + i32::try_from(id).expect("small id");
            boxes.push(fixed(id, Vec3i::new(x, 0, 0)));
        }
        boxes.push(fixed(36, Vec3i::new(-8, 0, 0)));

        let mut broad_phase = RotatingBroadPhase3d::default();
        let (hit, work) =
            sampled_rotating_recontact_search_profiled(&boxes, config(64), &mut broad_phase)
                .expect("ordered stress search");
        let hit = hit.expect("front-loaded fixture has a contact");
        assert_eq!(hit.pair.right, BodyId(36));
        assert!(
            work.candidate_pairs >= 30,
            "stress fixture should retain a large broad-phase candidate set: {work:?}"
        );
        assert!(
            work.coarse_rows < 32,
            "global row bound should stop before half the grid: {work:?}"
        );
        assert!(
            work.coarse_pair_evaluations < work.candidate_pairs.saturating_mul(32),
            "sample-major traversal should stop all pairs together: {work:?}"
        );
        assert_matches_reference(&boxes, config(64));
    }
}
