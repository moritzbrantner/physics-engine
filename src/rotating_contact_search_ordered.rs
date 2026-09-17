use std::{cmp::Ordering, collections::BTreeMap};

use crate::{
    BodyId, ObbContactSeed3d, OrientedBox3d, RigidBox3d, RigidBoxFreeFlightConfig3d,
    RotationalSweepPair3d, sample_rigid_box_free_flight,
};
use crate::{
    oriented_box::{PreparedObb3d, obb_contact_seed_prepared},
    rotating_broad_phase::RotatingBroadPhase3d,
};

pub use crate::rotating_contact_search_reference::{
    RotatingContactSearchConfig3d, RotatingContactSearchError3d, RotatingContactSearchHit3d,
    SampledContactTime3d,
};

const MAX_CACHED_COARSE_SAMPLES: usize = 4_096;

pub(crate) fn coarse_sample_limit(time: SampledContactTime3d, sample_count: u16) -> u32 {
    let scaled_numerator = u64::from(time.numerator) * u64::from(sample_count);
    let limit = scaled_numerator.div_ceil(u64::from(time.denominator));
    u32::try_from(limit)
        .expect("coarse sample limit fits u32")
        .min(u32::from(sample_count))
}

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
        config: RigidBoxFreeFlightConfig3d,
        numerator: u32,
    ) -> Result<PreparedObb3d, RotatingContactSearchError3d> {
        let sample_index = usize::try_from(numerator - 1).expect("coarse numerator fits usize");
        if let Some(sampled) = self.samples_by_body[body_index].get(sample_index).copied() {
            return Ok(sampled);
        }

        let shape = sample_rigid_box_free_flight(rigid_box, config, numerator, self.denominator)?
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
struct PairState3d {
    pair: RotationalSweepPair3d,
    left_index: usize,
    right_index: usize,
}

/// Searches the conservative candidate set in global coarse-time order.
///
/// The reference pair-major implementation remains compiled as the semantics oracle. Both searches use
/// the same broad-phase candidates, canonical free-flight samples, prepared SAT geometry, coarse grid,
/// refinement count and stable `(time, pair)` ordering. The only change is traversal order: every pair is
/// evaluated at coarse row 1 before any pair reaches row 2. Once a complete row contains contact, no later
/// row can refine to an earlier time because every candidate was observed clear on all preceding rows.
/// All contacts in that first hit row are refined before the stable minimum is selected.
///
/// This remains sampled rotational CCD. It deliberately does not reinterpret the existing approximation
/// or cache state across collision response; it only avoids evaluating coarse rows that are provably later
/// than the globally earliest observed contact row.
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
    let denominator = u32::from(config.sample_count);
    let mut coarse_samples = CoarseSampleCache3d::new(boxes.len(), denominator);
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
        let prepared_left =
            coarse_samples.prepare_geometry(left_index, boxes[left_index].oriented_box());
        let prepared_right =
            coarse_samples.prepare_geometry(right_index, boxes[right_index].oriented_box());
        work.initial_pair_evaluations = work.initial_pair_evaluations.saturating_add(1);
        work.exact_contact_evaluations = work.exact_contact_evaluations.saturating_add(1);
        if let Some(contact) = obb_contact_seed_prepared(&prepared_left, &prepared_right)? {
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

    for numerator in 1..=denominator {
        work.coarse_rows = work.coarse_rows.saturating_add(1);
        let mut row_best: Option<RotatingContactSearchHit3d> = None;
        for state in &states {
            work.coarse_pair_evaluations = work.coarse_pair_evaluations.saturating_add(1);
            let sampled_left = coarse_samples.sample_geometry(
                state.left_index,
                &boxes[state.left_index],
                config.free_flight,
                numerator,
            )?;
            let sampled_right = coarse_samples.sample_geometry(
                state.right_index,
                &boxes[state.right_index],
                config.free_flight,
                numerator,
            )?;
            work.exact_contact_evaluations = work.exact_contact_evaluations.saturating_add(1);
            let Some(contact) = obb_contact_seed_prepared(&sampled_left, &sampled_right)? else {
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
            crate::obb_contact_seed(sampled_left.oriented_box(), sampled_right.oriented_box())?
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
        RigidBoxFreeFlightConfig3d, Vec3i, rotating_broad_phase::RotatingBroadPhase3d,
        rotating_contact_search_reference::sampled_rotating_contact_search_with_broad_phase as reference_search,
    };

    use super::{RotatingContactSearchConfig3d, sampled_rotating_contact_search_profiled};

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
    fn global_time_order_stops_after_the_first_contact_row() {
        let moving = dynamic(50, Vec3i::new(-6, 0, 0), Vec3i::new(32, 0, 0));
        let boxes = [
            moving,
            fixed(1, Vec3i::new(-2, 0, 0)),
            fixed(2, Vec3i::new(2, 0, 0)),
            fixed(3, Vec3i::new(6, 0, 0)),
            fixed(4, Vec3i::new(10, 0, 0)),
        ];
        let mut broad_phase = RotatingBroadPhase3d::default();
        let (hit, work) =
            sampled_rotating_contact_search_profiled(&boxes, config(32), &mut broad_phase)
                .expect("ordered first-contact search");
        assert!(hit.is_some());
        assert!(
            work.coarse_rows < 32,
            "search should not scan the full time grid"
        );
        assert_eq!(
            work.coarse_pair_evaluations,
            work.candidate_pairs * work.coarse_rows,
            "every candidate is visited exactly once per completed coarse row"
        );
        assert_matches_reference(&boxes, config(32));
    }
}
