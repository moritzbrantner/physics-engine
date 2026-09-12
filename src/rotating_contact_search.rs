use std::{cmp::Ordering, collections::BTreeMap, error::Error, fmt};

use crate::rotating_broad_phase::RotatingBroadPhase3d;
use crate::{
    BodyId, ObbContactSeed3d, OrientedBox3d, OrientedBoxError3d, RigidBox3d,
    RigidBoxFreeFlightConfig3d, RigidBoxFreeFlightError3d, RotatingBroadPhaseError3d,
    RotationalSweepPair3d, obb_contact_seed, sample_rigid_box_free_flight,
};

const MAX_CACHED_COARSE_SAMPLES: usize = 4_096;

/// Rational upper-bound time returned by deterministic sampled rotating-contact search.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SampledContactTime3d {
    pub numerator: u32,
    pub denominator: u32,
}

impl SampledContactTime3d {
    pub const ZERO: Self = Self {
        numerator: 0,
        denominator: 1,
    };

    fn canonical(numerator: u32, denominator: u32) -> Self {
        let divisor = greatest_common_divisor(u64::from(numerator), u64::from(denominator));
        let divisor = u32::try_from(divisor).expect("gcd of u32 values fits u32");
        Self {
            numerator: numerator / divisor,
            denominator: denominator / divisor,
        }
    }
}

/// Sampling policy for rotating OBB contact search over one collision-free interval.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RotatingContactSearchConfig3d {
    pub free_flight: RigidBoxFreeFlightConfig3d,
    pub sample_count: u16,
    pub refinement_steps: u8,
}

impl RotatingContactSearchConfig3d {
    #[must_use]
    pub const fn new(
        free_flight: RigidBoxFreeFlightConfig3d,
        sample_count: u16,
        refinement_steps: u8,
    ) -> Self {
        Self {
            free_flight,
            sample_count,
            refinement_steps,
        }
    }
}

/// Earliest sampled rotating-box contact retained by the configured search grid.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RotatingContactSearchHit3d {
    /// Conservative upper sample/refinement bound at which contact is known to exist.
    pub time: SampledContactTime3d,
    pub pair: RotationalSweepPair3d,
    pub contact: ObbContactSeed3d,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RotatingContactSearchError3d {
    ZeroSampleCount,
    ResolutionTooFine {
        sample_count: u16,
        refinement_steps: u8,
    },
    MissingCandidateBody(BodyId),
    BroadPhase(RotatingBroadPhaseError3d),
    FreeFlight(RigidBoxFreeFlightError3d),
    Contact(OrientedBoxError3d),
}

impl fmt::Display for RotatingContactSearchError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ZeroSampleCount => write!(
                formatter,
                "rotating contact search requires at least one coarse sample"
            ),
            Self::ResolutionTooFine {
                sample_count,
                refinement_steps,
            } => write!(
                formatter,
                "rotating contact search resolution {sample_count} × 2^{refinement_steps} exceeds the supported u32 fraction contract"
            ),
            Self::MissingCandidateBody(id) => write!(
                formatter,
                "rotating contact search broad phase referenced missing body {}",
                id.0
            ),
            Self::BroadPhase(error) => {
                write!(
                    formatter,
                    "rotating contact search broad phase failed: {error}"
                )
            }
            Self::FreeFlight(error) => {
                write!(
                    formatter,
                    "rotating contact search free flight failed: {error}"
                )
            }
            Self::Contact(error) => {
                write!(
                    formatter,
                    "rotating contact search OBB query failed: {error}"
                )
            }
        }
    }
}

impl Error for RotatingContactSearchError3d {}

impl From<RotatingBroadPhaseError3d> for RotatingContactSearchError3d {
    fn from(value: RotatingBroadPhaseError3d) -> Self {
        Self::BroadPhase(value)
    }
}

impl From<RigidBoxFreeFlightError3d> for RotatingContactSearchError3d {
    fn from(value: RigidBoxFreeFlightError3d) -> Self {
        Self::FreeFlight(value)
    }
}

impl From<OrientedBoxError3d> for RotatingContactSearchError3d {
    fn from(value: OrientedBoxError3d) -> Self {
        Self::Contact(value)
    }
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
        if self.cached_entries < MAX_CACHED_COARSE_SAMPLES
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

/// Searches candidate rotating-box pairs on a deterministic fixed sample grid and refines the first
/// observed contact bracket.
///
/// Broad phase comes from the engine's conservative rotating broad phase. Every narrow-phase sample is
/// rebuilt directly from the interval start through [`sample_rigid_box_free_flight`], then evaluated with
/// [`obb_contact_seed`]. Coarse samples are cached by body index and coarse-grid numerator under the
/// search's fixed denominator, so candidate pairs that share a body reuse identical free-flight work
/// without a tree lookup; the cache is bounded and refinement remains pair-local.
/// For each candidate pair, the first coarse sample with contact brackets the transition against the
/// previous coarse sample; binary refinement returns the earliest known contact side of that bracket. Pair
/// ties are resolved by stable [`BodyId`] ordering.
///
/// This is deliberately a **sampled approximation**, not analytic rotational CCD. A contact island that
/// begins and ends entirely between adjacent coarse samples can be missed. Increasing `sample_count`
/// narrows that gap; `refinement_steps` only sharpens a contact bracket that was already observed.
/// Consumers must not reinterpret a sampled hit as proof that no earlier unsampled rotational contact
/// exists.
///
/// # Errors
///
/// Returns [`RotatingContactSearchError3d`] for invalid resolution, duplicate/missing body identity,
/// malformed free-flight state, or OBB contact arithmetic failure.
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
    validate_resolution(config)?;
    let pairs = broad_phase.candidate_pairs(boxes, config.free_flight)?;
    if pairs.is_empty() {
        return Ok(None);
    }

    let by_id: BTreeMap<BodyId, usize> = boxes
        .iter()
        .enumerate()
        .map(|(index, rigid_box)| (rigid_box.body().id(), index))
        .collect();
    let denominator = u32::from(config.sample_count);
    let mut coarse_samples = CoarseSampleCache3d::new(boxes.len(), denominator);
    let mut best: Option<RotatingContactSearchHit3d> = None;
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
        let left = &boxes[left_index];
        let right = &boxes[right_index];
        let Some(hit) = search_pair(
            left_index,
            left,
            right_index,
            right,
            pair,
            config,
            &mut coarse_samples,
        )?
        else {
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
    left_index: usize,
    left: &RigidBox3d,
    right_index: usize,
    right: &RigidBox3d,
    pair: RotationalSweepPair3d,
    config: RotatingContactSearchConfig3d,
    coarse_samples: &mut CoarseSampleCache3d,
) -> Result<Option<RotatingContactSearchHit3d>, RotatingContactSearchError3d> {
    if let Some(contact) = obb_contact_seed(left.oriented_box(), right.oriented_box())? {
        return Ok(Some(RotatingContactSearchHit3d {
            time: SampledContactTime3d::ZERO,
            pair,
            contact,
        }));
    }

    let denominator = u32::from(config.sample_count);
    for numerator in 1..=denominator {
        let sampled_left =
            coarse_samples.sample_oriented_box(left_index, left, config.free_flight, numerator)?;
        let sampled_right = coarse_samples.sample_oriented_box(
            right_index,
            right,
            config.free_flight,
            numerator,
        )?;
        let Some(contact) = obb_contact_seed(sampled_left, sampled_right)? else {
            continue;
        };

        let bracket = ContactBracket3d {
            lower_numerator: numerator - 1,
            upper_numerator: numerator,
            denominator,
            upper_contact: contact,
        };
        return Ok(Some(refine_contact_bracket(
            left, right, pair, config, bracket,
        )?));
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
            .lower_numerator
            .checked_add(bracket.upper_numerator)
            .ok_or(RotatingContactSearchError3d::ResolutionTooFine {
                sample_count: config.sample_count,
                refinement_steps: config.refinement_steps,
            })?;
        bracket.denominator = bracket.denominator.checked_mul(2).ok_or(
            RotatingContactSearchError3d::ResolutionTooFine {
                sample_count: config.sample_count,
                refinement_steps: config.refinement_steps,
            },
        )?;
        bracket.lower_numerator = bracket.lower_numerator.checked_mul(2).ok_or(
            RotatingContactSearchError3d::ResolutionTooFine {
                sample_count: config.sample_count,
                refinement_steps: config.refinement_steps,
            },
        )?;
        bracket.upper_numerator = bracket.upper_numerator.checked_mul(2).ok_or(
            RotatingContactSearchError3d::ResolutionTooFine {
                sample_count: config.sample_count,
                refinement_steps: config.refinement_steps,
            },
        )?;

        let (sampled_left, sampled_right) = sample_pair(
            left,
            right,
            config.free_flight,
            midpoint_numerator,
            bracket.denominator,
        )?;
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
        time: SampledContactTime3d::canonical(bracket.upper_numerator, bracket.denominator),
        pair,
        contact: bracket.upper_contact,
    })
}

fn sample_pair(
    left: &RigidBox3d,
    right: &RigidBox3d,
    config: RigidBoxFreeFlightConfig3d,
    numerator: u32,
    denominator: u32,
) -> Result<(RigidBox3d, RigidBox3d), RotatingContactSearchError3d> {
    Ok((
        sample_rigid_box_free_flight(left, config, numerator, denominator)?,
        sample_rigid_box_free_flight(right, config, numerator, denominator)?,
    ))
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
        RigidBoxFreeFlightConfig3d, RotationalSweepPair3d, Vec3i, obb_contact_seed,
        sample_rigid_box_free_flight,
    };

    use super::{
        CoarseSampleCache3d, MAX_CACHED_COARSE_SAMPLES, RotatingContactSearchConfig3d,
        RotatingContactSearchError3d, SampledContactTime3d, sampled_rotating_contact_search,
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

    fn config(sample_count: u16, refinement_steps: u8) -> RotatingContactSearchConfig3d {
        RotatingContactSearchConfig3d::new(
            RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1),
            sample_count,
            refinement_steps,
        )
    }

    #[test]
    fn initial_contact_returns_zero_time() {
        let hit = sampled_rotating_contact_search(
            &[
                dynamic(4, Vec3i::ZERO, Vec3i::ZERO),
                fixed(9, Vec3i::new(2, 0, 0)),
            ],
            config(4, 3),
        )
        .expect("valid sampled search")
        .expect("initial touching contact");

        assert_eq!(hit.time, SampledContactTime3d::ZERO);
        assert_eq!(
            hit.pair,
            RotationalSweepPair3d {
                left: BodyId(4),
                right: BodyId(9),
            }
        );
    }

    #[test]
    fn linear_contact_is_refined_to_known_contact_side() {
        let moving = dynamic(2, Vec3i::new(-10, 0, 0), Vec3i::new(20, 0, 0));
        let obstacle = fixed(8, Vec3i::ZERO);
        let hit =
            sampled_rotating_contact_search(&[moving.clone(), obstacle.clone()], config(4, 3))
                .expect("valid sampled search")
                .expect("linear path should contact");

        assert_eq!(
            hit.time,
            SampledContactTime3d {
                numerator: 3,
                denominator: 8,
            }
        );
        let sampled_moving = sample_rigid_box_free_flight(
            &moving,
            RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1),
            hit.time.numerator,
            hit.time.denominator,
        )
        .expect("valid returned sample");
        assert!(
            obb_contact_seed(sampled_moving.oriented_box(), obstacle.oriented_box())
                .expect("valid returned OBBs")
                .is_some()
        );
    }

    #[test]
    fn shared_body_pair_ties_keep_stable_pair_order() {
        let moving = dynamic(11, Vec3i::new(-10, 0, 0), Vec3i::new(20, 0, 0));
        let first = fixed(3, Vec3i::ZERO);
        let second = fixed(8, Vec3i::ZERO);
        let hit = sampled_rotating_contact_search(&[second, moving, first], config(8, 2))
            .expect("valid shared-body search")
            .expect("shared body should contact both obstacles");

        assert_eq!(
            hit.pair,
            RotationalSweepPair3d {
                left: BodyId(3),
                right: BodyId(11),
            }
        );
    }

    #[test]
    fn coarse_sample_cache_is_bounded_and_reuses_geometry() {
        let rigid_box = fixed(1, Vec3i::ZERO);
        let free_flight = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1);
        let mut cache = CoarseSampleCache3d::new(1, 32);

        let first = cache
            .sample_oriented_box(0, &rigid_box, free_flight, 1)
            .expect("first coarse sample");
        let repeated = cache
            .sample_oriented_box(0, &rigid_box, free_flight, 1)
            .expect("repeated coarse sample");
        assert_eq!(first, repeated);
        assert_eq!(cache.len(), 1);

        let denominator = u32::try_from(MAX_CACHED_COARSE_SAMPLES + 2).expect("small cache bound");
        let mut bounded_cache = CoarseSampleCache3d::new(1, denominator);
        for numerator in 1..=denominator {
            bounded_cache
                .sample_oriented_box(0, &rigid_box, free_flight, numerator)
                .expect("bounded coarse sample");
        }
        assert_eq!(bounded_cache.len(), MAX_CACHED_COARSE_SAMPLES);
    }

    #[test]
    fn search_result_is_stable_across_input_order() {
        let moving = dynamic(11, Vec3i::new(-10, 0, 0), Vec3i::new(20, 0, 0));
        let obstacle = fixed(3, Vec3i::ZERO);
        let forward =
            sampled_rotating_contact_search(&[moving.clone(), obstacle.clone()], config(8, 2))
                .expect("valid forward search");
        let reverse = sampled_rotating_contact_search(&[obstacle, moving], config(8, 2))
            .expect("valid reverse search");

        assert_eq!(forward, reverse);
    }

    #[test]
    fn separated_motion_returns_no_hit() {
        assert_eq!(
            sampled_rotating_contact_search(
                &[
                    dynamic(1, Vec3i::new(-100, 0, 0), Vec3i::ZERO),
                    fixed(2, Vec3i::new(100, 0, 0)),
                ],
                config(8, 2),
            )
            .expect("valid separated search"),
            None
        );
    }

    #[test]
    fn zero_samples_and_unrepresentable_refinement_fail_closed() {
        let boxes = [
            dynamic(1, Vec3i::ZERO, Vec3i::ZERO),
            fixed(2, Vec3i::new(2, 0, 0)),
        ];
        assert_eq!(
            sampled_rotating_contact_search(&boxes, config(0, 0)),
            Err(RotatingContactSearchError3d::ZeroSampleCount)
        );
        assert_eq!(
            sampled_rotating_contact_search(&boxes, config(u16::MAX, 17)),
            Err(RotatingContactSearchError3d::ResolutionTooFine {
                sample_count: u16::MAX,
                refinement_steps: 17,
            })
        );
    }
}
