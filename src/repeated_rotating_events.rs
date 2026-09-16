use std::{
    collections::{BTreeMap, BTreeSet},
    error::Error,
    fmt,
};

use crate::{
    RigidBox3d, RigidBoxFreeFlightConfig3d, RigidBoxFreeFlightError3d, RotatingContactFrontier3d,
    RotatingContactFrontierError3d, RotatingContactResponseError3d, RotatingContactSearchConfig3d,
    RotatingContactSearchHit3d, RotationalSweepPair3d, SampledContactTime3d, obb_contact_seed,
};
use crate::{
    rotating_broad_phase::RotatingBroadPhase3d,
    rotating_contact_frontier::{
        earliest_rotating_contact_frontier_with_broad_phase,
        next_rotating_contact_frontier_with_broad_phase,
    },
    rotating_contact_response::{
        RotatingContactResponseScratch3d,
        resolve_rotating_contact_frontier_with_activity_and_scratch,
    },
};

pub const MAX_REPEATED_ROTATING_EVENTS: u16 = 64;

/// Bounded policy for advancing through sampled rotating collision events.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RepeatedRotatingEventConfig3d {
    pub search: RotatingContactSearchConfig3d,
    pub solver_passes: u8,
    pub max_events: u16,
}

impl RepeatedRotatingEventConfig3d {
    #[must_use]
    pub const fn new(
        search: RotatingContactSearchConfig3d,
        solver_passes: u8,
        max_events: u16,
    ) -> Self {
        Self {
            search,
            solver_passes,
            max_events,
        }
    }
}

/// One resolved sampled event, expressed relative to the segment that began after the previous event.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RotatingResolvedEvent3d {
    pub time: SampledContactTime3d,
    pub contacts: Vec<RotatingContactSearchHit3d>,
    pub response_passes: u8,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RepeatedRotatingEventWorkStats3d {
    pub event_response_passes: u64,
    pub stabilization_passes: u64,
    pub stabilizations_hitting_limit: u64,
    pub stabilization_candidate_pairs: u64,
    pub stabilization_exact_contacts: u64,
    pub stabilization_active_bodies: u64,
}

#[derive(Clone, Debug)]
struct CurrentContactStabilization3d {
    boxes: Vec<RigidBox3d>,
    observed_pairs: BTreeSet<RotationalSweepPair3d>,
    exhausted_with_changes: bool,
}

/// Result of consuming every sampled rotating event admitted before the first unresolved tail.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RepeatedRotatingEventAdvance3d {
    /// State immediately after the last resolved sampled frontier, or the unchanged input if no event
    /// was admitted.
    pub boxes: Vec<RigidBox3d>,
    pub events: Vec<RotatingResolvedEvent3d>,
    /// Requested time that remains deliberately unconsumed.
    ///
    /// The ratio is preserved exactly in deterministic fixed-capacity limb storage. It is not rounded
    /// back to the narrower public constructor inputs, so a collision in the suffix remains part of the
    /// same requested interval and is still available to the world-level tail solver.
    ///
    /// The remaining segment is not automatically free-flown because it may contain persistent/resting
    /// contacts that the world-level tail solver must stabilize.
    pub remaining: RigidBoxFreeFlightConfig3d,
    pub work: RepeatedRotatingEventWorkStats3d,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RepeatedRotatingEventError3d {
    ZeroEventLimit,
    EventLimit(u16),
    NegativeTimestepNumerator(i128),
    NonPositiveTimestepDenominator(i128),
    InvalidRemainder(SampledContactTime3d),
    RatioTooLarge,
    Frontier(RotatingContactFrontierError3d),
    Response(RotatingContactResponseError3d),
}

impl fmt::Display for RepeatedRotatingEventError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ZeroEventLimit => write!(
                formatter,
                "repeated rotating event advance requires at least one event slot"
            ),
            Self::EventLimit(limit) => write!(
                formatter,
                "repeated rotating event advance reached its {limit}-event limit while another sampled event remained"
            ),
            Self::NegativeTimestepNumerator(value) => write!(
                formatter,
                "repeated rotating event timestep numerator must be non-negative, got {value}"
            ),
            Self::NonPositiveTimestepDenominator(value) => write!(
                formatter,
                "repeated rotating event timestep denominator must be positive, got {value}"
            ),
            Self::InvalidRemainder(time) => write!(
                formatter,
                "repeated rotating event response returned invalid remaining fraction after {}/{}",
                time.numerator, time.denominator
            ),
            Self::RatioTooLarge => write!(
                formatter,
                "repeated rotating event exact remaining-time arithmetic exceeded deterministic exact-ratio capacity"
            ),
            Self::Frontier(error) => write!(
                formatter,
                "repeated rotating event frontier failed: {error}"
            ),
            Self::Response(error) => write!(
                formatter,
                "repeated rotating event response failed: {error}"
            ),
        }
    }
}

impl Error for RepeatedRotatingEventError3d {}

impl From<RotatingContactFrontierError3d> for RepeatedRotatingEventError3d {
    fn from(value: RotatingContactFrontierError3d) -> Self {
        Self::Frontier(value)
    }
}

impl From<RotatingContactResponseError3d> for RepeatedRotatingEventError3d {
    fn from(value: RotatingContactResponseError3d) -> Self {
        Self::Response(value)
    }
}

/// Advances through a bounded sequence of sampled rotating collision events while preserving the exact
/// remaining requested interval.
///
/// The first event is selected with [`crate::earliest_rotating_contact_frontier`]. After response, the
/// remaining rational timestep becomes the next segment. Before that next segment is searched, the current
/// contact frontier is stabilized through the configured bounded solver-pass budget. Each pass refreshes
/// the current zero-time contact set and applies one simultaneous response pass. This keeps resting and
/// newly-created support constraints in the authoritative solver without demanding exact global
/// idempotence from quantized contact projection. Current-frontier discovery reuses the persistent
/// conservative broad phase at a zero timestep, then exact-filters every candidate with the current OBB
/// geometry; it deliberately does not re-enter the sampled temporal search. Later positive events are then
/// selected with [`crate::next_rotating_contact_frontier`], so a persistent time-zero pair cannot monopolize
/// event discovery.
///
/// If bounded response or current-contact stabilization still changes an island on its final configured
/// pass, and the next reconstructed positive frontier contains no pair outside the contacts observed for
/// that island, the positive frontier is treated as continuation rather than a fresh impact. The solver
/// advances exactly to that sampled re-contact state and returns the contacted suffix to the world-level
/// persistent-tail solver. This prevents projection-created clear samples from manufacturing an unbounded
/// sequence of new impact events without suppressing a genuinely new pair or changing the event cap,
/// tolerances, or sampled search resolution.
///
/// Every admitted frontier is resolved before the next segment is searched. Event times in
/// [`RotatingResolvedEvent3d`] are therefore **segment-relative**, not absolute fractions of the original
/// requested interval. Zero-time stabilization passes consume no requested time and are not counted as
/// sampled events. Remaining-time composition canonicalizes each sampled remainder, cross-cancels it
/// against the current exact ratio, and stores the reduced result in deterministic limb storage. No
/// positive suffix is silently discarded merely because its reduced numerator or denominator exceeds
/// `i128`.
///
/// This function is intentionally not yet a complete frame step. When no further positive sampled event
/// is found, the final tail is returned in [`RepeatedRotatingEventAdvance3d::remaining`] rather than being
/// free-flown through potentially persistent contacts. Persistent/resting-contact stabilization over that
/// exact tail belongs to the world-level solver. The search itself remains sampled rotational collision
/// handling, not analytic rotational CCD, so an event island wholly between adjacent coarse samples can
/// still be missed.
///
/// # Errors
///
/// Returns [`RepeatedRotatingEventError3d`] for invalid bounds/timestep configuration, deterministic
/// exact-ratio capacity exhaustion, frontier/response failures, or when an actual additional sampled
/// event exists beyond `max_events`.
pub fn advance_repeated_rotating_events(
    boxes: &[RigidBox3d],
    config: RepeatedRotatingEventConfig3d,
) -> Result<RepeatedRotatingEventAdvance3d, RepeatedRotatingEventError3d> {
    let mut broad_phase = RotatingBroadPhase3d::default();
    advance_repeated_rotating_events_with_broad_phase(boxes, config, &mut broad_phase)
}

pub(crate) fn advance_repeated_rotating_events_with_broad_phase(
    boxes: &[RigidBox3d],
    config: RepeatedRotatingEventConfig3d,
    broad_phase: &mut RotatingBroadPhase3d,
) -> Result<RepeatedRotatingEventAdvance3d, RepeatedRotatingEventError3d> {
    validate_config(config)?;

    let mut work = RepeatedRotatingEventWorkStats3d::default();
    let mut response_scratch = RotatingContactResponseScratch3d::default();
    let mut remaining = config.search.free_flight;
    let first_search = search_with_free_flight(config.search, remaining);
    let Some(first_frontier) =
        earliest_rotating_contact_frontier_with_broad_phase(boxes, first_search, broad_phase)?
    else {
        return Ok(RepeatedRotatingEventAdvance3d {
            boxes: boxes.to_vec(),
            events: Vec::new(),
            remaining,
            work,
        });
    };

    let mut state;
    let mut events = Vec::new();
    let mut frontier = first_frontier;

    loop {
        if events.len() >= usize::from(config.max_events) {
            return Err(RepeatedRotatingEventError3d::EventLimit(config.max_events));
        }

        let (response, modified_body_ids) =
            resolve_rotating_contact_frontier_with_activity_and_scratch(
                frontier,
                config.solver_passes,
                &mut response_scratch,
            )?;
        remaining = scale_remaining_time(remaining, response.remaining_numerator, response.time)?;
        let response_time = response.time;
        let response_contacts = response.contacts;
        let response_passes = response.passes_used;
        let response_exhausted = response_passes == config.solver_passes;
        work.event_response_passes = work
            .event_response_passes
            .saturating_add(u64::from(response_passes));
        let stabilization = stabilize_current_contacts(
            response.boxes,
            config.solver_passes,
            broad_phase,
            &modified_body_ids,
            &response_contacts,
            &mut response_scratch,
            &mut work,
        )?;
        state = stabilization.boxes;
        let continuation_pairs = stabilization.observed_pairs;
        let solver_exhausted = response_exhausted || stabilization.exhausted_with_changes;
        events.push(RotatingResolvedEvent3d {
            time: response_time,
            contacts: response_contacts,
            response_passes,
        });

        if remaining.timestep_is_zero() {
            break;
        }
        let next_search = search_with_free_flight(config.search, remaining);
        let Some(next) =
            next_rotating_contact_frontier_with_broad_phase(&state, next_search, broad_phase)?
        else {
            break;
        };
        let continuing_island = solver_exhausted
            && !next.contacts.is_empty()
            && next
                .contacts
                .iter()
                .all(|contact| continuation_pairs.contains(&contact.pair));
        if continuing_island {
            remaining = scale_remaining_time(remaining, next.remaining_numerator, next.time)?;
            state = next.boxes;
            break;
        }
        frontier = next;
    }

    Ok(RepeatedRotatingEventAdvance3d {
        boxes: state,
        events,
        remaining,
        work,
    })
}

fn stabilize_current_contacts(
    mut boxes: Vec<RigidBox3d>,
    solver_passes: u8,
    broad_phase: &mut RotatingBroadPhase3d,
    initial_active: &[crate::BodyId],
    initial_contacts: &[RotatingContactSearchHit3d],
    response_scratch: &mut RotatingContactResponseScratch3d,
    work: &mut RepeatedRotatingEventWorkStats3d,
) -> Result<CurrentContactStabilization3d, RepeatedRotatingEventError3d> {
    let mut active = initial_active.to_vec();
    active.sort_unstable();
    active.dedup();
    let mut observed_pairs = initial_contacts
        .iter()
        .map(|contact| contact.pair)
        .collect::<BTreeSet<_>>();
    let mut contacts = initial_contacts
        .iter()
        .cloned()
        .map(|mut contact| {
            contact.time = SampledContactTime3d::ZERO;
            (contact.pair, contact)
        })
        .collect::<BTreeMap<_, _>>();
    let mut exhausted_with_changes = false;

    for pass in 0..solver_passes {
        if active.is_empty() {
            break;
        }
        work.stabilization_active_bodies = work
            .stabilization_active_bodies
            .saturating_add(u64::try_from(active.len()).unwrap_or(u64::MAX));
        let current = refresh_current_contacts_for_changed_bodies(
            &boxes,
            &active,
            &mut contacts,
            broad_phase,
            response_scratch,
        )?;
        work.stabilization_candidate_pairs = work
            .stabilization_candidate_pairs
            .saturating_add(u64::try_from(current.candidate_pairs).unwrap_or(u64::MAX));
        work.stabilization_exact_contacts = work
            .stabilization_exact_contacts
            .saturating_add(u64::try_from(current.recomputed_contacts).unwrap_or(u64::MAX));
        let Some(frontier) = current.frontier else {
            active.clear();
            break;
        };
        observed_pairs.extend(frontier.contacts.iter().map(|contact| contact.pair));
        work.stabilization_passes = work.stabilization_passes.saturating_add(1);
        let (response, modified_body_ids) =
            resolve_rotating_contact_frontier_with_activity_and_scratch(
                frontier,
                1,
                response_scratch,
            )?;
        boxes = response.boxes;
        active = modified_body_ids;
        if active.is_empty() {
            break;
        }
        if pass.saturating_add(1) == solver_passes {
            exhausted_with_changes = true;
        }
    }

    if exhausted_with_changes {
        work.stabilizations_hitting_limit = work.stabilizations_hitting_limit.saturating_add(1);
    }
    Ok(CurrentContactStabilization3d {
        boxes,
        observed_pairs,
        exhausted_with_changes,
    })
}

#[derive(Clone, Debug)]
struct CurrentContactFrontierResult3d {
    frontier: Option<RotatingContactFrontier3d>,
    candidate_pairs: usize,
    recomputed_contacts: usize,
}

/// Refreshes only exact contact edges whose geometry can have changed.
///
/// Contacts whose two endpoints are unchanged are retained as authoritative exact evidence. Every cached
/// edge touching an active body is discarded and revalidated from current geometry, while the targeted
/// broad phase discovers any newly-created neighbors of those active bodies. The resulting frontier still
/// contains *all* current contacts, preserving the simultaneous solver semantics of a full-world refresh
/// without recomputing unchanged edges.
fn refresh_current_contacts_for_changed_bodies(
    boxes: &[RigidBox3d],
    active: &[crate::BodyId],
    contacts: &mut BTreeMap<crate::RotationalSweepPair3d, RotatingContactSearchHit3d>,
    broad_phase: &mut RotatingBroadPhase3d,
    body_index: &RotatingContactResponseScratch3d,
) -> Result<CurrentContactFrontierResult3d, RotatingContactFrontierError3d> {
    let active_set = active.iter().copied().collect::<BTreeSet<_>>();
    contacts
        .retain(|pair, _| !active_set.contains(&pair.left) && !active_set.contains(&pair.right));

    let mut active_boxes = Vec::with_capacity(active.len());
    for id in active {
        let rigid_box = body_index
            .indexed_box(boxes, *id)
            .ok_or(RotatingContactFrontierError3d::MissingBody(*id))?;
        active_boxes.push(rigid_box);
    }
    let candidates = broad_phase.candidate_pairs_for_changed_current_bodies(active_boxes)?;
    let candidate_pairs = candidates.len();
    let mut recomputed_contacts = 0_usize;

    for pair in candidates {
        let left = body_index
            .indexed_box(boxes, pair.left)
            .ok_or(RotatingContactFrontierError3d::MissingBody(pair.left))?;
        let right = body_index
            .indexed_box(boxes, pair.right)
            .ok_or(RotatingContactFrontierError3d::MissingBody(pair.right))?;
        let Some(contact) = obb_contact_seed(left.oriented_box(), right.oriented_box())? else {
            continue;
        };
        recomputed_contacts = recomputed_contacts.saturating_add(1);
        contacts.insert(
            pair,
            RotatingContactSearchHit3d {
                time: SampledContactTime3d::ZERO,
                pair,
                contact,
            },
        );
    }

    let frontier = if contacts.is_empty() {
        None
    } else {
        Some(RotatingContactFrontier3d {
            boxes: boxes.to_vec(),
            time: SampledContactTime3d::ZERO,
            contacts: contacts.values().cloned().collect(),
            remaining_numerator: 1,
        })
    };
    Ok(CurrentContactFrontierResult3d {
        frontier,
        candidate_pairs,
        recomputed_contacts,
    })
}

#[cfg(test)]
fn current_contact_frontier(
    boxes: &[RigidBox3d],
) -> Result<Option<RotatingContactFrontier3d>, RotatingContactFrontierError3d> {
    let mut broad_phase = RotatingBroadPhase3d::default();
    let zero_time = RigidBoxFreeFlightConfig3d::new(crate::Vec3i::ZERO, 0, 1);
    broad_phase.candidate_pairs(boxes, zero_time)?;
    let active = boxes
        .iter()
        .map(|rigid_box| rigid_box.body().id())
        .collect::<Vec<_>>();
    let mut contacts = BTreeMap::new();
    let mut body_index = RotatingContactResponseScratch3d::default();
    body_index.ensure_body_index(boxes);
    Ok(refresh_current_contacts_for_changed_bodies(
        boxes,
        &active,
        &mut contacts,
        &mut broad_phase,
        &body_index,
    )?
    .frontier)
}

fn validate_config(
    config: RepeatedRotatingEventConfig3d,
) -> Result<(), RepeatedRotatingEventError3d> {
    if config.max_events == 0 {
        return Err(RepeatedRotatingEventError3d::ZeroEventLimit);
    }
    if config.max_events > MAX_REPEATED_ROTATING_EVENTS {
        return Err(RepeatedRotatingEventError3d::EventLimit(
            MAX_REPEATED_ROTATING_EVENTS,
        ));
    }
    match config.search.free_flight.exact_timestep() {
        Ok(_) => Ok(()),
        Err(RigidBoxFreeFlightError3d::NegativeTimestepNumerator(value)) => Err(
            RepeatedRotatingEventError3d::NegativeTimestepNumerator(value),
        ),
        Err(RigidBoxFreeFlightError3d::NonPositiveTimestepDenominator(value)) => {
            Err(RepeatedRotatingEventError3d::NonPositiveTimestepDenominator(value))
        }
        Err(_) => Err(RepeatedRotatingEventError3d::RatioTooLarge),
    }
}

fn search_with_free_flight(
    search: RotatingContactSearchConfig3d,
    free_flight: RigidBoxFreeFlightConfig3d,
) -> RotatingContactSearchConfig3d {
    RotatingContactSearchConfig3d {
        free_flight,
        ..search
    }
}

fn scale_remaining_time(
    current: RigidBoxFreeFlightConfig3d,
    remaining_numerator: u32,
    event_time: SampledContactTime3d,
) -> Result<RigidBoxFreeFlightConfig3d, RepeatedRotatingEventError3d> {
    if event_time.denominator == 0 || remaining_numerator > event_time.denominator {
        return Err(RepeatedRotatingEventError3d::InvalidRemainder(event_time));
    }

    current
        .scaled_fraction(remaining_numerator, event_time.denominator)
        .map_err(|error| match error {
            RigidBoxFreeFlightError3d::NegativeTimestepNumerator(value) => {
                RepeatedRotatingEventError3d::NegativeTimestepNumerator(value)
            }
            RigidBoxFreeFlightError3d::NonPositiveTimestepDenominator(value) => {
                RepeatedRotatingEventError3d::NonPositiveTimestepDenominator(value)
            }
            _ => RepeatedRotatingEventError3d::RatioTooLarge,
        })
}

#[cfg(test)]
mod tests {
    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, MATERIAL_SCALE, Material, Orientation3d,
        RigidBody, RigidBox3d, RigidBoxFreeFlightConfig3d, RotatingContactSearchConfig3d, Vec3i,
    };

    use super::{
        RepeatedRotatingEventConfig3d, RepeatedRotatingEventError3d,
        advance_repeated_rotating_events, current_contact_frontier, scale_remaining_time,
    };

    fn dynamic(id: u64, position: Vec3i, velocity: Vec3i, material: Material) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::dynamic(BodyId(id), position, velocity, Vec3i::new(1, 1, 1))
                .with_material(material),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid dynamic box")
    }

    fn fixed(id: u64, position: Vec3i, material: Material) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::fixed(BodyId(id), position, Vec3i::new(1, 1, 1)).with_material(material),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid fixed box")
    }

    fn config(max_events: u16) -> RepeatedRotatingEventConfig3d {
        RepeatedRotatingEventConfig3d::new(
            RotatingContactSearchConfig3d::new(
                RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1),
                32,
                4,
            ),
            8,
            max_events,
        )
    }

    #[test]
    fn no_event_leaves_the_full_tail_unconsumed() {
        let boxes = [
            dynamic(1, Vec3i::new(-100, 0, 0), Vec3i::ZERO, Material::new(0)),
            fixed(2, Vec3i::new(100, 0, 0), Material::new(0)),
        ];
        let advance =
            advance_repeated_rotating_events(&boxes, config(8)).expect("valid no-event advance");

        assert_eq!(advance.boxes, boxes);
        assert!(advance.events.is_empty());
        assert_eq!(advance.remaining, config(8).search.free_flight);
    }

    #[test]
    fn current_frontier_does_not_look_ahead_into_future_motion() {
        let boxes = [
            dynamic(1, Vec3i::ZERO, Vec3i::new(100, 0, 0), Material::new(0)),
            fixed(2, Vec3i::new(20, 0, 0), Material::new(0)),
        ];

        assert!(
            current_contact_frontier(&boxes)
                .expect("current contact query")
                .is_none()
        );
    }

    #[test]
    fn time_zero_event_is_resolved_without_consuming_the_tail() {
        let boxes = [
            dynamic(1, Vec3i::ZERO, Vec3i::new(60, 0, 0), Material::new(0)),
            fixed(2, Vec3i::new(2, 0, 0), Material::new(0)),
        ];
        let advance = advance_repeated_rotating_events(&boxes, config(8))
            .expect("valid initial-contact advance");

        assert_eq!(advance.events.len(), 1);
        assert_eq!(advance.events[0].time, crate::SampledContactTime3d::ZERO);
        assert_eq!(advance.boxes[0].body().velocity().x, 0);
        assert_eq!(advance.remaining, config(8).search.free_flight);
    }

    #[test]
    fn multiple_wall_impacts_are_resolved_in_one_requested_interval() {
        let elastic = Material::new(MATERIAL_SCALE);
        let boxes = [
            fixed(1, Vec3i::new(-20, 0, 0), elastic),
            dynamic(2, Vec3i::ZERO, Vec3i::new(100, 0, 0), elastic),
            fixed(3, Vec3i::new(20, 0, 0), elastic),
        ];
        let advance = advance_repeated_rotating_events(&boxes, config(8))
            .expect("valid repeated impact advance");

        assert!(advance.events.len() >= 2);
        assert_eq!(advance.events[0].contacts[0].pair.left, BodyId(2));
        assert_eq!(advance.events[0].contacts[0].pair.right, BodyId(3));
        assert_eq!(advance.events[1].contacts[0].pair.left, BodyId(1));
        assert_eq!(advance.events[1].contacts[0].pair.right, BodyId(2));
    }

    #[test]
    fn event_limit_fails_only_when_a_real_additional_event_exists() {
        let elastic = Material::new(MATERIAL_SCALE);
        let boxes = [
            fixed(1, Vec3i::new(-20, 0, 0), elastic),
            dynamic(2, Vec3i::ZERO, Vec3i::new(100, 0, 0), elastic),
            fixed(3, Vec3i::new(20, 0, 0), elastic),
        ];

        assert_eq!(
            advance_repeated_rotating_events(&boxes, config(1)),
            Err(RepeatedRotatingEventError3d::EventLimit(1))
        );
    }

    #[test]
    fn exact_remaining_time_reduction_preserves_rational_value() {
        let current = RigidBoxFreeFlightConfig3d::new(Vec3i::new(0, -10, 0), 2, 3);
        assert_eq!(
            scale_remaining_time(
                current,
                5,
                crate::SampledContactTime3d {
                    numerator: 3,
                    denominator: 8,
                },
            )
            .expect("representable scaled remainder"),
            RigidBoxFreeFlightConfig3d::new(Vec3i::new(0, -10, 0), 5, 12)
        );
        assert_eq!(
            scale_remaining_time(
                current,
                0,
                crate::SampledContactTime3d {
                    numerator: 1,
                    denominator: 1,
                },
            )
            .expect("zero remainder"),
            RigidBoxFreeFlightConfig3d::new(Vec3i::new(0, -10, 0), 0, 1)
        );
    }

    #[test]
    fn reducible_remainder_is_normalized_before_exact_composition() {
        let current = RigidBoxFreeFlightConfig3d::new_wide(Vec3i::ZERO, i128::MAX, 1);
        let scaled = scale_remaining_time(
            current,
            256,
            crate::SampledContactTime3d {
                numerator: 256,
                denominator: 512,
            },
        )
        .expect("reducible sampled remainder must not manufacture overflow");

        assert_eq!(scaled.timestep_i128(), Some((i128::MAX, 2)));
    }

    #[test]
    fn denominator_growth_stays_exact_beyond_i32() {
        let mut remaining = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 60);
        let event = crate::SampledContactTime3d {
            numerator: 1,
            denominator: 512,
        };
        let mut expected_numerator = 1_u128;
        let mut expected_denominator = 60_u128;
        for _ in 0..8 {
            remaining = scale_remaining_time(remaining, 511, event)
                .expect("exact widened denominator composition");
            expected_numerator = expected_numerator.checked_mul(511).expect("test numerator");
            expected_denominator = expected_denominator
                .checked_mul(512)
                .expect("test denominator");
            let (numerator, denominator) = remaining
                .timestep_i128()
                .expect("eight sampled factors still fit i128");
            assert_eq!(numerator as u128, expected_numerator);
            assert_eq!(denominator as u128, expected_denominator);
        }
        assert!(
            remaining.timestep_i128().expect("eight factors fit i128").1 > i128::from(i32::MAX)
        );
    }

    #[test]
    fn exact_remaining_time_survives_beyond_i128() {
        let mut remaining = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 60);
        let event = crate::SampledContactTime3d {
            numerator: 1,
            denominator: 512,
        };
        for _ in 0..20 {
            remaining = scale_remaining_time(remaining, 511, event)
                .expect("bounded repeated-event exact ratio");
        }
        assert!(remaining.timestep_i128().is_none());
        assert!(!remaining.timestep_is_zero());
    }

    #[test]
    fn repeated_event_advance_is_bit_for_bit_repeatable() {
        let elastic = Material::new(MATERIAL_SCALE);
        let boxes = [
            fixed(1, Vec3i::new(-20, 0, 0), elastic),
            dynamic(2, Vec3i::ZERO, Vec3i::new(100, 0, 0), elastic),
            fixed(3, Vec3i::new(20, 0, 0), elastic),
        ];

        let first = advance_repeated_rotating_events(&boxes, config(8)).expect("first advance");
        let second = advance_repeated_rotating_events(&boxes, config(8)).expect("second advance");
        assert_eq!(first, second);
    }

    #[test]
    fn zero_event_limit_fails_closed() {
        let boxes = [
            dynamic(1, Vec3i::ZERO, Vec3i::ZERO, Material::new(0)),
            fixed(2, Vec3i::new(2, 0, 0), Material::new(0)),
        ];
        assert_eq!(
            advance_repeated_rotating_events(&boxes, config(0)),
            Err(RepeatedRotatingEventError3d::ZeroEventLimit)
        );
    }
}
