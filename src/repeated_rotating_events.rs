use std::{
    collections::{BTreeMap, BTreeSet},
    error::Error,
    fmt,
};

use crate::{
    RigidBox3d, RigidBoxFreeFlightConfig3d, RigidBoxFreeFlightError3d, RotatingContactFrontier3d,
    RotatingContactFrontierError3d, RotatingContactResponseError3d, RotatingContactSearchConfig3d,
    RotatingContactSearchHit3d, RotationalSweepPair3d, SampledContactTime3d, obb_contact_seed,
    resolve_rotating_contact_frontier,
};
use crate::{
    rotating_broad_phase::RotatingBroadPhase3d,
    rotating_contact_frontier::{
        earliest_rotating_contact_frontier_with_broad_phase,
        next_rotating_contact_frontier_with_persistent_pairs_and_broad_phase,
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
/// the current zero-time contact set and applies one simultaneous response pass. Every resolved or stabilized
/// pair remains in contact history, preserving the existing anti-churn behavior.
///
/// Historical state does not permanently override exact geometry. When a historical pair is already clear
/// at one segment start, shares a body with a later selected frontier, and is still clear at that exact
/// frontier before response changes motion, the pair receives a **one-segment start-clear exemption** after
/// that response. Its history marker is only suspended while selecting the immediately following frontier,
/// then restored regardless of the search result. This lets an unrelated shared-body response expose a real
/// first-cell collision without converting solver projection or a transient gap into durable clearance.
/// Positive coarse clearance observed by the ordinary re-contact search remains the only mechanism that can
/// actually delete a persistent history marker.
///
/// Current-frontier discovery reuses the persistent conservative broad phase at a zero timestep, then
/// exact-filters every candidate with current OBB geometry. Later positive events are selected by the same
/// sampled search and remain bounded sampled rotational handling, not analytic CCD.
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

    let mut remaining = config.search.free_flight;
    let first_search = search_with_free_flight(config.search, remaining);
    let Some(first_frontier) =
        earliest_rotating_contact_frontier_with_broad_phase(boxes, first_search, broad_phase)?
    else {
        return Ok(RepeatedRotatingEventAdvance3d {
            boxes: boxes.to_vec(),
            events: Vec::new(),
            remaining,
        });
    };

    let mut state;
    let mut events = Vec::new();
    let mut frontier = first_frontier;
    let mut persistent_pairs = BTreeSet::new();
    let mut start_clear_history_candidates = BTreeSet::new();

    loop {
        if events.len() >= usize::from(config.max_events) {
            return Err(RepeatedRotatingEventError3d::EventLimit(config.max_events));
        }

        let next_segment_exemptions = proven_shared_history_exemptions(
            &persistent_pairs,
            &start_clear_history_candidates,
            &frontier,
        )?;

        let response = resolve_rotating_contact_frontier(frontier, config.solver_passes)?;
        remaining = scale_remaining_time(remaining, response.remaining_numerator, response.time)?;
        let response_time = response.time;
        let response_contacts = response.contacts;
        let response_passes = response.passes_used;
        let mut observed_pairs = response_contacts
            .iter()
            .map(|contact| contact.pair)
            .collect::<BTreeSet<_>>();
        let (stabilized, stabilization_pairs) =
            stabilize_current_contacts(response.boxes, config.solver_passes, broad_phase)?;
        observed_pairs.extend(stabilization_pairs);
        state = stabilized;
        persistent_pairs.extend(observed_pairs);
        events.push(RotatingResolvedEvent3d {
            time: response_time,
            contacts: response_contacts,
            response_passes,
        });

        if remaining.timestep_is_zero() {
            break;
        }

        let next_candidates = start_clear_history(&persistent_pairs, &state)?;
        let next_search = search_with_free_flight(config.search, remaining);
        let next = next_frontier_with_one_shot_history_exemptions(
            &state,
            next_search,
            &mut persistent_pairs,
            &next_segment_exemptions,
            broad_phase,
        )?;
        let Some(next) = next else {
            break;
        };
        start_clear_history_candidates = next_candidates;
        frontier = next;
    }

    Ok(RepeatedRotatingEventAdvance3d {
        boxes: state,
        events,
        remaining,
    })
}

fn stabilize_current_contacts(
    mut boxes: Vec<RigidBox3d>,
    solver_passes: u8,
    broad_phase: &mut RotatingBroadPhase3d,
) -> Result<(Vec<RigidBox3d>, BTreeSet<RotationalSweepPair3d>), RepeatedRotatingEventError3d> {
    let mut persistent_pairs = BTreeSet::new();
    for _ in 0..solver_passes {
        let Some(frontier) = current_contact_frontier_with_broad_phase(&boxes, broad_phase)? else {
            break;
        };
        persistent_pairs.extend(frontier.contacts.iter().map(|contact| contact.pair));
        let response = resolve_rotating_contact_frontier(frontier, 1)?;
        if response.boxes == boxes {
            break;
        }
        boxes = response.boxes;
    }
    Ok((boxes, persistent_pairs))
}

fn start_clear_history(
    persistent_pairs: &BTreeSet<RotationalSweepPair3d>,
    boxes: &[RigidBox3d],
) -> Result<BTreeSet<RotationalSweepPair3d>, RotatingContactFrontierError3d> {
    let indices = body_indices(boxes);
    let mut clear_pairs = BTreeSet::new();
    for pair in persistent_pairs {
        if !pair_is_touching(*pair, boxes, &indices)? {
            clear_pairs.insert(*pair);
        }
    }
    Ok(clear_pairs)
}

fn proven_shared_history_exemptions(
    persistent_pairs: &BTreeSet<RotationalSweepPair3d>,
    candidates: &BTreeSet<RotationalSweepPair3d>,
    frontier: &RotatingContactFrontier3d,
) -> Result<BTreeSet<RotationalSweepPair3d>, RotatingContactFrontierError3d> {
    if candidates.is_empty() || frontier.contacts.is_empty() {
        return Ok(BTreeSet::new());
    }

    let mut active_bodies = BTreeSet::new();
    for contact in &frontier.contacts {
        active_bodies.insert(contact.pair.left);
        active_bodies.insert(contact.pair.right);
    }
    let indices = body_indices(&frontier.boxes);
    let mut exemptions = BTreeSet::new();

    for pair in candidates {
        if !persistent_pairs.contains(pair)
            || (!active_bodies.contains(&pair.left) && !active_bodies.contains(&pair.right))
        {
            continue;
        }
        if !pair_is_touching(*pair, &frontier.boxes, &indices)? {
            exemptions.insert(*pair);
        }
    }
    Ok(exemptions)
}

fn next_frontier_with_one_shot_history_exemptions(
    boxes: &[RigidBox3d],
    config: RotatingContactSearchConfig3d,
    persistent_pairs: &mut BTreeSet<RotationalSweepPair3d>,
    exemptions: &BTreeSet<RotationalSweepPair3d>,
    broad_phase: &mut RotatingBroadPhase3d,
) -> Result<Option<RotatingContactFrontier3d>, RotatingContactFrontierError3d> {
    let suspended = exemptions
        .iter()
        .copied()
        .filter(|pair| persistent_pairs.remove(pair))
        .collect::<Vec<_>>();
    let result = next_rotating_contact_frontier_with_persistent_pairs_and_broad_phase(
        boxes,
        config,
        persistent_pairs,
        broad_phase,
    );
    for pair in suspended {
        persistent_pairs.insert(pair);
    }
    result
}

fn body_indices(boxes: &[RigidBox3d]) -> BTreeMap<crate::BodyId, usize> {
    boxes
        .iter()
        .enumerate()
        .map(|(index, rigid_box)| (rigid_box.body().id(), index))
        .collect()
}

fn pair_is_touching(
    pair: RotationalSweepPair3d,
    boxes: &[RigidBox3d],
    indices: &BTreeMap<crate::BodyId, usize>,
) -> Result<bool, RotatingContactFrontierError3d> {
    let left_index = *indices
        .get(&pair.left)
        .ok_or(RotatingContactFrontierError3d::MissingBody(pair.left))?;
    let right_index = *indices
        .get(&pair.right)
        .ok_or(RotatingContactFrontierError3d::MissingBody(pair.right))?;
    Ok(obb_contact_seed(
        boxes[left_index].oriented_box(),
        boxes[right_index].oriented_box(),
    )?
    .is_some())
}

fn current_contact_frontier_with_broad_phase(
    boxes: &[RigidBox3d],
    broad_phase: &mut RotatingBroadPhase3d,
) -> Result<Option<RotatingContactFrontier3d>, RotatingContactFrontierError3d> {
    let zero_time = RigidBoxFreeFlightConfig3d::new(crate::Vec3i::ZERO, 0, 1);
    let candidates = broad_phase.candidate_pairs(boxes, zero_time)?;
    if candidates.is_empty() {
        return Ok(None);
    }

    let indices = boxes
        .iter()
        .enumerate()
        .map(|(index, rigid_box)| (rigid_box.body().id(), index))
        .collect::<BTreeMap<_, _>>();
    let mut contacts = Vec::new();
    for pair in candidates {
        let left_index = *indices
            .get(&pair.left)
            .ok_or(RotatingContactFrontierError3d::MissingBody(pair.left))?;
        let right_index = *indices
            .get(&pair.right)
            .ok_or(RotatingContactFrontierError3d::MissingBody(pair.right))?;
        let Some(contact) = obb_contact_seed(
            boxes[left_index].oriented_box(),
            boxes[right_index].oriented_box(),
        )?
        else {
            continue;
        };
        contacts.push(RotatingContactSearchHit3d {
            time: SampledContactTime3d::ZERO,
            pair,
            contact,
        });
    }

    if contacts.is_empty() {
        return Ok(None);
    }

    Ok(Some(RotatingContactFrontier3d {
        boxes: boxes.to_vec(),
        time: SampledContactTime3d::ZERO,
        contacts,
        remaining_numerator: 1,
    }))
}

#[cfg(test)]
fn current_contact_frontier(
    boxes: &[RigidBox3d],
) -> Result<Option<RotatingContactFrontier3d>, RotatingContactFrontierError3d> {
    let mut broad_phase = RotatingBroadPhase3d::default();
    current_contact_frontier_with_broad_phase(boxes, &mut broad_phase)
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
    use std::collections::BTreeSet;

    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, MATERIAL_SCALE, Material, Orientation3d,
        RigidBody, RigidBox3d, RigidBoxFreeFlightConfig3d, RotatingContactSearchConfig3d,
        RotationalSweepPair3d, SampledContactTime3d, Vec3i, next_rotating_contact_frontier,
    };

    use super::{
        RepeatedRotatingEventConfig3d, RepeatedRotatingEventError3d,
        advance_repeated_rotating_events, current_contact_frontier,
        next_frontier_with_one_shot_history_exemptions, proven_shared_history_exemptions,
        scale_remaining_time, start_clear_history,
    };
    use crate::rotating_broad_phase::RotatingBroadPhase3d;

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
        assert_eq!(advance.events[0].time, SampledContactTime3d::ZERO);
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
    fn clear_segment_start_preserves_first_cell_alternating_recontact() {
        let elastic = Material::new(MATERIAL_SCALE);
        let boxes = [
            fixed(1, Vec3i::new(-3, 0, 0), elastic),
            dynamic(2, Vec3i::ZERO, Vec3i::new(100, 0, 0), elastic),
            fixed(3, Vec3i::new(3, 0, 0), elastic),
        ];
        let narrow = RepeatedRotatingEventConfig3d::new(
            RotatingContactSearchConfig3d::new(
                RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 10),
                2,
                4,
            ),
            8,
            8,
        );
        let advance = advance_repeated_rotating_events(&boxes, narrow)
            .expect("first-cell alternating impacts remain discoverable");

        assert!(advance.events.len() >= 3);
        assert_eq!(advance.events[0].contacts[0].pair.left, BodyId(2));
        assert_eq!(advance.events[0].contacts[0].pair.right, BodyId(3));
        assert_eq!(advance.events[1].contacts[0].pair.left, BodyId(1));
        assert_eq!(advance.events[1].contacts[0].pair.right, BodyId(2));
        assert_eq!(advance.events[2].contacts[0].pair.left, BodyId(2));
        assert_eq!(advance.events[2].contacts[0].pair.right, BodyId(3));
    }

    #[test]
    fn stale_shared_history_becomes_one_segment_exemption_without_deletion() {
        let elastic = Material::new(MATERIAL_SCALE);
        let historical = RotationalSweepPair3d {
            left: BodyId(1),
            right: BodyId(2),
        };
        let boxes = [
            dynamic(1, Vec3i::new(3, 0, 0), Vec3i::new(8, 0, 0), elastic),
            fixed(2, Vec3i::ZERO, elastic),
            fixed(3, Vec3i::new(6, 0, 0), elastic),
        ];
        let search = RotatingContactSearchConfig3d::new(
            RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1),
            8,
            1,
        );
        let frontier = next_rotating_contact_frontier(&boxes, search)
            .expect("valid shared-body frontier")
            .expect("moving body should reach the other wall");
        let persistent = BTreeSet::from([historical]);
        let candidates = start_clear_history(&persistent, &boxes)
            .expect("classify genuine interval-start separation");
        let exemptions = proven_shared_history_exemptions(&persistent, &candidates, &frontier)
            .expect("prove stale history through the shared-body frontier");

        assert!(candidates.contains(&historical));
        assert!(exemptions.contains(&historical));
        assert!(persistent.contains(&historical));
    }

    #[test]
    fn one_segment_exemption_finds_first_cell_contact_and_restores_history() {
        let historical = RotationalSweepPair3d {
            left: BodyId(1),
            right: BodyId(2),
        };
        let boxes = [
            dynamic(
                1,
                Vec3i::new(-3, 0, 0),
                Vec3i::new(4, 0, 0),
                Material::new(0),
            ),
            fixed(2, Vec3i::ZERO, Material::new(0)),
        ];
        let search = RotatingContactSearchConfig3d::new(
            RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1),
            4,
            0,
        );
        let mut persistent = BTreeSet::from([historical]);
        let exemptions = BTreeSet::from([historical]);
        let mut broad_phase = RotatingBroadPhase3d::default();
        let frontier = next_frontier_with_one_shot_history_exemptions(
            &boxes,
            search,
            &mut persistent,
            &exemptions,
            &mut broad_phase,
        )
        .expect("valid one-shot history exemption")
        .expect("real first-cell collision must remain visible");

        assert_eq!(frontier.contacts[0].pair, historical);
        assert_eq!(
            frontier.time,
            SampledContactTime3d {
                numerator: 1,
                denominator: 4,
            }
        );
        assert!(persistent.contains(&historical));
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
                SampledContactTime3d {
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
                SampledContactTime3d {
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
            SampledContactTime3d {
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
        let event = SampledContactTime3d {
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
        let event = SampledContactTime3d {
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
