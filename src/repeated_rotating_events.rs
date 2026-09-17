use std::{
    collections::{BTreeMap, BTreeSet},
    error::Error,
    fmt,
};

use crate::{
    RigidBox3d, RigidBoxFreeFlightConfig3d, RigidBoxFreeFlightError3d, RotatingContactFrontier3d,
    RotatingContactFrontierError3d, RotatingContactResponseError3d, RotatingContactSearchConfig3d,
    RotatingContactSearchHit3d, SampledContactTime3d, obb_contact_seed,
    sample_rigid_box_free_flight,
};
use crate::{
    rotating_broad_phase::RotatingBroadPhase3d,
    rotating_contact_response::{
        RotatingContactResponseScratch3d,
        resolve_rotating_contact_frontier_with_activity_and_scratch,
    },
    rotating_contact_search::sampled_rotating_contact_search_with_broad_phase,
    rotating_recontact_search::sampled_rotating_recontact_search_with_broad_phase,
};

pub const MAX_REPEATED_ROTATING_EVENTS: u16 = 64;

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

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RepeatedRotatingEventAdvance3d {
    pub boxes: Vec<RigidBox3d>,
    pub events: Vec<RotatingResolvedEvent3d>,
    pub remaining: RigidBoxFreeFlightConfig3d,
    pub work: RepeatedRotatingEventWorkStats3d,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct RepeatedRotatingEventProgress3d {
    pub events: Vec<RotatingResolvedEvent3d>,
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

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct BodyMotionDelta3d {
    world_index: usize,
    position: Option<crate::Vec3i>,
    velocity: Option<crate::Vec3i>,
    orientation: Option<crate::Orientation3d>,
    angular_velocity: Option<crate::AngularVelocity3d>,
}

impl BodyMotionDelta3d {
    fn between(world_index: usize, before: &RigidBox3d, after: &RigidBox3d) -> Option<Self> {
        let delta = Self {
            world_index,
            position: (before.body.position != after.body.position).then_some(after.body.position),
            velocity: (before.body.velocity != after.body.velocity).then_some(after.body.velocity),
            orientation: (before.angular.orientation != after.angular.orientation)
                .then_some(after.angular.orientation),
            angular_velocity: (before.angular.angular_velocity != after.angular.angular_velocity)
                .then_some(after.angular.angular_velocity),
        };
        (delta.position.is_some()
            || delta.velocity.is_some()
            || delta.orientation.is_some()
            || delta.angular_velocity.is_some())
        .then_some(delta)
    }

    fn apply(self, boxes: &mut [RigidBox3d]) {
        let rigid_box = boxes
            .get_mut(self.world_index)
            .expect("motion delta index came from the current world layout");
        if let Some(position) = self.position {
            rigid_box.body.position = position;
        }
        if let Some(velocity) = self.velocity {
            rigid_box.body.velocity = velocity;
        }
        if let Some(orientation) = self.orientation {
            rigid_box.angular.orientation = orientation;
        }
        if let Some(angular_velocity) = self.angular_velocity {
            rigid_box.angular.angular_velocity = angular_velocity;
        }
    }
}

/// Advances through sampled events and returns an owned result for standalone callers.
///
/// The public boundary intentionally materializes one caller-owned working world because the borrowed input
/// must remain unchanged. Internal world stepping uses [`advance_repeated_rotating_events_with_broad_phase`]
/// directly on its existing mutable working buffer and therefore does not create a second full-world copy.
pub fn advance_repeated_rotating_events(
    boxes: &[RigidBox3d],
    config: RepeatedRotatingEventConfig3d,
) -> Result<RepeatedRotatingEventAdvance3d, RepeatedRotatingEventError3d> {
    let mut broad_phase = RotatingBroadPhase3d::default();
    let mut state = boxes.to_vec();
    let progress =
        advance_repeated_rotating_events_with_broad_phase(&mut state, config, &mut broad_phase)?;
    Ok(RepeatedRotatingEventAdvance3d {
        boxes: state,
        events: progress.events,
        remaining: progress.remaining,
        work: progress.work,
    })
}

/// Advances sampled events in place on a caller-owned working world.
///
/// Temporal search remains read-only. Once a hit is admitted, free flight is sampled exactly once into a
/// compact changed-field delta batch and committed to `boxes`. The broad phase is then synchronized at zero
/// time against that current state, and all equal-time contacts are exact-filtered there. Response therefore
/// receives an evidence-only zero-time frontier and stages only the active contact island. The function never
/// clones the complete input world; callers choose the transactional boundary appropriate to their authority.
pub(crate) fn advance_repeated_rotating_events_with_broad_phase(
    boxes: &mut [RigidBox3d],
    config: RepeatedRotatingEventConfig3d,
    broad_phase: &mut RotatingBroadPhase3d,
) -> Result<RepeatedRotatingEventProgress3d, RepeatedRotatingEventError3d> {
    validate_config(config)?;

    let mut work = RepeatedRotatingEventWorkStats3d::default();
    let mut response_scratch = RotatingContactResponseScratch3d::default();
    let mut remaining = config.search.free_flight;
    response_scratch.ensure_body_index(boxes);

    let first_search = search_with_free_flight(config.search, remaining);
    let Some(first_hit) =
        sampled_rotating_contact_search_with_broad_phase(boxes, first_search, broad_phase)
            .map_err(RotatingContactFrontierError3d::from)?
    else {
        return Ok(RepeatedRotatingEventProgress3d {
            events: Vec::new(),
            remaining,
            work,
        });
    };
    advance_state_to_time(boxes, first_search.free_flight, first_hit.time)?;
    let mut event_time = first_hit.time;
    let mut frontier =
        current_frontier_from_admitted_hit(boxes, first_hit, broad_phase, &response_scratch)?;

    let mut events = Vec::new();
    loop {
        if events.len() >= usize::from(config.max_events) {
            return Err(RepeatedRotatingEventError3d::EventLimit(config.max_events));
        }

        let (response, modified_body_ids) =
            resolve_rotating_contact_frontier_with_activity_and_scratch(
                boxes,
                &frontier,
                config.solver_passes,
                &mut response_scratch,
            )?;
        remaining = scale_remaining_time(
            remaining,
            event_remaining_numerator(event_time)?,
            event_time,
        )?;
        let response_contacts = response.contacts;
        let response_passes = response.passes_used;
        work.event_response_passes = work
            .event_response_passes
            .saturating_add(u64::from(response_passes));
        stabilize_current_contacts(
            boxes,
            config.solver_passes,
            broad_phase,
            &modified_body_ids,
            &response_contacts,
            &mut response_scratch,
            &mut work,
        )?;
        events.push(RotatingResolvedEvent3d {
            time: event_time,
            contacts: response_contacts,
            response_passes,
        });

        if remaining.timestep_is_zero() {
            break;
        }

        let next_search = search_with_free_flight(config.search, remaining);
        let Some(next_hit) =
            sampled_rotating_recontact_search_with_broad_phase(boxes, next_search, broad_phase)
                .map_err(RotatingContactFrontierError3d::from)?
        else {
            break;
        };
        advance_state_to_time(boxes, next_search.free_flight, next_hit.time)?;
        event_time = next_hit.time;
        frontier =
            current_frontier_from_admitted_hit(boxes, next_hit, broad_phase, &response_scratch)?;
    }

    Ok(RepeatedRotatingEventProgress3d {
        events,
        remaining,
        work,
    })
}

fn advance_state_to_time(
    boxes: &mut [RigidBox3d],
    free_flight: RigidBoxFreeFlightConfig3d,
    time: SampledContactTime3d,
) -> Result<(), RepeatedRotatingEventError3d> {
    if time == SampledContactTime3d::ZERO {
        return Ok(());
    }

    let mut deltas = Vec::new();
    for (world_index, rigid_box) in boxes.iter().enumerate() {
        let sampled =
            sample_rigid_box_free_flight(rigid_box, free_flight, time.numerator, time.denominator)
                .map_err(RotatingContactFrontierError3d::from)?;
        if let Some(delta) = BodyMotionDelta3d::between(world_index, rigid_box, &sampled) {
            deltas.push(delta);
        }
    }
    for delta in deltas {
        delta.apply(boxes);
    }
    Ok(())
}

fn current_frontier_from_admitted_hit(
    boxes: &[RigidBox3d],
    admitted: RotatingContactSearchHit3d,
    broad_phase: &mut RotatingBroadPhase3d,
    body_index: &RotatingContactResponseScratch3d,
) -> Result<RotatingContactFrontier3d, RotatingContactFrontierError3d> {
    let zero_time = RigidBoxFreeFlightConfig3d::new(crate::Vec3i::ZERO, 0, 1);
    let candidates = broad_phase.candidate_pairs(boxes, zero_time)?;
    let mut contacts = Vec::new();

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
        contacts.push(RotatingContactSearchHit3d {
            time: admitted.time,
            pair,
            contact,
        });
    }

    let reconstructed = contacts
        .iter()
        .find(|contact| contact.pair == admitted.pair)
        .ok_or(RotatingContactFrontierError3d::EarliestContactMissing(
            admitted.pair,
        ))?;
    if reconstructed.contact != admitted.contact {
        return Err(RotatingContactFrontierError3d::EarliestContactChanged(
            admitted.pair,
        ));
    }

    Ok(RotatingContactFrontier3d {
        free_flight: zero_time,
        time: SampledContactTime3d::ZERO,
        contacts,
        remaining_numerator: 0,
    })
}

fn event_remaining_numerator(
    time: SampledContactTime3d,
) -> Result<u32, RepeatedRotatingEventError3d> {
    if time.denominator == 0 || time.numerator > time.denominator {
        return Err(RepeatedRotatingEventError3d::InvalidRemainder(time));
    }
    time.denominator
        .checked_sub(time.numerator)
        .ok_or(RepeatedRotatingEventError3d::InvalidRemainder(time))
}

/// Stabilizes complete connected contact islands reached from bodies changed by the event response.
///
/// Each event response has already solved all simultaneous contacts once. Stabilization keeps the cached
/// zero-time contact graph so every affected island is still solved as one simultaneous frontier on every pass,
/// preserving the existing convergence semantics. Contacts in components disconnected from every body changed
/// by the previous pass are not re-solved. This removes unrelated work without turning island propagation into
/// one-contact-edge-per-pass behavior.
fn stabilize_current_contacts(
    boxes: &mut [RigidBox3d],
    solver_passes: u8,
    broad_phase: &mut RotatingBroadPhase3d,
    initial_active: &[crate::BodyId],
    initial_contacts: &[RotatingContactSearchHit3d],
    response_scratch: &mut RotatingContactResponseScratch3d,
    work: &mut RepeatedRotatingEventWorkStats3d,
) -> Result<(), RepeatedRotatingEventError3d> {
    response_scratch.ensure_body_index(boxes);
    let mut active = initial_active.to_vec();
    active.sort_unstable();
    active.dedup();
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
            boxes,
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
        work.stabilization_passes = work.stabilization_passes.saturating_add(1);
        let (_, modified_body_ids) = resolve_rotating_contact_frontier_with_activity_and_scratch(
            boxes,
            &frontier,
            1,
            response_scratch,
        )?;
        active = modified_body_ids;
        active.sort_unstable();
        active.dedup();
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
    Ok(())
}

#[derive(Clone, Debug)]
struct CurrentContactFrontierResult3d {
    frontier: Option<RotatingContactFrontier3d>,
    candidate_pairs: usize,
    recomputed_contacts: usize,
}

fn connected_contacts_for_active(
    contacts: &BTreeMap<crate::RotationalSweepPair3d, RotatingContactSearchHit3d>,
    active: &[crate::BodyId],
) -> Vec<RotatingContactSearchHit3d> {
    let mut reachable = active.iter().copied().collect::<BTreeSet<_>>();
    loop {
        let mut changed = false;
        for pair in contacts.keys() {
            if reachable.contains(&pair.left) || reachable.contains(&pair.right) {
                changed |= reachable.insert(pair.left);
                changed |= reachable.insert(pair.right);
            }
        }
        if !changed {
            break;
        }
    }
    contacts
        .iter()
        .filter(|(pair, _)| reachable.contains(&pair.left) || reachable.contains(&pair.right))
        .map(|(_, contact)| *contact)
        .collect()
}

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

    let island_contacts = connected_contacts_for_active(contacts, active);
    let frontier = if island_contacts.is_empty() {
        None
    } else {
        Some(RotatingContactFrontier3d {
            free_flight: RigidBoxFreeFlightConfig3d::new(crate::Vec3i::ZERO, 0, 1),
            time: SampledContactTime3d::ZERO,
            contacts: island_contacts,
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
    use std::{collections::BTreeMap, hint::black_box, time::Instant};

    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, MATERIAL_SCALE, Material, Orientation3d,
        RigidBody, RigidBox3d, RigidBoxFreeFlightConfig3d, RotatingContactSearchConfig3d, Vec3i,
        rotating_contact_response::RotatingContactResponseScratch3d,
    };

    use super::{
        RepeatedRotatingEventConfig3d, RepeatedRotatingEventError3d, RotatingBroadPhase3d,
        advance_repeated_rotating_events, advance_repeated_rotating_events_with_broad_phase,
        current_contact_frontier, refresh_current_contacts_for_changed_bodies,
        scale_remaining_time,
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
    fn in_place_advance_matches_owned_public_result() {
        let elastic = Material::new(MATERIAL_SCALE);
        let boxes = vec![
            fixed(1, Vec3i::new(-20, 0, 0), elastic),
            dynamic(2, Vec3i::ZERO, Vec3i::new(100, 0, 0), elastic),
            fixed(3, Vec3i::new(20, 0, 0), elastic),
        ];
        let expected = advance_repeated_rotating_events(&boxes, config(8)).expect("owned advance");
        let mut actual_boxes = boxes;
        let mut broad_phase = RotatingBroadPhase3d::default();
        let actual = advance_repeated_rotating_events_with_broad_phase(
            &mut actual_boxes,
            config(8),
            &mut broad_phase,
        )
        .expect("in-place advance");

        assert_eq!(actual_boxes, expected.boxes);
        assert_eq!(actual.events, expected.events);
        assert_eq!(actual.remaining, expected.remaining);
        assert_eq!(actual.work, expected.work);
    }

    #[test]
    fn changed_contact_query_excludes_disconnected_stable_islands() {
        let material = Material::new(0);
        let boxes = [
            dynamic(1, Vec3i::ZERO, Vec3i::ZERO, material),
            fixed(2, Vec3i::new(2, 0, 0), material),
            dynamic(3, Vec3i::new(100, 0, 0), Vec3i::ZERO, material),
            fixed(4, Vec3i::new(102, 0, 0), material),
        ];
        let mut broad_phase = RotatingBroadPhase3d::default();
        let zero_time = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 0, 1);
        broad_phase
            .candidate_pairs(&boxes, zero_time)
            .expect("prime current broad phase");
        let all_contacts = current_contact_frontier(&boxes)
            .expect("all current contacts")
            .expect("two touching islands");
        assert_eq!(all_contacts.contacts.len(), 2);
        let mut contacts = all_contacts
            .contacts
            .into_iter()
            .map(|contact| (contact.pair, contact))
            .collect::<BTreeMap<_, _>>();
        let mut body_index = RotatingContactResponseScratch3d::default();
        body_index.ensure_body_index(&boxes);

        let current = refresh_current_contacts_for_changed_bodies(
            &boxes,
            &[BodyId(1)],
            &mut contacts,
            &mut broad_phase,
            &body_index,
        )
        .expect("active-island contacts");
        let frontier = current.frontier.expect("body 1 touches body 2");

        assert_eq!(frontier.contacts.len(), 1);
        assert_eq!(frontier.contacts[0].pair.left, BodyId(1));
        assert_eq!(frontier.contacts[0].pair.right, BodyId(2));
    }

    #[test]
    #[ignore = "release performance evidence"]
    fn repeated_event_working_world_handoff_benchmark() {
        const BODY_COUNT: usize = 4_096;
        const ITERATIONS: usize = 512;
        let boxes = (0..BODY_COUNT)
            .map(|index| {
                fixed(
                    index as u64 + 1,
                    Vec3i::new(index as i32 * 8, 0, 0),
                    Material::new(0),
                )
            })
            .collect::<Vec<_>>();

        let clone_started = Instant::now();
        let mut checksum = 0_u64;
        for _ in 0..ITERATIONS {
            let duplicate = black_box(&boxes).to_vec();
            checksum = checksum.saturating_add(
                duplicate
                    .last()
                    .map(|body| body.body().id().0)
                    .unwrap_or_default(),
            );
            black_box(duplicate);
        }
        let clone_elapsed = clone_started.elapsed();

        let mut working = boxes;
        let in_place_started = Instant::now();
        let mut in_place_checksum = 0_u64;
        for _ in 0..ITERATIONS {
            in_place_checksum = in_place_checksum.saturating_add(
                black_box(&mut working)
                    .last()
                    .map(|body| body.body().id().0)
                    .unwrap_or_default(),
            );
        }
        let in_place_elapsed = in_place_started.elapsed();

        assert_eq!(checksum, in_place_checksum);
        let speedup = clone_elapsed.as_secs_f64() / in_place_elapsed.as_secs_f64();
        println!(
            "repeated-event working-world handoff {BODY_COUNT} bodies × {ITERATIONS}: duplicate={clone_elapsed:?}, in_place={in_place_elapsed:?}, speedup={speedup:.2}x, full_world_body_clones_per_frame=1->0"
        );
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
