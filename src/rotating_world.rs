use std::{collections::BTreeMap, error::Error, fmt};

use crate::{
    ANGULAR_VELOCITY_SCALE, BodyId, BodyKind, OrientedBox3d, OrientedBoxError3d,
    RepeatedRotatingEventConfig3d, RepeatedRotatingEventError3d, RigidBox3d,
    RigidBoxFreeFlightConfig3d, RigidBoxFreeFlightError3d, RotatingContactFrontier3d,
    RotatingContactResponseError3d, RotatingContactSearchConfig3d, RotatingContactSearchHit3d,
    SampledContactTime3d, Vec3i, obb_contact_seed, oriented_box_vertices,
    resolve_rotating_contact_frontier, sample_rigid_box_free_flight,
};
use crate::{
    current_contact_query::{
        BodyCurrentContact3d, body_current_contacts_for_body, body_current_overlap_ids,
    },
    repeated_rotating_events::advance_repeated_rotating_events_with_broad_phase,
    rotating_broad_phase::{RotatingBroadPhase3d, RotatingBroadPhaseError3d},
};

const MAX_PERSISTENT_TAIL_SLICES: u32 = 1_024;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RotatingWorldConfig3d {
    pub gravity: Vec3i,
    pub sample_count: u16,
    pub refinement_steps: u8,
    pub solver_passes: u8,
    pub max_events: u16,
}

impl Default for RotatingWorldConfig3d {
    fn default() -> Self {
        Self {
            gravity: Vec3i::new(0, -10, 0),
            sample_count: 32,
            refinement_steps: 4,
            solver_passes: 8,
            max_events: 32,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RotatingWorldStepStats3d {
    pub body_count: usize,
    pub sampled_events: usize,
    pub tail_contacts: usize,
    /// Persistent-tail slices actually executed, including discarded replay work.
    pub tail_slices: u64,
    /// Persistent-tail attempts discarded because an impulse required a finer deterministic slice.
    pub tail_replays: u64,
    /// Conservative current-overlap candidates that reached exact OBB testing in the tail solver.
    pub tail_candidate_pairs: u64,
    /// Current-overlap broad-phase queries issued by the tail solver.
    pub tail_broad_phase_queries: u64,
    /// Tail broad-phase queries that rebuilt the balanced fat-AABB topology.
    pub tail_broad_phase_rebuilds: u64,
    /// Tail broad-phase queries that reused the retained fat-AABB topology.
    pub tail_broad_phase_reuses: u64,
    /// Conservative sampled-event broad-phase queries issued during this step.
    pub broad_phase_queries: u64,
    /// Sampled-event queries that had to rebuild the balanced fat-AABB topology.
    pub broad_phase_rebuilds: u64,
    /// Sampled-event queries that reused the existing fat-AABB topology and exact-filtered its candidate leaves.
    pub broad_phase_reuses: u64,
    pub broad_phase_incremental_updates: u64,
    pub broad_phase_reinserts: u64,
    pub broad_phase_rotations: u64,
    pub broad_phase_partial_queries: u64,
    pub broad_phase_partial_body_updates: u64,
    pub event_response_passes: u64,
    pub stabilization_passes: u64,
    pub stabilizations_hitting_limit: u64,
    pub stabilization_candidate_pairs: u64,
    pub stabilization_exact_contacts: u64,
    pub stabilization_active_bodies: u64,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct RotatingWorldStepReport3d {
    /// Stable `BodyId`-ordered bodies whose observable rigid-body or sleep state changed this step.
    ///
    /// Higher integration layers consume this precise delta instead of cloning every dynamic body.
    pub changed_body_ids: Vec<BodyId>,
    pub stats: RotatingWorldStepStats3d,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct TailStepStats3d {
    contacts: usize,
    slices: u64,
    replays: u64,
    candidate_pairs: u64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RotatingWorldError3d {
    DuplicateBody(BodyId),
    MissingBody(BodyId),
    FixedBodyVelocity(BodyId),
    NegativeTimestepNumerator(i32),
    NonPositiveTimestepDenominator(i32),
    PersistentTailResolutionLimit(u32),
    PersistentTailMotionUnsafe(BodyId),
    PersistentTailArithmeticOverflow(BodyId),
    Repeated(RepeatedRotatingEventError3d),
    FreeFlight(RigidBoxFreeFlightError3d),
    Contact(OrientedBoxError3d),
    Response(RotatingContactResponseError3d),
}

impl fmt::Display for RotatingWorldError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DuplicateBody(id) => write!(formatter, "duplicate rotating body {}", id.0),
            Self::MissingBody(id) => write!(formatter, "rotating body {} does not exist", id.0),
            Self::FixedBodyVelocity(id) => write!(
                formatter,
                "fixed rotating body {} cannot receive linear velocity",
                id.0
            ),
            Self::NegativeTimestepNumerator(value) => write!(
                formatter,
                "rotating world timestep numerator must be non-negative, got {value}"
            ),
            Self::NonPositiveTimestepDenominator(value) => write!(
                formatter,
                "rotating world timestep denominator must be positive, got {value}"
            ),
            Self::PersistentTailResolutionLimit(limit) => write!(
                formatter,
                "persistent rotating contact tail needs more than {limit} deterministic slices"
            ),
            Self::PersistentTailMotionUnsafe(id) => write!(
                formatter,
                "persistent rotating contact motion became unsafe for body {} at the maximum tail resolution",
                id.0
            ),
            Self::PersistentTailArithmeticOverflow(id) => write!(
                formatter,
                "persistent rotating contact motion bound overflowed for body {}",
                id.0
            ),
            Self::Repeated(error) => write!(formatter, "rotating event advance failed: {error}"),
            Self::FreeFlight(error) => {
                write!(formatter, "rotating tail free flight failed: {error}")
            }
            Self::Contact(error) => {
                write!(formatter, "rotating tail contact query failed: {error}")
            }
            Self::Response(error) => write!(formatter, "rotating tail response failed: {error}"),
        }
    }
}

impl Error for RotatingWorldError3d {}

impl From<RepeatedRotatingEventError3d> for RotatingWorldError3d {
    fn from(value: RepeatedRotatingEventError3d) -> Self {
        Self::Repeated(value)
    }
}

impl From<RigidBoxFreeFlightError3d> for RotatingWorldError3d {
    fn from(value: RigidBoxFreeFlightError3d) -> Self {
        Self::FreeFlight(value)
    }
}

impl From<OrientedBoxError3d> for RotatingWorldError3d {
    fn from(value: OrientedBoxError3d) -> Self {
        Self::Contact(value)
    }
}

impl From<RotatingContactResponseError3d> for RotatingWorldError3d {
    fn from(value: RotatingContactResponseError3d) -> Self {
        Self::Response(value)
    }
}

/// Deterministic rotating-cuboid world built from the engine's sampled OBB event pipeline.
///
/// Collision discovery remains explicitly sampled rotational handling rather than analytic rotational
/// CCD. The event pipeline resolves every admitted impact first. A contact-free exact tail still uses
/// one direct free-flight sample. When the tail begins with persistent contact, the remaining rational
/// time is instead consumed through bounded deterministic slices. Bodies participating in the current
/// persistent frontier are limited to less than their narrowest full thickness of conservative
/// center-plus-rotational motion before the next OBB stabilization pass, so a time-zero contact cannot
/// be free-flown completely through before the next constraint solve. Tail slicing scales the canonical
/// exact ratio directly, so repeated-event precision is preserved without forcing the tail back into a
/// narrower integer pair. If an impulse makes the selected resolution too coarse, the exact tail is
/// replayed from its starting state with a finer deterministic resolution; exceeding the hard bound fails
/// closed.
///
/// The world retains independent fat-AABB broad-phase trees for sampled sweeps and persistent-tail
/// current-overlap queries. Exact conservative bounds remain candidate truth in both paths; retained tree
/// topology only prunes exact OBB work and cannot change collision truth.
#[derive(Clone, Debug)]
pub struct RotatingWorld3d {
    config: RotatingWorldConfig3d,
    boxes: BTreeMap<BodyId, RigidBox3d>,
    broad_phase: RotatingBroadPhase3d,
    tail_broad_phase: RotatingBroadPhase3d,
}

impl RotatingWorld3d {
    #[must_use]
    pub fn new(config: RotatingWorldConfig3d) -> Self {
        Self {
            config,
            boxes: BTreeMap::new(),
            broad_phase: RotatingBroadPhase3d::default(),
            tail_broad_phase: RotatingBroadPhase3d::default(),
        }
    }

    #[must_use]
    pub const fn config(&self) -> RotatingWorldConfig3d {
        self.config
    }

    pub fn add_box(&mut self, rigid_box: RigidBox3d) -> Result<(), RotatingWorldError3d> {
        let id = rigid_box.body().id();
        if self.boxes.contains_key(&id) {
            return Err(RotatingWorldError3d::DuplicateBody(id));
        }
        self.boxes.insert(id, rigid_box);
        Ok(())
    }

    pub fn remove_box(&mut self, id: BodyId) -> Option<RigidBox3d> {
        self.boxes.remove(&id)
    }

    #[must_use]
    pub fn box_by_id(&self, id: BodyId) -> Option<&RigidBox3d> {
        self.boxes.get(&id)
    }

    pub fn boxes(&self) -> impl Iterator<Item = &RigidBox3d> {
        self.boxes.values()
    }

    /// Returns exact current contacts for one known body without discovering that body from query geometry
    /// or constructing the full-world contact graph.
    pub fn body_contacts(
        &self,
        body: BodyId,
    ) -> Result<Vec<BodyCurrentContact3d>, RotatingWorldError3d> {
        body_current_contacts_for_body(&self.boxes, body)
    }

    /// Replaces one dynamic body's linear velocity while preserving its rotational state.
    ///
    /// This is intended for controlled bodies and external impulses that are already expressed in the
    /// engine's canonical velocity units. Fixed bodies reject non-zero velocity.
    pub fn set_linear_velocity(
        &mut self,
        id: BodyId,
        velocity: Vec3i,
    ) -> Result<(), RotatingWorldError3d> {
        let rigid_box = self
            .boxes
            .get_mut(&id)
            .ok_or(RotatingWorldError3d::MissingBody(id))?;
        if rigid_box.body.kind() == BodyKind::Fixed {
            if velocity != Vec3i::ZERO {
                return Err(RotatingWorldError3d::FixedBodyVelocity(id));
            }
            return Ok(());
        }
        rigid_box.body.velocity = velocity;
        Ok(())
    }

    /// Returns stable `BodyId`-ordered OBB overlaps for an arbitrary oriented query box.
    ///
    /// When the query is exactly one current dynamic body's OBB, the engine reuses the deterministic
    /// zero-time current-contact graph. Arbitrary query geometry and fixed-body queries keep the existing
    /// direct exact scan, so the public query contract is unchanged.
    pub fn overlap_query(&self, query: OrientedBox3d) -> Result<Vec<BodyId>, RotatingWorldError3d> {
        oriented_box_vertices(query)?;
        if let Some(body) = self
            .boxes
            .values()
            .find(|rigid_box| {
                rigid_box.body().kind() == BodyKind::Dynamic && rigid_box.oriented_box() == query
            })
            .map(|rigid_box| rigid_box.body().id())
        {
            return body_current_overlap_ids(&self.boxes, body);
        }

        let mut hits = Vec::new();
        for (id, rigid_box) in &self.boxes {
            if obb_contact_seed(query, rigid_box.oriented_box())?.is_some() {
                hits.push(*id);
            }
        }
        Ok(hits)
    }

    pub fn step(
        &mut self,
        timestep_numerator: i32,
        timestep_denominator: i32,
    ) -> Result<RotatingWorldStepReport3d, RotatingWorldError3d> {
        if timestep_numerator < 0 {
            return Err(RotatingWorldError3d::NegativeTimestepNumerator(
                timestep_numerator,
            ));
        }
        if timestep_denominator <= 0 {
            return Err(RotatingWorldError3d::NonPositiveTimestepDenominator(
                timestep_denominator,
            ));
        }
        if timestep_numerator == 0 {
            return Ok(RotatingWorldStepReport3d {
                changed_body_ids: Vec::new(),
                stats: RotatingWorldStepStats3d {
                    body_count: self.boxes.len(),
                    ..RotatingWorldStepStats3d::default()
                },
            });
        }

        let broad_phase_before = self.broad_phase.stats();
        let tail_broad_phase_before = self.tail_broad_phase.stats();
        let boxes = self.boxes.values().cloned().collect::<Vec<_>>();
        let free_flight = RigidBoxFreeFlightConfig3d::new(
            self.config.gravity,
            timestep_numerator,
            timestep_denominator,
        );
        let advance = advance_repeated_rotating_events_with_broad_phase(
            &boxes,
            RepeatedRotatingEventConfig3d::new(
                RotatingContactSearchConfig3d::new(
                    free_flight,
                    self.config.sample_count,
                    self.config.refinement_steps,
                ),
                self.config.solver_passes,
                self.config.max_events,
            ),
            &mut self.broad_phase,
        )?;

        let sampled_events = advance.events.len();
        let (boxes, tail) = if advance.remaining.timestep_is_zero() {
            (advance.boxes, TailStepStats3d::default())
        } else {
            consume_tail(
                advance.boxes,
                advance.remaining,
                self.config.solver_passes,
                &mut self.tail_broad_phase,
            )?
        };
        let changed_body_ids = boxes
            .iter()
            .filter_map(|rigid_box| {
                let id = rigid_box.body().id();
                (self.boxes.get(&id) != Some(rigid_box)).then_some(id)
            })
            .collect::<Vec<_>>();
        self.boxes = boxes
            .into_iter()
            .map(|rigid_box| (rigid_box.body().id(), rigid_box))
            .collect();

        let broad_phase_after = self.broad_phase.stats();
        let tail_broad_phase_after = self.tail_broad_phase.stats();
        Ok(RotatingWorldStepReport3d {
            changed_body_ids,
            stats: RotatingWorldStepStats3d {
                body_count: self.boxes.len(),
                sampled_events,
                tail_contacts: tail.contacts,
                tail_slices: tail.slices,
                tail_replays: tail.replays,
                tail_candidate_pairs: tail.candidate_pairs,
                tail_broad_phase_queries: tail_broad_phase_after
                    .queries
                    .saturating_sub(tail_broad_phase_before.queries),
                tail_broad_phase_rebuilds: tail_broad_phase_after
                    .rebuilds
                    .saturating_sub(tail_broad_phase_before.rebuilds),
                tail_broad_phase_reuses: tail_broad_phase_after
                    .reuses
                    .saturating_sub(tail_broad_phase_before.reuses),
                broad_phase_queries: broad_phase_after
                    .queries
                    .saturating_sub(broad_phase_before.queries),
                broad_phase_rebuilds: broad_phase_after
                    .rebuilds
                    .saturating_sub(broad_phase_before.rebuilds),
                broad_phase_reuses: broad_phase_after
                    .reuses
                    .saturating_sub(broad_phase_before.reuses),
                broad_phase_incremental_updates: broad_phase_after
                    .incremental_updates
                    .saturating_sub(broad_phase_before.incremental_updates),
                broad_phase_reinserts: broad_phase_after
                    .reinserts
                    .saturating_sub(broad_phase_before.reinserts),
                broad_phase_rotations: broad_phase_after
                    .rotations
                    .saturating_sub(broad_phase_before.rotations),
                broad_phase_partial_queries: broad_phase_after
                    .partial_queries
                    .saturating_sub(broad_phase_before.partial_queries),
                broad_phase_partial_body_updates: broad_phase_after
                    .partial_body_updates
                    .saturating_sub(broad_phase_before.partial_body_updates),
                event_response_passes: advance.work.event_response_passes,
                stabilization_passes: advance.work.stabilization_passes,
                stabilizations_hitting_limit: advance.work.stabilizations_hitting_limit,
                stabilization_candidate_pairs: advance.work.stabilization_candidate_pairs,
                stabilization_exact_contacts: advance.work.stabilization_exact_contacts,
                stabilization_active_bodies: advance.work.stabilization_active_bodies,
            },
        })
    }
}

fn consume_tail(
    boxes: Vec<RigidBox3d>,
    remaining: RigidBoxFreeFlightConfig3d,
    solver_passes: u8,
    broad_phase: &mut RotatingBroadPhase3d,
) -> Result<(Vec<RigidBox3d>, TailStepStats3d), RotatingWorldError3d> {
    let mut stats = TailStepStats3d::default();
    let initial_contacts = contact_frontier(&boxes, broad_phase, &mut stats)?;
    if initial_contacts.is_empty() {
        let (boxes, contacts) =
            free_flight_and_stabilize(boxes, remaining, solver_passes, broad_phase, &mut stats)?;
        stats.contacts = contacts;
        return Ok((boxes, stats));
    }

    let mut slice_count = persistent_tail_slice_count(&boxes, remaining, &initial_contacts)?;
    loop {
        let slice_config = tail_slice_config(remaining, slice_count)?;
        let mut current = boxes.clone();
        let mut contact_count = 0_usize;
        let mut unsafe_body = None;

        for _ in 0..slice_count {
            stats.slices = stats.slices.saturating_add(1);
            let current_contacts = contact_frontier(&current, broad_phase, &mut stats)?;
            if let Some(id) = first_unsafe_tail_body(&current, slice_config, &current_contacts)? {
                unsafe_body = Some(id);
                break;
            }
            let (next, contacts) = free_flight_and_stabilize(
                current,
                slice_config,
                solver_passes,
                broad_phase,
                &mut stats,
            )?;
            current = next;
            contact_count = contact_count.saturating_add(contacts);
        }

        if unsafe_body.is_none() {
            stats.contacts = contact_count;
            return Ok((current, stats));
        }
        stats.replays = stats.replays.saturating_add(1);
        let target = slice_count
            .saturating_mul(2)
            .min(MAX_PERSISTENT_TAIL_SLICES);
        if target <= slice_count {
            return Err(RotatingWorldError3d::PersistentTailMotionUnsafe(
                unsafe_body.expect("unsafe tail body was observed"),
            ));
        }
        slice_count = next_representable_tail_slice_count(remaining, target)?;
    }
}

fn free_flight_and_stabilize(
    boxes: Vec<RigidBox3d>,
    config: RigidBoxFreeFlightConfig3d,
    solver_passes: u8,
    broad_phase: &mut RotatingBroadPhase3d,
    stats: &mut TailStepStats3d,
) -> Result<(Vec<RigidBox3d>, usize), RotatingWorldError3d> {
    let mut sampled = boxes
        .iter()
        .map(|rigid_box| sample_rigid_box_free_flight(rigid_box, config, 1, 1))
        .collect::<Result<Vec<_>, _>>()?;
    let contacts = contact_frontier(&sampled, broad_phase, stats)?;
    let contact_count = contacts.len();
    if contacts.is_empty() {
        return Ok((sampled, 0));
    }

    let frontier = RotatingContactFrontier3d {
        free_flight: RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 0, 1),
        time: SampledContactTime3d::ZERO,
        contacts,
        remaining_numerator: 0,
    };
    resolve_rotating_contact_frontier(&mut sampled, &frontier, solver_passes)?;
    Ok((sampled, contact_count))
}

fn persistent_tail_slice_count(
    boxes: &[RigidBox3d],
    remaining: RigidBoxFreeFlightConfig3d,
    contacts: &[RotatingContactSearchHit3d],
) -> Result<u32, RotatingWorldError3d> {
    for slices in 1..=MAX_PERSISTENT_TAIL_SLICES {
        let Ok(config) = tail_slice_config(remaining, slices) else {
            continue;
        };
        if first_unsafe_tail_body(boxes, config, contacts)?.is_none() {
            return Ok(slices);
        }
    }

    Err(RotatingWorldError3d::PersistentTailResolutionLimit(
        MAX_PERSISTENT_TAIL_SLICES,
    ))
}

fn next_representable_tail_slice_count(
    remaining: RigidBoxFreeFlightConfig3d,
    minimum: u32,
) -> Result<u32, RotatingWorldError3d> {
    for slices in minimum..=MAX_PERSISTENT_TAIL_SLICES {
        if tail_slice_config(remaining, slices).is_ok() {
            return Ok(slices);
        }
    }
    Err(RotatingWorldError3d::PersistentTailResolutionLimit(
        MAX_PERSISTENT_TAIL_SLICES,
    ))
}

fn tail_slice_config(
    remaining: RigidBoxFreeFlightConfig3d,
    slices: u32,
) -> Result<RigidBoxFreeFlightConfig3d, RotatingWorldError3d> {
    if slices == 0 {
        return Err(RotatingWorldError3d::PersistentTailResolutionLimit(
            MAX_PERSISTENT_TAIL_SLICES,
        ));
    }
    remaining.scaled_fraction(1, slices).map_err(|_| {
        RotatingWorldError3d::PersistentTailResolutionLimit(MAX_PERSISTENT_TAIL_SLICES)
    })
}

fn first_unsafe_tail_body(
    boxes: &[RigidBox3d],
    config: RigidBoxFreeFlightConfig3d,
    contacts: &[RotatingContactSearchHit3d],
) -> Result<Option<BodyId>, RotatingWorldError3d> {
    for rigid_box in boxes {
        let id = rigid_box.body().id();
        if !contacts
            .iter()
            .any(|contact| contact.pair.left == id || contact.pair.right == id)
        {
            continue;
        }
        if !tail_motion_within_extent(rigid_box, config)? {
            return Ok(Some(id));
        }
    }
    Ok(None)
}

fn tail_motion_within_extent(
    rigid_box: &RigidBox3d,
    config: RigidBoxFreeFlightConfig3d,
) -> Result<bool, RotatingWorldError3d> {
    if rigid_box.body().kind() == BodyKind::Fixed || config.timestep_is_zero() {
        return Ok(true);
    }

    let id = rigid_box.body().id();
    let timestep = config
        .exact_timestep()
        .map_err(|_| RotatingWorldError3d::PersistentTailArithmeticOverflow(id))?;
    let half = rigid_box.body().half_extents();
    let minimum_half = u128::from(half.x.min(half.y).min(half.z).unsigned_abs());
    let motion_budget = minimum_half
        .checked_mul(2)
        .ok_or(RotatingWorldError3d::PersistentTailArithmeticOverflow(id))?;

    let velocity = rigid_box.body().velocity();
    let velocity_components = [velocity.x, velocity.y, velocity.z];
    let gravity_components = [config.gravity.x, config.gravity.y, config.gravity.z];
    let mut translation_bound = 0_u128;
    for axis in 0..3 {
        let gravity_delta = timestep
            .mul_ceil_u128(u128::from(gravity_components[axis].unsigned_abs()))
            .map_err(|_| RotatingWorldError3d::PersistentTailArithmeticOverflow(id))?;
        let speed_bound = u128::from(velocity_components[axis].unsigned_abs())
            .checked_add(gravity_delta)
            .ok_or(RotatingWorldError3d::PersistentTailArithmeticOverflow(id))?;
        let travel = timestep
            .mul_ceil_u128(speed_bound)
            .map_err(|_| RotatingWorldError3d::PersistentTailArithmeticOverflow(id))?;
        translation_bound = translation_bound.max(travel);
    }

    let angular = rigid_box.angular().angular_velocity;
    let angular_speed_l1 = if rigid_box.rotation_locked() {
        0
    } else {
        u128::from(angular.x.unsigned_abs())
            .checked_add(u128::from(angular.y.unsigned_abs()))
            .and_then(|value| value.checked_add(u128::from(angular.z.unsigned_abs())))
            .ok_or(RotatingWorldError3d::PersistentTailArithmeticOverflow(id))?
    };
    let radius_bound = u128::from(half.x.unsigned_abs())
        .checked_add(u128::from(half.y.unsigned_abs()))
        .and_then(|value| value.checked_add(u128::from(half.z.unsigned_abs())))
        .ok_or(RotatingWorldError3d::PersistentTailArithmeticOverflow(id))?;
    let angular_numerator_motion = timestep
        .mul_ceil_u128(
            radius_bound
                .checked_mul(angular_speed_l1)
                .ok_or(RotatingWorldError3d::PersistentTailArithmeticOverflow(id))?,
        )
        .map_err(|_| RotatingWorldError3d::PersistentTailArithmeticOverflow(id))?;
    let angular_motion = ceil_div(
        angular_numerator_motion,
        u128::from(ANGULAR_VELOCITY_SCALE.unsigned_abs()),
        id,
    )?;

    let total_motion = translation_bound
        .checked_add(angular_motion)
        .ok_or(RotatingWorldError3d::PersistentTailArithmeticOverflow(id))?;
    Ok(total_motion <= motion_budget)
}

fn ceil_div(value: u128, denominator: u128, id: BodyId) -> Result<u128, RotatingWorldError3d> {
    if denominator == 0 {
        return Err(RotatingWorldError3d::PersistentTailArithmeticOverflow(id));
    }
    let quotient = value / denominator;
    if value.is_multiple_of(denominator) {
        Ok(quotient)
    } else {
        quotient
            .checked_add(1)
            .ok_or(RotatingWorldError3d::PersistentTailArithmeticOverflow(id))
    }
}

fn contact_frontier(
    boxes: &[RigidBox3d],
    broad_phase: &mut RotatingBroadPhase3d,
    stats: &mut TailStepStats3d,
) -> Result<Vec<RotatingContactSearchHit3d>, RotatingWorldError3d> {
    let pairs = broad_phase
        .candidate_pairs(boxes, RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 0, 1))
        .map_err(map_tail_broad_phase_error)?;
    stats.candidate_pairs = stats
        .candidate_pairs
        .saturating_add(u64::try_from(pairs.len()).unwrap_or(u64::MAX));

    let mut contacts = Vec::new();
    for pair in pairs {
        let left = boxes
            .iter()
            .find(|rigid_box| rigid_box.body().id() == pair.left)
            .expect("tail broad phase candidate left body must exist");
        let right = boxes
            .iter()
            .find(|rigid_box| rigid_box.body().id() == pair.right)
            .expect("tail broad phase candidate right body must exist");
        let Some(contact) = obb_contact_seed(left.oriented_box(), right.oriented_box())? else {
            continue;
        };
        contacts.push(RotatingContactSearchHit3d {
            time: SampledContactTime3d::ZERO,
            pair,
            contact,
        });
    }
    Ok(contacts)
}

fn map_tail_broad_phase_error(error: RotatingBroadPhaseError3d) -> RotatingWorldError3d {
    match error {
        RotatingBroadPhaseError3d::DuplicateBodyId(id) => RotatingWorldError3d::DuplicateBody(id),
        RotatingBroadPhaseError3d::IncrementalQueryUnsynchronized(id) => {
            RotatingWorldError3d::MissingBody(id)
        }
        RotatingBroadPhaseError3d::FreeFlight(error) => RotatingWorldError3d::FreeFlight(error),
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        ANGULAR_VELOCITY_SCALE, AngularState3d, AngularVelocity3d, BodyId, Orientation3d,
        OrientedBox3d, RigidBody, RigidBox3d, RigidBoxFreeFlightConfig3d, Vec3i,
    };

    use super::{
        RotatingBroadPhase3d, RotatingWorld3d, RotatingWorldConfig3d, RotatingWorldError3d,
        TailStepStats3d, contact_frontier, tail_motion_within_extent, tail_slice_config,
    };

    fn dynamic(id: u64, position: Vec3i, velocity: Vec3i, half: Vec3i) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::dynamic(BodyId(id), position, velocity, half),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid dynamic box")
    }

    fn fixed(id: u64, position: Vec3i, half: Vec3i) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::fixed(BodyId(id), position, half),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid fixed box")
    }

    fn world(gravity: Vec3i) -> RotatingWorld3d {
        RotatingWorld3d::new(RotatingWorldConfig3d {
            gravity,
            sample_count: 32,
            refinement_steps: 4,
            solver_passes: 8,
            max_events: 16,
        })
    }

    #[test]
    fn widened_tail_denominator_can_be_sliced_exactly() {
        let remaining =
            RigidBoxFreeFlightConfig3d::new_wide(Vec3i::ZERO, 35_582_088, 2_147_483_647);
        let sliced = tail_slice_config(remaining, 2).expect("wide tail slice");
        assert_eq!(sliced.timestep_i128(), Some((17_791_044, 2_147_483_647)));
        assert!(
            sliced
                .timestep_i128()
                .expect("ratio remains narrow in this regression")
                .1
                > i128::from(i32::MAX) - 1
        );
    }

    #[test]
    fn widened_tail_motion_bound_avoids_intermediate_overflow() {
        let rigid_box = RigidBox3d::new(
            RigidBody::dynamic(
                BodyId(9),
                Vec3i::ZERO,
                Vec3i::new(10, 0, 0),
                Vec3i::new(100, 100, 100),
            ),
            AngularState3d::new(
                Orientation3d::IDENTITY,
                AngularVelocity3d::new(0, 0, ANGULAR_VELOCITY_SCALE / 10),
            ),
        )
        .expect("valid rotating box");
        let mut config = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1);
        for _ in 0..20 {
            config = config
                .scaled_fraction(511, 512)
                .expect("bounded exact tail growth");
        }
        assert!(config.timestep_i128().is_none());

        assert!(
            tail_motion_within_extent(&rigid_box, config)
                .expect("wide tail bound remains representable")
        );
    }

    #[test]
    fn tail_frontier_broad_phase_prunes_separated_pairs_without_changing_contact_truth() {
        let mut boxes = vec![
            dynamic(1, Vec3i::new(0, 1, 0), Vec3i::ZERO, Vec3i::new(1, 1, 1)),
            fixed(2, Vec3i::new(0, -1, 0), Vec3i::new(20, 1, 20)),
        ];
        for id in 3..35 {
            boxes.push(fixed(
                id,
                Vec3i::new((id as i32) * 100, 100, 100),
                Vec3i::new(1, 1, 1),
            ));
        }
        let all_pairs = boxes.len().saturating_mul(boxes.len().saturating_sub(1)) / 2;
        let mut broad_phase = RotatingBroadPhase3d::default();
        let mut stats = TailStepStats3d::default();

        let contacts =
            contact_frontier(&boxes, &mut broad_phase, &mut stats).expect("broad-phase frontier");

        assert_eq!(contacts.len(), 1);
        assert_eq!(contacts[0].pair.left, BodyId(1));
        assert_eq!(contacts[0].pair.right, BodyId(2));
        assert!(
            stats.candidate_pairs < u64::try_from(all_pairs).unwrap_or(u64::MAX),
            "tail broad phase did not prune any exact OBB tests"
        );
    }

    #[test]
    fn controlled_velocity_and_overlap_queries_preserve_stable_identity() {
        let empty_world = world(Vec3i::ZERO);
        assert!(
            empty_world
                .overlap_query(OrientedBox3d::new(
                    Vec3i::ZERO,
                    Vec3i::new(1, 0, 1),
                    Orientation3d::IDENTITY,
                ))
                .is_err(),
            "empty-world queries must still validate their geometry"
        );

        let mut world = world(Vec3i::ZERO);
        world
            .add_box(dynamic(1, Vec3i::ZERO, Vec3i::ZERO, Vec3i::new(2, 2, 2)))
            .expect("dynamic");
        world
            .add_box(fixed(2, Vec3i::new(5, 0, 0), Vec3i::new(2, 2, 2)))
            .expect("fixed");
        world
            .set_linear_velocity(BodyId(1), Vec3i::new(60, 0, 0))
            .expect("controlled velocity");
        assert_eq!(
            world
                .box_by_id(BodyId(1))
                .expect("dynamic")
                .body()
                .velocity(),
            Vec3i::new(60, 0, 0)
        );
        assert_eq!(
            world.set_linear_velocity(BodyId(2), Vec3i::new(1, 0, 0)),
            Err(RotatingWorldError3d::FixedBodyVelocity(BodyId(2)))
        );

        let hits = world
            .overlap_query(OrientedBox3d::new(
                Vec3i::new(2, 0, 0),
                Vec3i::new(1, 1, 1),
                Orientation3d::IDENTITY,
            ))
            .expect("valid overlap query");
        assert_eq!(hits, vec![BodyId(1), BodyId(2)]);
    }

    #[test]
    fn persistent_broad_phase_reuses_topology_across_unchanged_steps() {
        let mut world = world(Vec3i::ZERO);
        world
            .add_box(dynamic(1, Vec3i::ZERO, Vec3i::ZERO, Vec3i::new(1, 1, 1)))
            .expect("dynamic");
        world
            .add_box(fixed(2, Vec3i::new(100, 0, 0), Vec3i::new(1, 1, 1)))
            .expect("fixed");

        let first = world.step(1, 60).expect("first static frame");
        let second = world.step(1, 60).expect("second static frame");

        assert_eq!(first.stats.broad_phase_queries, 1);
        assert_eq!(first.stats.broad_phase_rebuilds, 1);
        assert_eq!(first.stats.broad_phase_reuses, 0);
        assert_eq!(second.stats.broad_phase_queries, 1);
        assert_eq!(second.stats.broad_phase_rebuilds, 0);
        assert_eq!(second.stats.broad_phase_reuses, 1);
    }

    #[test]
    fn persistent_floor_contact_cannot_tunnel_across_a_large_tail() {
        let mut world = world(Vec3i::new(0, -100, 0));
        world
            .add_box(fixed(1, Vec3i::new(0, -1, 0), Vec3i::new(20, 1, 20)))
            .expect("floor");
        world
            .add_box(dynamic(
                2,
                Vec3i::new(0, 1, 0),
                Vec3i::ZERO,
                Vec3i::new(1, 1, 1),
            ))
            .expect("box");

        world
            .step(1, 1)
            .expect("persistent tail must remain constrained");

        let body = world.box_by_id(BodyId(2)).expect("box remains").body();
        assert!(
            body.position().y >= 1,
            "box tunneled through a time-zero floor contact: {body:?}"
        );
    }

    #[test]
    fn resting_floor_contact_consumes_the_full_frame() {
        let mut world = world(Vec3i::new(0, -3_600, 0));
        world
            .add_box(fixed(1, Vec3i::new(0, -1, 0), Vec3i::new(20, 1, 20)))
            .expect("floor");
        world
            .add_box(dynamic(
                2,
                Vec3i::new(0, 1, 0),
                Vec3i::ZERO,
                Vec3i::new(1, 1, 1),
            ))
            .expect("box");

        for _ in 0..8 {
            world.step(1, 60).expect("stable frame");
        }

        let body = world.box_by_id(BodyId(2)).expect("box remains").body();
        assert!(
            body.position().y >= 1,
            "box sank through the floor: {body:?}"
        );
        assert!(
            body.velocity().y >= 0,
            "resting box kept downward velocity: {body:?}"
        );
    }

    #[test]
    fn off_center_projectile_impact_generates_spin() {
        let mut world = world(Vec3i::ZERO);
        world
            .add_box(dynamic(10, Vec3i::ZERO, Vec3i::ZERO, Vec3i::new(3, 3, 3)))
            .expect("target");
        world
            .add_box(dynamic(
                20,
                Vec3i::new(-12, 2, 0),
                Vec3i::new(720, 0, 0),
                Vec3i::new(1, 1, 1),
            ))
            .expect("projectile");

        world.step(1, 60).expect("impact frame");

        let target = world.box_by_id(BodyId(10)).expect("target remains");
        assert!(
            !target.angular().angular_velocity.is_zero(),
            "off-center impact did not generate angular velocity"
        );
    }

    #[test]
    fn identical_steps_are_bit_for_bit_repeatable() {
        let mut first = world(Vec3i::new(0, -3_600, 0));
        first
            .add_box(fixed(1, Vec3i::new(0, -1, 0), Vec3i::new(20, 1, 20)))
            .expect("floor");
        first
            .add_box(dynamic(
                2,
                Vec3i::new(0, 8, 0),
                Vec3i::new(120, 0, 0),
                Vec3i::new(2, 2, 2),
            ))
            .expect("box");
        let mut second = first.clone();

        for _ in 0..12 {
            first.step(1, 60).expect("first frame");
            second.step(1, 60).expect("second frame");
        }

        assert_eq!(
            first.boxes().cloned().collect::<Vec<_>>(),
            second.boxes().cloned().collect::<Vec<_>>()
        );
    }
}
