use std::{cmp::Ordering, collections::BTreeMap, fmt};

use crate::{
    BodyId, BodyKind, ContactNormal, MATERIAL_SCALE, Material, RigidBody, TimeOfImpact, Vec3i,
    collision::{MotionAabb, MotionSweepHit, Ratio, SUBTICK_SCALE, sweep_motion},
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct WorldConfig {
    pub gravity: Vec3i,
    pub max_events_per_step: usize,
    pub stabilization_passes: usize,
}

impl Default for WorldConfig {
    fn default() -> Self {
        Self {
            gravity: Vec3i::new(0, -1, 0),
            max_events_per_step: 256,
            stabilization_passes: 8,
        }
    }
}

/// Computational work performed by the translational compatibility world.
/// Sleep/wake, layers and rotating solver authority are unavailable on this API;
/// they are deliberately absent rather than reported as invented zero counters.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct TranslationalStepWork {
    pub cached_stationary_step: bool,
    pub dynamic_bodies: usize,
    pub initially_moving_bodies: usize,
    pub staged_bodies: usize,
    /// Staging-vector capacity increases during this call; warmed active calls reuse capacity.
    pub staged_state_capacity_growths: usize,
    /// Active staging payload capacity, excluding allocator overhead. No-op calls use zero.
    pub staged_state_capacity_bytes: usize,
    pub quantized_bodies: usize,
    pub body_map_rebuilds: usize,
    pub body_map_insertions: usize,
    pub committed_position_deltas: usize,
    pub committed_velocity_deltas: usize,
    pub broad_phase_queries: usize,
    pub sweep_bound_preparations: usize,
    /// Actual bound preparations for BodyKind::Fixed, across every query in this call.
    pub fixed_sweep_bound_preparations: usize,
    /// Actual bound preparations for BodyKind::Dynamic, across every query in this call.
    pub dynamic_sweep_bound_preparations: usize,
    pub broad_phase_sorts: usize,
    pub broad_phase_pair_sorts: usize,
    /// Entry, active-index and candidate-pair vector capacity increases across all queries.
    pub broad_phase_capacity_growths: usize,
    pub active_bound_checks: usize,
    pub fixed_pair_rejections: usize,
    pub stabilization_queries: usize,
    pub stabilization_pair_checks: usize,
    pub stabilization_exact_tests: usize,
    pub current_contact_tests: usize,
    pub candidate_buffer_peak_capacity_bytes: usize,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct StepStats {
    pub body_count: usize,
    pub work: TranslationalStepWork,
    /// Candidate pair comparisons after the sweep-and-prune X-axis rejection.
    pub pair_checks: usize,
    /// Pairs whose swept bounds overlap in all three axes.
    pub swept_candidates: usize,
    pub toi_tests: usize,
    pub collision_events: usize,
    pub contact_resolutions: usize,
    pub stabilization_passes: usize,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CollisionEvent {
    pub left: BodyId,
    pub right: BodyId,
    pub normal: ContactNormal,
    pub time: TimeOfImpact,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct StepReport {
    pub events: Vec<CollisionEvent>,
    pub stats: StepStats,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PhysicsError {
    DuplicateBody(BodyId),
    MissingBody(BodyId),
    InvalidHalfExtents(BodyId),
    ZeroMass(BodyId),
    RestitutionOutOfRange(BodyId, u16),
    FixedBodyVelocity(BodyId),
    NonPositiveTicks(i32),
    ArithmeticOverflow(BodyId),
    EventLimit(usize),
}

impl fmt::Display for PhysicsError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DuplicateBody(id) => write!(formatter, "duplicate physics body {}", id.0),
            Self::MissingBody(id) => write!(formatter, "physics body {} does not exist", id.0),
            Self::InvalidHalfExtents(id) => {
                write!(formatter, "physics body {} has negative half extents", id.0)
            }
            Self::ZeroMass(id) => write!(formatter, "dynamic physics body {} has zero mass", id.0),
            Self::RestitutionOutOfRange(id, value) => write!(
                formatter,
                "physics body {} has restitution {value}, expected 0..={MATERIAL_SCALE}",
                id.0
            ),
            Self::FixedBodyVelocity(id) => write!(
                formatter,
                "fixed physics body {} has non-zero velocity",
                id.0
            ),
            Self::NonPositiveTicks(ticks) => {
                write!(
                    formatter,
                    "physics step requires positive ticks, got {ticks}"
                )
            }
            Self::ArithmeticOverflow(id) => {
                write!(formatter, "physics arithmetic overflow for body {}", id.0)
            }
            Self::EventLimit(limit) => write!(
                formatter,
                "physics step exceeded its deterministic collision-event limit of {limit}"
            ),
        }
    }
}

impl std::error::Error for PhysicsError {}

/// Deterministic body world. `BTreeMap` order is part of the repeatability contract.
#[derive(Clone, Debug)]
pub struct World {
    config: WorldConfig,
    bodies: BTreeMap<BodyId, RigidBody>,
    dynamic_body_count: usize,
    stationary_step_is_no_op: bool,
    // Empty at every returned step boundary; the body map is the physical authority.
    staged_states: Vec<BodyState>,
    broad_phase: BroadPhaseScratch,
}

impl Default for World {
    fn default() -> Self {
        Self::new(WorldConfig::default())
    }
}

impl World {
    #[must_use]
    pub fn new(config: WorldConfig) -> Self {
        Self {
            config,
            bodies: BTreeMap::new(),
            dynamic_body_count: 0,
            stationary_step_is_no_op: false,
            staged_states: Vec::new(),
            broad_phase: BroadPhaseScratch::default(),
        }
    }

    #[must_use]
    pub const fn config(&self) -> WorldConfig {
        self.config
    }

    pub fn add_body(&mut self, body: RigidBody) -> Result<(), PhysicsError> {
        validate_body(&body)?;
        if self.bodies.contains_key(&body.id) {
            return Err(PhysicsError::DuplicateBody(body.id));
        }
        self.dynamic_body_count += usize::from(body.kind == BodyKind::Dynamic);
        self.bodies.insert(body.id, body);
        self.stationary_step_is_no_op = false;
        Ok(())
    }

    pub fn replace_body(&mut self, body: RigidBody) -> Result<(), PhysicsError> {
        validate_body(&body)?;
        let previous = self
            .bodies
            .get_mut(&body.id)
            .ok_or(PhysicsError::MissingBody(body.id))?;
        if *previous != body {
            self.dynamic_body_count -= usize::from(previous.kind == BodyKind::Dynamic);
            self.dynamic_body_count += usize::from(body.kind == BodyKind::Dynamic);
            *previous = body;
            self.stationary_step_is_no_op = false;
        }
        Ok(())
    }

    pub fn remove_body(&mut self, id: BodyId) -> Option<RigidBody> {
        let removed = self.bodies.remove(&id);
        if let Some(body) = &removed {
            self.dynamic_body_count -= usize::from(body.kind == BodyKind::Dynamic);
            self.stationary_step_is_no_op = false;
        }
        removed
    }

    #[must_use]
    pub fn body(&self, id: BodyId) -> Option<&RigidBody> {
        self.bodies.get(&id)
    }

    pub fn bodies(&self) -> impl Iterator<Item = &RigidBody> {
        self.bodies.values()
    }

    pub fn set_velocity(&mut self, id: BodyId, velocity: Vec3i) -> Result<(), PhysicsError> {
        let body = self
            .bodies
            .get_mut(&id)
            .ok_or(PhysicsError::MissingBody(id))?;
        if body.kind == BodyKind::Fixed && velocity != Vec3i::ZERO {
            return Err(PhysicsError::FixedBodyVelocity(id));
        }
        if body.velocity != velocity {
            body.velocity = velocity;
            self.stationary_step_is_no_op = false;
        }
        Ok(())
    }

    pub fn set_position(&mut self, id: BodyId, position: Vec3i) -> Result<(), PhysicsError> {
        let body = self
            .bodies
            .get_mut(&id)
            .ok_or(PhysicsError::MissingBody(id))?;
        if body.position != position {
            body.position = position;
            self.stationary_step_is_no_op = false;
        }
        Ok(())
    }

    pub fn set_material(&mut self, id: BodyId, material: Material) -> Result<(), PhysicsError> {
        if material.restitution_milli() > MATERIAL_SCALE {
            return Err(PhysicsError::RestitutionOutOfRange(
                id,
                material.restitution_milli(),
            ));
        }
        let body = self
            .bodies
            .get_mut(&id)
            .ok_or(PhysicsError::MissingBody(id))?;
        if body.material != material {
            body.material = material;
            self.stationary_step_is_no_op = false;
        }
        Ok(())
    }

    /// Advances the world on one continuous Q32.32 timeline.
    ///
    /// Gravity is applied first, then a deterministic swept sweep-and-prune broad phase limits the
    /// candidate set. The globally earliest AABB impact is resolved and the remainder of the
    /// requested interval continues with the post-impact velocity, so fast bodies cannot tunnel
    /// through thin fixed bodies merely because no frame landed on the contact point.
    pub fn step(&mut self, ticks: i32) -> Result<StepReport, PhysicsError> {
        if ticks <= 0 {
            return Err(PhysicsError::NonPositiveTicks(ticks));
        }

        let mut report = StepReport {
            events: Vec::new(),
            stats: StepStats {
                body_count: self.bodies.len(),
                work: TranslationalStepWork {
                    dynamic_bodies: self.dynamic_body_count,
                    ..TranslationalStepWork::default()
                },
                ..StepStats::default()
            },
        };
        if self.stationary_step_is_no_op {
            report.stats.work.cached_stationary_step = true;
            return Ok(report);
        }
        let mut states = std::mem::take(&mut self.staged_states);
        let mut broad_phase = std::mem::take(&mut self.broad_phase);
        let result = self.step_staged(ticks, &mut states, &mut broad_phase, report);
        states.clear();
        broad_phase.clear();
        self.staged_states = states;
        self.broad_phase = broad_phase;
        result
    }

    /// Retained step-scratch payload only, excluding authoritative bodies and allocator overhead.
    /// Capacity survives successful and returned-error steps, including stationary no-op calls.
    #[must_use]
    pub fn retained_step_scratch_bytes(&self) -> usize {
        self.staged_states.capacity() * std::mem::size_of::<BodyState>()
            + self.broad_phase.capacity_bytes()
    }

    /// Releases disposable step capacity without changing bodies, stationary evidence or replay.
    /// A later active step may allocate again; no query or simulation history is discarded.
    pub fn release_step_scratch(&mut self) {
        self.staged_states = Vec::new();
        self.broad_phase = BroadPhaseScratch::default();
    }

    fn step_staged(
        &mut self,
        ticks: i32,
        states: &mut Vec<BodyState>,
        broad_phase: &mut BroadPhaseScratch,
        mut report: StepReport,
    ) -> Result<StepReport, PhysicsError> {
        debug_assert!(states.is_empty());
        let previous_capacity = states.capacity();
        states.reserve_exact(self.bodies.len());
        states.extend(self.bodies.values().cloned().map(BodyState::new));
        report.stats.work.staged_state_capacity_growths =
            usize::from(states.capacity() > previous_capacity);
        report.stats.work.staged_bodies = states.len();
        report.stats.work.staged_state_capacity_bytes =
            states.capacity() * std::mem::size_of::<BodyState>();
        for state in &mut *states {
            state.apply_gravity(self.config.gravity, ticks)?;
            report.stats.work.initially_moving_bodies +=
                usize::from(state.body.velocity != Vec3i::ZERO);
        }
        let mut contacts_converged = stabilize_contacts(
            states,
            self.config.stabilization_passes,
            broad_phase,
            &mut report.stats,
        )?;
        // Gravity has already been applied for the complete interval. With zero
        // post-contact velocities, every remaining swept position is unchanged.
        let has_motion = states
            .iter()
            .any(|state| state.body.velocity != Vec3i::ZERO);

        let mut remaining_subticks = i128::from(ticks) * SUBTICK_SCALE;
        let mut elapsed_subticks = 0_i128;
        let mut event_count = 0_usize;

        while has_motion && remaining_subticks > 0 {
            let hits =
                find_earliest_hits(states, remaining_subticks, broad_phase, &mut report.stats);
            if hits.is_empty() {
                advance_all(states, remaining_subticks)?;
                break;
            }

            if event_count.saturating_add(hits.len()) > self.config.max_events_per_step {
                return Err(PhysicsError::EventLimit(self.config.max_events_per_step));
            }

            let advance_subticks = hits[0].hit.time.ceil().max(1).min(remaining_subticks);
            advance_all(states, advance_subticks)?;
            remaining_subticks -= advance_subticks;
            elapsed_subticks += advance_subticks;

            for hit in &hits {
                project_to_contact(states, hit.left, hit.right, hit.hit.normal)?;
            }
            for hit in hits {
                let (left_id, right_id) = (states[hit.left].body.id, states[hit.right].body.id);
                if resolve_contact_velocity(states, hit.left, hit.right, hit.hit.normal)? {
                    report.stats.contact_resolutions += 1;
                }
                let event_time = u64::try_from(elapsed_subticks)
                    .map_err(|_| PhysicsError::ArithmeticOverflow(left_id))?;
                report.events.push(CollisionEvent {
                    left: left_id,
                    right: right_id,
                    normal: hit.hit.normal,
                    time: TimeOfImpact::from_subticks(event_time),
                });
                report.stats.collision_events += 1;
                event_count += 1;
            }

            contacts_converged = stabilize_contacts(
                states,
                self.config.stabilization_passes,
                broad_phase,
                &mut report.stats,
            )?;
        }

        // Validate every fallible conversion before publishing any delta.
        // The state layout retains BodyId order and membership throughout stepping.
        let mut stationary = (self.config.gravity == Vec3i::ZERO || self.dynamic_body_count == 0)
            && contacts_converged;
        for state in &mut *states {
            state.quantize_position()?;
            report.stats.work.quantized_bodies += 1;
            stationary &= state.body.velocity == Vec3i::ZERO
                && state.center_scaled
                    == [
                        i128::from(state.body.position.x) * SUBTICK_SCALE,
                        i128::from(state.body.position.y) * SUBTICK_SCALE,
                        i128::from(state.body.position.z) * SUBTICK_SCALE,
                    ];
        }
        for (state, body) in states.iter().zip(self.bodies.values_mut()) {
            debug_assert_eq!(state.body.id, body.id);
            if state.body.position != body.position {
                body.position = state.body.position;
                report.stats.work.committed_position_deltas += 1;
            }
            if state.body.velocity != body.velocity {
                body.velocity = state.body.velocity;
                report.stats.work.committed_velocity_deltas += 1;
            }
        }
        self.stationary_step_is_no_op = stationary;
        Ok(report)
    }
}

fn validate_body(body: &RigidBody) -> Result<(), PhysicsError> {
    if body.half_extents.x < 0 || body.half_extents.y < 0 || body.half_extents.z < 0 {
        return Err(PhysicsError::InvalidHalfExtents(body.id));
    }
    if body.kind == BodyKind::Dynamic && body.mass_units == 0 {
        return Err(PhysicsError::ZeroMass(body.id));
    }
    if body.material.restitution_milli() > MATERIAL_SCALE {
        return Err(PhysicsError::RestitutionOutOfRange(
            body.id,
            body.material.restitution_milli(),
        ));
    }
    if body.kind == BodyKind::Fixed && body.velocity != Vec3i::ZERO {
        return Err(PhysicsError::FixedBodyVelocity(body.id));
    }
    Ok(())
}

#[derive(Clone, Debug)]
struct BodyState {
    body: RigidBody,
    center_scaled: [i128; 3],
}

impl BodyState {
    fn new(body: RigidBody) -> Self {
        Self {
            center_scaled: [
                i128::from(body.position.x) * SUBTICK_SCALE,
                i128::from(body.position.y) * SUBTICK_SCALE,
                i128::from(body.position.z) * SUBTICK_SCALE,
            ],
            body,
        }
    }

    fn motion(&self) -> MotionAabb {
        MotionAabb {
            center_scaled: self.center_scaled,
            half_scaled: [
                i128::from(self.body.half_extents.x) * SUBTICK_SCALE,
                i128::from(self.body.half_extents.y) * SUBTICK_SCALE,
                i128::from(self.body.half_extents.z) * SUBTICK_SCALE,
            ],
            velocity: [
                self.body.velocity.x,
                self.body.velocity.y,
                self.body.velocity.z,
            ],
        }
    }

    fn apply_gravity(&mut self, gravity: Vec3i, ticks: i32) -> Result<(), PhysicsError> {
        if self.body.kind == BodyKind::Fixed {
            return Ok(());
        }
        for axis in 0..3 {
            let acceleration = i128::from(gravity.component(axis)) * i128::from(ticks);
            let velocity = i128::from(self.body.velocity.component(axis)) + acceleration;
            let velocity = i32::try_from(velocity)
                .map_err(|_| PhysicsError::ArithmeticOverflow(self.body.id))?;
            self.body.velocity.set_component(axis, velocity);
        }
        Ok(())
    }

    fn advance(&mut self, subticks: i128) -> Result<(), PhysicsError> {
        if self.body.kind == BodyKind::Fixed {
            return Ok(());
        }
        for axis in 0..3 {
            let delta = i128::from(self.body.velocity.component(axis))
                .checked_mul(subticks)
                .ok_or(PhysicsError::ArithmeticOverflow(self.body.id))?;
            self.center_scaled[axis] = self.center_scaled[axis]
                .checked_add(delta)
                .ok_or(PhysicsError::ArithmeticOverflow(self.body.id))?;
        }
        Ok(())
    }

    fn quantize_position(&mut self) -> Result<(), PhysicsError> {
        self.body.position = Vec3i::new(
            quantize_axis(self.center_scaled[0], self.body.id)?,
            quantize_axis(self.center_scaled[1], self.body.id)?,
            quantize_axis(self.center_scaled[2], self.body.id)?,
        );
        Ok(())
    }
}

#[derive(Clone, Copy, Debug)]
struct Contact {
    normal: ContactNormal,
    penetration_scaled: i128,
}

#[derive(Clone, Copy, Debug)]
struct IndexedHit {
    left: usize,
    right: usize,
    hit: MotionSweepHit,
}

#[derive(Clone, Copy, Debug)]
struct BroadPhaseBounds {
    body_index: usize,
    min: [i128; 3],
    max: [i128; 3],
}

#[derive(Clone, Debug, Default)]
struct BroadPhaseScratch {
    entries: Vec<BroadPhaseBounds>,
    active: Vec<usize>,
    pairs: Vec<(usize, usize)>,
}

impl BroadPhaseScratch {
    fn clear(&mut self) {
        self.entries.clear();
        self.active.clear();
        self.pairs.clear();
    }

    fn capacity_bytes(&self) -> usize {
        self.entries.capacity() * std::mem::size_of::<BroadPhaseBounds>()
            + self.active.capacity() * std::mem::size_of::<usize>()
            + self.pairs.capacity() * std::mem::size_of::<(usize, usize)>()
    }
}

fn find_earliest_hits(
    states: &[BodyState],
    remaining_subticks: i128,
    scratch: &mut BroadPhaseScratch,
    stats: &mut StepStats,
) -> Vec<IndexedHit> {
    let pair_checks = broad_phase_pairs(states, remaining_subticks, scratch, &mut stats.work);
    stats.pair_checks += pair_checks;

    let mut earliest_time: Option<Ratio> = None;
    let mut hits = Vec::new();

    for &(left, right) in &scratch.pairs {
        stats.work.current_contact_tests += 1;
        if contact_between(&states[left], &states[right]).is_some() {
            continue;
        }

        stats.swept_candidates += 1;
        stats.toi_tests += 1;
        let Some(hit) = sweep_motion(
            states[left].motion(),
            states[right].motion(),
            remaining_subticks,
        ) else {
            continue;
        };

        match earliest_time {
            None => {
                earliest_time = Some(hit.time);
                hits.push(IndexedHit { left, right, hit });
            }
            Some(time) => match hit.time.compare(time) {
                Ordering::Less => {
                    earliest_time = Some(hit.time);
                    hits.clear();
                    hits.push(IndexedHit { left, right, hit });
                }
                Ordering::Equal => hits.push(IndexedHit { left, right, hit }),
                Ordering::Greater => {}
            },
        }
    }
    hits
}

/// Produces deterministic swept candidate pairs using X-axis sweep-and-prune followed by Y/Z
/// interval rejection. The swept bounds cover every position each body can occupy during the
/// remaining constant-velocity interval, so this stage may produce false positives but must not
/// reject a genuine TOI candidate.
fn broad_phase_pairs(
    states: &[BodyState],
    horizon_subticks: i128,
    scratch: &mut BroadPhaseScratch,
    work: &mut TranslationalStepWork,
) -> usize {
    work.broad_phase_queries += 1;
    work.sweep_bound_preparations += states.len();
    work.broad_phase_sorts += 1;
    scratch.clear();
    let previous_capacity = scratch.entries.capacity();
    scratch.entries.reserve_exact(states.len());
    work.broad_phase_capacity_growths +=
        usize::from(scratch.entries.capacity() > previous_capacity);
    scratch
        .entries
        .extend(states.iter().enumerate().map(|(body_index, state)| {
            match state.body.kind {
                BodyKind::Fixed => work.fixed_sweep_bound_preparations += 1,
                BodyKind::Dynamic => work.dynamic_sweep_bound_preparations += 1,
            }
            swept_bounds(body_index, state, horizon_subticks)
        }));
    let BroadPhaseScratch {
        entries,
        active,
        pairs,
    } = scratch;
    entries.sort_by(|left, right| {
        left.min[0]
            .cmp(&right.min[0])
            .then_with(|| {
                states[left.body_index]
                    .body
                    .id
                    .cmp(&states[right.body_index].body.id)
            })
            .then_with(|| left.body_index.cmp(&right.body_index))
    });

    let mut pair_checks = 0_usize;

    for current_index in 0..entries.len() {
        let current = entries[current_index];
        work.active_bound_checks += active.len();
        active.retain(|other_index| entries[*other_index].max[0] >= current.min[0]);

        for &other_index in active.iter() {
            let other = entries[other_index];
            let (left, right) = ordered_pair(other.body_index, current.body_index);
            if states[left].body.kind == BodyKind::Fixed
                && states[right].body.kind == BodyKind::Fixed
            {
                work.fixed_pair_rejections += 1;
                continue;
            }

            pair_checks += 1;
            if intervals_overlap(other.min[1], other.max[1], current.min[1], current.max[1])
                && intervals_overlap(other.min[2], other.max[2], current.min[2], current.max[2])
            {
                push_broad_phase(pairs, (left, right), work);
            }
        }
        push_broad_phase(active, current_index, work);
    }

    work.candidate_buffer_peak_capacity_bytes = work.candidate_buffer_peak_capacity_bytes.max(
        entries.capacity() * std::mem::size_of::<BroadPhaseBounds>()
            + active.capacity() * std::mem::size_of::<usize>()
            + pairs.capacity() * std::mem::size_of::<(usize, usize)>(),
    );
    work.broad_phase_pair_sorts += 1;
    pairs.sort_unstable();
    pair_checks
}

fn push_broad_phase<T>(buffer: &mut Vec<T>, value: T, work: &mut TranslationalStepWork) {
    let previous_capacity = buffer.capacity();
    buffer.push(value);
    work.broad_phase_capacity_growths += usize::from(buffer.capacity() > previous_capacity);
}

fn swept_bounds(body_index: usize, state: &BodyState, horizon_subticks: i128) -> BroadPhaseBounds {
    let motion = state.motion();
    let mut min = [0_i128; 3];
    let mut max = [0_i128; 3];

    for axis in 0..3 {
        let start = motion.center_scaled[axis];
        let end = start + i128::from(motion.velocity[axis]) * horizon_subticks;
        min[axis] = start.min(end) - motion.half_scaled[axis];
        max[axis] = start.max(end) + motion.half_scaled[axis];
    }

    BroadPhaseBounds {
        body_index,
        min,
        max,
    }
}

const fn ordered_pair(left: usize, right: usize) -> (usize, usize) {
    if left < right {
        (left, right)
    } else {
        (right, left)
    }
}

const fn intervals_overlap(
    left_min: i128,
    left_max: i128,
    right_min: i128,
    right_max: i128,
) -> bool {
    left_min <= right_max && right_min <= left_max
}

fn stabilize_contacts(
    states: &mut [BodyState],
    max_passes: usize,
    scratch: &mut BroadPhaseScratch,
    stats: &mut StepStats,
) -> Result<bool, PhysicsError> {
    for _ in 0..max_passes {
        stats.work.stabilization_queries += 1;
        let checks = broad_phase_pairs(states, 0, scratch, &mut stats.work);
        stats.work.stabilization_pair_checks += checks;
        let mut changed = false;

        for &(left, right) in &scratch.pairs {
            stats.work.stabilization_exact_tests += 1;
            let Some(contact) = contact_between(&states[left], &states[right]) else {
                continue;
            };
            if contact.penetration_scaled > 0 {
                project_to_contact(states, left, right, contact.normal)?;
                changed = true;
            }
            if resolve_contact_velocity(states, left, right, contact.normal)? {
                stats.contact_resolutions += 1;
                changed = true;
            }
        }

        if !changed {
            return Ok(true);
        }
        stats.stabilization_passes += 1;
    }
    Ok(max_passes == 0)
}

fn contact_between(left: &BodyState, right: &BodyState) -> Option<Contact> {
    let mut penetration = i128::MAX;
    let mut axis = 0_usize;
    for candidate_axis in 0..3 {
        let distance = left.center_scaled[candidate_axis] - right.center_scaled[candidate_axis];
        let extent = i128::from(left.body.half_extents.component(candidate_axis)) * SUBTICK_SCALE
            + i128::from(right.body.half_extents.component(candidate_axis)) * SUBTICK_SCALE;
        let candidate_penetration = extent - distance.abs();
        if candidate_penetration < 0 {
            return None;
        }
        if candidate_penetration < penetration {
            penetration = candidate_penetration;
            axis = candidate_axis;
        }
    }

    let distance = left.center_scaled[axis] - right.center_scaled[axis];
    let relative_velocity = i64::from(left.body.velocity.component(axis))
        - i64::from(right.body.velocity.component(axis));
    let component = match distance.cmp(&0) {
        Ordering::Greater => 1,
        Ordering::Less => -1,
        Ordering::Equal => match relative_velocity.cmp(&0) {
            Ordering::Greater => -1,
            Ordering::Less => 1,
            Ordering::Equal => {
                if left.body.id < right.body.id {
                    -1
                } else {
                    1
                }
            }
        },
    };
    Some(Contact {
        normal: ContactNormal::for_axis(axis, component),
        penetration_scaled: penetration,
    })
}

fn project_to_contact(
    states: &mut [BodyState],
    left_index: usize,
    right_index: usize,
    normal: ContactNormal,
) -> Result<(), PhysicsError> {
    let (left, right) = two_mut(states, left_index, right_index);
    let axis = normal.axis();
    let extent = i128::from(left.body.half_extents.component(axis)) * SUBTICK_SCALE
        + i128::from(right.body.half_extents.component(axis)) * SUBTICK_SCALE;
    let target_difference = i128::from(normal.component()) * extent;
    let current_difference = left.center_scaled[axis] - right.center_scaled[axis];
    let correction = target_difference - current_difference;
    if correction == 0 {
        return Ok(());
    }

    match (left.body.kind, right.body.kind) {
        (BodyKind::Fixed, BodyKind::Fixed) => {}
        (BodyKind::Dynamic, BodyKind::Fixed) => {
            left.center_scaled[axis] = left.center_scaled[axis]
                .checked_add(correction)
                .ok_or(PhysicsError::ArithmeticOverflow(left.body.id))?;
        }
        (BodyKind::Fixed, BodyKind::Dynamic) => {
            right.center_scaled[axis] = right.center_scaled[axis]
                .checked_sub(correction)
                .ok_or(PhysicsError::ArithmeticOverflow(right.body.id))?;
        }
        (BodyKind::Dynamic, BodyKind::Dynamic) => {
            let left_mass = i128::from(left.body.mass_units);
            let right_mass = i128::from(right.body.mass_units);
            let total_mass = left_mass + right_mass;
            let weighted = correction
                .checked_mul(right_mass)
                .ok_or(PhysicsError::ArithmeticOverflow(left.body.id))?;
            let left_correction = weighted / total_mass;
            let right_correction = correction - left_correction;
            left.center_scaled[axis] = left.center_scaled[axis]
                .checked_add(left_correction)
                .ok_or(PhysicsError::ArithmeticOverflow(left.body.id))?;
            right.center_scaled[axis] = right.center_scaled[axis]
                .checked_sub(right_correction)
                .ok_or(PhysicsError::ArithmeticOverflow(right.body.id))?;
        }
    }
    Ok(())
}

fn resolve_contact_velocity(
    states: &mut [BodyState],
    left_index: usize,
    right_index: usize,
    normal: ContactNormal,
) -> Result<bool, PhysicsError> {
    let (left, right) = two_mut(states, left_index, right_index);
    let axis = normal.axis();
    let left_velocity = left.body.velocity.component(axis);
    let right_velocity = right.body.velocity.component(axis);
    let left_value = i128::from(left_velocity);
    let right_value = i128::from(right_velocity);
    let relative_velocity = left_value - right_value;
    if relative_velocity * i128::from(normal.component()) >= 0 {
        return Ok(false);
    }

    let restitution = i128::from(
        left.body
            .material
            .restitution_milli()
            .min(right.body.material.restitution_milli()),
    );
    let scale = i128::from(MATERIAL_SCALE);

    match (left.body.kind, right.body.kind) {
        (BodyKind::Fixed, BodyKind::Fixed) => return Ok(false),
        (BodyKind::Dynamic, BodyKind::Fixed) => {
            let value = right_value - (left_value - right_value) * restitution / scale;
            left.body.velocity.set_component(
                axis,
                i32::try_from(value).map_err(|_| PhysicsError::ArithmeticOverflow(left.body.id))?,
            );
        }
        (BodyKind::Fixed, BodyKind::Dynamic) => {
            let value = left_value - (right_value - left_value) * restitution / scale;
            right.body.velocity.set_component(
                axis,
                i32::try_from(value)
                    .map_err(|_| PhysicsError::ArithmeticOverflow(right.body.id))?,
            );
        }
        (BodyKind::Dynamic, BodyKind::Dynamic) => {
            let left_mass = i128::from(left.body.mass_units);
            let right_mass = i128::from(right.body.mass_units);
            let denominator = scale * (left_mass + right_mass);
            let momentum = scale * (left_mass * left_value + right_mass * right_value);
            let relative = left_value - right_value;
            let next_left = (momentum - restitution * right_mass * relative) / denominator;
            let next_right = (momentum + restitution * left_mass * relative) / denominator;
            left.body.velocity.set_component(
                axis,
                i32::try_from(next_left)
                    .map_err(|_| PhysicsError::ArithmeticOverflow(left.body.id))?,
            );
            right.body.velocity.set_component(
                axis,
                i32::try_from(next_right)
                    .map_err(|_| PhysicsError::ArithmeticOverflow(right.body.id))?,
            );
        }
    }
    Ok(true)
}

fn advance_all(states: &mut [BodyState], subticks: i128) -> Result<(), PhysicsError> {
    for state in states {
        state.advance(subticks)?;
    }
    Ok(())
}

fn two_mut(
    states: &mut [BodyState],
    left: usize,
    right: usize,
) -> (&mut BodyState, &mut BodyState) {
    debug_assert!(left < right);
    let (before_right, from_right) = states.split_at_mut(right);
    (&mut before_right[left], &mut from_right[0])
}

fn quantize_axis(value: i128, id: BodyId) -> Result<i32, PhysicsError> {
    let half = SUBTICK_SCALE / 2;
    let rounded = if value >= 0 {
        (value + half) / SUBTICK_SCALE
    } else {
        (value - half) / SUBTICK_SCALE
    };
    i32::try_from(rounded).map_err(|_| PhysicsError::ArithmeticOverflow(id))
}

#[cfg(test)]
mod maintenance_tests {
    use super::*;
    use std::{hint::black_box, time::Instant};

    mod fixed_bound_workload {
        use crate as physics_engine;
        include!("../tests/support/fixed_bound_workload.rs");
    }

    #[test]
    #[ignore = "isolated preparation-stage timing; run explicitly in release mode"]
    fn fixed_heavy_bound_preparation_stage() {
        let input = fixed_bound_workload::input();
        // The shared control also checks actual query counters and rebuilding parity.
        black_box(fixed_bound_workload::run());
        for count in [8, 64, 128] {
            let mut world = input.world(count, input.gravity);
            world.step(1).unwrap();
            let mut states = world
                .bodies()
                .cloned()
                .map(BodyState::new)
                .collect::<Vec<_>>();
            for state in &mut states {
                state.apply_gravity(input.gravity, 1).unwrap();
            }
            let (fixed, dynamic): (Vec<_>, Vec<_>) = states
                .iter()
                .enumerate()
                .partition(|(_, state)| state.body.kind == BodyKind::Fixed);
            assert_eq!(fixed.len(), 261);
            assert_eq!(dynamic.len(), count);
            for horizon in [0, SUBTICK_SCALE] {
                for trial in 0..9 {
                    let measure = |rows: &[(usize, &BodyState)]| {
                        let start = Instant::now();
                        for _ in 0..120 {
                            for &(index, state) in rows {
                                black_box(swept_bounds(
                                    index,
                                    black_box(state),
                                    black_box(horizon),
                                ));
                            }
                        }
                        start.elapsed().as_secs_f64() * 1000.0
                    };
                    let (fixed_ms, dynamic_ms) = if trial % 2 == 0 {
                        (measure(&fixed), measure(&dynamic))
                    } else {
                        let dynamic_ms = measure(&dynamic);
                        (measure(&fixed), dynamic_ms)
                    };
                    println!(
                        "FIXED_BOUND_STAGE {{\"dynamic_bodies\":{count},\"fixed_bodies\":261,\"trial\":{trial},\"repetitions\":120,\"horizon_subticks\":{horizon},\"fixed_ms\":{fixed_ms},\"dynamic_ms\":{dynamic_ms}}}"
                    );
                }
            }
        }
    }

    fn stationary_world(count: u64) -> World {
        let mut world = World::new(WorldConfig {
            gravity: Vec3i::ZERO,
            ..WorldConfig::default()
        });
        for id in 0..count {
            let rank = id.wrapping_mul(7_919) % count;
            let position = Vec3i::new(
                i32::try_from(rank % 32).unwrap() * 100 - 1600,
                20,
                i32::try_from(rank / 32).unwrap() * 100 - 3200,
            );
            world
                .add_body(RigidBody::dynamic(
                    BodyId(id),
                    position,
                    Vec3i::ZERO,
                    Vec3i::new(10, 20, 10),
                ))
                .unwrap();
        }
        world
    }

    use super::rebuilding_reference as reference;

    fn worlds(config: WorldConfig, bodies: Vec<RigidBody>) -> (World, reference::World) {
        let mut candidate = World::new(config);
        let mut oracle = reference::World::new(reference::WorldConfig {
            gravity: config.gravity,
            max_events_per_step: config.max_events_per_step,
            stabilization_passes: config.stabilization_passes,
        });
        for body in bodies {
            candidate.add_body(body.clone()).unwrap();
            oracle.add_body(body).unwrap();
        }
        assert_eq!(candidate.config().gravity, oracle.config().gravity);
        (candidate, oracle)
    }

    fn step_matches(candidate: &mut World, oracle: &mut reference::World, ticks: i32) {
        let before = candidate.bodies().cloned().collect::<Vec<_>>();
        let actual = candidate.step(ticks);
        assert!(candidate.staged_states.is_empty());
        assert!(candidate.broad_phase.entries.is_empty());
        assert!(candidate.broad_phase.active.is_empty());
        assert!(candidate.broad_phase.pairs.is_empty());
        let expected = oracle.step(ticks);
        match (actual, expected) {
            (Ok(actual), Ok(expected)) => {
                let events = actual
                    .events
                    .iter()
                    .map(|e| (e.left, e.right, e.normal, e.time))
                    .collect::<Vec<_>>();
                let oracle_events = expected
                    .events
                    .iter()
                    .map(|e| (e.left, e.right, e.normal, e.time))
                    .collect::<Vec<_>>();
                assert_eq!(events, oracle_events);
                assert_eq!(
                    (
                        actual.stats.collision_events,
                        actual.stats.contact_resolutions,
                        actual.stats.stabilization_passes,
                        actual.stats.toi_tests
                    ),
                    (
                        expected.stats.collision_events,
                        expected.stats.contact_resolutions,
                        expected.stats.stabilization_passes,
                        expected.stats.toi_tests
                    )
                );
                assert_eq!(actual.stats.work.body_map_rebuilds, 0);
                assert_eq!(actual.stats.work.body_map_insertions, 0);
            }
            (Err(actual), Err(expected)) => {
                assert_eq!(actual.to_string(), expected.to_string());
                assert_eq!(
                    candidate.bodies().cloned().collect::<Vec<_>>(),
                    before,
                    "failure published partial state"
                );
            }
            (actual, expected) => {
                panic!("different result: candidate={actual:?}, oracle={expected:?}")
            }
        }
        assert_eq!(
            candidate.bodies().cloned().collect::<Vec<_>>(),
            oracle.bodies().cloned().collect::<Vec<_>>()
        );
        assert_eq!(
            candidate.dynamic_body_count,
            candidate
                .bodies()
                .filter(|b| b.kind() == BodyKind::Dynamic)
                .count()
        );
    }

    #[test]
    fn retained_broad_phase_matches_exhaustive_pairs_after_horizon_and_population_changes() {
        let make_state = |id: u64| {
            let position = Vec3i::new(
                (id % 5) as i32 * 8 - 20,
                (id % 3) as i32 * 5 - 10,
                id as i32 - 16,
            );
            BodyState::new(if id.is_multiple_of(7) {
                RigidBody::fixed(BodyId(id), position, Vec3i::new(5, 5, 5))
            } else {
                RigidBody::dynamic(
                    BodyId(id),
                    position,
                    Vec3i::new(id as i32 % 7 - 3, 0, 1),
                    Vec3i::new(5, 5, 5),
                )
            })
        };
        let mut states = (0..32).map(make_state).collect::<Vec<_>>();
        let mut scratch = BroadPhaseScratch::default();
        for round in 0..12 {
            if round == 3 {
                states.truncate(8);
            }
            if round == 6 {
                states.extend((8..48).map(make_state));
            }
            if round == 9 {
                states[1] = BodyState::new(RigidBody::fixed(
                    BodyId(1),
                    Vec3i::new(-100, 0, 0),
                    Vec3i::new(1, 1, 1),
                ));
            }
            for horizon in [0, SUBTICK_SCALE, 5 * SUBTICK_SCALE] {
                let mut expected = Vec::new();
                for left in 0..states.len() {
                    for right in left + 1..states.len() {
                        if states[left].body.kind == BodyKind::Fixed
                            && states[right].body.kind == BodyKind::Fixed
                        {
                            continue;
                        }
                        let a = swept_bounds(left, &states[left], horizon);
                        let b = swept_bounds(right, &states[right], horizon);
                        if (0..3)
                            .all(|axis| a.min[axis] <= b.max[axis] && b.min[axis] <= a.max[axis])
                        {
                            expected.push((left, right));
                        }
                    }
                }
                broad_phase_pairs(
                    &states,
                    horizon,
                    &mut scratch,
                    &mut TranslationalStepWork::default(),
                );
                assert_eq!(scratch.pairs, expected);
                let pointers = (
                    scratch.entries.as_ptr(),
                    scratch.active.as_ptr(),
                    scratch.pairs.as_ptr(),
                );
                let mut warmed = TranslationalStepWork::default();
                broad_phase_pairs(&states, horizon, &mut scratch, &mut warmed);
                assert_eq!(scratch.pairs, expected);
                assert_eq!(warmed.broad_phase_capacity_growths, 0);
                assert_eq!(
                    (
                        scratch.entries.as_ptr(),
                        scratch.active.as_ptr(),
                        scratch.pairs.as_ptr()
                    ),
                    pointers
                );
            }
        }
    }

    #[test]
    fn retained_staging_growth_and_shrink_match_the_exhaustive_rebuilding_oracle() {
        let body = |id| {
            RigidBody::dynamic(
                BodyId(id),
                Vec3i::new(id as i32 * 100 - 10_000, 20, -30),
                if id == 1 {
                    Vec3i::new(1, 0, 0)
                } else {
                    Vec3i::ZERO
                },
                Vec3i::new(10, 20, 10),
            )
        };
        let (mut candidate, mut oracle) = worlds(
            WorldConfig {
                gravity: Vec3i::ZERO,
                ..Default::default()
            },
            (0..16).map(body).collect(),
        );
        step_matches(&mut candidate, &mut oracle, 1);
        let small_capacity = candidate.retained_step_scratch_bytes();
        for id in 16..129 {
            candidate.add_body(body(id)).unwrap();
            oracle.add_body(body(id)).unwrap();
        }
        step_matches(&mut candidate, &mut oracle, 1);
        let high_water = candidate.retained_step_scratch_bytes();
        assert!(high_water > small_capacity);
        for id in 16..129 {
            assert_eq!(
                candidate.remove_body(BodyId(id)),
                oracle.remove_body(BodyId(id))
            );
        }
        step_matches(&mut candidate, &mut oracle, 1);
        assert_eq!(candidate.retained_step_scratch_bytes(), high_water);
        candidate.release_step_scratch();
        step_matches(&mut candidate, &mut oracle, 1);
    }

    #[test]
    fn staging_allocation_is_reused_and_logically_empty_at_step_boundaries() {
        let mut world = stationary_world(128);
        world.set_velocity(BodyId(9), Vec3i::new(1, 0, 0)).unwrap();
        let first = world.step(1).unwrap();
        assert_eq!(first.stats.work.staged_state_capacity_growths, 1);
        let allocation = world.staged_states.as_ptr();
        let payload = world.retained_step_scratch_bytes();
        for _ in 0..32 {
            let report = world.step(1).unwrap();
            assert_eq!(report.stats.work.staged_state_capacity_growths, 0);
            assert_eq!(report.stats.work.staged_bodies, 128);
            assert_eq!(world.staged_states.as_ptr(), allocation);
            assert!(world.staged_states.is_empty());
            assert_eq!(world.retained_step_scratch_bytes(), payload);
        }
        world
            .set_position(BodyId(9), Vec3i::new(i32::MAX, 20, -3200))
            .unwrap();
        let before = world.bodies().cloned().collect::<Vec<_>>();
        for _ in 0..2 {
            assert_eq!(
                world.step(1),
                Err(PhysicsError::ArithmeticOverflow(BodyId(9)))
            );
            assert_eq!(world.staged_states.as_ptr(), allocation);
            assert!(world.staged_states.is_empty());
            assert_eq!(world.bodies().cloned().collect::<Vec<_>>(), before);
        }
        world.set_velocity(BodyId(9), Vec3i::new(-1, 0, 0)).unwrap();
        assert_eq!(
            world
                .step(1)
                .unwrap()
                .stats
                .work
                .staged_state_capacity_growths,
            0
        );
    }

    #[test]
    fn unchanged_stationary_ticks_do_no_staging_queries_or_map_rebuilds() {
        let mut world = stationary_world(512);
        let first = world.step(1).unwrap();
        assert_eq!(first.stats.work.staged_bodies, 512);
        assert_eq!(first.stats.work.broad_phase_queries, 1);
        let before = world.bodies().cloned().collect::<Vec<_>>();
        // Repeated identical commands from a controller do not invalidate derived state.
        world.set_velocity(BodyId(0), Vec3i::ZERO).unwrap();
        world
            .set_position(BodyId(0), world.body(BodyId(0)).unwrap().position())
            .unwrap();
        world
            .set_material(BodyId(0), world.body(BodyId(0)).unwrap().material())
            .unwrap();
        world
            .replace_body(world.body(BodyId(0)).unwrap().clone())
            .unwrap();
        assert_eq!(world.remove_body(BodyId(99999)), None);
        for _ in 0..120 {
            let report = world.step(1).unwrap();
            assert_eq!(
                report.stats.work,
                TranslationalStepWork {
                    cached_stationary_step: true,
                    dynamic_bodies: 512,
                    ..TranslationalStepWork::default()
                }
            );
            assert!(report.events.is_empty());
        }
        assert_eq!(world.bodies().cloned().collect::<Vec<_>>(), before);
        assert_eq!(world.step(0), Err(PhysicsError::NonPositiveTicks(0)));
    }

    #[test]
    fn fixed_only_and_empty_worlds_reuse_evidence_with_nonzero_gravity() {
        let mut world = World::default();
        for count in [0, 8] {
            for id in 0..count {
                world
                    .add_body(RigidBody::fixed(
                        BodyId(id),
                        Vec3i::new(id as i32, 0, 0),
                        Vec3i::new(1, 1, 1),
                    ))
                    .unwrap();
            }
            world.step(1).unwrap();
            let report = world.step(i32::MAX).unwrap();
            assert_eq!(
                report.stats.work,
                TranslationalStepWork {
                    cached_stationary_step: true,
                    ..TranslationalStepWork::default()
                }
            );
            assert_eq!(report.stats.body_count, count as usize);
        }
    }

    #[test]
    fn successful_steps_retain_body_map_storage_and_only_publish_physical_deltas() {
        let mut world = stationary_world(128);
        world.set_velocity(BodyId(9), Vec3i::new(1, 0, 0)).unwrap();
        let slots = world
            .bodies()
            .map(|body| (body.id(), std::ptr::from_ref(body)))
            .collect::<Vec<_>>();
        for _ in 0..32 {
            let report = world.step(1).unwrap();
            assert_eq!(report.stats.work.committed_position_deltas, 1);
            assert_eq!(report.stats.work.committed_velocity_deltas, 0);
            assert_eq!(
                world
                    .bodies()
                    .map(|body| (body.id(), std::ptr::from_ref(body)))
                    .collect::<Vec<_>>(),
                slots
            );
        }
    }

    #[test]
    fn retained_stationary_evidence_matches_rebuild_after_every_mutation_kind() {
        let bodies = stationary_world(32).bodies().cloned().collect();
        let (mut candidate, mut oracle) = worlds(
            WorldConfig {
                gravity: Vec3i::ZERO,
                ..WorldConfig::default()
            },
            bodies,
        );
        for _ in 0..3 {
            step_matches(&mut candidate, &mut oracle, 1);
        }
        for tick in 0..96 {
            let id = BodyId(tick % 32);
            let velocity = Vec3i::new(if tick % 2 == 0 { 3 } else { -3 }, 0, 1);
            candidate.set_velocity(id, velocity).unwrap();
            oracle.set_velocity(id, velocity).unwrap();
            if tick % 7 == 0 {
                let position = Vec3i::new(-400 + tick as i32, 20, -300);
                candidate.set_position(id, position).unwrap();
                oracle.set_position(id, position).unwrap();
            }
            if tick % 9 == 0 {
                let material = Material::new(500).with_friction(200);
                candidate.set_material(id, material).unwrap();
                oracle.set_material(id, material).unwrap();
            }
            if tick % 11 == 0 {
                let replacement = RigidBody::fixed(
                    id,
                    candidate.body(id).unwrap().position(),
                    Vec3i::new(7, 15, 9),
                );
                candidate.replace_body(replacement.clone()).unwrap();
                oracle.replace_body(replacement).unwrap();
            }
            step_matches(&mut candidate, &mut oracle, 1);
            let replacement = RigidBody::dynamic(
                id,
                candidate.body(id).unwrap().position(),
                Vec3i::ZERO,
                Vec3i::new(10, 20, 10),
            )
            .with_mass(3);
            candidate.replace_body(replacement.clone()).unwrap();
            oracle.replace_body(replacement).unwrap();
            assert_eq!(candidate.remove_body(id), oracle.remove_body(id));
            let replacement = RigidBody::dynamic(
                id,
                Vec3i::new(-500 + tick as i32, 20, -500),
                Vec3i::ZERO,
                Vec3i::new(11, 17, 8),
            );
            candidate.add_body(replacement.clone()).unwrap();
            oracle.add_body(replacement).unwrap();
            step_matches(&mut candidate, &mut oracle, 2);
            assert_eq!(candidate.body(id), oracle.body(id));
        }
    }

    #[test]
    fn late_quantization_and_event_limit_failures_preserve_all_authoritative_bodies() {
        for event_limit in [0, 256] {
            let bodies = vec![
                RigidBody::dynamic(
                    BodyId(1),
                    Vec3i::new(-10, 0, 0),
                    Vec3i::new(40, 0, 0),
                    Vec3i::new(1, 1, 1),
                ),
                RigidBody::fixed(BodyId(2), Vec3i::ZERO, Vec3i::new(1, 8, 8)),
                RigidBody::dynamic(
                    BodyId(99),
                    Vec3i::new(i32::MAX - 1, 0, 0),
                    Vec3i::new(4, 0, 0),
                    Vec3i::new(1, 1, 1),
                ),
            ];
            let (mut candidate, mut oracle) = worlds(
                WorldConfig {
                    gravity: Vec3i::ZERO,
                    max_events_per_step: event_limit,
                    ..WorldConfig::default()
                },
                bodies,
            );
            assert!(candidate.step(1).is_err());
            step_matches(&mut candidate, &mut oracle, 1);
            candidate.set_velocity(BodyId(99), Vec3i::ZERO).unwrap();
            oracle.set_velocity(BodyId(99), Vec3i::ZERO).unwrap();
            candidate.remove_body(BodyId(2));
            oracle.remove_body(BodyId(2));
            step_matches(&mut candidate, &mut oracle, 1);
        }
    }

    #[test]
    fn gravity_supported_dense_spawn_and_zero_stabilization_controls_match_exhaustive_reference() {
        for gravity in [Vec3i::ZERO, Vec3i::new(0, -1, 0)] {
            for passes in [0, 1, 8] {
                let mut bodies = vec![RigidBody::fixed(
                    BodyId(0),
                    Vec3i::new(0, -10, 0),
                    Vec3i::new(1000, 10, 1000),
                )];
                for id in 1..=16 {
                    bodies.push(RigidBody::dynamic(
                        BodyId(id),
                        Vec3i::new((id % 4) as i32 * 18 - 40, 20, (id / 4) as i32 * 18 - 40),
                        Vec3i::ZERO,
                        Vec3i::new(10, 20, 10),
                    ));
                }
                let (mut candidate, mut oracle) = worlds(
                    WorldConfig {
                        gravity,
                        stabilization_passes: passes,
                        ..WorldConfig::default()
                    },
                    bodies,
                );
                for _ in 0..32 {
                    step_matches(&mut candidate, &mut oracle, 1);
                }
                // Removing a support is a real dependency change; gravity must run again.
                assert_eq!(
                    candidate.remove_body(BodyId(0)),
                    oracle.remove_body(BodyId(0))
                );
                for _ in 0..16 {
                    step_matches(&mut candidate, &mut oracle, 1);
                }
            }
        }
    }

    #[test]
    #[ignore = "deterministic translational work ratchet; invoked by the existing performance harness"]
    fn translational_necessary_work_ratchet() {
        for (mode, count, moving) in [
            ("quiet", 128, 0),
            ("quiet", 512, 0),
            ("quiet", 2048, 0),
            ("local", 512, 8),
            ("supported", 512, 0),
        ] {
            let mut world = stationary_world(count);
            if mode == "supported" {
                world.config.gravity = Vec3i::new(0, -1, 0);
                world
                    .add_body(RigidBody::fixed(
                        BodyId(100_000),
                        Vec3i::new(0, -10, 0),
                        Vec3i::new(100_000, 10, 100_000),
                    ))
                    .unwrap();
            }
            let initial = world.bodies().cloned().collect::<Vec<_>>();
            world.step(1).unwrap();
            let mut staged = 0;
            let mut quantized = 0;
            let mut queries = 0;
            let mut bounds = 0;
            let mut sorts = 0;
            let mut map_rebuilds = 0;
            let mut map_insertions = 0;
            let mut stabilization_tests = 0;
            let mut scratch_capacity = 0;
            let mut position_deltas = 0;
            let mut times = Vec::new();
            for tick in 0..64 {
                for id in 0..moving.max(1) {
                    let velocity = Vec3i::new(
                        if id < moving {
                            if tick % 2 == 0 { 1 } else { -1 }
                        } else {
                            0
                        },
                        0,
                        0,
                    );
                    world.set_velocity(BodyId(id), velocity).unwrap();
                }
                let start = Instant::now();
                let report = black_box(world.step(1).unwrap());
                times.push(start.elapsed().as_secs_f64() * 1000.0);
                assert!(report.events.is_empty());
                let work = report.stats.work;
                staged += work.staged_bodies as u64;
                quantized += work.quantized_bodies as u64;
                queries += work.broad_phase_queries as u64;
                bounds += work.sweep_bound_preparations as u64;
                sorts += work.broad_phase_sorts as u64;
                map_rebuilds += work.body_map_rebuilds as u64;
                map_insertions += work.body_map_insertions as u64;
                stabilization_tests += work.stabilization_exact_tests as u64;
                scratch_capacity = scratch_capacity.max(
                    work.staged_state_capacity_bytes + work.candidate_buffer_peak_capacity_bytes,
                );
                position_deltas += work.committed_position_deltas as u64;
                assert_eq!(work.committed_position_deltas, moving as usize);
            }
            let final_bodies = world.bodies().cloned().collect::<Vec<_>>();
            for (before, after) in initial.iter().zip(&final_bodies) {
                assert_eq!(before.position(), after.position());
                assert_eq!(before.material(), after.material());
                assert_eq!(before.mass_units(), after.mass_units());
            }
            assert_eq!(map_rebuilds, 0);
            assert_eq!(map_insertions, 0);
            if mode == "quiet" {
                assert_eq!(staged, 0);
                assert_eq!(queries, 0);
                assert_eq!(scratch_capacity, 0);
            }
            crate::performance_ratchet::record(
                &format!("translational/{mode}/{count}"),
                &[
                    ("staged_bodies", staged),
                    ("quantized_bodies", quantized),
                    ("broad_phase_queries", queries),
                    ("bounds_prepared", bounds),
                    ("sorts", sorts),
                    ("body_map_rebuilds", map_rebuilds),
                    ("body_map_insertions", map_insertions),
                    ("stabilization_tests", stabilization_tests),
                    ("working_vector_capacity_bytes", scratch_capacity as u64),
                ],
                &[
                    ("bodies", world.bodies.len() as u64),
                    ("completed_ticks", 64),
                    ("position_deltas", position_deltas),
                ],
                &[("elapsed_ms", times.iter().sum())],
            );
        }
    }

    #[test]
    #[ignore = "stage timing evidence; run explicitly in release mode"]
    fn translational_world_maintenance_baseline() {
        for count in [128, 512, 2048] {
            let mut world = stationary_world(count);
            let expected = world.bodies().cloned().collect::<Vec<_>>();
            let start = Instant::now();
            let mut last = StepStats::default();
            for _ in 0..120 {
                last = black_box(world.step(1).unwrap()).stats;
            }
            let whole = start.elapsed().as_secs_f64() * 1000.0;
            assert_eq!(world.bodies().cloned().collect::<Vec<_>>(), expected);
            let mut staging_ms = 0.0;
            let mut query_ms = 0.0;
            let mut rebuild_ms = 0.0;
            for _ in 0..120 {
                let start = Instant::now();
                let states = black_box(&world.bodies)
                    .values()
                    .cloned()
                    .map(BodyState::new)
                    .collect::<Vec<_>>();
                staging_ms += start.elapsed().as_secs_f64() * 1000.0;
                let start = Instant::now();
                black_box(broad_phase_pairs(
                    black_box(&states),
                    SUBTICK_SCALE,
                    &mut BroadPhaseScratch::default(),
                    &mut TranslationalStepWork::default(),
                ));
                query_ms += start.elapsed().as_secs_f64() * 1000.0;
                let start = Instant::now();
                let mut next = BTreeMap::new();
                for mut state in states {
                    state.quantize_position().unwrap();
                    next.insert(state.body.id, state.body);
                }
                black_box(next);
                rebuild_ms += start.elapsed().as_secs_f64() * 1000.0;
            }
            println!(
                "TRANSLATIONAL_BASELINE count={count} ticks=120 whole_ms={whole} staging_ms={staging_ms} one_query_ms={query_ms} quantize_map_rebuild_ms={rebuild_ms} stats={last:?}"
            );
        }
    }
}

#[cfg(test)]
#[path = "../tests/reference/translational_world.rs"]
mod rebuilding_reference;
