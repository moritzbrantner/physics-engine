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

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct StepStats {
    pub body_count: usize,
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
        self.bodies.insert(body.id, body);
        Ok(())
    }

    pub fn replace_body(&mut self, body: RigidBody) -> Result<(), PhysicsError> {
        validate_body(&body)?;
        if !self.bodies.contains_key(&body.id) {
            return Err(PhysicsError::MissingBody(body.id));
        }
        self.bodies.insert(body.id, body);
        Ok(())
    }

    pub fn remove_body(&mut self, id: BodyId) -> Option<RigidBody> {
        self.bodies.remove(&id)
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
        body.velocity = velocity;
        Ok(())
    }

    pub fn set_position(&mut self, id: BodyId, position: Vec3i) -> Result<(), PhysicsError> {
        let body = self
            .bodies
            .get_mut(&id)
            .ok_or(PhysicsError::MissingBody(id))?;
        body.position = position;
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
        body.material = material;
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

        let mut states = self
            .bodies
            .values()
            .cloned()
            .map(BodyState::new)
            .collect::<Vec<_>>();
        for state in &mut states {
            state.apply_gravity(self.config.gravity, ticks)?;
        }

        let mut report = StepReport {
            events: Vec::new(),
            stats: StepStats {
                body_count: states.len(),
                ..StepStats::default()
            },
        };
        stabilize_contacts(
            &mut states,
            self.config.stabilization_passes,
            &mut report.stats,
        )?;

        let mut remaining_subticks = i128::from(ticks) * SUBTICK_SCALE;
        let mut elapsed_subticks = 0_i128;
        let mut event_count = 0_usize;

        while remaining_subticks > 0 {
            let hits = find_earliest_hits(&states, remaining_subticks, &mut report.stats);
            if hits.is_empty() {
                advance_all(&mut states, remaining_subticks)?;
                break;
            }

            if event_count.saturating_add(hits.len()) > self.config.max_events_per_step {
                return Err(PhysicsError::EventLimit(self.config.max_events_per_step));
            }

            let advance_subticks = hits[0].hit.time.ceil().max(1).min(remaining_subticks);
            advance_all(&mut states, advance_subticks)?;
            remaining_subticks -= advance_subticks;
            elapsed_subticks += advance_subticks;

            for hit in &hits {
                project_to_contact(&mut states, hit.left, hit.right, hit.hit.normal)?;
            }
            for hit in hits {
                let (left_id, right_id) = (states[hit.left].body.id, states[hit.right].body.id);
                if resolve_contact_velocity(&mut states, hit.left, hit.right, hit.hit.normal)? {
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

            stabilize_contacts(
                &mut states,
                self.config.stabilization_passes,
                &mut report.stats,
            )?;
        }

        let mut next_bodies = BTreeMap::new();
        for mut state in states {
            state.quantize_position()?;
            next_bodies.insert(state.body.id, state.body);
        }
        self.bodies = next_bodies;
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

fn find_earliest_hits(
    states: &[BodyState],
    remaining_subticks: i128,
    stats: &mut StepStats,
) -> Vec<IndexedHit> {
    let (candidate_pairs, pair_checks) = broad_phase_pairs(states, remaining_subticks);
    stats.pair_checks += pair_checks;

    let mut earliest_time: Option<Ratio> = None;
    let mut hits = Vec::new();

    for (left, right) in candidate_pairs {
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
fn broad_phase_pairs(states: &[BodyState], horizon_subticks: i128) -> (Vec<(usize, usize)>, usize) {
    let mut entries = states
        .iter()
        .enumerate()
        .map(|(body_index, state)| swept_bounds(body_index, state, horizon_subticks))
        .collect::<Vec<_>>();
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

    let mut active = Vec::<usize>::new();
    let mut pairs = Vec::new();
    let mut pair_checks = 0_usize;

    for current_index in 0..entries.len() {
        let current = entries[current_index];
        active.retain(|other_index| entries[*other_index].max[0] >= current.min[0]);

        for &other_index in &active {
            let other = entries[other_index];
            let (left, right) = ordered_pair(other.body_index, current.body_index);
            if states[left].body.kind == BodyKind::Fixed
                && states[right].body.kind == BodyKind::Fixed
            {
                continue;
            }

            pair_checks += 1;
            if intervals_overlap(other.min[1], other.max[1], current.min[1], current.max[1])
                && intervals_overlap(other.min[2], other.max[2], current.min[2], current.max[2])
            {
                pairs.push((left, right));
            }
        }
        active.push(current_index);
    }

    pairs.sort_unstable();
    (pairs, pair_checks)
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
    stats: &mut StepStats,
) -> Result<(), PhysicsError> {
    for _ in 0..max_passes {
        let (candidate_pairs, _) = broad_phase_pairs(states, 0);
        let mut changed = false;

        for (left, right) in candidate_pairs {
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
            break;
        }
        stats.stabilization_passes += 1;
    }
    Ok(())
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
