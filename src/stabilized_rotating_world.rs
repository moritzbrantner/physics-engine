use std::collections::{BTreeMap, BTreeSet};

use crate::{
    AngularVelocity3d, BodyId, BodyKind, OrientedBox3d, RigidBox3d, RigidBoxFreeFlightConfig3d,
    RotatingContactResponseError3d, RotatingWorldConfig3d, RotatingWorldError3d,
    RotatingWorldStepReport3d, RotationalSweepBounds3d, Vec3i, obb_contact_seed,
    obb_response::resolve_obb_contact, rigid_box_free_flight_sweep_bounds,
    rotating_world::RotatingWorld3d as InnerRotatingWorld3d,
};

const MAX_FIXED_POSITION_STABILIZATION_PASSES: u8 = 16;
const SLEEP_STABLE_STEPS: u8 = 12;
const SLEEP_LINEAR_SPEED_LIMIT: u32 = 120;
const SLEEP_ANGULAR_SPEED_LIMIT: u32 = 75_000;

/// Engine-owned rotating world with bounded fixed-boundary stabilization and deterministic sleeping.
///
/// The inner rotating solver remains the sole authority for impulses, dynamic/dynamic response,
/// restitution, friction, and angular response. After one requested non-zero world step, this facade only
/// removes residual integer penetration between fixed and dynamic bodies. The fixed-boundary pass evaluates
/// every constraint from one shared snapshot and commits at most one combined positional correction per
/// dynamic body per pass, so adjacent or duplicated fixed surfaces cannot sequentially move the same body
/// several times inside one stabilization pass. Every velocity and orientation result from the simultaneous
/// solver is preserved.
///
/// Dynamic bodies whose linear and angular speeds remain below the deterministic sleep thresholds for a
/// bounded number of consecutive steps are put to sleep only when they are already motionless or a current
/// contact can physically dissipate the residual motion. A contact qualifies when its normal constraint is
/// still opposing relative approach, or when non-zero pair friction belongs to a low-motion support chain
/// that is ultimately anchored by fixed geometry or an existing sleeper. Gravity-supported bodies settle
/// position-only against already anchored supports before becoming sleepers, so quantized equal-mass
/// projection cannot freeze residual penetration into a resting stack. This preserves ordinary low-speed
/// free-flight and frictionless tangential inertia while letting gravity-loaded rough stacks converge to
/// sleep as one supported island instead of repeatedly waking their lower members. Sleeping bodies are
/// presented to the inner solver as fixed proxies, so gravity and persistent-contact stabilization cannot
/// keep nudging a settled body. Before each step, conservative free-flight sweep bounds wake every sleeping
/// body that an awake dynamic body may reach, including transitive sleeping islands. Explicit velocity
/// changes also wake their target, adding fixed geometry invalidates existing sleepers conservatively, and
/// any successful body removal invalidates sleep because world membership and contact topology changed.
///
/// The requested frame is staged on a clone and committed only after the authoritative step, fixed-boundary
/// stabilization, proxy restoration, and sleep-state update succeed. Failed frames therefore leave the
/// public world unchanged. A zero timestep keeps the inner world's exact no-op contract and deliberately
/// skips stabilization and sleep bookkeeping.
///
/// Multiple fixed boundaries can still constrain one body, so the position-only pass is independently
/// bounded and fails closed if those fixed constraints cannot reach an idempotent state.
#[derive(Clone, Debug)]
pub struct RotatingWorld3d {
    inner: InnerRotatingWorld3d,
    sleeping: BTreeSet<BodyId>,
    sleep_streaks: BTreeMap<BodyId, u8>,
}

impl RotatingWorld3d {
    #[must_use]
    pub fn new(config: RotatingWorldConfig3d) -> Self {
        Self {
            inner: InnerRotatingWorld3d::new(config),
            sleeping: BTreeSet::new(),
            sleep_streaks: BTreeMap::new(),
        }
    }

    #[must_use]
    pub fn config(&self) -> RotatingWorldConfig3d {
        self.inner.config()
    }

    pub fn add_box(&mut self, rigid_box: RigidBox3d) -> Result<(), RotatingWorldError3d> {
        let invalidates_sleep = rigid_box.body.kind == BodyKind::Fixed;
        self.inner.add_box(rigid_box)?;
        if invalidates_sleep {
            self.wake_all_sleepers();
        }
        Ok(())
    }

    pub fn remove_box(&mut self, id: BodyId) -> Option<RigidBox3d> {
        let removed = self.inner.remove_box(id)?;
        self.wake_all_sleepers();
        Some(removed)
    }

    #[must_use]
    pub fn box_by_id(&self, id: BodyId) -> Option<&RigidBox3d> {
        self.inner.box_by_id(id)
    }

    pub fn boxes(&self) -> impl Iterator<Item = &RigidBox3d> {
        self.inner.boxes()
    }

    #[must_use]
    pub fn is_sleeping(&self, id: BodyId) -> bool {
        self.sleeping.contains(&id)
    }

    #[must_use]
    pub fn sleeping_body_count(&self) -> usize {
        self.sleeping.len()
    }

    pub fn set_linear_velocity(
        &mut self,
        id: BodyId,
        velocity: Vec3i,
    ) -> Result<(), RotatingWorldError3d> {
        self.inner.set_linear_velocity(id, velocity)?;
        self.sleeping.remove(&id);
        self.sleep_streaks.remove(&id);
        Ok(())
    }

    pub fn overlap_query(&self, query: OrientedBox3d) -> Result<Vec<BodyId>, RotatingWorldError3d> {
        self.inner.overlap_query(query)
    }

    pub fn step(
        &mut self,
        timestep_numerator: i32,
        timestep_denominator: i32,
    ) -> Result<RotatingWorldStepReport3d, RotatingWorldError3d> {
        if timestep_numerator <= 0 || timestep_denominator <= 0 {
            return self.inner.step(timestep_numerator, timestep_denominator);
        }

        let mut staged = self.clone();
        staged.wake_sleepers_for_sweeps(timestep_numerator, timestep_denominator)?;
        staged.freeze_sleeping_bodies()?;
        let report = staged
            .inner
            .step(timestep_numerator, timestep_denominator)?;
        staged.stabilize_fixed_boundaries()?;
        staged.restore_sleeping_bodies()?;
        staged.update_sleep_state()?;
        *self = staged;
        Ok(report)
    }

    fn wake_sleepers_for_sweeps(
        &mut self,
        timestep_numerator: i32,
        timestep_denominator: i32,
    ) -> Result<(), RotatingWorldError3d> {
        if self.sleeping.is_empty() {
            return Ok(());
        }

        let awake_config = RigidBoxFreeFlightConfig3d::new(
            self.config().gravity,
            timestep_numerator,
            timestep_denominator,
        );
        let stationary_config = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 0, 1);
        let mut awake_bounds = Vec::new();
        let mut sleeper_bounds = BTreeMap::new();

        for rigid_box in self.inner.boxes() {
            if rigid_box.body.kind != BodyKind::Dynamic {
                continue;
            }
            let id = rigid_box.body.id;
            if self.sleeping.contains(&id) {
                sleeper_bounds.insert(
                    id,
                    rigid_box_free_flight_sweep_bounds(rigid_box, stationary_config)?,
                );
            } else {
                awake_bounds.push((
                    id,
                    rigid_box_free_flight_sweep_bounds(rigid_box, awake_config)?,
                ));
            }
        }

        loop {
            let newly_awake = self
                .sleeping
                .iter()
                .copied()
                .filter(|id| {
                    sleeper_bounds.get(id).is_some_and(|sleeping_bounds| {
                        awake_bounds
                            .iter()
                            .any(|(_, bounds)| sweep_bounds_overlap(*bounds, *sleeping_bounds))
                    })
                })
                .collect::<Vec<_>>();
            if newly_awake.is_empty() {
                break;
            }

            for id in newly_awake {
                self.sleeping.remove(&id);
                self.sleep_streaks.remove(&id);
                sleeper_bounds.remove(&id);
                let rigid_box = self
                    .inner
                    .box_by_id(id)
                    .ok_or(RotatingWorldError3d::MissingBody(id))?;
                awake_bounds.push((
                    id,
                    rigid_box_free_flight_sweep_bounds(rigid_box, awake_config)?,
                ));
            }
        }

        Ok(())
    }

    fn freeze_sleeping_bodies(&mut self) -> Result<(), RotatingWorldError3d> {
        for id in self.sleeping.iter().copied().collect::<Vec<_>>() {
            self.set_sleep_proxy(id, true)?;
        }
        Ok(())
    }

    fn restore_sleeping_bodies(&mut self) -> Result<(), RotatingWorldError3d> {
        for id in self.sleeping.iter().copied().collect::<Vec<_>>() {
            self.set_sleep_proxy(id, false)?;
        }
        Ok(())
    }

    fn set_sleep_proxy(&mut self, id: BodyId, sleeping: bool) -> Result<(), RotatingWorldError3d> {
        let mut rigid_box = self
            .inner
            .remove_box(id)
            .ok_or(RotatingWorldError3d::MissingBody(id))?;
        rigid_box.body.kind = if sleeping {
            BodyKind::Fixed
        } else {
            BodyKind::Dynamic
        };
        if sleeping {
            rigid_box.body.velocity = Vec3i::ZERO;
            rigid_box.angular.angular_velocity = AngularVelocity3d::default();
        }
        self.inner.add_box(rigid_box)
    }

    fn update_sleep_state(&mut self) -> Result<(), RotatingWorldError3d> {
        let motion = self
            .inner
            .boxes()
            .filter(|rigid_box| {
                rigid_box.body.kind == BodyKind::Dynamic
                    && !self.sleeping.contains(&rigid_box.body.id)
            })
            .map(|rigid_box| {
                (
                    rigid_box.body.id,
                    low_motion(rigid_box),
                    motion_is_zero(rigid_box),
                    rigid_box.clone(),
                )
            })
            .collect::<Vec<_>>();
        let mut seen = BTreeSet::new();
        let mut direct_sleep = Vec::new();
        let mut supported_sleep = BTreeSet::new();

        for (id, is_low_motion, is_stationary, rigid_box) in motion {
            seen.insert(id);
            let has_dissipative_contact = if is_low_motion && !is_stationary {
                self.has_dissipative_sleep_contact(&rigid_box)?
            } else {
                false
            };
            if is_low_motion && (is_stationary || has_dissipative_contact) {
                let streak = self.sleep_streaks.entry(id).or_default();
                *streak = streak.saturating_add(1);
                if *streak >= SLEEP_STABLE_STEPS {
                    if self.has_gravity_support_chain(id)? {
                        supported_sleep.insert(id);
                    } else {
                        direct_sleep.push(id);
                    }
                }
            } else {
                self.sleep_streaks.remove(&id);
            }
        }
        self.sleep_streaks.retain(|id, _| seen.contains(id));

        for id in direct_sleep {
            self.put_body_to_sleep(id)?;
            self.sleeping.insert(id);
            self.sleep_streaks.remove(&id);
        }

        loop {
            let mut ready = Vec::new();
            for id in supported_sleep.iter().copied() {
                let rigid_box = self
                    .inner
                    .box_by_id(id)
                    .ok_or(RotatingWorldError3d::MissingBody(id))?;
                if !self.direct_gravity_sleep_supports(rigid_box)?.is_empty() {
                    ready.push(id);
                }
            }
            if ready.is_empty() {
                break;
            }

            for id in ready {
                supported_sleep.remove(&id);
                if !self.settle_body_on_sleep_supports(id)? {
                    continue;
                }
                self.put_body_to_sleep(id)?;
                self.sleeping.insert(id);
                self.sleep_streaks.remove(&id);
            }
        }
        Ok(())
    }

    fn has_dissipative_sleep_contact(
        &self,
        rigid_box: &RigidBox3d,
    ) -> Result<bool, RotatingWorldError3d> {
        let id = rigid_box.body.id;
        for other_id in self.inner.overlap_query(rigid_box.oriented_box())? {
            if other_id == id {
                continue;
            }
            let other = self
                .inner
                .box_by_id(other_id)
                .ok_or(RotatingWorldError3d::MissingBody(other_id))?;
            let (left, right) = if id < other_id {
                (rigid_box, other)
            } else {
                (other, rigid_box)
            };
            let Some(contact) = obb_contact_seed(left.oriented_box(), right.oriented_box())? else {
                continue;
            };

            let relative_velocity = relative_vector(right.body.velocity, left.body.velocity);
            if checked_axis_dot(relative_velocity, contact.axis, id)? < 0 {
                return Ok(true);
            }
        }

        self.has_gravity_support_chain(id)
    }

    fn has_gravity_support_chain(&self, start: BodyId) -> Result<bool, RotatingWorldError3d> {
        let gravity = self.config().gravity;
        if gravity == Vec3i::ZERO {
            return Ok(false);
        }
        let gravity_vector = relative_vector(gravity, Vec3i::ZERO);
        let mut pending = BTreeSet::from([start]);
        let mut visited = BTreeSet::new();

        while let Some(id) = pending.pop_first() {
            if !visited.insert(id) {
                continue;
            }
            let rigid_box = self
                .inner
                .box_by_id(id)
                .ok_or(RotatingWorldError3d::MissingBody(id))?;
            if rigid_box.body.kind != BodyKind::Dynamic {
                continue;
            }

            for other_id in self.inner.overlap_query(rigid_box.oriented_box())? {
                if other_id == id {
                    continue;
                }
                let other = self
                    .inner
                    .box_by_id(other_id)
                    .ok_or(RotatingWorldError3d::MissingBody(other_id))?;
                let (left, right) = if id < other_id {
                    (rigid_box, other)
                } else {
                    (other, rigid_box)
                };
                let Some(contact) = obb_contact_seed(left.oriented_box(), right.oriented_box())?
                else {
                    continue;
                };
                let pair_friction = left
                    .body
                    .material
                    .friction_milli()
                    .max(right.body.material.friction_milli());
                if pair_friction == 0 {
                    continue;
                }

                if !gravity_pushes_into_other(gravity_vector, contact.axis, id, left.body.id)? {
                    continue;
                }

                if other.body.kind == BodyKind::Fixed || self.sleeping.contains(&other_id) {
                    return Ok(true);
                }
                if other.body.kind == BodyKind::Dynamic
                    && low_motion(other)
                    && !visited.contains(&other_id)
                {
                    pending.insert(other_id);
                }
            }
        }
        Ok(false)
    }

    fn direct_gravity_sleep_supports(
        &self,
        rigid_box: &RigidBox3d,
    ) -> Result<Vec<BodyId>, RotatingWorldError3d> {
        let gravity = self.config().gravity;
        if gravity == Vec3i::ZERO {
            return Ok(Vec::new());
        }
        let gravity_vector = relative_vector(gravity, Vec3i::ZERO);
        let id = rigid_box.body.id;
        let mut supports = Vec::new();

        for other_id in self.inner.overlap_query(rigid_box.oriented_box())? {
            if other_id == id {
                continue;
            }
            let other = self
                .inner
                .box_by_id(other_id)
                .ok_or(RotatingWorldError3d::MissingBody(other_id))?;
            if other.body.kind != BodyKind::Fixed && !self.sleeping.contains(&other_id) {
                continue;
            }
            let (left, right) = if id < other_id {
                (rigid_box, other)
            } else {
                (other, rigid_box)
            };
            let Some(contact) = obb_contact_seed(left.oriented_box(), right.oriented_box())? else {
                continue;
            };
            let pair_friction = left
                .body
                .material
                .friction_milli()
                .max(right.body.material.friction_milli());
            if pair_friction == 0 {
                continue;
            }
            if gravity_pushes_into_other(gravity_vector, contact.axis, id, left.body.id)? {
                supports.push(other_id);
            }
        }
        Ok(supports)
    }

    fn settle_body_on_sleep_supports(&mut self, id: BodyId) -> Result<bool, RotatingWorldError3d> {
        let mut candidate = self
            .inner
            .box_by_id(id)
            .cloned()
            .ok_or(RotatingWorldError3d::MissingBody(id))?;
        let original_position = candidate.body.position;
        let mut converged = false;

        for _ in 0..MAX_FIXED_POSITION_STABILIZATION_PASSES {
            let supports = self.direct_gravity_sleep_supports(&candidate)?;
            if supports.is_empty() {
                converged = true;
                break;
            }

            let before = candidate.body.position;
            let mut corrections = PositionCorrectionAccumulator::default();
            for support_id in supports {
                let mut support = self
                    .inner
                    .box_by_id(support_id)
                    .cloned()
                    .ok_or(RotatingWorldError3d::MissingBody(support_id))?;
                support.body.kind = BodyKind::Fixed;
                support.body.velocity = Vec3i::ZERO;
                support.angular.angular_velocity = AngularVelocity3d::default();

                let response = if id < support_id {
                    resolve_obb_contact(candidate.clone(), support, false)
                } else {
                    resolve_obb_contact(support, candidate.clone(), false)
                }
                .map_err(|error| {
                    RotatingWorldError3d::Response(RotatingContactResponseError3d::Pair(error))
                })?;
                let projected = if id < support_id {
                    response.left.body.position
                } else {
                    response.right.body.position
                };
                if projected != before {
                    corrections.accumulate(id, before, projected)?;
                }
            }

            if corrections.is_empty() {
                converged = true;
                break;
            }
            let projected = corrections.target_position(id, before)?;
            if projected == before {
                break;
            }
            candidate.body.position = projected;
        }

        if !converged {
            return Err(RotatingWorldError3d::PersistentTailResolutionLimit(
                u32::from(MAX_FIXED_POSITION_STABILIZATION_PASSES),
            ));
        }
        if !self.sleep_projection_is_constraint_safe(&candidate)? {
            return Ok(false);
        }
        if candidate.body.position == original_position {
            return Ok(true);
        }

        let mut rigid_box = self
            .inner
            .remove_box(id)
            .ok_or(RotatingWorldError3d::MissingBody(id))?;
        rigid_box.body.position = candidate.body.position;
        self.inner.add_box(rigid_box)?;
        Ok(true)
    }

    fn sleep_projection_is_constraint_safe(
        &self,
        candidate: &RigidBox3d,
    ) -> Result<bool, RotatingWorldError3d> {
        let id = candidate.body.id;
        for other_id in self.inner.overlap_query(candidate.oriented_box())? {
            if other_id == id {
                continue;
            }
            let other = self
                .inner
                .box_by_id(other_id)
                .ok_or(RotatingWorldError3d::MissingBody(other_id))?;
            if other.body.kind != BodyKind::Fixed && !self.sleeping.contains(&other_id) {
                continue;
            }

            let mut anchored = other.clone();
            anchored.body.kind = BodyKind::Fixed;
            anchored.body.velocity = Vec3i::ZERO;
            anchored.angular.angular_velocity = AngularVelocity3d::default();
            let response = if id < other_id {
                resolve_obb_contact(candidate.clone(), anchored, false)
            } else {
                resolve_obb_contact(anchored, candidate.clone(), false)
            }
            .map_err(|error| {
                RotatingWorldError3d::Response(RotatingContactResponseError3d::Pair(error))
            })?;
            let projected = if id < other_id {
                response.left.body.position
            } else {
                response.right.body.position
            };
            if projected != candidate.body.position {
                return Ok(false);
            }
        }
        Ok(true)
    }

    fn put_body_to_sleep(&mut self, id: BodyId) -> Result<(), RotatingWorldError3d> {
        let mut rigid_box = self
            .inner
            .remove_box(id)
            .ok_or(RotatingWorldError3d::MissingBody(id))?;
        if rigid_box.body.kind == BodyKind::Dynamic {
            rigid_box.body.velocity = Vec3i::ZERO;
            rigid_box.angular.angular_velocity = AngularVelocity3d::default();
        }
        self.inner.add_box(rigid_box)
    }

    fn wake_all_sleepers(&mut self) {
        self.sleeping.clear();
        self.sleep_streaks.clear();
    }

    fn stabilize_fixed_boundaries(&mut self) -> Result<(), RotatingWorldError3d> {
        let mut boxes = self.inner.boxes().cloned().collect::<Vec<_>>();
        if boxes.len() < 2 {
            return Ok(());
        }

        let candidate_pairs = fixed_dynamic_pairs(&boxes);
        if candidate_pairs.is_empty() {
            return Ok(());
        }

        let mut corrections = vec![PositionCorrectionAccumulator::default(); boxes.len()];
        let mut converged = false;
        let mut any_changed = false;
        for _ in 0..MAX_FIXED_POSITION_STABILIZATION_PASSES {
            let snapshot = boxes.clone();
            for correction in &mut corrections {
                correction.clear();
            }

            let mut had_projection = false;
            for &(left_index, right_index, dynamic_index) in &candidate_pairs {
                let response = resolve_obb_contact(
                    snapshot[left_index].clone(),
                    snapshot[right_index].clone(),
                    false,
                )
                .map_err(|error| {
                    RotatingWorldError3d::Response(RotatingContactResponseError3d::Pair(error))
                })?;
                let projected = if dynamic_index == left_index {
                    response.left.body.position
                } else {
                    response.right.body.position
                };
                let before = snapshot[dynamic_index].body.position;
                if projected != before {
                    corrections[dynamic_index].accumulate(
                        snapshot[dynamic_index].body.id,
                        before,
                        projected,
                    )?;
                    had_projection = true;
                }
            }

            if !had_projection {
                converged = true;
                break;
            }

            let mut changed = false;
            for (index, correction) in corrections.iter().enumerate() {
                if correction.is_empty() {
                    continue;
                }
                let id = snapshot[index].body.id;
                let projected = correction.target_position(id, snapshot[index].body.position)?;
                if projected != snapshot[index].body.position {
                    boxes[index].body.position = projected;
                    changed = true;
                    any_changed = true;
                }
            }

            if !changed {
                break;
            }
        }

        if !converged {
            return Err(RotatingWorldError3d::PersistentTailResolutionLimit(
                u32::from(MAX_FIXED_POSITION_STABILIZATION_PASSES),
            ));
        }
        if !any_changed {
            return Ok(());
        }

        let ids = boxes
            .iter()
            .map(|rigid_box| rigid_box.body.id)
            .collect::<Vec<_>>();
        for id in ids {
            let removed = self
                .inner
                .remove_box(id)
                .ok_or(RotatingWorldError3d::MissingBody(id))?;
            debug_assert_eq!(removed.body.id, id);
        }
        for rigid_box in boxes {
            self.inner.add_box(rigid_box)?;
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct PositionCorrectionGroup {
    delta: [i64; 3],
    strength: u64,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
struct PositionCorrectionAccumulator {
    groups: BTreeMap<[i64; 3], PositionCorrectionGroup>,
}

impl PositionCorrectionAccumulator {
    fn accumulate(
        &mut self,
        id: BodyId,
        before: Vec3i,
        projected: Vec3i,
    ) -> Result<(), RotatingWorldError3d> {
        let delta = [
            i64::from(projected.x) - i64::from(before.x),
            i64::from(projected.y) - i64::from(before.y),
            i64::from(projected.z) - i64::from(before.z),
        ];
        if delta == [0; 3] {
            return Ok(());
        }
        let direction = primitive_position_direction(delta, id)?;
        let strength = delta
            .into_iter()
            .map(i64::unsigned_abs)
            .max()
            .unwrap_or_default();
        let candidate = PositionCorrectionGroup { delta, strength };
        match self.groups.get_mut(&direction) {
            Some(existing) if existing.strength < strength => *existing = candidate,
            Some(_) => {}
            None => {
                self.groups.insert(direction, candidate);
            }
        }
        Ok(())
    }

    fn target_position(&self, id: BodyId, before: Vec3i) -> Result<Vec3i, RotatingWorldError3d> {
        let mut combined = [0_i64; 3];
        for group in self.groups.values() {
            for (axis, combined_axis) in combined.iter_mut().enumerate() {
                *combined_axis = (*combined_axis)
                    .checked_add(group.delta[axis])
                    .ok_or_else(|| position_correction_overflow(id))?;
            }
        }
        Ok(Vec3i::new(
            checked_position_component(before.x, combined[0], id)?,
            checked_position_component(before.y, combined[1], id)?,
            checked_position_component(before.z, combined[2], id)?,
        ))
    }

    fn is_empty(&self) -> bool {
        self.groups.is_empty()
    }

    fn clear(&mut self) {
        self.groups.clear();
    }
}

fn primitive_position_direction(
    delta: [i64; 3],
    id: BodyId,
) -> Result<[i64; 3], RotatingWorldError3d> {
    let divisor = delta
        .into_iter()
        .map(i64::unsigned_abs)
        .fold(0_u64, gcd_u64);
    if divisor == 0 {
        return Err(position_correction_overflow(id));
    }
    let divisor = i64::try_from(divisor).map_err(|_| position_correction_overflow(id))?;
    Ok([delta[0] / divisor, delta[1] / divisor, delta[2] / divisor])
}

fn checked_position_component(
    value: i32,
    delta: i64,
    id: BodyId,
) -> Result<i32, RotatingWorldError3d> {
    let value = i64::from(value)
        .checked_add(delta)
        .ok_or_else(|| position_correction_overflow(id))?;
    i32::try_from(value).map_err(|_| position_correction_overflow(id))
}

fn position_correction_overflow(id: BodyId) -> RotatingWorldError3d {
    RotatingWorldError3d::Response(RotatingContactResponseError3d::ArithmeticOverflow(id))
}

fn relative_vector(right: Vec3i, left: Vec3i) -> [i128; 3] {
    [
        i128::from(right.x) - i128::from(left.x),
        i128::from(right.y) - i128::from(left.y),
        i128::from(right.z) - i128::from(left.z),
    ]
}

fn checked_axis_dot(
    vector: [i128; 3],
    axis: [i128; 3],
    id: BodyId,
) -> Result<i128, RotatingWorldError3d> {
    vector
        .into_iter()
        .zip(axis)
        .try_fold(0_i128, |sum, (component, axis_component)| {
            let product = component
                .checked_mul(axis_component)
                .ok_or_else(|| position_correction_overflow(id))?;
            sum.checked_add(product)
                .ok_or_else(|| position_correction_overflow(id))
        })
}

fn gravity_pushes_into_other(
    gravity: [i128; 3],
    contact_axis: [i128; 3],
    id: BodyId,
    left_id: BodyId,
) -> Result<bool, RotatingWorldError3d> {
    let dot = checked_axis_dot(gravity, contact_axis, id)?;
    Ok(if id == left_id { dot > 0 } else { dot < 0 })
}

fn gcd_u64(mut left: u64, mut right: u64) -> u64 {
    while right != 0 {
        let remainder = left % right;
        left = right;
        right = remainder;
    }
    left
}

fn low_motion(rigid_box: &RigidBox3d) -> bool {
    let velocity = rigid_box.body.velocity();
    let linear_speed = velocity
        .x
        .unsigned_abs()
        .max(velocity.y.unsigned_abs())
        .max(velocity.z.unsigned_abs());
    let angular = rigid_box.angular().angular_velocity;
    let angular_speed = angular
        .x
        .unsigned_abs()
        .max(angular.y.unsigned_abs())
        .max(angular.z.unsigned_abs());
    linear_speed <= SLEEP_LINEAR_SPEED_LIMIT && angular_speed <= SLEEP_ANGULAR_SPEED_LIMIT
}

fn motion_is_zero(rigid_box: &RigidBox3d) -> bool {
    rigid_box.body.velocity() == Vec3i::ZERO && rigid_box.angular().angular_velocity.is_zero()
}

fn sweep_bounds_overlap(left: RotationalSweepBounds3d, right: RotationalSweepBounds3d) -> bool {
    (0..3).all(|axis| {
        left.minimum[axis] <= right.maximum[axis] && right.minimum[axis] <= left.maximum[axis]
    })
}

fn fixed_dynamic_pairs(boxes: &[RigidBox3d]) -> Vec<(usize, usize, usize)> {
    let mut pairs = Vec::new();
    for left_index in 0..boxes.len() {
        for right_index in (left_index + 1)..boxes.len() {
            let dynamic_index = match (boxes[left_index].body.kind, boxes[right_index].body.kind) {
                (BodyKind::Fixed, BodyKind::Dynamic) => Some(right_index),
                (BodyKind::Dynamic, BodyKind::Fixed) => Some(left_index),
                _ => None,
            };
            if let Some(dynamic_index) = dynamic_index {
                pairs.push((left_index, right_index, dynamic_index));
            }
        }
    }
    pairs
}

#[cfg(test)]
mod tests {
    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, Material, Orientation3d, RigidBody, RigidBox3d,
        RotatingWorldConfig3d, Vec3i,
    };

    use super::{PositionCorrectionAccumulator, RotatingWorld3d, SLEEP_STABLE_STEPS};

    fn dynamic(id: u64, position: Vec3i, velocity: Vec3i) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::dynamic(BodyId(id), position, velocity, Vec3i::new(1, 1, 1)),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid dynamic box")
    }

    fn zero_gravity_world() -> RotatingWorld3d {
        RotatingWorld3d::new(RotatingWorldConfig3d {
            gravity: Vec3i::ZERO,
            sample_count: 8,
            refinement_steps: 2,
            solver_passes: 4,
            max_events: 8,
        })
    }

    #[test]
    fn parallel_position_corrections_do_not_stack() {
        let id = BodyId(9);
        let mut corrections = PositionCorrectionAccumulator::default();
        corrections
            .accumulate(id, Vec3i::ZERO, Vec3i::new(2, 0, 0))
            .expect("first correction");
        corrections
            .accumulate(id, Vec3i::ZERO, Vec3i::new(3, 0, 0))
            .expect("stronger parallel correction");

        assert_eq!(
            corrections
                .target_position(id, Vec3i::ZERO)
                .expect("combined correction"),
            Vec3i::new(3, 0, 0)
        );
    }

    #[test]
    fn independent_position_corrections_commit_once_together() {
        let id = BodyId(10);
        let mut corrections = PositionCorrectionAccumulator::default();
        corrections
            .accumulate(id, Vec3i::ZERO, Vec3i::new(2, 0, 0))
            .expect("horizontal correction");
        corrections
            .accumulate(id, Vec3i::ZERO, Vec3i::new(0, 3, 0))
            .expect("vertical correction");

        assert_eq!(
            corrections
                .target_position(id, Vec3i::ZERO)
                .expect("combined correction"),
            Vec3i::new(2, 3, 0)
        );
    }

    #[test]
    fn low_motion_body_sleeps_and_remains_stationary() {
        let id = BodyId(1);
        let mut world = zero_gravity_world();
        world
            .add_box(dynamic(id.0, Vec3i::new(10, 20, 30), Vec3i::ZERO))
            .expect("add body");

        for _ in 0..SLEEP_STABLE_STEPS {
            world.step(1, 60).expect("settle body");
        }
        assert!(world.is_sleeping(id));
        assert_eq!(world.sleeping_body_count(), 1);
        let sleeping = world.box_by_id(id).expect("sleeping body").clone();

        for _ in 0..20 {
            world.step(1, 60).expect("sleeping step");
        }
        assert_eq!(world.box_by_id(id), Some(&sleeping));
    }

    #[test]
    fn explicit_velocity_change_wakes_sleeping_body() {
        let id = BodyId(2);
        let mut world = zero_gravity_world();
        world
            .add_box(dynamic(id.0, Vec3i::ZERO, Vec3i::ZERO))
            .expect("add body");
        for _ in 0..SLEEP_STABLE_STEPS {
            world.step(1, 60).expect("settle body");
        }
        assert!(world.is_sleeping(id));

        world
            .set_linear_velocity(id, Vec3i::new(60, 0, 0))
            .expect("wake body");
        assert!(!world.is_sleeping(id));
        world.step(1, 60).expect("awake step");
        assert_eq!(
            world.box_by_id(id).expect("awake body").body().position(),
            Vec3i::new(1, 0, 0)
        );
    }

    #[test]
    fn approaching_dynamic_body_wakes_sleeping_body_before_contact() {
        let sleeper = BodyId(3);
        let mut world = zero_gravity_world();
        world
            .add_box(dynamic(sleeper.0, Vec3i::ZERO, Vec3i::ZERO))
            .expect("add sleeper");
        for _ in 0..SLEEP_STABLE_STEPS {
            world.step(1, 60).expect("settle sleeper");
        }
        assert!(world.is_sleeping(sleeper));

        world
            .add_box(dynamic(4, Vec3i::new(-5, 0, 0), Vec3i::new(240, 0, 0)))
            .expect("add approaching body");
        world.step(1, 60).expect("impact step");
        assert!(!world.is_sleeping(sleeper));
    }

    #[test]
    fn supported_sleep_projection_rejects_new_fixed_penetration() {
        let lower = BodyId(20);
        let upper = BodyId(21);
        let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
            gravity: Vec3i::new(0, -3_600, 0),
            sample_count: 32,
            refinement_steps: 4,
            solver_passes: 8,
            max_events: 32,
        });
        let material = Material::new(0).with_friction(1_000);
        let rigid_box = |body| {
            RigidBox3d::new(
                body,
                AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
            )
            .expect("valid box")
        };

        world
            .add_box(rigid_box(RigidBody::fixed(
                BodyId(10),
                Vec3i::new(0, -16, 0),
                Vec3i::new(300, 16, 300),
            )))
            .expect("floor");
        world
            .add_box(rigid_box(
                RigidBody::dynamic(
                    lower,
                    Vec3i::new(0, 18, 0),
                    Vec3i::ZERO,
                    Vec3i::new(18, 18, 18),
                )
                .with_mass(2)
                .with_material(material),
            ))
            .expect("lower crate");
        world
            .add_box(rigid_box(
                RigidBody::dynamic(
                    upper,
                    Vec3i::new(0, 52, 0),
                    Vec3i::ZERO,
                    Vec3i::new(18, 18, 18),
                )
                .with_mass(2)
                .with_material(material),
            ))
            .expect("upper crate");
        world
            .add_box(rigid_box(RigidBody::fixed(
                BodyId(11),
                Vec3i::new(0, 87, 0),
                Vec3i::new(300, 16, 300),
            )))
            .expect("ceiling");
        world.sleeping.insert(lower);

        assert!(
            !world
                .settle_body_on_sleep_supports(upper)
                .expect("bounded support projection")
        );
        assert_eq!(
            world
                .box_by_id(upper)
                .expect("upper crate")
                .body()
                .position(),
            Vec3i::new(0, 52, 0)
        );
    }
}
