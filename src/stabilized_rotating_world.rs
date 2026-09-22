use std::collections::{BTreeMap, BTreeSet};

#[cfg(test)]
use crate::rotating_broad_phase::{RotatingBroadPhase3d, RotatingBroadPhaseError3d};

use crate::{
    ANGULAR_VELOCITY_SCALE, AngularVelocity3d, BallisticSphere3d, BodyCurrentContact3d, BodyId,
    BodyKind, InteractionCategory3d, InteractionExecutionPlan3d, InteractionPolicies3d,
    InteractionPolicy3d, MotionAuthority3d, Orientation3d, OrientedBox3d, RigidBox3d,
    RigidBoxFreeFlightConfig3d, RotatingContactResponseError3d, RotatingWorldConfig3d,
    RotatingWorldError3d, RotatingWorldStepReport3d, RotationalSweepBounds3d, SleepMode3d,
    SolverParticipation3d, Vec3i, WakePropagation3d, obb_contact_seed,
    obb_response::resolve_obb_contact, rigid_box_free_flight_sweep_bounds,
    rotating_world::RotatingWorld3d as InnerRotatingWorld3d,
};

const MAX_FIXED_POSITION_STABILIZATION_PASSES: u8 = 16;
#[cfg(test)]
const SLEEP_STABLE_STEPS_AT_60_HZ: u8 = 12;
const SLEEP_TIME_SCALE: u128 = 1_u128 << 64;
const SLEEP_STABLE_DURATION_Q64: u128 = SLEEP_TIME_SCALE / 5;
const AGGRESSIVE_SLEEP_STABLE_DURATION_Q64: u128 = SLEEP_TIME_SCALE / 20;
const SLEEP_LINEAR_SPEED_LIMIT: u32 = 120;

/// Engine-owned rotating world with bounded fixed-boundary stabilization and deterministic sleeping.
///
/// The inner rotating solver remains the sole authority for impulses, dynamic/dynamic response,
/// restitution, friction, and angular response. After one requested non-zero world step, this facade only
/// removes residual integer penetration between fixed and dynamic bodies. The fixed-boundary pass evaluates
/// every constraint from one shared snapshot and commits at most one combined positional correction per
/// dynamic body per pass, so adjacent or duplicated fixed surfaces cannot sequentially move the same body
/// several times inside one stabilization pass. Every velocity and orientation result from the simultaneous
/// solver is preserved. Fixed-boundary stabilization consumes the exact changed-body set from the step
/// contract: each changed dynamic queries only its own current overlap neighborhood, and iterative projection
/// re-queries only that candidate geometry. Unchanged dynamics are not cloned, indexed, or traversed.
///
/// Dynamic bodies whose conservative point-speed bound remains below the deterministic sleep threshold for a
/// continuous simulated duration are put to sleep only when they are already motionless or a current contact
/// can physically dissipate the residual motion. The point-speed bound combines translation with a
/// body-size-aware bound on rotational surface motion, so fixed-point angular units are judged in the same
/// spatial units as linear velocity instead of against a shape-independent angular constant. Sleep stability
/// time is accumulated in deterministic Q64 units from the requested rational timestep, so equivalent elapsed
/// simulation time reaches the same sleep threshold independently of 30, 60, or 120 Hz step partitioning. A
/// contact qualifies when its normal constraint is still opposing relative approach, or when non-zero pair
/// friction belongs to a low-motion support chain that is ultimately anchored by fixed geometry or an existing
/// sleeper. Gravity-supported bodies settle position-only against already anchored supports before becoming
/// sleepers, so quantized equal-mass projection cannot freeze residual penetration into a resting stack. This
/// preserves ordinary low-speed free-flight and frictionless tangential inertia while letting gravity-loaded
/// rough stacks converge to sleep as one supported island instead of repeatedly waking their lower members.
/// Sleeping bodies are presented to the inner solver as fixed proxies, so gravity and persistent-contact
/// stabilization cannot keep nudging a settled body. Before each step, conservative free-flight sweep bounds
/// wake every sleeping body that an awake dynamic body may reach, including transitive sleeping islands.
/// Explicit velocity changes also wake their target, adding fixed geometry invalidates existing sleepers
/// conservatively, and any successful body removal invalidates sleep because world membership and contact
/// topology changed.
///
/// Non-zero commands execute in place. Malformed timesteps still fail before mutation, and an inner-solver
/// failure restores temporary sleeping proxies before returning. The facade deliberately no longer clones
/// the complete world merely to provide frame-level rollback around later stabilization or sleep bookkeeping:
/// if those later phases fail after the authoritative inner step committed, the successfully advanced
/// physical state is retained and the error is reported. Whole-frame rollback is not part of the production
/// command contract. A zero timestep keeps the inner world's exact no-op contract and deliberately skips
/// stabilization and sleep bookkeeping.
///
/// Multiple fixed boundaries can still constrain one body, so the position-only pass is independently
/// bounded and fails closed if those fixed constraints cannot reach an idempotent state.
#[derive(Clone, Debug)]
pub struct RotatingWorld3d {
    inner: InnerRotatingWorld3d,
    sleeping: BTreeSet<BodyId>,
    sleep_candidates: BTreeSet<BodyId>,
    sleep_stable_time_q64: BTreeMap<BodyId, u128>,
    pending_fixed_boundary_body_ids: BTreeSet<BodyId>,
    interaction_policies: InteractionPolicies3d,
}

impl RotatingWorld3d {
    #[must_use]
    pub fn new(config: RotatingWorldConfig3d) -> Self {
        Self {
            inner: InnerRotatingWorld3d::new(config),
            sleeping: BTreeSet::new(),
            sleep_candidates: BTreeSet::new(),
            sleep_stable_time_q64: BTreeMap::new(),
            pending_fixed_boundary_body_ids: BTreeSet::new(),
            interaction_policies: InteractionPolicies3d::default(),
        }
    }

    #[must_use]
    pub fn config(&self) -> RotatingWorldConfig3d {
        self.inner.config()
    }

    #[must_use]
    pub fn body_interaction_category(&self, id: BodyId) -> InteractionCategory3d {
        self.interaction_policies.body_category(id)
    }

    pub fn set_body_interaction_category(
        &mut self,
        id: BodyId,
        category: InteractionCategory3d,
    ) -> Result<Option<InteractionCategory3d>, RotatingWorldError3d> {
        if self.inner.box_by_id(id).is_none() {
            return Err(RotatingWorldError3d::MissingBody(id));
        }
        Ok(self.interaction_policies.set_body_category(id, category))
    }

    pub(crate) fn clear_body_interaction_category(
        &mut self,
        id: BodyId,
    ) -> Option<InteractionCategory3d> {
        self.interaction_policies.clear_body_category(id)
    }

    pub fn set_default_interaction_policy(&mut self, policy: InteractionPolicy3d) {
        self.interaction_policies.set_default_policy(policy);
    }

    #[must_use]
    pub fn interaction_policy_for_bodies(
        &self,
        source: BodyId,
        target: BodyId,
    ) -> InteractionPolicy3d {
        self.interaction_policies.policy_for_bodies(source, target)
    }

    #[must_use]
    pub fn interaction_execution_plan_for_bodies(
        &self,
        source: BodyId,
        target: BodyId,
    ) -> InteractionExecutionPlan3d {
        self.interaction_policies
            .execution_plan_for_bodies(source, target)
    }

    pub fn set_pair_interaction_policy(
        &mut self,
        left: InteractionCategory3d,
        right: InteractionCategory3d,
        policy: InteractionPolicy3d,
    ) -> Option<InteractionPolicy3d> {
        self.interaction_policies
            .set_pair_policy(left, right, policy)
    }

    pub fn clear_pair_interaction_policy(
        &mut self,
        left: InteractionCategory3d,
        right: InteractionCategory3d,
    ) -> Option<InteractionPolicy3d> {
        self.interaction_policies.clear_pair_policy(left, right)
    }

    pub fn set_directional_interaction_policy(
        &mut self,
        source: InteractionCategory3d,
        target: InteractionCategory3d,
        policy: InteractionPolicy3d,
    ) -> Option<InteractionPolicy3d> {
        self.interaction_policies
            .set_directional_policy(source, target, policy)
    }

    pub fn clear_directional_interaction_policy(
        &mut self,
        source: InteractionCategory3d,
        target: InteractionCategory3d,
    ) -> Option<InteractionPolicy3d> {
        self.interaction_policies
            .clear_directional_policy(source, target)
    }

    pub fn add_box(&mut self, rigid_box: RigidBox3d) -> Result<(), RotatingWorldError3d> {
        let id = rigid_box.body.id;
        let kind = rigid_box.body.kind;
        let affected_dynamic_ids = if kind == BodyKind::Fixed {
            let layers = rigid_box.collision_layers();
            self.inner
                .overlap_query(rigid_box.oriented_box())?
                .into_iter()
                .filter(|other_id| {
                    self.inner.box_by_id(*other_id).is_some_and(|other| {
                        other.body.kind == BodyKind::Dynamic
                            && layers.collides_with(other.collision_layers())
                    })
                })
                .collect::<Vec<_>>()
        } else {
            Vec::new()
        };

        self.inner.add_box(rigid_box)?;
        if kind == BodyKind::Dynamic {
            self.pending_fixed_boundary_body_ids.insert(id);
            self.sleep_candidates.insert(id);
        } else {
            self.pending_fixed_boundary_body_ids
                .extend(affected_dynamic_ids);
            self.wake_all_sleepers();
        }
        Ok(())
    }

    pub fn remove_box(&mut self, id: BodyId) -> Option<RigidBox3d> {
        let removed = self.inner.remove_box(id)?;
        self.pending_fixed_boundary_body_ids.remove(&id);
        self.sleep_candidates.remove(&id);
        self.sleeping.remove(&id);
        self.sleep_stable_time_q64.remove(&id);
        self.wake_all_sleepers();
        Some(removed)
    }

    pub fn add_ballistic_sphere(
        &mut self,
        projectile: BallisticSphere3d,
        retire_on_contact: bool,
    ) -> Result<(), RotatingWorldError3d> {
        self.inner
            .add_ballistic_sphere(projectile, retire_on_contact)
    }

    pub fn remove_ballistic_sphere(&mut self, id: BodyId) -> Option<BallisticSphere3d> {
        self.inner.remove_ballistic_sphere(id)
    }

    #[must_use]
    pub fn ballistic_sphere_by_id(&self, id: BodyId) -> Option<&BallisticSphere3d> {
        self.inner.ballistic_sphere_by_id(id)
    }

    pub fn ballistic_spheres(&self) -> impl Iterator<Item = &BallisticSphere3d> {
        self.inner.ballistic_spheres()
    }

    #[must_use]
    pub fn ballistic_sphere_count(&self) -> usize {
        self.inner.ballistic_sphere_count()
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
        self.sleep_candidates.insert(id);
        self.sleep_stable_time_q64.remove(&id);
        self.pending_fixed_boundary_body_ids.insert(id);
        Ok(())
    }

    pub fn set_orientations(
        &mut self,
        updates: &[(BodyId, Orientation3d)],
    ) -> Result<(), RotatingWorldError3d> {
        self.inner.set_orientations(updates)?;
        for (id, _) in updates {
            self.sleeping.remove(id);
            self.sleep_candidates.insert(*id);
            self.sleep_stable_time_q64.remove(id);
            self.pending_fixed_boundary_body_ids.insert(*id);
        }
        Ok(())
    }

    pub fn overlap_query(&self, query: OrientedBox3d) -> Result<Vec<BodyId>, RotatingWorldError3d> {
        self.inner.overlap_query(query)
    }

    pub fn body_contacts(
        &self,
        body: BodyId,
    ) -> Result<Vec<BodyCurrentContact3d>, RotatingWorldError3d> {
        self.inner.body_contacts(body)
    }

    pub fn body_overlaps(&self, body: BodyId) -> Result<Vec<BodyId>, RotatingWorldError3d> {
        self.inner.body_overlaps(body)
    }

    pub fn step(
        &mut self,
        timestep_numerator: i32,
        timestep_denominator: i32,
    ) -> Result<RotatingWorldStepReport3d, RotatingWorldError3d> {
        self.step_with_parked(timestep_numerator, timestep_denominator, &BTreeMap::new())
    }

    pub(crate) fn contact_work_counters(&self) -> [u64; 3] {
        self.inner.contact_work_counters()
    }

    pub(crate) fn step_with_parked(
        &mut self,
        timestep_numerator: i32,
        timestep_denominator: i32,
        parked: &BTreeMap<BodyId, RigidBox3d>,
    ) -> Result<RotatingWorldStepReport3d, RotatingWorldError3d> {
        if timestep_numerator <= 0 || timestep_denominator <= 0 {
            return self.inner.step(timestep_numerator, timestep_denominator);
        }

        let sleep_time_increment =
            sleep_time_increment_q64(timestep_numerator, timestep_denominator);
        let mut changed_body_ids =
            self.wake_sleepers_for_sweeps(timestep_numerator, timestep_denominator)?;
        let mut fixed_boundary_subjects = changed_body_ids.clone();
        fixed_boundary_subjects.extend(self.pending_fixed_boundary_body_ids.iter().copied());
        self.freeze_sleeping_bodies()?;
        let guard = crate::contact_wake::ContactWakeGuard3d {
            parked,
            policies: &self.interaction_policies,
        };
        let mut report = match self.inner.step_guarded(
            timestep_numerator,
            timestep_denominator,
            (!parked.is_empty()).then_some(&guard),
        ) {
            Ok(report) => report,
            Err(error) => {
                let _ = self.restore_sleeping_bodies();
                return Err(error);
            }
        };
        changed_body_ids.extend(report.changed_body_ids.iter().copied());
        fixed_boundary_subjects.extend(changed_body_ids.iter().copied());
        match self.stabilize_fixed_boundaries(&fixed_boundary_subjects) {
            Ok(stabilized) => {
                changed_body_ids.extend(stabilized);
                self.pending_fixed_boundary_body_ids.clear();
            }
            Err(error) => {
                let _ = self.restore_sleeping_bodies();
                return Err(error);
            }
        }
        self.restore_sleeping_bodies()?;
        let mut sleep_subjects = self.sleep_candidates.clone();
        sleep_subjects.extend(changed_body_ids.iter().copied());
        changed_body_ids.extend(self.update_sleep_state(sleep_time_increment, &sleep_subjects)?);
        report.changed_body_ids = changed_body_ids.into_iter().collect();
        Ok(report)
    }

    fn wake_sleepers_for_sweeps(
        &mut self,
        timestep_numerator: i32,
        timestep_denominator: i32,
    ) -> Result<BTreeSet<BodyId>, RotatingWorldError3d> {
        let mut awakened = BTreeSet::new();
        if self.sleeping.is_empty() {
            return Ok(awakened);
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
            if rigid_box.body.kind != BodyKind::Dynamic
                || rigid_box.solver_participation() != SolverParticipation3d::Solid
            {
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
                        awake_bounds.iter().any(|(awake_id, bounds)| {
                            self.interaction_policies
                                .execution_plan_for_bodies(*awake_id, *id)
                                .wake_propagation()
                                == WakePropagation3d::Full
                                && sweep_bounds_overlap(*bounds, *sleeping_bounds)
                                && !self.inner.box_by_id(*awake_id).is_some_and(|awake| {
                                    self.inner.box_by_id(*id).is_some_and(|sleeping| {
                                        crate::linear_contact::sweep_is_passive_support(
                                            awake, *bounds, sleeping,
                                        )
                                    })
                                })
                        })
                    })
                })
                .collect::<Vec<_>>();
            if newly_awake.is_empty() {
                break;
            }

            for id in newly_awake {
                self.sleeping.remove(&id);
                self.sleep_candidates.insert(id);
                self.sleep_stable_time_q64.remove(&id);
                sleeper_bounds.remove(&id);
                awakened.insert(id);
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

        Ok(awakened)
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

    fn update_sleep_state(
        &mut self,
        sleep_time_increment: u128,
        subjects: &BTreeSet<BodyId>,
    ) -> Result<BTreeSet<BodyId>, RotatingWorldError3d> {
        let mut direct_sleep = Vec::new();
        let mut supported_sleep = BTreeSet::new();
        let mut changed_body_ids = BTreeSet::new();

        for id in subjects.iter().copied() {
            if self.sleeping.contains(&id) {
                self.sleep_candidates.remove(&id);
                continue;
            }
            let Some(rigid_box) = self.inner.box_by_id(id).cloned() else {
                self.sleep_candidates.remove(&id);
                self.sleep_stable_time_q64.remove(&id);
                continue;
            };
            if rigid_box.body.kind != BodyKind::Dynamic {
                self.sleep_candidates.remove(&id);
                self.sleep_stable_time_q64.remove(&id);
                continue;
            }
            let is_low_motion = low_motion(&rigid_box);
            let is_stationary = motion_is_zero(&rigid_box);
            let sleep_mode = rigid_box.sleep_mode();
            if sleep_mode == SleepMode3d::Never {
                self.sleep_candidates.remove(&id);
                self.sleep_stable_time_q64.remove(&id);
                continue;
            }
            let has_dissipative_contact = if is_low_motion && !is_stationary {
                self.has_dissipative_sleep_contact(&rigid_box)?
            } else {
                false
            };
            if is_low_motion && (is_stationary || has_dissipative_contact) {
                self.sleep_candidates.insert(id);
                let stable_time = self.sleep_stable_time_q64.entry(id).or_default();
                *stable_time = stable_time.saturating_add(sleep_time_increment);
                let stable_duration = match sleep_mode {
                    SleepMode3d::Normal => SLEEP_STABLE_DURATION_Q64,
                    SleepMode3d::Aggressive => AGGRESSIVE_SLEEP_STABLE_DURATION_Q64,
                    SleepMode3d::Never => unreachable!("never-sleep bodies are filtered above"),
                };
                if *stable_time >= stable_duration {
                    if self.has_gravity_support_chain(id)? {
                        supported_sleep.insert(id);
                    } else {
                        direct_sleep.push(id);
                    }
                }
            } else {
                // Motion itself remains a dependency: an awake dynamic can become sleep-eligible on a
                // later step even when no topology or explicit control delta occurs. Keep the retained
                // candidate membership, but reset only the stability deadline.
                self.sleep_candidates.insert(id);
                self.sleep_stable_time_q64.remove(&id);
            }
        }

        for id in direct_sleep {
            self.put_body_to_sleep(id)?;
            self.sleeping.insert(id);
            self.sleep_candidates.remove(&id);
            self.sleep_stable_time_q64.remove(&id);
            changed_body_ids.insert(id);
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
                self.sleep_candidates.remove(&id);
                self.sleep_stable_time_q64.remove(&id);
                changed_body_ids.insert(id);
            }
        }
        Ok(changed_body_ids)
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
        self.sleep_candidates.extend(self.sleeping.iter().copied());
        self.sleeping.clear();
        self.sleep_stable_time_q64.clear();
    }

    fn stabilize_fixed_boundaries(
        &mut self,
        subjects: &BTreeSet<BodyId>,
    ) -> Result<BTreeSet<BodyId>, RotatingWorldError3d> {
        let mut changed_body_ids = BTreeSet::new();

        for id in subjects.iter().copied() {
            let Some(current) = self.inner.box_by_id(id) else {
                return Err(RotatingWorldError3d::MissingBody(id));
            };
            if current.body.kind != BodyKind::Dynamic
                || current.motion_authority() == MotionAuthority3d::External
                || current.solver_participation() != SolverParticipation3d::Solid
            {
                continue;
            }

            let mut candidate = current.clone();
            let original_position = candidate.body.position;
            let mut converged = false;

            for pass in 0..MAX_FIXED_POSITION_STABILIZATION_PASSES {
                let before = candidate.body.position;
                let mut corrections = PositionCorrectionAccumulator::default();

                for fixed_id in self.inner.overlap_query(candidate.oriented_box())? {
                    if fixed_id == id {
                        continue;
                    }
                    let fixed = self
                        .inner
                        .box_by_id(fixed_id)
                        .ok_or(RotatingWorldError3d::MissingBody(fixed_id))?;
                    if fixed.body.kind != BodyKind::Fixed
                        || !candidate
                            .collision_layers()
                            .collides_with(fixed.collision_layers())
                    {
                        continue;
                    }
                    let pass_limit = self
                        .interaction_policies
                        .execution_plan_for_bodies(id, fixed_id)
                        .fixed_boundary_stabilization_pass_limit(
                            MAX_FIXED_POSITION_STABILIZATION_PASSES,
                        );
                    if pass >= pass_limit {
                        continue;
                    }

                    let response = if id < fixed_id {
                        resolve_obb_contact(candidate.clone(), fixed.clone(), false)
                    } else {
                        resolve_obb_contact(fixed.clone(), candidate.clone(), false)
                    }
                    .map_err(|error| {
                        RotatingWorldError3d::Response(RotatingContactResponseError3d::Pair(error))
                    })?;
                    let projected = if id < fixed_id {
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
            if candidate.body.position == original_position {
                continue;
            }

            let mut rigid_box = self
                .inner
                .remove_box(id)
                .ok_or(RotatingWorldError3d::MissingBody(id))?;
            rigid_box.body.position = candidate.body.position;
            self.inner.add_box(rigid_box)?;
            changed_body_ids.insert(id);
        }

        Ok(changed_body_ids)
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

fn sleep_time_increment_q64(timestep_numerator: i32, timestep_denominator: i32) -> u128 {
    let numerator = u128::from(timestep_numerator.unsigned_abs()) * SLEEP_TIME_SCALE;
    numerator.div_ceil(u128::from(timestep_denominator.unsigned_abs()))
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
    let linear_speed = u128::from(
        velocity
            .x
            .unsigned_abs()
            .max(velocity.y.unsigned_abs())
            .max(velocity.z.unsigned_abs()),
    );
    let angular = rigid_box.angular().angular_velocity;
    let angular_speed_l1 = if rigid_box.rotation_locked() {
        0
    } else {
        u128::from(angular.x.unsigned_abs())
            .saturating_add(u128::from(angular.y.unsigned_abs()))
            .saturating_add(u128::from(angular.z.unsigned_abs()))
    };
    let half = rigid_box.body.half_extents();
    let radius_bound = u128::from(half.x.unsigned_abs())
        .saturating_add(u128::from(half.y.unsigned_abs()))
        .saturating_add(u128::from(half.z.unsigned_abs()));
    let rotational_surface_speed = radius_bound
        .saturating_mul(angular_speed_l1)
        .div_ceil(u128::from(ANGULAR_VELOCITY_SCALE.unsigned_abs()));

    linear_speed.saturating_add(rotational_surface_speed) <= u128::from(SLEEP_LINEAR_SPEED_LIMIT)
}

fn motion_is_zero(rigid_box: &RigidBox3d) -> bool {
    rigid_box.body.velocity() == Vec3i::ZERO && rigid_box.angular().angular_velocity.is_zero()
}

fn sweep_bounds_overlap(left: RotationalSweepBounds3d, right: RotationalSweepBounds3d) -> bool {
    (0..3).all(|axis| {
        left.minimum[axis] <= right.maximum[axis] && right.minimum[axis] <= left.maximum[axis]
    })
}

#[cfg(test)]
fn fixed_dynamic_pairs(
    boxes: &[RigidBox3d],
    broad_phase: &mut RotatingBroadPhase3d,
) -> Result<Vec<(usize, usize, usize)>, RotatingWorldError3d> {
    let indices = boxes
        .iter()
        .enumerate()
        .map(|(index, rigid_box)| (rigid_box.body.id, index))
        .collect::<BTreeMap<_, _>>();
    let candidates = broad_phase
        .response_candidate_pairs(boxes, RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 0, 1))
        .map_err(map_fixed_boundary_broad_phase_error)?;
    let mut pairs = Vec::with_capacity(candidates.len());
    for pair in candidates {
        let left_index = *indices
            .get(&pair.left)
            .ok_or(RotatingWorldError3d::MissingBody(pair.left))?;
        let right_index = *indices
            .get(&pair.right)
            .ok_or(RotatingWorldError3d::MissingBody(pair.right))?;
        let dynamic_index = match (boxes[left_index].body.kind, boxes[right_index].body.kind) {
            (BodyKind::Fixed, BodyKind::Dynamic) => Some(right_index),
            (BodyKind::Dynamic, BodyKind::Fixed) => Some(left_index),
            _ => None,
        };
        if let Some(dynamic_index) = dynamic_index {
            pairs.push((left_index, right_index, dynamic_index));
        }
    }
    Ok(pairs)
}

#[cfg(test)]
fn map_fixed_boundary_broad_phase_error(error: RotatingBroadPhaseError3d) -> RotatingWorldError3d {
    match error {
        RotatingBroadPhaseError3d::DuplicateBodyId(id) => RotatingWorldError3d::DuplicateBody(id),
        RotatingBroadPhaseError3d::IncrementalQueryUnsynchronized(id) => {
            RotatingWorldError3d::MissingBody(id)
        }
        RotatingBroadPhaseError3d::FreeFlight(error) => RotatingWorldError3d::FreeFlight(error),
    }
}

#[cfg(test)]
fn fixed_dynamic_pairs_reference(boxes: &[RigidBox3d]) -> Vec<(usize, usize, usize)> {
    let mut pairs = Vec::new();
    for left_index in 0..boxes.len() {
        for right_index in (left_index + 1)..boxes.len() {
            let dynamic_index = match (boxes[left_index].body.kind, boxes[right_index].body.kind) {
                (BodyKind::Fixed, BodyKind::Dynamic) => Some(right_index),
                (BodyKind::Dynamic, BodyKind::Fixed) => Some(left_index),
                _ => None,
            };
            if let Some(dynamic_index) = dynamic_index
                && boxes[left_index]
                    .collision_layers()
                    .collides_with(boxes[right_index].collision_layers())
            {
                pairs.push((left_index, right_index, dynamic_index));
            }
        }
    }
    pairs
}

#[cfg(test)]
mod tests {
    use std::{collections::BTreeSet, hint::black_box, time::Instant};

    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, InteractionCategory3d, InteractionPolicy3d,
        Material, Orientation3d, RigidBody, RigidBox3d, RotatingWorldConfig3d, Vec3i,
        obb_contact_seed, rotating_broad_phase::RotatingBroadPhase3d,
    };

    use super::{
        PositionCorrectionAccumulator, RotatingWorld3d, SLEEP_STABLE_STEPS_AT_60_HZ,
        fixed_dynamic_pairs, fixed_dynamic_pairs_reference,
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

    fn sparse_fixed_dynamic_scene(pair_count: usize) -> Vec<RigidBox3d> {
        let mut boxes = Vec::with_capacity(pair_count.saturating_mul(2));
        for index in 0..pair_count {
            let base = i32::try_from(index).expect("benchmark index fits i32") * 32;
            boxes.push(fixed(
                u64::try_from(index).expect("benchmark id fits u64") * 2 + 1,
                Vec3i::new(base, 0, 0),
            ));
            boxes.push(dynamic(
                u64::try_from(index).expect("benchmark id fits u64") * 2 + 2,
                Vec3i::new(base + 2, 0, 0),
                Vec3i::ZERO,
            ));
        }
        boxes.sort_by_key(|rigid_box| rigid_box.body.id);
        boxes
    }

    fn exact_fixed_contacts(
        boxes: &[RigidBox3d],
        pairs: &[(usize, usize, usize)],
    ) -> Vec<(BodyId, BodyId)> {
        let mut contacts = pairs
            .iter()
            .filter_map(|&(left, right, _)| {
                obb_contact_seed(boxes[left].oriented_box(), boxes[right].oriented_box())
                    .expect("benchmark geometry is valid")
                    .map(|_| (boxes[left].body.id, boxes[right].body.id))
            })
            .collect::<Vec<_>>();
        contacts.sort_unstable();
        contacts
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
    fn moving_sleep_candidate_stays_scheduled_until_motion_can_settle() {
        let id = BodyId(1);
        let mut world = zero_gravity_world();
        world
            .add_box(dynamic(id.0, Vec3i::ZERO, Vec3i::new(500, 0, 0)))
            .expect("add moving body");

        world.step(1, 60).expect("moving step");

        assert!(!world.is_sleeping(id));
        assert!(
            world.sleep_candidates.contains(&id),
            "awake dynamics must stay scheduled for later sleep eligibility"
        );
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
    fn fixed_boundary_broad_phase_preserves_exact_current_contacts() {
        for pair_count in [8_usize, 32, 128] {
            let boxes = sparse_fixed_dynamic_scene(pair_count);
            let reference = fixed_dynamic_pairs_reference(&boxes);
            let mut broad_phase = RotatingBroadPhase3d::default();
            let pruned = fixed_dynamic_pairs(&boxes, &mut broad_phase).expect("broad phase");
            assert_eq!(
                exact_fixed_contacts(&boxes, &pruned),
                exact_fixed_contacts(&boxes, &reference),
                "exact fixed contacts drifted at {pair_count} sparse pairs"
            );
        }
    }

    #[test]
    #[ignore = "release-mode scaling evidence for fixed-boundary broad-phase pruning"]
    fn fixed_boundary_broad_phase_scaling_benchmark() {
        for pair_count in [32_usize, 64, 128, 256] {
            let boxes = sparse_fixed_dynamic_scene(pair_count);
            let iterations = 100_usize;

            let reference_start = Instant::now();
            let mut reference_candidates = 0_usize;
            for _ in 0..iterations {
                reference_candidates = reference_candidates.saturating_add(black_box(
                    fixed_dynamic_pairs_reference(black_box(&boxes)).len(),
                ));
            }
            let reference_elapsed = reference_start.elapsed();

            let mut broad_phase = RotatingBroadPhase3d::default();
            let broad_start = Instant::now();
            let mut broad_candidates = 0_usize;
            for _ in 0..iterations {
                broad_candidates = broad_candidates.saturating_add(black_box(
                    fixed_dynamic_pairs(black_box(&boxes), &mut broad_phase)
                        .expect("broad phase")
                        .len(),
                ));
            }
            let broad_elapsed = broad_start.elapsed();

            assert_eq!(reference_candidates, pair_count * pair_count * iterations);
            assert_eq!(broad_candidates, pair_count * iterations);
            println!(
                "fixed-boundary candidates: pairs={pair_count}, iterations={iterations}, all_pairs={}, broad_phase={}, all_pairs_elapsed={reference_elapsed:?}, broad_phase_elapsed={broad_elapsed:?}",
                reference_candidates / iterations,
                broad_candidates / iterations,
            );
        }
    }

    #[test]
    fn fixed_boundary_stabilization_consumes_only_named_dynamic_subjects() {
        let target = BodyId(1);
        let mut world = zero_gravity_world();
        world
            .add_box(fixed(1000, Vec3i::ZERO))
            .expect("fixed obstacle");
        world
            .add_box(dynamic(target.0, Vec3i::ZERO, Vec3i::ZERO))
            .expect("overlapping target");
        for id in 2..=66 {
            world
                .add_box(dynamic(
                    id,
                    Vec3i::new(i32::try_from(id).expect("small id") * 32, 0, 0),
                    Vec3i::ZERO,
                ))
                .expect("unrelated dynamic");
        }

        let before_unrelated = world.box_by_id(BodyId(66)).expect("unrelated body").clone();
        let changed = world
            .stabilize_fixed_boundaries(&BTreeSet::from([target]))
            .expect("precise fixed-boundary stabilization");

        assert_eq!(changed, BTreeSet::from([target]));
        assert_ne!(
            world
                .box_by_id(target)
                .expect("target remains")
                .body
                .position,
            Vec3i::ZERO
        );
        assert_eq!(world.box_by_id(BodyId(66)), Some(&before_unrelated));
    }

    #[test]
    fn pair_policy_can_disable_fixed_boundary_stabilization_for_one_pair() {
        let target = BodyId(1);
        let terrain = BodyId(1000);
        let crate_category = InteractionCategory3d::new(1);
        let terrain_category = InteractionCategory3d::new(2);
        let mut world = zero_gravity_world();
        world
            .add_box(fixed(terrain.0, Vec3i::ZERO))
            .expect("fixed obstacle");
        world
            .add_box(dynamic(target.0, Vec3i::ZERO, Vec3i::ZERO))
            .expect("overlapping target");
        world
            .set_body_interaction_category(target, crate_category)
            .expect("categorize crate");
        world
            .set_body_interaction_category(terrain, terrain_category)
            .expect("categorize terrain");
        world.set_pair_interaction_policy(
            crate_category,
            terrain_category,
            InteractionPolicy3d::default().with_fixed_boundary_stabilization_pass_limit(0),
        );

        let changed = world
            .stabilize_fixed_boundaries(&BTreeSet::from([target]))
            .expect("policy-bounded stabilization");

        assert!(changed.is_empty());
        assert_eq!(
            world
                .box_by_id(target)
                .expect("target remains")
                .body()
                .position(),
            Vec3i::ZERO
        );
    }

    #[test]
    #[ignore = "release-mode evidence that fixed-boundary work follows changed subjects"]
    fn fixed_boundary_changed_subject_scaling_benchmark() {
        for unrelated in [32_u64, 128, 512] {
            let target = BodyId(1);
            let mut world = zero_gravity_world();
            world
                .add_box(fixed(1000, Vec3i::ZERO))
                .expect("fixed obstacle");
            world
                .add_box(dynamic(target.0, Vec3i::ZERO, Vec3i::ZERO))
                .expect("overlapping target");
            for id in 2..=(unrelated + 1) {
                world
                    .add_box(dynamic(
                        id,
                        Vec3i::new(i32::try_from(id).expect("small id") * 32, 0, 0),
                        Vec3i::ZERO,
                    ))
                    .expect("unrelated dynamic");
            }

            let started = Instant::now();
            let changed = black_box(
                world
                    .stabilize_fixed_boundaries(&BTreeSet::from([target]))
                    .expect("precise stabilization"),
            );
            let elapsed = started.elapsed();
            assert_eq!(changed, BTreeSet::from([target]));
            println!(
                "precise fixed-boundary stabilization: unrelated_dynamics={unrelated}, subjects=1, elapsed={elapsed:?}"
            );
        }
    }

    #[test]
    fn low_motion_body_sleeps_and_remains_stationary() {
        let id = BodyId(1);
        let mut world = zero_gravity_world();
        world
            .add_box(dynamic(id.0, Vec3i::new(10, 20, 30), Vec3i::ZERO))
            .expect("add body");

        for _ in 0..SLEEP_STABLE_STEPS_AT_60_HZ {
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
    fn equivalent_simulated_time_owns_sleep_threshold() {
        for (denominator, steps) in [(30, 6_u8), (60, 12_u8), (120, 24_u8)] {
            let id = BodyId(u64::from(denominator as u32));
            let mut world = zero_gravity_world();
            world
                .add_box(dynamic(id.0, Vec3i::ZERO, Vec3i::ZERO))
                .expect("add stable body");

            for _ in 0..steps - 1 {
                world.step(1, denominator).expect("pre-threshold step");
            }
            assert!(
                !world.is_sleeping(id),
                "body slept before 0.2 simulated seconds at {denominator} Hz"
            );

            world.step(1, denominator).expect("threshold step");
            assert!(
                world.is_sleeping(id),
                "body did not sleep after 0.2 simulated seconds at {denominator} Hz"
            );
        }
    }

    #[test]
    fn explicit_velocity_change_wakes_sleeping_body() {
        let id = BodyId(2);
        let mut world = zero_gravity_world();
        world
            .add_box(dynamic(id.0, Vec3i::ZERO, Vec3i::ZERO))
            .expect("add body");
        for _ in 0..SLEEP_STABLE_STEPS_AT_60_HZ {
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
        for _ in 0..SLEEP_STABLE_STEPS_AT_60_HZ {
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
