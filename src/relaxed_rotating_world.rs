use std::collections::{BTreeMap, BTreeSet};

use crate::{
    BallisticSphere3d, BodyCurrentContact3d, BodyId, BodyKind, InteractionCategory3d,
    InteractionExecutionPlan3d, InteractionPolicy3d, Orientation3d, OrientedBox3d,
    PerformanceCounterU64, RigidBox3d, RigidBoxFreeFlightConfig3d, RotatingBroadPhaseError3d,
    RotatingWorldConfig3d, RotatingWorldError3d, RotatingWorldStepReport3d,
    RotatingWorldStepStats3d, SolverParticipation3d, Vec3i, WakePropagation3d,
    rigid_box_free_flight_sweep_bounds, rotating_broad_phase::RotatingBoundsIndex3d,
    strict_stabilized_rotating_world::RotatingWorld3d as StrictRotatingWorld3d,
};

/// Performance-oriented rotating world that parks settled dynamics as persistent fixed proxies.
///
/// The strict stabilized world is the sole authority for every awake body. This layer retains original
/// body state only for deliberately parked dynamics, because the active solver currently represents those
/// bodies with fixed collision proxies. Awake bodies are never mirrored into a second full-world map.
///
/// Before collision response, the core tests the actual swept rigid/ballistic contact against
/// parked proxies. An admitted contact requests activation of its dynamic contact island. The
/// uncommitted attempt is discarded and repeated with real dynamic mass/inertia; the existing
/// working rigid buffer and a staged ballistic lane prevent partial physical commits. Every retry
/// activates at least one parked body, bounding retries by parked membership. A broad-phase near
/// miss, projectile creation, or removal outside a contact island does not wake the tower. Passive
/// linear-actuator support and explicit no-wake policies remain authoritative. Actual fixed geometry
/// does not connect otherwise independent dynamic islands through a shared floor. Fixed-geometry
/// additions still invalidate parked state conservatively.
///
/// Parking is a bounded representation transition, not a world snapshot. Only the bodies actually parked
/// have retained original state. Parking copies that body's dynamic state for public queries; waking
/// restores its response kind in place. Ordinary active stepping performs no wrapper-level body
/// synchronization pass.
#[derive(Clone, Debug)]
pub struct RotatingWorld3d {
    active: StrictRotatingWorld3d,
    parked: BTreeMap<BodyId, RigidBox3d>,
    parked_wake_index: RotatingBoundsIndex3d,
    active_dynamic_count: usize,
    interval_parked: Option<BTreeMap<BodyId, Option<RigidBox3d>>>,
    #[cfg(test)]
    dependency_queries: std::cell::Cell<usize>,
}

impl RotatingWorld3d {
    #[must_use]
    pub fn new(config: RotatingWorldConfig3d) -> Self {
        Self {
            active: StrictRotatingWorld3d::new(config),
            parked: BTreeMap::new(),
            parked_wake_index: RotatingBoundsIndex3d::default(),
            active_dynamic_count: 0,
            interval_parked: None,
            #[cfg(test)]
            dependency_queries: std::cell::Cell::new(0),
        }
    }

    #[must_use]
    pub fn config(&self) -> RotatingWorldConfig3d {
        self.active.config()
    }

    #[must_use]
    pub fn body_interaction_category(&self, id: BodyId) -> InteractionCategory3d {
        self.active.body_interaction_category(id)
    }

    pub fn set_body_interaction_category(
        &mut self,
        id: BodyId,
        category: InteractionCategory3d,
    ) -> Result<Option<InteractionCategory3d>, RotatingWorldError3d> {
        self.active.set_body_interaction_category(id, category)
    }

    pub fn set_default_interaction_policy(&mut self, policy: InteractionPolicy3d) {
        self.active.set_default_interaction_policy(policy);
    }

    #[must_use]
    pub fn interaction_policy_for_bodies(
        &self,
        source: BodyId,
        target: BodyId,
    ) -> InteractionPolicy3d {
        self.active.interaction_policy_for_bodies(source, target)
    }

    #[must_use]
    pub fn interaction_execution_plan_for_bodies(
        &self,
        source: BodyId,
        target: BodyId,
    ) -> InteractionExecutionPlan3d {
        self.active
            .interaction_execution_plan_for_bodies(source, target)
    }

    pub fn set_pair_interaction_policy(
        &mut self,
        left: InteractionCategory3d,
        right: InteractionCategory3d,
        policy: InteractionPolicy3d,
    ) -> Option<InteractionPolicy3d> {
        self.active.set_pair_interaction_policy(left, right, policy)
    }

    pub fn clear_pair_interaction_policy(
        &mut self,
        left: InteractionCategory3d,
        right: InteractionCategory3d,
    ) -> Option<InteractionPolicy3d> {
        self.active.clear_pair_interaction_policy(left, right)
    }

    pub fn set_directional_interaction_policy(
        &mut self,
        source: InteractionCategory3d,
        target: InteractionCategory3d,
        policy: InteractionPolicy3d,
    ) -> Option<InteractionPolicy3d> {
        self.active
            .set_directional_interaction_policy(source, target, policy)
    }

    pub fn clear_directional_interaction_policy(
        &mut self,
        source: InteractionCategory3d,
        target: InteractionCategory3d,
    ) -> Option<InteractionPolicy3d> {
        self.active
            .clear_directional_interaction_policy(source, target)
    }

    pub fn add_box(&mut self, rigid_box: RigidBox3d) -> Result<(), RotatingWorldError3d> {
        let id = rigid_box.body().id();
        if self.active.box_by_id(id).is_some() {
            return Err(RotatingWorldError3d::DuplicateBody(id));
        }

        if rigid_box.body().kind() == BodyKind::Fixed {
            self.unpark_all()?;
        }
        let dynamic = rigid_box.body().kind() == BodyKind::Dynamic;
        self.active.add_box(rigid_box)?;
        if dynamic {
            self.active_dynamic_count = self.active_dynamic_count.saturating_add(1);
        }
        Ok(())
    }

    pub fn remove_box(&mut self, id: BodyId) -> Option<RigidBox3d> {
        self.box_by_id(id)?;
        let dependents = self
            .contact_dependents(id, false)
            .unwrap_or_else(|_| self.parked.keys().copied().collect());
        let removed = if self.parked.contains_key(&id) {
            self.active.remove_box(id)?;
            self.parked_wake_index.remove(id);
            self.parked.remove(&id)?
        } else {
            let removed = self.active.remove_box(id)?;
            if removed.body().kind() == BodyKind::Dynamic {
                self.active_dynamic_count = self.active_dynamic_count.saturating_sub(1);
            }
            removed
        };
        self.active.clear_body_interaction_category(id);

        for dependent in dependents {
            self.unpark_body(dependent)
                .expect("parked bodies are valid and disjoint from active dynamics");
        }
        Some(removed)
    }

    pub fn add_ballistic_sphere(
        &mut self,
        projectile: BallisticSphere3d,
        retire_on_contact: bool,
    ) -> Result<(), RotatingWorldError3d> {
        // Creation is not collision evidence. Parked targets remain collision-testable proxies
        // until the chronological narrow phase admits an impact during a nonzero step.
        self.active
            .add_ballistic_sphere(projectile, retire_on_contact)
    }

    pub fn remove_ballistic_sphere(&mut self, id: BodyId) -> Option<BallisticSphere3d> {
        self.active.remove_ballistic_sphere(id)
    }

    #[must_use]
    pub fn ballistic_sphere_by_id(&self, id: BodyId) -> Option<&BallisticSphere3d> {
        self.active.ballistic_sphere_by_id(id)
    }

    pub fn ballistic_spheres(&self) -> impl Iterator<Item = &BallisticSphere3d> {
        self.active.ballistic_spheres()
    }

    #[must_use]
    pub fn ballistic_sphere_count(&self) -> usize {
        self.active.ballistic_sphere_count()
    }

    #[must_use]
    pub fn box_by_id(&self, id: BodyId) -> Option<&RigidBox3d> {
        self.parked.get(&id).or_else(|| self.active.box_by_id(id))
    }

    /// Iterates bodies in the active solver's stable `BodyId` order, substituting the retained original
    /// dynamic state for parked fixed proxies without materializing a second world collection.
    pub fn boxes(&self) -> impl Iterator<Item = &RigidBox3d> {
        let parked = &self.parked;
        self.active
            .boxes()
            .map(move |rigid_box| parked.get(&rigid_box.body().id()).unwrap_or(rigid_box))
    }

    #[must_use]
    pub fn is_sleeping(&self, id: BodyId) -> bool {
        self.parked.contains_key(&id) || self.active.is_sleeping(id)
    }

    #[must_use]
    pub fn sleeping_body_count(&self) -> usize {
        self.parked
            .len()
            .saturating_add(self.active.sleeping_body_count())
    }

    pub fn set_linear_velocity(
        &mut self,
        id: BodyId,
        velocity: Vec3i,
    ) -> Result<(), RotatingWorldError3d> {
        self.unpark(id)?;
        self.active.set_linear_velocity(id, velocity)
    }

    pub fn set_orientations(
        &mut self,
        updates: &[(BodyId, Orientation3d)],
    ) -> Result<(), RotatingWorldError3d> {
        for (id, _) in updates {
            self.unpark(*id)?;
        }
        self.active.set_orientations(updates)
    }

    pub fn overlap_query(&self, query: OrientedBox3d) -> Result<Vec<BodyId>, RotatingWorldError3d> {
        self.active.overlap_query(query)
    }

    pub fn body_contacts(
        &self,
        body: BodyId,
    ) -> Result<Vec<BodyCurrentContact3d>, RotatingWorldError3d> {
        self.active.body_contacts(body)
    }

    pub fn body_overlaps(&self, body: BodyId) -> Result<Vec<BodyId>, RotatingWorldError3d> {
        self.active.body_overlaps(body)
    }

    /// Advances a box-only interval atomically. Reports are cleared on any returned error.
    /// Only touched motion/sleep/parking state is journaled; scene membership cannot change here.
    pub fn advance_interval(
        &mut self,
        config: crate::RotatingIntervalConfig3d,
        reports: &mut Vec<RotatingWorldStepReport3d>,
    ) -> Result<crate::RotatingIntervalWork3d, Box<crate::RotatingIntervalFailure3d>> {
        use crate::{RotatingIntervalError3d, RotatingIntervalFailure3d, RotatingIntervalWork3d};
        reports.clear();
        let invalid = |error| {
            Box::new(RotatingIntervalFailure3d {
                error,
                work: RotatingIntervalWork3d::default(),
            })
        };
        let denominator = config.substep_denominator().map_err(invalid)?;
        if self.active.ballistic_sphere_count() != 0 {
            return Err(invalid(RotatingIntervalError3d::BallisticBodiesUnsupported));
        }
        let active_count_before = self.active_dynamic_count;
        let counters_before = self.active.contact_work_counters();
        self.interval_parked = Some(BTreeMap::new());
        self.active.begin_interval();
        let mut work = RotatingIntervalWork3d::default();
        let mut error = None;
        for _ in 0..config.substeps {
            match self.step(config.timestep_numerator, denominator) {
                Ok(report) => {
                    reports.push(report);
                    work.completed_substeps += 1;
                }
                Err(cause) => {
                    error = Some(cause);
                    break;
                }
            }
        }
        if error.is_none() && config.timestep_numerator != 0 {
            let (visits, changed) = self
                .active
                .damp_interval_angular_velocity(config.angular_damping_milli);
            work.damping_body_visits = visits;
            work.damping_changes = changed.len();
            if !changed.is_empty()
                && let Some(report) = reports.last_mut()
            {
                let mut ids = report
                    .changed_body_ids
                    .iter()
                    .copied()
                    .collect::<BTreeSet<_>>();
                ids.extend(changed);
                report.changed_body_ids = ids.into_iter().collect();
            }
        }
        let counters_after = self.active.contact_work_counters();
        work.contact_work = std::array::from_fn(|index| {
            counters_after[index].saturating_sub(counters_before[index])
        });
        let rollback = error.is_some();
        (work.motion_before_images, work.sleep_before_images) =
            self.active.finish_interval(rollback);
        let parked = self.interval_parked.take().expect("interval was begun");
        work.parked_before_images = parked.len();
        if rollback {
            for (id, original) in parked {
                self.parked_wake_index.remove(id);
                if let Some(body) = original {
                    self.parked_wake_index
                        .insert_stationary(&body)
                        .expect("original parked bounds were validated");
                    self.parked.insert(id, body);
                } else {
                    self.parked.remove(&id);
                }
            }
            self.active_dynamic_count = active_count_before;
            reports.clear();
        }
        if let Some(error) = error {
            Err(Box::new(RotatingIntervalFailure3d {
                error: RotatingIntervalError3d::World(error),
                work,
            }))
        } else {
            Ok(work)
        }
    }

    fn record_interval_parked(&mut self, id: BodyId) {
        if let Some(journal) = &mut self.interval_parked {
            journal
                .entry(id)
                .or_insert_with(|| self.parked.get(&id).cloned());
        }
    }

    pub fn step(
        &mut self,
        timestep_numerator: i32,
        timestep_denominator: i32,
    ) -> Result<RotatingWorldStepReport3d, RotatingWorldError3d> {
        if timestep_numerator < 0 || timestep_denominator <= 0 {
            return self.active.step(timestep_numerator, timestep_denominator);
        }
        if timestep_numerator == 0 {
            return self.active.step(timestep_numerator, timestep_denominator);
        }
        if self.active_dynamic_count == 0 && self.active.ballistic_sphere_count() == 0 {
            return Ok(self.quiescent_report());
        }
        let mut changed_body_ids = BTreeSet::new();
        let mut probe_work = [PerformanceCounterU64::default(); 4];
        let mut wake_retries = PerformanceCounterU64::default();
        let mut report = loop {
            let before = self.active.contact_work_counters();
            match self.active.step_with_parked(
                timestep_numerator,
                timestep_denominator,
                &self.parked,
            ) {
                Ok(report) => break report,
                Err(error) => {
                    let Some(id) = parked_contact_request(error) else {
                        return Err(error);
                    };
                    // Every retry consumes at least one parked body, so this is bounded by
                    // the original parked count, not an enlarged collision-event budget.
                    if !self.parked.contains_key(&id) {
                        return Err(error);
                    }
                    let after = self.active.contact_work_counters();
                    for (sum, (after, before)) in
                        probe_work.iter_mut().zip(after.into_iter().zip(before))
                    {
                        *sum = sum.saturating_add(after.saturating_sub(before));
                    }
                    let island = self.contact_dependents(id, true)?;
                    for body in island {
                        if self.unpark_body(body)? {
                            changed_body_ids.insert(body);
                        }
                    }
                    wake_retries = wake_retries.saturating_add(1);
                }
            }
        };
        crate::performance_counter!({
            report.stats.parked_wake_retries = wake_retries.value();
            report.stats.wake_probe_broad_phase_queries = probe_work[0].value();
            report.stats.wake_probe_tail_broad_phase_queries = probe_work[1].value();
            report.stats.wake_probe_response_passes = probe_work[2].value();
            report.stats.continuation_contact_evaluations = report
                .stats
                .continuation_contact_evaluations
                .saturating_add(probe_work[3].value());
            report.stats.parked_bodies_woken = changed_body_ids.len();
        });
        changed_body_ids.extend(report.changed_body_ids.iter().copied());
        self.park_new_sleepers(&changed_body_ids)?;
        report.changed_body_ids = changed_body_ids.into_iter().collect();
        crate::performance_counter!({
            report.stats.body_count = self
                .active
                .boxes()
                .count()
                .saturating_add(self.active.ballistic_sphere_count());
        });
        Ok(report)
    }

    fn quiescent_report(&self) -> RotatingWorldStepReport3d {
        RotatingWorldStepReport3d {
            changed_body_ids: Vec::new(),
            stats: if cfg!(feature = "performance-counters") {
                RotatingWorldStepStats3d {
                    body_count: self
                        .active
                        .boxes()
                        .count()
                        .saturating_add(self.active.ballistic_sphere_count()),
                    ..RotatingWorldStepStats3d::default()
                }
            } else {
                RotatingWorldStepStats3d::default()
            },
        }
    }

    fn park_new_sleepers(
        &mut self,
        changed_body_ids: &BTreeSet<BodyId>,
    ) -> Result<(), RotatingWorldError3d> {
        let sleepers = changed_body_ids
            .iter()
            .copied()
            .filter(|id| {
                self.active.box_by_id(*id).is_some_and(|rigid_box| {
                    rigid_box.body().kind() == BodyKind::Dynamic && self.active.is_sleeping(*id)
                })
            })
            .collect::<Vec<_>>();

        for id in sleepers {
            let rigid_box = self
                .active
                .box_by_id(id)
                .cloned()
                .ok_or(RotatingWorldError3d::MissingBody(id))?;
            self.record_interval_parked(id);
            self.active.transition_parked_body(id, BodyKind::Fixed)?;
            self.active_dynamic_count = self.active_dynamic_count.saturating_sub(1);
            self.parked.insert(id, rigid_box);
            self.parked_wake_index
                .insert_stationary(&self.parked[&id])
                .map_err(map_broad_phase_error)?;
        }
        Ok(())
    }

    fn unpark(&mut self, id: BodyId) -> Result<(), RotatingWorldError3d> {
        self.unpark_body(id)?;
        Ok(())
    }

    fn unpark_body(&mut self, id: BodyId) -> Result<bool, RotatingWorldError3d> {
        if !self.parked.contains_key(&id) {
            return Ok(false);
        }
        self.record_interval_parked(id);
        self.active.transition_parked_body(id, BodyKind::Dynamic)?;
        self.parked.remove(&id);
        self.parked_wake_index.remove(id);
        self.active_dynamic_count = self.active_dynamic_count.saturating_add(1);
        Ok(true)
    }

    fn unpark_all(&mut self) -> Result<(), RotatingWorldError3d> {
        let ids = self.parked.keys().copied().collect::<Vec<_>>();
        for id in ids {
            self.unpark_body(id)?;
        }
        Ok(())
    }

    /// Traverse dynamic contact dependencies only. A shared fixed floor is not an island edge.
    /// This runs on activation/removal, never on a projectile near miss.
    fn contact_dependents(
        &self,
        seed: BodyId,
        respect_wake_policy: bool,
    ) -> Result<BTreeSet<BodyId>, RotatingWorldError3d> {
        let mut pending = BTreeSet::from([seed]);
        let mut visited = BTreeSet::new();
        let mut parked = BTreeSet::new();
        while let Some(id) = pending.pop_first() {
            if !visited.insert(id) {
                continue;
            }
            let Some(source) = self.box_by_id(id) else {
                continue;
            };
            if self.parked.contains_key(&id) {
                parked.insert(id);
            }
            let bounds = rigid_box_free_flight_sweep_bounds(
                source,
                RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 0, 1),
            )?;
            #[cfg(test)]
            self.dependency_queries
                .set(self.dependency_queries.get() + 1);
            let mut candidates = self.parked_wake_index.overlapping_ids(bounds).body_ids;
            // The raw current-contact graph excludes parked↔parked (fixed proxy) pairs;
            // the parked index above supplies those edges using original dynamic geometry.
            candidates.extend(
                self.active
                    .body_contacts(id)?
                    .into_iter()
                    .map(|contact| contact.other),
            );
            for next in candidates {
                if visited.contains(&next) {
                    continue;
                }
                let Some(target) = self.box_by_id(next) else {
                    continue;
                };
                if target.body().kind() == BodyKind::Dynamic
                    && target.solver_participation() == SolverParticipation3d::Solid
                    && source
                        .collision_layers()
                        .collides_with(target.collision_layers())
                    && (!respect_wake_policy
                        || self
                            .active
                            .interaction_execution_plan_for_bodies(id, next)
                            .wake_propagation()
                            == WakePropagation3d::Full)
                    && crate::obb_contact_seed(source.oriented_box(), target.oriented_box())?
                        .is_some()
                {
                    pending.insert(next);
                }
            }
        }
        Ok(parked)
    }
}

fn parked_contact_request(error: RotatingWorldError3d) -> Option<BodyId> {
    use crate::{RepeatedRotatingEventError3d, RotatingContactResponseError3d};
    match error {
        RotatingWorldError3d::Response(RotatingContactResponseError3d::ParkedBodyContact(id))
        | RotatingWorldError3d::Repeated(RepeatedRotatingEventError3d::Response(
            RotatingContactResponseError3d::ParkedBodyContact(id),
        )) => Some(id),
        _ => None,
    }
}

fn map_broad_phase_error(error: RotatingBroadPhaseError3d) -> RotatingWorldError3d {
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
        AngularState3d, AngularVelocity3d, BodyId, InteractionCategory3d, Orientation3d, RigidBody,
        RigidBox3d, RotatingWorldConfig3d, RotatingWorldStepStats3d, Vec3i,
    };

    use super::RotatingWorld3d;

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

    fn world() -> RotatingWorld3d {
        RotatingWorld3d::new(RotatingWorldConfig3d {
            gravity: Vec3i::ZERO,
            sample_count: 8,
            refinement_steps: 2,
            solver_passes: 4,
            max_events: 8,
        })
    }

    fn settle(world: &mut RotatingWorld3d, id: BodyId) {
        for _ in 0..60 {
            world.step(1, 60).expect("settling step");
            if world.is_sleeping(id) {
                return;
            }
        }
        panic!("body did not reach parked sleep state");
    }

    #[test]
    fn removing_body_clears_its_interaction_category_before_id_reuse() {
        let id = BodyId(1);
        let category = InteractionCategory3d::new(9);
        let mut world = world();
        world
            .add_box(dynamic(id.0, Vec3i::ZERO, Vec3i::ZERO))
            .expect("add categorized body");
        world
            .set_body_interaction_category(id, category)
            .expect("categorize body");
        assert_eq!(world.body_interaction_category(id), category);

        world.remove_box(id).expect("remove categorized body");
        world
            .add_box(dynamic(id.0, Vec3i::ZERO, Vec3i::ZERO))
            .expect("reuse body id");

        assert_eq!(
            world.body_interaction_category(id),
            InteractionCategory3d::DEFAULT
        );
    }

    #[test]
    fn fully_parked_world_skips_all_solver_work_and_keeps_exact_pose() {
        let id = BodyId(1);
        let mut world = world();
        world
            .add_box(dynamic(id.0, Vec3i::new(10, 20, 30), Vec3i::ZERO))
            .expect("add body");
        settle(&mut world, id);
        let settled = world.box_by_id(id).expect("parked body").clone();

        let report = world.step(1, 60).expect("quiescent step");
        assert_eq!(
            report.stats,
            RotatingWorldStepStats3d {
                body_count: 1,
                ..RotatingWorldStepStats3d::default()
            }
        );
        assert_eq!(world.box_by_id(id), Some(&settled));
        assert!(world.is_sleeping(id));
    }

    #[test]
    fn passive_support_stays_parked_and_collision_testable() {
        let sleeper = BodyId(2);
        let mut world = world();
        world
            .add_box(dynamic(sleeper.0, Vec3i::ZERO, Vec3i::ZERO))
            .expect("add sleeper");
        settle(&mut world, sleeper);
        let sleeper_before = world.box_by_id(sleeper).expect("parked sleeper").clone();

        world
            .add_box(
                dynamic(3, Vec3i::new(0, 3, 0), Vec3i::new(0, -1, 0))
                    .with_linear_push(Vec3i::new(0, -1, 0)),
            )
            .expect("add actuator");
        world.step(1, 1).expect("passive landing step");

        assert!(world.is_sleeping(sleeper));
        assert_eq!(world.box_by_id(sleeper), Some(&sleeper_before));
        assert_eq!(
            world
                .box_by_id(BodyId(3))
                .expect("actuator")
                .body()
                .position()
                .y,
            2
        );
    }

    #[test]
    fn conservative_sweep_wakes_parked_body_before_an_impact() {
        let sleeper = BodyId(4);
        let mut world = world();
        world
            .add_box(dynamic(sleeper.0, Vec3i::ZERO, Vec3i::ZERO))
            .expect("add sleeper");
        settle(&mut world, sleeper);

        world
            .add_box(dynamic(5, Vec3i::new(-10, 0, 0), Vec3i::new(20, 0, 0)))
            .expect("add mover");
        world.step(1, 1).expect("impact step");

        assert!(!world.is_sleeping(sleeper));
    }

    #[test]
    fn fixed_topology_change_wakes_parked_bodies() {
        let sleeper = BodyId(6);
        let mut world = world();
        world
            .add_box(dynamic(sleeper.0, Vec3i::ZERO, Vec3i::ZERO))
            .expect("add sleeper");
        settle(&mut world, sleeper);
        assert!(world.is_sleeping(sleeper));

        world
            .add_box(fixed(7, Vec3i::new(100, 0, 0)))
            .expect("add fixed geometry");
        assert!(!world.is_sleeping(sleeper));
    }

    #[test]
    fn awake_bodies_are_not_retained_in_a_second_state_store() {
        let mut world = world();
        world
            .add_box(dynamic(9, Vec3i::ZERO, Vec3i::new(1, 0, 0)))
            .expect("add awake body");

        assert!(world.parked.is_empty());
        assert!(world.active.box_by_id(BodyId(9)).is_some());
        assert_eq!(world.boxes().count(), 1);
    }
    #[test]
    #[ignore = "release-mode contact-cache reuse across sleep transitions"]
    fn sleep_transition_contact_reuse_benchmark() {
        for count in [32, 128, 512] {
            let mut world = world();
            for index in 0..count {
                world
                    .add_box(dynamic(
                        index as u64 + 1,
                        Vec3i::new((index / 2) * 16 + (index % 2) * 2, 0, 0),
                        Vec3i::ZERO,
                    ))
                    .unwrap();
            }
            settle(&mut world, BodyId(1));
            let query_all = |world: &RotatingWorld3d| {
                for id in 1..=count as u64 {
                    let contacts = world.body_contacts(BodyId(id)).unwrap();
                    assert_eq!(contacts.len(), 1);
                    assert_eq!(
                        contacts[0].other,
                        BodyId(if id % 2 == 1 { id + 1 } else { id - 1 })
                    );
                }
            };
            query_all(&world);
            let before = world.active.current_contact_cache_stats();
            let start = std::time::Instant::now();
            for _ in 0..4 {
                world.set_linear_velocity(BodyId(1), Vec3i::ZERO).unwrap();
                query_all(&world);
                settle(&mut world, BodyId(1));
                query_all(&world);
                assert_eq!(world.sleeping_body_count(), count as usize);
            }
            let elapsed = start.elapsed();
            let after = world.active.current_contact_cache_stats();
            crate::performance_ratchet::record(
                &format!("sleep-transition/{count}"),
                &[
                    ("graph_builds", after.0 - before.0),
                    ("subject_rebuilds", after.1 - before.1),
                ],
                &[
                    ("queries", 8 * count as u64),
                    ("contacts", 8 * count as u64),
                    ("cycles", 4),
                ],
                &[("elapsed_ms", elapsed.as_secs_f64() * 1_000.0)],
            );
        }
    }

    #[test]
    fn contact_island_queries_each_source_once_in_a_touching_chain() {
        for count in [8, 32] {
            let mut world = world();
            for index in 0..count {
                world
                    .add_box(dynamic(
                        index as u64 + 1,
                        Vec3i::new(index * 2, 0, 0),
                        Vec3i::ZERO,
                    ))
                    .unwrap();
            }
            world
                .add_box(dynamic(999, Vec3i::new(10_000, 0, 0), Vec3i::ZERO))
                .unwrap();
            settle(&mut world, BodyId(1));
            world.dependency_queries.set(0);
            // This is the contact-island traversal used after the narrow phase admits a hit.
            // Sweep proximity is deliberately not an activation mechanism.
            let island = world.contact_dependents(BodyId(1), true).unwrap();
            assert_eq!(island, (1..=count as u64).map(BodyId).collect());
            assert_eq!(world.dependency_queries.get(), count as usize);
            for id in &island {
                world.unpark(*id).unwrap();
                assert!(!world.is_sleeping(*id));
            }
            assert!(world.is_sleeping(BodyId(999)));
            crate::performance_ratchet::record(
                &format!("wake-chain/{count}"),
                &[("queries", world.dependency_queries.get() as u64)],
                &[("awakened", island.len() as u64)],
                &[],
            );
        }
    }

    #[test]
    fn waking_one_body_keeps_other_sleepers_prepared() {
        for count in [32, 128] {
            let mut world = world();
            for index in 0..count {
                world
                    .add_box(dynamic(
                        index as u64 + 1,
                        Vec3i::new(index * 16, 0, 0),
                        Vec3i::ZERO,
                    ))
                    .unwrap();
            }
            settle(&mut world, BodyId(1));
            let preparations = world.parked_wake_index.bounds_preparations;
            world.dependency_queries.set(0);
            world
                .set_linear_velocity(BodyId(1), Vec3i::new(-1, 0, 0))
                .unwrap();
            assert_eq!(
                world.parked_wake_index.bounds_preparations, preparations,
                "waking one body must not re-prepare unchanged sleeping geometry"
            );
            assert_eq!(world.sleeping_body_count(), count as usize - 1);
            let report = world.step(1, 60).unwrap();
            assert_eq!(report.stats.parked_wake_retries, 0);
            assert_eq!(report.stats.parked_bodies_woken, 0);
            assert_eq!(
                world.dependency_queries.get(),
                0,
                "a miss does not traverse contact islands"
            );
            crate::performance_ratchet::record(
                &format!("wake-local/{count}"),
                &[
                    (
                        "bounds_prepared",
                        (world.parked_wake_index.bounds_preparations - preparations) as u64,
                    ),
                    ("queries", world.dependency_queries.get() as u64),
                ],
                &[("sleepers", world.sleeping_body_count() as u64)],
                &[],
            );
        }
    }
}
