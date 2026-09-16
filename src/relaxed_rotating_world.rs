use std::collections::{BTreeMap, BTreeSet};

use crate::{
    AngularVelocity3d, BodyCurrentContact3d, BodyId, BodyKind, OrientedBox3d, RigidBox3d,
    RigidBoxFreeFlightConfig3d, RotatingWorldConfig3d, RotatingWorldError3d,
    RotatingWorldStepReport3d, RotatingWorldStepStats3d, RotationalSweepBounds3d, Vec3i,
    rigid_box_free_flight_sweep_bounds,
    strict_stabilized_rotating_world::RotatingWorld3d as StrictRotatingWorld3d,
};

/// Performance-oriented rotating world that parks settled dynamics as persistent fixed proxies.
///
/// The strict stabilized world is the sole authority for every awake body. This layer retains original
/// body state only for deliberately parked dynamics, because the active solver currently represents those
/// bodies with fixed collision proxies. Awake bodies are never mirrored into a second full-world map.
///
/// Before an active step, a conservative free-flight sweep of every awake dynamic is checked against parked
/// bodies. A parked body whose collision-enabled bounds may be reached is restored to dynamic simulation,
/// except when the existing linear-actuator policy proves the sweep is only passive support. Passive
/// supports stay fixed and collision-testable, so character landings do not wake or drop a settled stack.
/// Newly restored bodies join the same sweep pass so disruptive wake-up propagates through a sleeping
/// island. Fixed geometry additions and any removals wake every parked body conservatively because support
/// topology may have changed.
///
/// Parking is a bounded representation transition, not a world snapshot. Only the bodies actually parked
/// have retained original state, and each wake/sleep transition copies at most that body when needed to
/// preserve the existing fail-closed proxy swap. Ordinary active stepping performs no wrapper-level body
/// synchronization pass.
#[derive(Clone, Debug)]
pub struct RotatingWorld3d {
    active: StrictRotatingWorld3d,
    parked: BTreeMap<BodyId, RigidBox3d>,
    active_dynamic_count: usize,
}

impl RotatingWorld3d {
    #[must_use]
    pub fn new(config: RotatingWorldConfig3d) -> Self {
        Self {
            active: StrictRotatingWorld3d::new(config),
            parked: BTreeMap::new(),
            active_dynamic_count: 0,
        }
    }

    #[must_use]
    pub fn config(&self) -> RotatingWorldConfig3d {
        self.active.config()
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
        let removed = if self.parked.contains_key(&id) {
            self.active.remove_box(id)?;
            self.parked.remove(&id)?
        } else {
            let removed = self.active.remove_box(id)?;
            if removed.body().kind() == BodyKind::Dynamic {
                self.active_dynamic_count = self.active_dynamic_count.saturating_sub(1);
            }
            removed
        };

        self.unpark_all()
            .expect("parked bodies are valid and disjoint from active dynamics");
        Some(removed)
    }

    #[must_use]
    pub fn box_by_id(&self, id: BodyId) -> Option<&RigidBox3d> {
        self.parked.get(&id).or_else(|| self.active.box_by_id(id))
    }

    /// Iterates bodies in the active solver's stable `BodyId` order, substituting the retained original
    /// dynamic state for parked fixed proxies without materializing a second world collection.
    pub fn boxes(&self) -> impl Iterator<Item = &RigidBox3d> {
        let parked = &self.parked;
        self.active.boxes().map(move |rigid_box| {
            parked
                .get(&rigid_box.body().id())
                .unwrap_or(rigid_box)
        })
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

    pub fn overlap_query(&self, query: OrientedBox3d) -> Result<Vec<BodyId>, RotatingWorldError3d> {
        self.active.overlap_query(query)
    }

    pub fn body_contacts(
        &self,
        body: BodyId,
    ) -> Result<Vec<BodyCurrentContact3d>, RotatingWorldError3d> {
        self.active.body_contacts(body)
    }

    pub fn step(
        &mut self,
        timestep_numerator: i32,
        timestep_denominator: i32,
    ) -> Result<RotatingWorldStepReport3d, RotatingWorldError3d> {
        if timestep_numerator < 0 || timestep_denominator <= 0 {
            return self.active.step(timestep_numerator, timestep_denominator);
        }
        if timestep_numerator == 0 || self.active_dynamic_count == 0 {
            return Ok(self.quiescent_report());
        }

        let mut changed_body_ids =
            self.wake_parked_for_sweeps(timestep_numerator, timestep_denominator)?;
        if self.active_dynamic_count == 0 {
            return Ok(self.quiescent_report());
        }

        let mut report = self.active.step(timestep_numerator, timestep_denominator)?;
        changed_body_ids.extend(report.changed_body_ids.iter().copied());
        self.park_new_sleepers(&changed_body_ids)?;
        report.changed_body_ids = changed_body_ids.into_iter().collect();
        report.stats.body_count = self.active.boxes().count();
        Ok(report)
    }

    fn quiescent_report(&self) -> RotatingWorldStepReport3d {
        RotatingWorldStepReport3d {
            changed_body_ids: Vec::new(),
            stats: RotatingWorldStepStats3d {
                body_count: self.active.boxes().count(),
                ..RotatingWorldStepStats3d::default()
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
                .remove_box(id)
                .ok_or(RotatingWorldError3d::MissingBody(id))?;
            let proxy = fixed_sleep_proxy(rigid_box.clone());
            self.active.add_box(proxy)?;
            self.active_dynamic_count = self.active_dynamic_count.saturating_sub(1);
            self.parked.insert(id, rigid_box);
        }
        Ok(())
    }

    fn unpark(&mut self, id: BodyId) -> Result<(), RotatingWorldError3d> {
        let Some(original) = self.parked.get(&id).cloned() else {
            return Ok(());
        };
        let proxy = self
            .active
            .remove_box(id)
            .ok_or(RotatingWorldError3d::MissingBody(id))?;
        if let Err(error) = self.active.add_box(original) {
            self.active
                .add_box(proxy)
                .expect("restoring a previously valid fixed sleep proxy cannot fail");
            return Err(error);
        }
        self.parked.remove(&id);
        self.active_dynamic_count = self.active_dynamic_count.saturating_add(1);
        Ok(())
    }

    fn unpark_all(&mut self) -> Result<(), RotatingWorldError3d> {
        let ids = self.parked.keys().copied().collect::<Vec<_>>();
        for id in ids {
            self.unpark(id)?;
        }
        Ok(())
    }

    fn wake_parked_for_sweeps(
        &mut self,
        timestep_numerator: i32,
        timestep_denominator: i32,
    ) -> Result<BTreeSet<BodyId>, RotatingWorldError3d> {
        let mut awakened = BTreeSet::new();
        if self.parked.is_empty() || self.active_dynamic_count == 0 {
            return Ok(awakened);
        }

        let awake_config = RigidBoxFreeFlightConfig3d::new(
            self.config().gravity,
            timestep_numerator,
            timestep_denominator,
        );
        let stationary_config = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 0, 1);
        let mut awake_bounds = self
            .active
            .boxes()
            .filter(|rigid_box| rigid_box.body().kind() == BodyKind::Dynamic)
            .map(|rigid_box| {
                Ok((
                    rigid_box.body().id(),
                    rigid_box_free_flight_sweep_bounds(rigid_box, awake_config)?,
                ))
            })
            .collect::<Result<Vec<_>, RotatingWorldError3d>>()?;
        let mut parked_bounds = self
            .parked
            .iter()
            .map(|(id, rigid_box)| {
                Ok((
                    *id,
                    rigid_box_free_flight_sweep_bounds(rigid_box, stationary_config)?,
                ))
            })
            .collect::<Result<BTreeMap<_, _>, RotatingWorldError3d>>()?;

        loop {
            let newly_awake = self
                .parked
                .iter()
                .filter_map(|(parked_id, parked_box)| {
                    let parked_bounds = parked_bounds.get(parked_id)?;
                    awake_bounds
                        .iter()
                        .any(|(awake_id, awake_bounds)| {
                            self.active.box_by_id(*awake_id).is_some_and(|awake_box| {
                                awake_box
                                    .collision_layers()
                                    .collides_with(parked_box.collision_layers())
                                    && sweep_bounds_overlap(*awake_bounds, *parked_bounds)
                                    && !crate::linear_contact::sweep_is_passive_support(
                                        awake_box,
                                        *awake_bounds,
                                        parked_box,
                                    )
                            })
                        })
                        .then_some(*parked_id)
                })
                .collect::<Vec<_>>();
            if newly_awake.is_empty() {
                break;
            }

            for id in newly_awake {
                self.unpark(id)?;
                awakened.insert(id);
                parked_bounds.remove(&id);
                let rigid_box = self
                    .active
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
}

fn fixed_sleep_proxy(mut rigid_box: RigidBox3d) -> RigidBox3d {
    rigid_box.body.kind = BodyKind::Fixed;
    rigid_box.body.velocity = Vec3i::ZERO;
    rigid_box.angular.angular_velocity = AngularVelocity3d::default();
    rigid_box
}

fn sweep_bounds_overlap(left: RotationalSweepBounds3d, right: RotationalSweepBounds3d) -> bool {
    (0..3).all(|axis| {
        left.minimum[axis] <= right.maximum[axis] && right.minimum[axis] <= left.maximum[axis]
    })
}

#[cfg(test)]
mod tests {
    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d,
        RotatingWorldConfig3d, RotatingWorldStepStats3d, Vec3i,
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
}
