use std::collections::{BTreeMap, BTreeSet};

use crate::{
    BodyId, BodyKind, OrientedBox3d, RigidBox3d, RigidBoxFreeFlightConfig3d,
    RotatingWorldConfig3d, RotatingWorldError3d, RotatingWorldStepReport3d,
    RotatingWorldStepStats3d, RotationalSweepBounds3d, Vec3i, obb_contact_seed,
    rigid_box_free_flight_sweep_bounds,
    strict_stabilized_rotating_world::RotatingWorld3d as StrictRotatingWorld3d,
};

/// Performance-oriented rotating world that removes settled bodies from active simulation work.
///
/// The strict stabilized world remains responsible while a body is awake: collision discovery, contact
/// response, stabilization, and the existing deterministic sleep admission policy are unchanged. Once that
/// world declares a dynamic body sleeping, this facade parks the body outside the active solver entirely.
/// Parked bodies keep their exact pose and zero motion and therefore do not participate in sampled contact
/// search, persistent-tail solving, current-contact graph rebuilding, or fixed-boundary stabilization.
///
/// Before an active step, a conservative free-flight sweep of every awake dynamic is checked against parked
/// bodies. Any parked body whose collision-enabled bounds may be reached is restored to the active world;
/// newly restored bodies join the same sweep pass so wake-up propagates through a sleeping island. Fixed
/// geometry additions and any removals wake every parked body conservatively because support topology may
/// have changed.
///
/// This deliberately treats quiescence as an optimization boundary rather than replaying sleeping dynamics
/// as fixed proxies every frame. It preserves active-world physics but no longer promises that invisible
/// sleep bookkeeping is bit-identical to the strict implementation. In particular, if an active step fails
/// after a conservative wake, the body may remain awake on the next attempt even though physical state is
/// unchanged. That tradeoff avoids cloning the complete parked scene on every active frame.
#[derive(Clone, Debug)]
pub struct RotatingWorld3d {
    active: StrictRotatingWorld3d,
    boxes: BTreeMap<BodyId, RigidBox3d>,
    parked: BTreeSet<BodyId>,
    active_dynamic_count: usize,
}

impl RotatingWorld3d {
    #[must_use]
    pub fn new(config: RotatingWorldConfig3d) -> Self {
        Self {
            active: StrictRotatingWorld3d::new(config),
            boxes: BTreeMap::new(),
            parked: BTreeSet::new(),
            active_dynamic_count: 0,
        }
    }

    #[must_use]
    pub fn config(&self) -> RotatingWorldConfig3d {
        self.active.config()
    }

    pub fn add_box(&mut self, rigid_box: RigidBox3d) -> Result<(), RotatingWorldError3d> {
        let id = rigid_box.body().id();
        if self.boxes.contains_key(&id) {
            return Err(RotatingWorldError3d::DuplicateBody(id));
        }

        if rigid_box.body().kind() == BodyKind::Fixed {
            self.unpark_all()?;
        }
        self.active.add_box(rigid_box.clone())?;
        if rigid_box.body().kind() == BodyKind::Dynamic {
            self.active_dynamic_count = self.active_dynamic_count.saturating_add(1);
        }
        self.boxes.insert(id, rigid_box);
        Ok(())
    }

    pub fn remove_box(&mut self, id: BodyId) -> Option<RigidBox3d> {
        let removed = if self.parked.remove(&id) {
            self.boxes.remove(&id)?
        } else {
            let removed = self.active.remove_box(id)?;
            if removed.body().kind() == BodyKind::Dynamic {
                self.active_dynamic_count = self.active_dynamic_count.saturating_sub(1);
            }
            self.boxes.remove(&id);
            removed
        };

        self.unpark_all()
            .expect("parked bodies are valid and disjoint from the active world");
        Some(removed)
    }

    #[must_use]
    pub fn box_by_id(&self, id: BodyId) -> Option<&RigidBox3d> {
        self.boxes.get(&id)
    }

    pub fn boxes(&self) -> impl Iterator<Item = &RigidBox3d> {
        self.boxes.values()
    }

    #[must_use]
    pub fn is_sleeping(&self, id: BodyId) -> bool {
        self.parked.contains(&id) || self.active.is_sleeping(id)
    }

    #[must_use]
    pub fn sleeping_body_count(&self) -> usize {
        self.parked.len().saturating_add(self.active.sleeping_body_count())
    }

    pub fn set_linear_velocity(
        &mut self,
        id: BodyId,
        velocity: Vec3i,
    ) -> Result<(), RotatingWorldError3d> {
        self.unpark(id)?;
        self.active.set_linear_velocity(id, velocity)?;
        let updated = self
            .active
            .box_by_id(id)
            .cloned()
            .ok_or(RotatingWorldError3d::MissingBody(id))?;
        self.boxes.insert(id, updated);
        Ok(())
    }

    pub fn overlap_query(&self, query: OrientedBox3d) -> Result<Vec<BodyId>, RotatingWorldError3d> {
        let mut hits = self.active.overlap_query(query)?;
        for id in &self.parked {
            let rigid_box = self
                .boxes
                .get(id)
                .ok_or(RotatingWorldError3d::MissingBody(*id))?;
            if obb_contact_seed(query, rigid_box.oriented_box())?.is_some() {
                hits.push(*id);
            }
        }
        hits.sort_unstable();
        hits.dedup();
        Ok(hits)
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

        self.wake_parked_for_sweeps(timestep_numerator, timestep_denominator)?;
        if self.active_dynamic_count == 0 {
            return Ok(self.quiescent_report());
        }

        let mut report = self
            .active
            .step(timestep_numerator, timestep_denominator)?;
        self.park_new_sleepers()?;
        self.sync_active_boxes();
        report.stats.body_count = self.boxes.len();
        Ok(report)
    }

    fn quiescent_report(&self) -> RotatingWorldStepReport3d {
        RotatingWorldStepReport3d {
            stats: RotatingWorldStepStats3d {
                body_count: self.boxes.len(),
                ..RotatingWorldStepStats3d::default()
            },
        }
    }

    fn park_new_sleepers(&mut self) -> Result<(), RotatingWorldError3d> {
        let sleepers = self
            .active
            .boxes()
            .filter(|rigid_box| {
                rigid_box.body().kind() == BodyKind::Dynamic
                    && self.active.is_sleeping(rigid_box.body().id())
            })
            .map(|rigid_box| rigid_box.body().id())
            .collect::<Vec<_>>();

        for id in sleepers {
            let rigid_box = self
                .active
                .remove_box(id)
                .ok_or(RotatingWorldError3d::MissingBody(id))?;
            self.active_dynamic_count = self.active_dynamic_count.saturating_sub(1);
            self.boxes.insert(id, rigid_box);
            self.parked.insert(id);
        }
        Ok(())
    }

    fn sync_active_boxes(&mut self) {
        let updates = self
            .active
            .boxes()
            .map(|rigid_box| (rigid_box.body().id(), rigid_box.clone()))
            .collect::<Vec<_>>();
        for (id, rigid_box) in updates {
            self.boxes.insert(id, rigid_box);
        }
    }

    fn unpark(&mut self, id: BodyId) -> Result<(), RotatingWorldError3d> {
        if !self.parked.remove(&id) {
            return Ok(());
        }
        let rigid_box = self
            .boxes
            .get(&id)
            .cloned()
            .ok_or(RotatingWorldError3d::MissingBody(id))?;
        if let Err(error) = self.active.add_box(rigid_box) {
            self.parked.insert(id);
            return Err(error);
        }
        self.active_dynamic_count = self.active_dynamic_count.saturating_add(1);
        Ok(())
    }

    fn unpark_all(&mut self) -> Result<(), RotatingWorldError3d> {
        let ids = self.parked.iter().copied().collect::<Vec<_>>();
        for id in ids {
            self.unpark(id)?;
        }
        Ok(())
    }

    fn wake_parked_for_sweeps(
        &mut self,
        timestep_numerator: i32,
        timestep_denominator: i32,
    ) -> Result<(), RotatingWorldError3d> {
        if self.parked.is_empty() || self.active_dynamic_count == 0 {
            return Ok(());
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
            .copied()
            .map(|id| {
                let rigid_box = self
                    .boxes
                    .get(&id)
                    .ok_or(RotatingWorldError3d::MissingBody(id))?;
                Ok((
                    id,
                    rigid_box_free_flight_sweep_bounds(rigid_box, stationary_config)?,
                ))
            })
            .collect::<Result<BTreeMap<_, _>, RotatingWorldError3d>>()?;

        loop {
            let newly_awake = self
                .parked
                .iter()
                .copied()
                .filter(|parked_id| {
                    let Some(parked_box) = self.boxes.get(parked_id) else {
                        return true;
                    };
                    let Some(parked_bounds) = parked_bounds.get(parked_id) else {
                        return true;
                    };
                    awake_bounds.iter().any(|(awake_id, awake_bounds)| {
                        self.boxes.get(awake_id).is_some_and(|awake_box| {
                            awake_box
                                .collision_layers()
                                .collides_with(parked_box.collision_layers())
                                && sweep_bounds_overlap(*awake_bounds, *parked_bounds)
                        })
                    })
                })
                .collect::<Vec<_>>();
            if newly_awake.is_empty() {
                break;
            }

            for id in newly_awake {
                self.unpark(id)?;
                parked_bounds.remove(&id);
                let rigid_box = self
                    .boxes
                    .get(&id)
                    .ok_or(RotatingWorldError3d::MissingBody(id))?;
                awake_bounds.push((
                    id,
                    rigid_box_free_flight_sweep_bounds(rigid_box, awake_config)?,
                ));
            }
        }
        Ok(())
    }
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
    fn conservative_sweep_wakes_parked_body_before_an_impact() {
        let sleeper = BodyId(2);
        let mut world = world();
        world
            .add_box(dynamic(sleeper.0, Vec3i::ZERO, Vec3i::ZERO))
            .expect("add sleeper");
        settle(&mut world, sleeper);

        world
            .add_box(dynamic(3, Vec3i::new(-10, 0, 0), Vec3i::new(20, 0, 0)))
            .expect("add mover");
        world.step(1, 1).expect("impact step");

        assert!(!world.is_sleeping(sleeper));
    }

    #[test]
    fn fixed_topology_change_wakes_parked_bodies() {
        let sleeper = BodyId(4);
        let mut world = world();
        world
            .add_box(dynamic(sleeper.0, Vec3i::ZERO, Vec3i::ZERO))
            .expect("add sleeper");
        settle(&mut world, sleeper);
        assert!(world.is_sleeping(sleeper));

        world
            .add_box(fixed(5, Vec3i::new(100, 0, 0)))
            .expect("add fixed geometry");
        assert!(!world.is_sleeping(sleeper));
    }
}
