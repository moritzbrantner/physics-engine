use std::collections::{BTreeMap, BTreeSet};

use crate::{
    AngularVelocity3d, BodyId, BodyKind, OrientedBox3d, RigidBox3d, RigidBoxFreeFlightConfig3d,
    RotatingContactResponseError3d, RotatingWorldConfig3d, RotatingWorldError3d,
    RotatingWorldStepReport3d, RotationalSweepBounds3d, Vec3i,
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
/// removes residual integer penetration between fixed and dynamic bodies. Only the dynamic body's projected
/// position is committed; every velocity and orientation result from the simultaneous solver is preserved.
///
/// Dynamic bodies whose linear and angular speeds remain below the deterministic sleep thresholds for a
/// bounded number of consecutive steps are put to sleep with zero residual velocity. Sleeping bodies are
/// presented to the inner solver as fixed proxies, so gravity and persistent-contact stabilization cannot
/// keep nudging a settled body. Before each step, conservative free-flight sweep bounds wake every sleeping
/// body that an awake dynamic body may reach, including transitive sleeping islands. Explicit velocity
/// changes also wake their target, and removing a fixed or sleeping support wakes all sleepers.
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
        self.inner.add_box(rigid_box)
    }

    pub fn remove_box(&mut self, id: BodyId) -> Option<RigidBox3d> {
        let removed = self.inner.remove_box(id)?;
        let was_sleeping = self.sleeping.remove(&id);
        self.sleep_streaks.remove(&id);
        if was_sleeping || removed.body.kind == BodyKind::Fixed {
            self.wake_all_sleepers();
        }
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
        if timestep_numerator == 0 || timestep_numerator < 0 || timestep_denominator <= 0 {
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
                        awake_bounds.iter().any(|(_, bounds)| {
                            sweep_bounds_overlap(*bounds, *sleeping_bounds)
                        })
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

    fn set_sleep_proxy(
        &mut self,
        id: BodyId,
        sleeping: bool,
    ) -> Result<(), RotatingWorldError3d> {
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
            .map(|rigid_box| (rigid_box.body.id, low_motion(rigid_box)))
            .collect::<Vec<_>>();
        let mut seen = BTreeSet::new();
        let mut to_sleep = Vec::new();

        for (id, is_low_motion) in motion {
            seen.insert(id);
            if is_low_motion {
                let streak = self.sleep_streaks.entry(id).or_default();
                *streak = streak.saturating_add(1);
                if *streak >= SLEEP_STABLE_STEPS {
                    to_sleep.push(id);
                }
            } else {
                self.sleep_streaks.remove(&id);
            }
        }
        self.sleep_streaks.retain(|id, _| seen.contains(id));

        for id in to_sleep {
            self.put_body_to_sleep(id)?;
            self.sleeping.insert(id);
            self.sleep_streaks.remove(&id);
        }
        Ok(())
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

        let mut converged = false;
        let mut any_changed = false;
        for _ in 0..MAX_FIXED_POSITION_STABILIZATION_PASSES {
            let mut changed = false;
            for &(left_index, right_index, dynamic_index) in &candidate_pairs {
                let response = resolve_obb_contact(
                    boxes[left_index].clone(),
                    boxes[right_index].clone(),
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
                if projected != boxes[dynamic_index].body.position {
                    boxes[dynamic_index].body.position = projected;
                    changed = true;
                    any_changed = true;
                }
            }
            if !changed {
                converged = true;
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
        AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d,
        RotatingWorldConfig3d, Vec3i,
    };

    use super::{RotatingWorld3d, SLEEP_STABLE_STEPS};

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
}
