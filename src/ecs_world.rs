use std::collections::{BTreeMap, BTreeSet};

use crate::fixed_geometry::{FixedGeometryPreparationCache3d, with_fixed_geometry_context};
use crate::{
    BodyCurrentContact3d, BodyId, FixedGeometryPreparationMode3d, FixedGeometryPreparationStats3d,
    OrientedBox3d, RigidBox3d, RotatingWorldConfig3d, RotatingWorldError3d,
    RotatingWorldStepReport3d, Vec3i,
    stabilized_rotating_world::RotatingWorld3d as PhysicsSystem3d,
};

#[derive(Clone, Debug)]
struct ComponentStore<T> {
    values: BTreeMap<BodyId, T>,
}

impl<T> Default for ComponentStore<T> {
    fn default() -> Self {
        Self {
            values: BTreeMap::new(),
        }
    }
}

impl<T> ComponentStore<T> {
    fn insert(&mut self, entity: BodyId, value: T) -> Option<T> {
        self.values.insert(entity, value)
    }

    fn get(&self, entity: BodyId) -> Option<&T> {
        self.values.get(&entity)
    }

    fn get_mut(&mut self, entity: BodyId) -> Option<&mut T> {
        self.values.get_mut(&entity)
    }

    fn remove(&mut self, entity: BodyId) -> Option<T> {
        self.values.remove(&entity)
    }

    fn values(&self) -> impl Iterator<Item = &T> {
        self.values.values()
    }

    fn len(&self) -> usize {
        self.values.len()
    }
}

/// ECS-backed consumer world for rotating rigid-body physics.
///
/// Entity membership and the `RigidBox3d` physics component live in deterministic component storage.
/// The performance-oriented rotating solver is retained as a system-owned resource so broad-phase caches,
/// sleeping state, collision discovery, and response authority stay inside the physics engine. After a
/// successful active system step, authoritative solver state is written back into the ECS component store.
/// When the physics system was already fully quiescent before the step, its parked-state contract guarantees
/// that no body state can change, so the ECS boundary skips the otherwise linear component-clone pass as
/// well. Component slots are reused in place during active stepping.
///
/// Optional fixed-geometry preparation is also owned here, at the stable public scene boundary. Only fixed
/// bodies inserted by the consumer are registered for preparation; internal persistent sleep proxies never
/// cross this boundary and therefore can never become baked static geometry.
///
/// This is the default public `RotatingWorld3d` integration. Consumers that deliberately need the raw
/// solver resource can use `PhysicsWorld3dKernel` instead.
#[derive(Clone, Debug)]
pub struct EcsRotatingWorld3d {
    entities: BTreeSet<BodyId>,
    rigid_boxes: ComponentStore<RigidBox3d>,
    physics: PhysicsSystem3d,
    fixed_geometry: FixedGeometryPreparationCache3d,
}

impl EcsRotatingWorld3d {
    #[must_use]
    pub fn new(config: RotatingWorldConfig3d) -> Self {
        Self {
            entities: BTreeSet::new(),
            rigid_boxes: ComponentStore::default(),
            physics: PhysicsSystem3d::new(config),
            fixed_geometry: FixedGeometryPreparationCache3d::default(),
        }
    }

    #[must_use]
    pub fn config(&self) -> RotatingWorldConfig3d {
        self.physics.config()
    }

    /// Switches between the existing runtime preparation path and retained prepare-at-load geometry.
    ///
    /// Enabling prepare-at-load immediately prepares every currently registered genuine fixed body. Later
    /// fixed additions are prepared on insertion, and removals invalidate their retained preparation.
    /// Dynamic bodies are never registered here, including bodies that the inner sleep system presents as
    /// persistent fixed proxies after they settle.
    pub fn set_fixed_geometry_preparation_mode(&mut self, mode: FixedGeometryPreparationMode3d) {
        self.fixed_geometry
            .set_mode(mode, self.rigid_boxes.values());
    }

    #[must_use]
    pub fn fixed_geometry_preparation_stats(&self) -> FixedGeometryPreparationStats3d {
        self.fixed_geometry.stats()
    }

    /// Adds one entity with its rotating rigid-body component.
    ///
    /// The physics system validates the component before ECS membership is committed, so failed inserts
    /// leave both representations unchanged.
    pub fn add_box(&mut self, rigid_box: RigidBox3d) -> Result<(), RotatingWorldError3d> {
        let entity = rigid_box.body().id();
        self.physics.add_box(rigid_box.clone())?;
        self.fixed_geometry.register_fixed(&rigid_box);
        let inserted = self.entities.insert(entity);
        debug_assert!(inserted);
        let previous = self.rigid_boxes.insert(entity, rigid_box);
        debug_assert!(previous.is_none());
        Ok(())
    }

    pub fn remove_box(&mut self, entity: BodyId) -> Option<RigidBox3d> {
        let removed = self.physics.remove_box(entity)?;
        self.fixed_geometry.unregister(entity);
        let was_alive = self.entities.remove(&entity);
        debug_assert!(was_alive);
        let component = self.rigid_boxes.remove(entity);
        debug_assert!(component.is_some());
        Some(removed)
    }

    #[must_use]
    pub fn box_by_id(&self, entity: BodyId) -> Option<&RigidBox3d> {
        self.rigid_boxes.get(entity)
    }

    /// Iterates physics components in stable entity-id order.
    pub fn boxes(&self) -> impl Iterator<Item = &RigidBox3d> {
        self.rigid_boxes.values()
    }

    /// Iterates live ECS entity ids in stable order.
    pub fn entities(&self) -> impl Iterator<Item = BodyId> + '_ {
        self.entities.iter().copied()
    }

    #[must_use]
    pub fn entity_count(&self) -> usize {
        self.entities.len()
    }

    #[must_use]
    pub fn is_sleeping(&self, entity: BodyId) -> bool {
        self.physics.is_sleeping(entity)
    }

    #[must_use]
    pub fn sleeping_body_count(&self) -> usize {
        self.physics.sleeping_body_count()
    }

    pub fn set_linear_velocity(
        &mut self,
        entity: BodyId,
        velocity: Vec3i,
    ) -> Result<(), RotatingWorldError3d> {
        self.physics.set_linear_velocity(entity, velocity)?;
        self.sync_entity_from_physics(entity);
        Ok(())
    }

    pub fn overlap_query(&self, query: OrientedBox3d) -> Result<Vec<BodyId>, RotatingWorldError3d> {
        self.with_prepared_fixed_geometry(|| self.physics.overlap_query(query))
    }

    /// Returns exact current contacts for one known entity.
    ///
    /// Unlike the arbitrary-geometry overlap API, this preserves the caller's known body identity all the way
    /// into the physics query and therefore does not materialize a complete contact graph merely to extract one
    /// subject's contacts.
    pub fn body_contacts(
        &self,
        body: BodyId,
    ) -> Result<Vec<BodyCurrentContact3d>, RotatingWorldError3d> {
        self.with_prepared_fixed_geometry(|| self.physics.body_contacts(body))
    }

    /// Runs the physics system and writes authoritative active results back to ECS components.
    /// Runs the physics system and writes back only bodies named by the authoritative step delta.
    pub fn step(
        &mut self,
        timestep_numerator: i32,
        timestep_denominator: i32,
    ) -> Result<RotatingWorldStepReport3d, RotatingWorldError3d> {
        let prepared = self.fixed_geometry.clone();
        let report = with_fixed_geometry_context(&prepared, || {
            self.physics.step(timestep_numerator, timestep_denominator)
        })?;
        for entity in report.changed_body_ids.iter().copied() {
            self.sync_entity_from_physics(entity);
        }
        debug_assert_eq!(self.rigid_boxes.len(), self.entities.len());
        Ok(report)
    }

    pub(crate) fn with_prepared_fixed_geometry<R>(&self, callback: impl FnOnce() -> R) -> R {
        with_fixed_geometry_context(&self.fixed_geometry, callback)
    }

    fn sync_entity_from_physics(&mut self, entity: BodyId) {
        let Some(rigid_box) = self.physics.box_by_id(entity) else {
            return;
        };
        let component = self
            .rigid_boxes
            .get_mut(entity)
            .expect("live physics entity must retain its ECS component");
        component.clone_from(rigid_box);
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, FixedGeometryPreparationMode3d, Orientation3d,
        RigidBody, RigidBox3d, RotatingWorldConfig3d, Vec3i,
    };

    use super::EcsRotatingWorld3d;

    fn world() -> EcsRotatingWorld3d {
        EcsRotatingWorld3d::new(RotatingWorldConfig3d {
            gravity: Vec3i::ZERO,
            sample_count: 8,
            refinement_steps: 2,
            solver_passes: 4,
            max_events: 8,
        })
    }

    fn dynamic(id: u64, position: Vec3i, velocity: Vec3i) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::dynamic(BodyId(id), position, velocity, Vec3i::new(1, 1, 1)),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid ECS physics component")
    }

    fn fixed(id: u64, position: Vec3i) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::fixed(BodyId(id), position, Vec3i::new(4, 1, 4)),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid fixed physics component")
    }

    #[test]
    fn entity_lifecycle_owns_the_physics_component() {
        let mut world = world();
        let body = dynamic(7, Vec3i::new(1, 2, 3), Vec3i::ZERO);
        world.add_box(body.clone()).expect("spawn entity");

        assert_eq!(world.entity_count(), 1);
        assert_eq!(world.entities().collect::<Vec<_>>(), vec![BodyId(7)]);
        assert_eq!(world.box_by_id(BodyId(7)), Some(&body));

        assert_eq!(world.remove_box(BodyId(7)), Some(body));
        assert_eq!(world.entity_count(), 0);
        assert!(world.box_by_id(BodyId(7)).is_none());
    }

    #[test]
    fn physics_system_writes_motion_back_to_components() {
        let mut world = world();
        world
            .add_box(dynamic(3, Vec3i::ZERO, Vec3i::new(60, 0, 0)))
            .expect("spawn moving entity");

        world.step(1, 60).expect("physics system step");

        assert_eq!(
            world
                .box_by_id(BodyId(3))
                .expect("synced component")
                .body()
                .position(),
            Vec3i::new(1, 0, 0)
        );
    }

    #[test]
    fn step_delta_names_only_observably_changed_entities() {
        let mut world = world();
        world
            .add_box(dynamic(1, Vec3i::ZERO, Vec3i::new(60, 0, 0)))
            .expect("moving entity");
        for id in 2..=66 {
            world
                .add_box(dynamic(
                    id,
                    Vec3i::new(i32::try_from(id).expect("small id") * 16, 0, 0),
                    Vec3i::ZERO,
                ))
                .expect("unrelated stationary entity");
        }

        let report = world.step(1, 60).expect("precise writeback step");

        assert_eq!(report.changed_body_ids, vec![BodyId(1)]);
        assert_eq!(
            world
                .box_by_id(BodyId(1))
                .expect("moving entity")
                .body()
                .position(),
            Vec3i::new(1, 0, 0)
        );
        assert_eq!(
            world
                .box_by_id(BodyId(66))
                .expect("stationary entity")
                .body()
                .position(),
            Vec3i::new(66 * 16, 0, 0)
        );
    }

    #[test]
    fn controlled_velocity_updates_solver_and_component_together() {
        let mut world = world();
        world
            .add_box(dynamic(5, Vec3i::ZERO, Vec3i::ZERO))
            .expect("spawn controlled entity");

        world
            .set_linear_velocity(BodyId(5), Vec3i::new(120, 0, 0))
            .expect("set ECS physics velocity");

        assert_eq!(
            world
                .box_by_id(BodyId(5))
                .expect("updated component")
                .body()
                .velocity(),
            Vec3i::new(120, 0, 0)
        );
    }

    #[test]
    fn prepare_at_load_tracks_only_scene_fixed_bodies_and_invalidates_removal() {
        let mut world = world();
        world.add_box(fixed(1, Vec3i::ZERO)).expect("fixed floor");
        world
            .add_box(dynamic(2, Vec3i::new(0, 2, 0), Vec3i::ZERO))
            .expect("dynamic crate");
        world.set_fixed_geometry_preparation_mode(FixedGeometryPreparationMode3d::PrepareAtLoad);
        let prepared = world.fixed_geometry_preparation_stats();
        assert_eq!(prepared.prepared_body_count, 1);
        assert_eq!(prepared.total_preparations, 1);

        world.remove_box(BodyId(1)).expect("remove fixed floor");
        assert_eq!(
            world.fixed_geometry_preparation_stats().prepared_body_count,
            0
        );
    }

    #[test]
    fn runtime_mode_retains_no_fixed_preparation() {
        let mut world = world();
        world.add_box(fixed(1, Vec3i::ZERO)).expect("fixed floor");
        assert_eq!(
            world.fixed_geometry_preparation_stats().prepared_body_count,
            0
        );
    }
}
