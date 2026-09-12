use std::collections::{BTreeMap, BTreeSet};

use crate::{
    BodyId, OrientedBox3d, RigidBox3d, RotatingWorldConfig3d, RotatingWorldError3d,
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
/// The existing stabilized rotating solver is retained as a system-owned resource so broad-phase caches,
/// sleeping state, exact collision discovery, and response authority stay inside the physics engine. After
/// every successful system step, authoritative solver state is written back into the ECS component store.
/// Component slots are reused in place during stepping, so the ECS boundary does not rebuild its storage
/// every frame.
///
/// This is the default public `RotatingWorld3d` integration. Consumers that deliberately need the raw
/// solver resource can use `PhysicsWorld3dKernel` instead.
#[derive(Clone, Debug)]
pub struct EcsRotatingWorld3d {
    entities: BTreeSet<BodyId>,
    rigid_boxes: ComponentStore<RigidBox3d>,
    physics: PhysicsSystem3d,
}

impl EcsRotatingWorld3d {
    #[must_use]
    pub fn new(config: RotatingWorldConfig3d) -> Self {
        Self {
            entities: BTreeSet::new(),
            rigid_boxes: ComponentStore::default(),
            physics: PhysicsSystem3d::new(config),
        }
    }

    #[must_use]
    pub fn config(&self) -> RotatingWorldConfig3d {
        self.physics.config()
    }

    /// Adds one entity with its rotating rigid-body component.
    ///
    /// The physics system validates the component before ECS membership is committed, so failed inserts
    /// leave both representations unchanged.
    pub fn add_box(&mut self, rigid_box: RigidBox3d) -> Result<(), RotatingWorldError3d> {
        let entity = rigid_box.body().id();
        self.physics.add_box(rigid_box.clone())?;
        let inserted = self.entities.insert(entity);
        debug_assert!(inserted);
        let previous = self.rigid_boxes.insert(entity, rigid_box);
        debug_assert!(previous.is_none());
        Ok(())
    }

    pub fn remove_box(&mut self, entity: BodyId) -> Option<RigidBox3d> {
        let removed = self.physics.remove_box(entity)?;
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
        self.physics.overlap_query(query)
    }

    /// Runs the physics system and writes authoritative results back to ECS components.
    pub fn step(
        &mut self,
        timestep_numerator: i32,
        timestep_denominator: i32,
    ) -> Result<RotatingWorldStepReport3d, RotatingWorldError3d> {
        let report = self
            .physics
            .step(timestep_numerator, timestep_denominator)?;
        self.sync_all_from_physics();
        Ok(report)
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

    fn sync_all_from_physics(&mut self) {
        for rigid_box in self.physics.boxes() {
            let entity = rigid_box.body().id();
            let component = self
                .rigid_boxes
                .get_mut(entity)
                .expect("physics system body must have an ECS component");
            component.clone_from(rigid_box);
        }
        debug_assert_eq!(self.rigid_boxes.len(), self.entities.len());
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d,
        RotatingWorldConfig3d, Vec3i,
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
}
