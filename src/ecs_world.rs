use crate::fixed_geometry::{FixedGeometryPreparationCache3d, with_fixed_geometry_context};
use crate::{
    BodyCurrentContact3d, BodyId, FixedGeometryPreparationMode3d, FixedGeometryPreparationStats3d,
    InteractionCategory3d, InteractionPolicy3d, OrientedBox3d, RigidBox3d, RotatingWorldConfig3d,
    RotatingWorldError3d, RotatingWorldStepReport3d, Vec3i,
    stabilized_rotating_world::RotatingWorld3d as PhysicsSystem3d,
};

/// Public rotating rigid-body world backed by one authoritative mutable physics state.
///
/// Earlier revisions mirrored every `RigidBox3d` into a second ECS component map and cloned changed
/// bodies back out of the physics system after each active step. That made the integration boundary own
/// two live copies of the same world and forced synchronization work that was unrelated to collision
/// semantics.
///
/// The physics system is now the sole body-state authority. Queries borrow that state directly and
/// mutations are delegated to the physics mutation boundary. The wrapper retains only derived fixed-
/// geometry preparation data, which is invalidated from explicit body lifecycle changes and never acts as
/// an alternate world representation.
///
/// This is the first migration step toward the repository's delta-only runtime architecture. The inner
/// solver still has legacy owned-state paths that will be removed separately; this boundary must not
/// reintroduce a mirrored body store while that migration proceeds.
#[derive(Clone, Debug)]
pub struct EcsRotatingWorld3d {
    physics: PhysicsSystem3d,
    fixed_geometry: FixedGeometryPreparationCache3d,
}

impl EcsRotatingWorld3d {
    #[must_use]
    pub fn new(config: RotatingWorldConfig3d) -> Self {
        Self {
            physics: PhysicsSystem3d::new(config),
            fixed_geometry: FixedGeometryPreparationCache3d::default(),
        }
    }

    #[must_use]
    pub fn config(&self) -> RotatingWorldConfig3d {
        self.physics.config()
    }

    #[must_use]
    pub fn body_interaction_category(&self, entity: BodyId) -> InteractionCategory3d {
        self.physics.body_interaction_category(entity)
    }

    pub fn set_body_interaction_category(
        &mut self,
        entity: BodyId,
        category: InteractionCategory3d,
    ) -> Result<Option<InteractionCategory3d>, RotatingWorldError3d> {
        self.physics.set_body_interaction_category(entity, category)
    }

    pub fn set_default_interaction_policy(&mut self, policy: InteractionPolicy3d) {
        self.physics.set_default_interaction_policy(policy);
    }

    pub fn set_pair_interaction_policy(
        &mut self,
        left: InteractionCategory3d,
        right: InteractionCategory3d,
        policy: InteractionPolicy3d,
    ) -> Option<InteractionPolicy3d> {
        self.physics
            .set_pair_interaction_policy(left, right, policy)
    }

    pub fn set_directional_interaction_policy(
        &mut self,
        source: InteractionCategory3d,
        target: InteractionCategory3d,
        policy: InteractionPolicy3d,
    ) -> Option<InteractionPolicy3d> {
        self.physics
            .set_directional_interaction_policy(source, target, policy)
    }

    /// Switches between the existing runtime preparation path and retained prepare-at-load geometry.
    ///
    /// Enabling prepare-at-load immediately prepares every currently registered genuine fixed body. Later
    /// fixed additions are prepared on insertion, and removals invalidate their retained preparation.
    /// Dynamic bodies are never registered here, including bodies that the inner sleep system presents as
    /// persistent fixed proxies after they settle.
    pub fn set_fixed_geometry_preparation_mode(&mut self, mode: FixedGeometryPreparationMode3d) {
        self.fixed_geometry.set_mode(mode, self.physics.boxes());
    }

    #[must_use]
    pub fn fixed_geometry_preparation_stats(&self) -> FixedGeometryPreparationStats3d {
        self.fixed_geometry.stats()
    }

    /// Adds one rotating rigid body to the authoritative world.
    ///
    /// Fixed-geometry preparation is registered only after physics validation succeeds, so failed inserts
    /// leave both authoritative and derived state unchanged.
    pub fn add_box(&mut self, rigid_box: RigidBox3d) -> Result<(), RotatingWorldError3d> {
        let entity = rigid_box.body().id();
        self.physics.add_box(rigid_box)?;
        let inserted = self
            .physics
            .box_by_id(entity)
            .expect("successful physics insertion must retain the body");
        self.fixed_geometry.register_fixed(inserted);
        Ok(())
    }

    pub fn remove_box(&mut self, entity: BodyId) -> Option<RigidBox3d> {
        let removed = self.physics.remove_box(entity)?;
        self.fixed_geometry.unregister(entity);
        Some(removed)
    }

    #[must_use]
    pub fn box_by_id(&self, entity: BodyId) -> Option<&RigidBox3d> {
        self.physics.box_by_id(entity)
    }

    /// Iterates authoritative physics bodies in stable entity-id order.
    pub fn boxes(&self) -> impl Iterator<Item = &RigidBox3d> {
        self.physics.boxes()
    }

    /// Iterates live entity ids in stable order without maintaining duplicate membership state.
    pub fn entities(&self) -> impl Iterator<Item = BodyId> + '_ {
        self.physics.boxes().map(|rigid_box| rigid_box.body().id())
    }

    #[must_use]
    pub fn entity_count(&self) -> usize {
        self.physics.boxes().count()
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
        self.physics.set_linear_velocity(entity, velocity)
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

    /// Runs the physics system against the single authoritative body state.
    ///
    /// `changed_body_ids` remains the precise observable delta report for callers, but no body clone/writeback
    /// pass is required at this boundary because there is no mirrored component store to synchronize.
    pub fn step(
        &mut self,
        timestep_numerator: i32,
        timestep_denominator: i32,
    ) -> Result<RotatingWorldStepReport3d, RotatingWorldError3d> {
        let prepared = self.fixed_geometry.clone();
        with_fixed_geometry_context(&prepared, || {
            self.physics.step(timestep_numerator, timestep_denominator)
        })
    }

    pub(crate) fn with_prepared_fixed_geometry<R>(&self, callback: impl FnOnce() -> R) -> R {
        with_fixed_geometry_context(&self.fixed_geometry, callback)
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
    fn physics_system_mutates_the_authoritative_body_directly() {
        let mut world = world();
        world
            .add_box(dynamic(3, Vec3i::ZERO, Vec3i::new(60, 0, 0)))
            .expect("spawn moving entity");

        world.step(1, 60).expect("physics system step");

        assert_eq!(
            world
                .box_by_id(BodyId(3))
                .expect("authoritative body")
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

        let report = world.step(1, 60).expect("precise delta step");

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
    fn controlled_velocity_mutates_the_authoritative_body_once() {
        let mut world = world();
        world
            .add_box(dynamic(5, Vec3i::ZERO, Vec3i::ZERO))
            .expect("spawn controlled entity");

        world
            .set_linear_velocity(BodyId(5), Vec3i::new(120, 0, 0))
            .expect("set physics velocity");

        assert_eq!(
            world
                .box_by_id(BodyId(5))
                .expect("updated authoritative body")
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
