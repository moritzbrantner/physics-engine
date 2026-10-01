//! Dependency/lifecycle controls shared by native and release WASM.
use physics_engine::{
    BodyId, BodyKind, Material, PhysicsError, RigidBody, Vec3i, World, WorldConfig,
};

fn check(world: &mut World, preparations: usize, invalidations: usize) {
    world.set_velocity(BodyId(30), Vec3i::new(1, 0, 1)).unwrap();
    let mut rebuilt = World::new(world.config());
    for body in world.bodies().cloned() {
        rebuilt.add_body(body).unwrap();
    }
    let actual = world.step(1).unwrap();
    let expected = rebuilt.step(1).unwrap();
    assert_eq!(actual.events, expected.events);
    assert_eq!(
        world.bodies().collect::<Vec<_>>(),
        rebuilt.bodies().collect::<Vec<_>>()
    );
    let fixed = world
        .bodies()
        .filter(|body| body.kind() == BodyKind::Fixed)
        .count();
    let dynamic = world.bodies().count() - fixed;
    let work = actual.stats.work;
    assert!(work.broad_phase_queries > 0);
    assert_eq!(work.fixed_sweep_bound_preparations, preparations);
    assert_eq!(work.fixed_bound_invalidations, invalidations);
    assert_eq!(
        work.fixed_sweep_bound_reuses + preparations,
        fixed * work.broad_phase_queries
    );
    assert_eq!(
        work.dynamic_sweep_bound_preparations,
        dynamic * work.broad_phase_queries
    );
    assert_eq!(
        work.sweep_bound_preparations,
        preparations + work.dynamic_sweep_bound_preparations
    );
}

pub fn run() {
    let half = Vec3i::new(1, 1, 1);
    let mut world = World::new(WorldConfig {
        gravity: Vec3i::ZERO,
        ..WorldConfig::default()
    });
    for body in [
        RigidBody::fixed(BodyId(40), Vec3i::new(200, 0, 0), half),
        RigidBody::dynamic(BodyId(30), Vec3i::new(0, 50, 0), Vec3i::ZERO, half),
        RigidBody::fixed(BodyId(20), Vec3i::new(100, 0, 0), half),
    ] {
        world.add_body(body).unwrap();
    }
    check(&mut world, 2, 0);
    check(&mut world, 0, 0);
    world
        .set_position(BodyId(20), Vec3i::new(100, 0, 0))
        .unwrap();
    world.set_material(BodyId(20), Material::new(500)).unwrap();
    check(&mut world, 0, 0);
    let same_geometry = world
        .body(BodyId(20))
        .unwrap()
        .clone()
        .with_material(Material::new(750));
    world.replace_body(same_geometry).unwrap();
    check(&mut world, 0, 0);
    world
        .set_position(BodyId(20), Vec3i::new(90, 0, 0))
        .unwrap();
    check(&mut world, 1, 1);
    world
        .replace_body(RigidBody::fixed(
            BodyId(20),
            Vec3i::new(90, 0, 0),
            Vec3i::new(2, 3, 4),
        ))
        .unwrap();
    check(&mut world, 1, 1);
    world
        .replace_body(RigidBody::dynamic(
            BodyId(20),
            Vec3i::new(90, 0, 0),
            Vec3i::ZERO,
            half,
        ))
        .unwrap();
    check(&mut world, 0, 1);
    world
        .replace_body(RigidBody::fixed(BodyId(20), Vec3i::new(80, 0, 0), half))
        .unwrap();
    check(&mut world, 1, 0);
    world.remove_body(BodyId(20)).unwrap();
    // A lower ID shifts every staged index, while only its own geometry is new.
    world
        .add_body(RigidBody::fixed(BodyId(10), Vec3i::new(70, 0, 0), half))
        .unwrap();
    check(&mut world, 1, 1);
    world
        .add_body(RigidBody::fixed(
            BodyId(20),
            Vec3i::new(60, 0, 0),
            Vec3i::new(5, 2, 1),
        ))
        .unwrap();
    check(&mut world, 1, 0);
    let before = world.bodies().cloned().collect::<Vec<_>>();
    assert_eq!(
        world.set_velocity(BodyId(20), Vec3i::new(1, 0, 0)),
        Err(PhysicsError::FixedBodyVelocity(BodyId(20)))
    );
    assert_eq!(
        world.set_position(BodyId(99), Vec3i::ZERO),
        Err(PhysicsError::MissingBody(BodyId(99)))
    );
    assert_eq!(
        world.replace_body(RigidBody::fixed(
            BodyId(20),
            Vec3i::ZERO,
            Vec3i::new(-1, 1, 1)
        )),
        Err(PhysicsError::InvalidHalfExtents(BodyId(20)))
    );
    assert_eq!(world.bodies().cloned().collect::<Vec<_>>(), before);
    check(&mut world, 0, 0);
    // Releasing capacity must preserve invalidations already performed by mutation.
    world
        .set_position(BodyId(20), Vec3i::new(55, 0, 0))
        .unwrap();
    let before_release = world.bodies().cloned().collect::<Vec<_>>();
    world.release_step_scratch();
    assert_eq!(world.retained_step_scratch_bytes(), 0);
    assert_eq!(world.bodies().cloned().collect::<Vec<_>>(), before_release);
    check(&mut world, 3, 1);
    check(&mut world, 0, 0);
    let mut cloned = world.clone();
    check(&mut cloned, 0, 0);
    world
        .set_position(BodyId(20), Vec3i::new(50, 0, 0))
        .unwrap();
    check(&mut world, 1, 1);
    check(&mut cloned, 0, 0);
}
