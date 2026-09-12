use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d,
    RotatingWorld3d, RotatingWorldConfig3d, Vec3i,
};

fn angular() -> AngularState3d {
    AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default())
}

#[test]
fn zero_timestep_does_not_project_existing_fixed_dynamic_overlap() {
    let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
        gravity: Vec3i::ZERO,
        ..RotatingWorldConfig3d::default()
    });
    world
        .add_box(
            RigidBox3d::new(
                RigidBody::fixed(BodyId(1), Vec3i::ZERO, Vec3i::new(10, 10, 10)),
                angular(),
            )
            .expect("fixed box"),
        )
        .expect("fixed body");
    world
        .add_box(
            RigidBox3d::new(
                RigidBody::dynamic(
                    BodyId(2),
                    Vec3i::new(15, 0, 0),
                    Vec3i::ZERO,
                    Vec3i::new(10, 10, 10),
                ),
                angular(),
            )
            .expect("dynamic box"),
        )
        .expect("dynamic body");

    let before = world
        .boxes()
        .map(|rigid_box| (rigid_box.body().id(), rigid_box.body().position()))
        .collect::<Vec<_>>();
    world.step(0, 1).expect("zero step remains valid");
    let after = world
        .boxes()
        .map(|rigid_box| (rigid_box.body().id(), rigid_box.body().position()))
        .collect::<Vec<_>>();

    assert_eq!(before, after);
}

#[test]
fn no_op_fixed_boundary_stabilization_preserves_broad_phase_reuse() {
    let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
        gravity: Vec3i::ZERO,
        ..RotatingWorldConfig3d::default()
    });
    world
        .add_box(
            RigidBox3d::new(
                RigidBody::fixed(BodyId(1), Vec3i::ZERO, Vec3i::new(10, 10, 10)),
                angular(),
            )
            .expect("fixed box"),
        )
        .expect("fixed body");
    world
        .add_box(
            RigidBox3d::new(
                RigidBody::dynamic(
                    BodyId(2),
                    Vec3i::new(100, 0, 0),
                    Vec3i::ZERO,
                    Vec3i::new(10, 10, 10),
                ),
                angular(),
            )
            .expect("dynamic box"),
        )
        .expect("dynamic body");

    let first = world.step(1, 1).expect("initial broad-phase step");
    assert!(first.stats.broad_phase_rebuilds > 0);

    let second = world.step(1, 1).expect("reused broad-phase step");
    assert_eq!(second.stats.broad_phase_rebuilds, 0);
    assert!(second.stats.broad_phase_reuses > 0);
}
