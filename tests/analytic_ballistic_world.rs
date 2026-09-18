use physics_engine::{
    AngularState3d, AngularVelocity3d, BallisticSphere3d, BodyId, MATERIAL_SCALE, Material,
    Orientation3d, RigidBody, RigidBox3d, RotatingWorld3d, RotatingWorldConfig3d, Vec3i,
};

fn world() -> RotatingWorld3d {
    RotatingWorld3d::new(RotatingWorldConfig3d {
        gravity: Vec3i::ZERO,
        sample_count: 32,
        refinement_steps: 4,
        solver_passes: 8,
        max_events: 32,
    })
}

fn fixed(id: u64, position: Vec3i, half: Vec3i, material: Material) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::fixed(BodyId(id), position, half).with_material(material),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid fixed target")
}

fn dynamic(id: u64, position: Vec3i, half: Vec3i) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::dynamic(BodyId(id), position, Vec3i::ZERO, half)
            .with_mass(4)
            .with_material(Material::new(0)),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid dynamic target")
}

#[test]
fn analytic_sphere_crosses_thin_sweep_and_retires_at_impact() {
    let mut world = world();
    world
        .add_box(fixed(
            1,
            Vec3i::ZERO,
            Vec3i::new(2, 20, 20),
            Material::new(0),
        ))
        .expect("add thin target");
    world
        .add_ballistic_sphere(
            BallisticSphere3d::new(
                BodyId(100),
                Vec3i::new(-30, 0, 0),
                Vec3i::new(3_600, 0, 0),
                2,
                1,
            )
            .expect("valid sphere"),
            true,
        )
        .expect("add analytic sphere");

    let report = world.step(1, 60).expect("analytic sphere step");

    assert_eq!(report.stats.ballistic_impacts, 1);
    assert_eq!(report.stats.ballistic_retired, 1);
    assert!(report.stats.ballistic_toi_tests >= 1);
    assert_eq!(world.ballistic_sphere_count(), 0);
    assert!(world.ballistic_sphere_by_id(BodyId(100)).is_none());
}

#[test]
fn off_center_analytic_sphere_transfers_linear_and_angular_impulse() {
    let mut world = world();
    world
        .add_box(dynamic(1, Vec3i::ZERO, Vec3i::new(5, 5, 5)))
        .expect("add dynamic target");
    world
        .add_ballistic_sphere(
            BallisticSphere3d::new(
                BodyId(100),
                Vec3i::new(-30, 4, 0),
                Vec3i::new(3_600, 0, 0),
                2,
                1,
            )
            .expect("valid sphere"),
            true,
        )
        .expect("add analytic sphere");

    let report = world.step(1, 60).expect("analytic impact step");
    let target = world.box_by_id(BodyId(1)).expect("dynamic target");

    assert_eq!(report.stats.ballistic_impacts, 1);
    assert!(target.body().velocity().x > 0);
    assert_ne!(target.angular().angular_velocity.z, 0);
}

#[test]
fn elastic_analytic_sphere_remains_live_and_reverses_after_fixed_impact() {
    let mut world = world();
    let elastic = Material::new(MATERIAL_SCALE);
    world
        .add_box(fixed(1, Vec3i::ZERO, Vec3i::new(2, 20, 20), elastic))
        .expect("add elastic target");
    world
        .add_ballistic_sphere(
            BallisticSphere3d::new(
                BodyId(100),
                Vec3i::new(-30, 0, 0),
                Vec3i::new(3_600, 0, 0),
                2,
                1,
            )
            .expect("valid sphere")
            .with_material(elastic),
            false,
        )
        .expect("add elastic analytic sphere");

    let report = world.step(1, 60).expect("elastic sphere step");
    let sphere = world
        .ballistic_sphere_by_id(BodyId(100))
        .expect("non-retiring sphere remains live");

    assert_eq!(report.stats.ballistic_impacts, 1);
    assert_eq!(report.stats.ballistic_retired, 0);
    assert!(sphere.velocity().x < 0);
}
