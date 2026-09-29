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
    assert!(report.stats.ballistic_target_bound_checks >= report.stats.ballistic_toi_tests);
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

#[test]
fn ballistic_impacts_preserve_external_motion_authority() {
    let mut world = world();
    let elastic = Material::new(MATERIAL_SCALE);
    let target = RigidBox3d::new(
        RigidBody::dynamic(BodyId(1), Vec3i::ZERO, Vec3i::ZERO, Vec3i::new(5, 5, 5))
            .with_material(elastic),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid target")
    .with_external_motion();
    world.add_box(target.clone()).expect("add external target");
    world
        .add_ballistic_sphere(
            BallisticSphere3d::new(
                BodyId(100),
                Vec3i::new(-30, 4, 0),
                Vec3i::new(3_600, 0, 0),
                2,
                1,
            )
            .expect("valid sphere")
            .with_material(elastic),
            false,
        )
        .expect("add sphere");

    let report = world.step(1, 60).expect("external target impact");

    assert_eq!(report.stats.ballistic_impacts, 1);
    assert_eq!(world.box_by_id(BodyId(1)), Some(&target));
    assert_eq!(
        world
            .ballistic_sphere_by_id(BodyId(100))
            .expect("live sphere")
            .velocity(),
        Vec3i::new(-3_600, 0, 0),
    );
}

fn bouncing_sphere_world(max_events: u16) -> RotatingWorld3d {
    let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
        max_events,
        ..world().config()
    });
    let elastic = Material::new(MATERIAL_SCALE);
    for (id, x) in [(1, -10), (2, 10)] {
        world
            .add_box(fixed(
                id,
                Vec3i::new(x, 0, 0),
                Vec3i::new(1, 10, 10),
                elastic,
            ))
            .expect("add wall");
    }
    world
        .add_ballistic_sphere(
            BallisticSphere3d::new(BodyId(100), Vec3i::ZERO, Vec3i::new(100, 0, 0), 1, 1)
                .expect("valid sphere")
                .with_material(elastic),
            false,
        )
        .expect("add sphere");
    world
}

#[test]
fn elastic_sphere_can_recontact_the_same_wall_within_one_step() {
    let mut world = bouncing_sphere_world(32);
    let report = world.step(1, 1).expect("repeated wall impacts");
    let sphere = world
        .ballistic_sphere_by_id(BodyId(100))
        .expect("live sphere");

    assert_eq!(report.stats.ballistic_impacts, 6);
    assert!(
        (-8..=8).contains(&sphere.position().x),
        "sphere tunneled: {sphere:?}"
    );
    assert_eq!(sphere.velocity(), Vec3i::new(100, 0, 0));
}

#[test]
fn repeated_ballistic_impacts_still_fail_closed_at_the_event_limit() {
    let mut world = bouncing_sphere_world(4);
    assert_eq!(
        world.step(1, 1),
        Err(physics_engine::RotatingWorldError3d::Repeated(
            physics_engine::RepeatedRotatingEventError3d::BallisticEventLimit(4)
        )),
    );
}

#[test]
fn fractional_response_mass_matches_closed_form_elastic_impact() {
    let mut world = world();
    let elastic = Material::new(MATERIAL_SCALE);
    let target = RigidBox3d::new(
        RigidBody::dynamic(BodyId(1), Vec3i::ZERO, Vec3i::ZERO, Vec3i::new(5, 5, 5))
            .with_mass(2)
            .with_material(elastic),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("dynamic target");
    world.add_box(target).expect("add target");
    world
        .add_ballistic_sphere(
            BallisticSphere3d::new(
                BodyId(100),
                Vec3i::new(-30, 0, 0),
                Vec3i::new(3000, 0, 0),
                2,
                1,
            )
            .expect("sphere")
            .with_response_mass(0.5)
            .expect("fractional mass")
            .with_material(elastic),
            false,
        )
        .expect("add sphere");
    let report = world.step(1, 60).expect("elastic collision");
    assert_eq!(report.stats.ballistic_impacts, 1);
    // Independent 1D elastic solution: v1=(m1-m2)/(m1+m2)*u1; v2=2*m1/(m1+m2)*u1.
    assert_eq!(
        world
            .ballistic_sphere_by_id(BodyId(100))
            .expect("sphere remains")
            .velocity(),
        Vec3i::new(-1800, 0, 0)
    );
    let target = world.box_by_id(BodyId(1)).expect("target remains");
    assert_eq!(target.body().velocity(), Vec3i::new(1200, 0, 0));
    assert_eq!(
        target.angular().angular_velocity,
        AngularVelocity3d::default()
    );
}
