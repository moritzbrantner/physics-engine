use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, Material, Orientation3d, RigidBody, RigidBox3d,
    RotatingWorld3d, RotatingWorldConfig3d, Vec3i,
};

const TICKS_PER_SECOND: i32 = 60;

fn rotating_box(body: RigidBody) -> RigidBox3d {
    RigidBox3d::new(
        body,
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid box")
}

#[test]
fn removing_an_awake_dynamic_support_wakes_dependent_sleeper_and_allows_fall() {
    let removed = BodyId(1);
    let remaining = BodyId(2);
    let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
        gravity: Vec3i::new(0, -3_600, 0),
        sample_count: 32,
        refinement_steps: 4,
        solver_passes: 8,
        max_events: 32,
    });
    world
        .add_box(rotating_box(RigidBody::fixed(
            BodyId(10),
            Vec3i::new(0, -16, 0),
            Vec3i::new(300, 16, 300),
        )))
        .expect("add floor");

    let material = Material::new(0).with_friction(1_000);
    for (id, y) in [(removed, 18), (remaining, 54)] {
        world
            .add_box(rotating_box(
                RigidBody::dynamic(id, Vec3i::new(0, y, 0), Vec3i::ZERO, Vec3i::new(18, 18, 18))
                    .with_mass(2)
                    .with_material(material),
            ))
            .expect("add dynamic body");
    }

    for _ in 0..240 {
        world
            .step(1, TICKS_PER_SECOND)
            .expect("settle supported stack");
    }
    assert!(world.is_sleeping(removed), "lower support never slept");
    assert!(world.is_sleeping(remaining), "upper body never slept");
    let before = world
        .box_by_id(remaining)
        .expect("upper body before removal")
        .body()
        .position();

    world
        .set_linear_velocity(removed, Vec3i::ZERO)
        .expect("wake lower support without moving it");
    assert!(!world.is_sleeping(removed));
    assert!(world.is_sleeping(remaining));

    world.remove_box(removed).expect("remove awake support");
    assert!(world.box_by_id(removed).is_none());
    assert!(world.box_by_id(remaining).is_some());
    assert!(
        !world.is_sleeping(remaining),
        "successful support removal must invalidate the remaining sleep/contact topology"
    );

    world
        .step(1, TICKS_PER_SECOND)
        .expect("gravity step after support removal");
    let falling = world
        .box_by_id(remaining)
        .expect("upper body after support removal")
        .body();
    assert!(
        falling.position().y < before.y || falling.velocity().y < 0,
        "woken dependent body must resume falling once its support is gone"
    );
}
