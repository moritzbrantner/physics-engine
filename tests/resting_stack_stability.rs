use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, Material, Orientation3d, RigidBody, RigidBox3d,
    RotatingWorld3d, RotatingWorldConfig3d, Vec3i,
};

fn rotating_box(body: RigidBody) -> RigidBox3d {
    RigidBox3d::new(
        body,
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid box")
}

fn assert_stable_stack(lower_id: BodyId, upper_id: BodyId) {
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
        .expect("floor");

    let material = Material::new(0).with_friction(1_000);
    for (id, y) in [(lower_id, 18), (upper_id, 54)] {
        world
            .add_box(rotating_box(
                RigidBody::dynamic(
                    id,
                    Vec3i::new(0, y, 0),
                    Vec3i::ZERO,
                    Vec3i::new(18, 18, 18),
                )
                .with_mass(2)
                .with_material(material),
            ))
            .expect("crate");
    }

    for _ in 0..240 {
        world.step(1, 60).expect("settle stack");
    }

    let lower = world.box_by_id(lower_id).expect("lower crate");
    let upper = world.box_by_id(upper_id).expect("upper crate");
    assert_eq!(lower.body().position(), Vec3i::new(0, 18, 0));
    assert_eq!(upper.body().position(), Vec3i::new(0, 54, 0));
    assert_eq!(lower.body().velocity(), Vec3i::ZERO);
    assert_eq!(upper.body().velocity(), Vec3i::ZERO);
    assert_eq!(lower.angular().angular_velocity, AngularVelocity3d::default());
    assert_eq!(upper.angular().angular_velocity, AngularVelocity3d::default());
    assert!(world.is_sleeping(lower_id));
    assert!(world.is_sleeping(upper_id));

    let settled_lower = lower.clone();
    let settled_upper = upper.clone();
    for _ in 0..60 {
        world.step(1, 60).expect("stable sleeping step");
    }
    assert_eq!(world.box_by_id(lower_id), Some(&settled_lower));
    assert_eq!(world.box_by_id(upper_id), Some(&settled_upper));
}

#[test]
fn supported_resting_stack_settles_without_penetration_or_wake_churn() {
    assert_stable_stack(BodyId(100), BodyId(101));
    assert_stable_stack(BodyId(201), BodyId(200));
}
