use physics_engine::{
    AngularState3d, AngularVelocity3d, BallisticSphere3d, BodyId, InteractionCategory3d,
    Orientation3d, RigidBody, RigidBox3d, RotatingWorld3d, RotatingWorldConfig3d, Vec3i,
};

fn body(id: u64, x: i32, velocity: i32) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::dynamic(
            BodyId(id),
            Vec3i::new(x, 0, 0),
            Vec3i::new(velocity, 0, 0),
            Vec3i::new(1, 1, 1),
        ),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .unwrap()
}

#[test]
fn rigid_and_mixed_lanes_retire_only_after_transferring_the_impact() {
    for mixed in [false, true] {
        let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
            gravity: Vec3i::ZERO,
            ..Default::default()
        });
        world
            .add_box(body(1, -10, 120).with_impact_retirement(true))
            .unwrap();
        world.add_box(body(2, 0, 0)).unwrap();
        let category = InteractionCategory3d::new(17);
        world
            .set_body_interaction_category(BodyId(1), category)
            .unwrap();
        if mixed {
            world
                .add_ballistic_sphere(
                    BallisticSphere3d::new(
                        BodyId(3),
                        Vec3i::new(0, 1000, 0),
                        Vec3i::new(0, 120, 0),
                        1,
                        1,
                    )
                    .unwrap(),
                    true,
                )
                .unwrap();
        }
        let report = world.step(1, 10).unwrap();
        assert_eq!(report.stats.rigid_bodies_retired_on_impact, 1);
        assert!(world.box_by_id(BodyId(1)).is_none());
        assert!(world.box_by_id(BodyId(2)).unwrap().body().velocity().x > 0);
        assert!(report.changed_body_ids.contains(&BodyId(1)));
        assert_eq!(
            world.body_interaction_category(BodyId(1)),
            InteractionCategory3d::DEFAULT
        );
        world.step(1, 60).unwrap(); // No stale sleeper, contact, or body-index references.
        world.add_box(body(1, -100, 0)).unwrap();
        assert_eq!(
            world.body_interaction_category(BodyId(1)),
            InteractionCategory3d::DEFAULT
        );
    }
}

#[test]
fn ordinary_rigid_bodies_are_not_implicitly_retired() {
    let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
        gravity: Vec3i::ZERO,
        ..Default::default()
    });
    world.add_box(body(1, -10, 120)).unwrap();
    world.add_box(body(2, 0, 0)).unwrap();
    let report = world.step(1, 10).unwrap();
    assert_eq!(report.stats.rigid_bodies_retired_on_impact, 0);
    assert!(world.box_by_id(BodyId(1)).is_some());
}
