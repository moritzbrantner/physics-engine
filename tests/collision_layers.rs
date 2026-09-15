use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, CollisionLayers3d, Orientation3d, RigidBody,
    RigidBox3d, RigidBoxFreeFlightConfig3d, RotatingWorld3d, RotatingWorldConfig3d, Vec3i,
    body_has_support, rotational_sweep_candidate_pairs,
};

fn dynamic(id: u64, position: Vec3i, layers: CollisionLayers3d) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::dynamic(BodyId(id), position, Vec3i::ZERO, Vec3i::new(2, 2, 2)),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid dynamic box")
    .with_collision_layers(layers)
}

fn fixed(id: u64, position: Vec3i, layers: CollisionLayers3d) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::fixed(BodyId(id), position, Vec3i::new(2, 2, 2)),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid fixed box")
    .with_collision_layers(layers)
}

#[test]
fn broad_phase_excludes_pairs_rejected_by_symmetric_collision_layers() {
    let character = CollisionLayers3d::new(0b0010, 0b0100);
    let crate_layers = CollisionLayers3d::new(0b0100, 0b0010);
    let projectile = CollisionLayers3d::new(0b1000, 0b1000);
    let boxes = [
        dynamic(1, Vec3i::ZERO, character),
        dynamic(2, Vec3i::new(1, 0, 0), crate_layers),
        dynamic(3, Vec3i::new(-1, 0, 0), projectile),
    ];

    let pairs = rotational_sweep_candidate_pairs(
        &boxes,
        RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 0, 1),
    )
    .expect("valid broad phase");

    assert_eq!(pairs.len(), 1);
    assert_eq!(pairs[0].left, BodyId(1));
    assert_eq!(pairs[0].right, BodyId(2));
}

#[test]
fn support_queries_do_not_reintroduce_disabled_pairs_from_the_contact_cache() {
    let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
        gravity: Vec3i::new(0, -60, 0),
        sample_count: 16,
        refinement_steps: 2,
        solver_passes: 4,
        max_events: 8,
    });
    let character = CollisionLayers3d::new(0b0010, 0b0100);
    let world_only = CollisionLayers3d::new(0b0001, 0b0001);
    world
        .add_box(dynamic(1, Vec3i::new(0, 4, 0), character))
        .expect("character");
    world
        .add_box(fixed(2, Vec3i::ZERO, world_only))
        .expect("floor");

    assert!(!body_has_support(&world, BodyId(1), Vec3i::new(0, -60, 0)).expect("support query"));
}

#[test]
fn default_layers_preserve_historical_collision_eligibility() {
    let boxes = [
        dynamic(1, Vec3i::ZERO, CollisionLayers3d::ALL),
        dynamic(2, Vec3i::new(1, 0, 0), CollisionLayers3d::default()),
    ];
    let pairs = rotational_sweep_candidate_pairs(
        &boxes,
        RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 0, 1),
    )
    .expect("valid broad phase");
    assert_eq!(pairs.len(), 1);
}

#[test]
fn fixed_boundary_stabilization_honors_either_layers_rejecting_the_pair() {
    let cases = [
        (
            CollisionLayers3d::new(0b0001, 0b0010),
            CollisionLayers3d::new(0b0010, 0b0100),
            false,
        ),
        (
            CollisionLayers3d::new(0b0001, 0b0100),
            CollisionLayers3d::new(0b0010, 0b0001),
            true,
        ),
    ];

    for (fixed_layers, dynamic_layers, add_dynamic_first) in cases {
        let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
            gravity: Vec3i::ZERO,
            ..RotatingWorldConfig3d::default()
        });
        let fixed_boundary = fixed(2, Vec3i::ZERO, fixed_layers);
        let dynamic_body = dynamic(1, Vec3i::new(1, 0, 0), dynamic_layers);
        if add_dynamic_first {
            world
                .add_box(dynamic_body)
                .expect("overlapping dynamic body");
            world.add_box(fixed_boundary).expect("fixed boundary");
        } else {
            world.add_box(fixed_boundary).expect("fixed boundary");
            world
                .add_box(dynamic_body)
                .expect("overlapping dynamic body");
        }

        let before = world
            .box_by_id(BodyId(1))
            .expect("dynamic body before step")
            .body()
            .position();
        world.step(1, 60).expect("disabled pair steps safely");
        let after = world
            .box_by_id(BodyId(1))
            .expect("dynamic body after step")
            .body()
            .position();

        assert_eq!(after, before);
    }
}
