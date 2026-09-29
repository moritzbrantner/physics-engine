use physics_engine::{
    AngularState3d, AngularVelocity3d, BallisticSphere3d, BodyId, CollisionLayers3d,
    InteractionPolicy3d, Material, Orientation3d, PhysicsWorld3dKernel, RigidBody, RigidBox3d,
    RotatingWorldConfig3d, Vec3i, WakePropagation3d,
};

fn body(id: u64, position: Vec3i, half: Vec3i) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::dynamic(BodyId(id), position, Vec3i::ZERO, half)
            .with_mass(2)
            .with_material(Material::new(1_000)),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .unwrap()
}

fn world() -> PhysicsWorld3dKernel {
    PhysicsWorld3dKernel::new(RotatingWorldConfig3d {
        gravity: Vec3i::ZERO,
        sample_count: 32,
        refinement_steps: 4,
        solver_passes: 8,
        max_events: 32,
    })
}

fn settle(world: &mut PhysicsWorld3dKernel) {
    for _ in 0..60 {
        world.step(1, 60).unwrap();
    }
    assert_eq!(
        world.sleeping_body_count(),
        world
            .boxes()
            .filter(|b| b.body().kind() == physics_engine::BodyKind::Dynamic)
            .count()
    );
}

fn sphere(id: u64, position: Vec3i, velocity: Vec3i) -> BallisticSphere3d {
    BallisticSphere3d::new(BodyId(id), position, velocity, 1, 1)
        .unwrap()
        .with_material(Material::new(1_000))
}

fn wall(id: u64, position: Vec3i, half: Vec3i) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::fixed(BodyId(id), position, half).with_material(Material::new(1_000)),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .unwrap()
}

#[test]
fn sphere_spawn_zero_step_and_distant_travel_preserve_parked_bodies() {
    let mut world = world();
    for id in 1..=32 {
        world
            .add_box(body(
                id,
                Vec3i::new(id as i32 * 4, 0, 0),
                Vec3i::new(1, 1, 1),
            ))
            .unwrap();
    }
    settle(&mut world);
    let before = world.boxes().cloned().collect::<Vec<_>>();
    world
        .add_ballistic_sphere(
            sphere(1000, Vec3i::new(0, 10, 0), Vec3i::new(100, 0, 0)),
            true,
        )
        .unwrap();
    assert_eq!(
        world.sleeping_body_count(),
        32,
        "creation cannot wake bodies"
    );
    world.step(0, 1).unwrap();
    assert_eq!(world.sleeping_body_count(), 32);
    for _ in 0..30 {
        let report = world.step(1, 60).unwrap();
        assert_eq!(world.sleeping_body_count(), 32);
        assert_eq!(world.boxes().cloned().collect::<Vec<_>>(), before);
        assert!(report.changed_body_ids.is_empty());
        assert_eq!(report.stats.parked_wake_retries, 0);
        assert_eq!(report.stats.parked_bodies_woken, 0);
        assert_eq!(report.stats.response_authority_body_count, 0);
        assert_eq!(report.stats.sampled_events, 0);
        assert_eq!(report.stats.stabilization_passes, 0);
    }
}

#[test]
fn overlapping_sweep_bounds_without_arrow_contact_do_not_wake() {
    let mut world = world();
    world
        .add_box(body(1, Vec3i::ZERO, Vec3i::new(18, 18, 18)))
        .unwrap();
    settle(&mut world);
    let before = world.box_by_id(BodyId(1)).unwrap().clone();
    // Actual arrow x range is 20..24 (a miss); its old rotational envelope reaches x=8.
    let arrow = RigidBox3d::new(
        RigidBody::dynamic(
            BodyId(1000),
            Vec3i::new(22, 0, 80),
            Vec3i::new(0, 0, -100),
            Vec3i::new(2, 2, 12),
        ),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .unwrap()
    .with_rotation_locked()
    .with_transient_contacts();
    world.add_box(arrow).unwrap();
    let report = world.step(1, 1).unwrap();
    assert!(world.is_sleeping(BodyId(1)));
    assert_eq!(world.box_by_id(BodyId(1)), Some(&before));
    assert_eq!(report.stats.parked_bodies_woken, 0);
    assert_eq!(report.stats.parked_wake_retries, 0);
    assert!(!report.changed_body_ids.contains(&BodyId(1)));
    world.remove_box(BodyId(1000)).unwrap();
    assert!(
        world.is_sleeping(BodyId(1)),
        "removing a miss must not wake its neighbors"
    );
}

#[test]
fn real_impact_wakes_target_and_matches_an_already_awake_target() {
    for ballistic in [false, true] {
        let mut world = world();
        world
            .add_box(body(1, Vec3i::ZERO, Vec3i::new(2, 2, 2)))
            .unwrap();
        world
            .add_box(body(2, Vec3i::new(0, 50, 0), Vec3i::new(2, 2, 2)))
            .unwrap();
        settle(&mut world);
        if ballistic {
            world
                .add_ballistic_sphere(
                    sphere(1000, Vec3i::new(-20, 0, 0), Vec3i::new(30, 0, 0)),
                    true,
                )
                .unwrap();
        } else {
            world
                .add_box(
                    RigidBox3d::new(
                        RigidBody::dynamic(
                            BodyId(1000),
                            Vec3i::new(-20, 0, 0),
                            Vec3i::new(30, 0, 0),
                            Vec3i::new(1, 1, 1),
                        ),
                        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
                    )
                    .unwrap()
                    .with_rotation_locked()
                    .with_transient_contacts(),
                )
                .unwrap();
        }
        let remote = world.box_by_id(BodyId(2)).unwrap().clone();
        let mut reference = world.clone();
        reference
            .set_linear_velocity(BodyId(1), Vec3i::ZERO)
            .unwrap();
        reference.step(1, 1).unwrap();
        let report = world.step(1, 1).unwrap();
        assert_eq!(report.stats.parked_bodies_woken, 1);
        assert_eq!(report.stats.parked_wake_retries, 1);
        assert!(world.is_sleeping(BodyId(2)));
        assert_eq!(world.box_by_id(BodyId(2)), Some(&remote));
        assert!(world.box_by_id(BodyId(1)).unwrap().body().velocity().x > 0);
        assert_eq!(
            world.boxes().cloned().collect::<Vec<_>>(),
            reference.boxes().cloned().collect::<Vec<_>>()
        );
        assert_eq!(
            world.ballistic_spheres().copied().collect::<Vec<_>>(),
            reference.ballistic_spheres().copied().collect::<Vec<_>>()
        );
    }
}

#[test]
fn same_step_ricochet_wakes_a_target_behind_the_original_launch() {
    for ballistic in [false, true] {
        let mut world = world();
        world
            .add_box(wall(10, Vec3i::new(20, 0, 0), Vec3i::new(1, 10, 10)))
            .unwrap();
        world
            .add_box(body(1, Vec3i::new(-10, 0, 0), Vec3i::new(1, 1, 1)))
            .unwrap();
        settle(&mut world);
        if ballistic {
            world
                .add_ballistic_sphere(sphere(1000, Vec3i::ZERO, Vec3i::new(60, 0, 0)), false)
                .unwrap();
        } else {
            world
                .add_box(
                    RigidBox3d::new(
                        RigidBody::dynamic(
                            BodyId(1000),
                            Vec3i::ZERO,
                            Vec3i::new(60, 0, 0),
                            Vec3i::new(1, 1, 1),
                        )
                        .with_material(Material::new(1000)),
                        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
                    )
                    .unwrap()
                    .with_rotation_locked()
                    .with_transient_contacts(),
                )
                .unwrap();
        }
        let mut reference = world.clone();
        reference
            .set_linear_velocity(BodyId(1), Vec3i::ZERO)
            .unwrap();
        reference.step(1, 1).unwrap();
        let report = world.step(1, 1).unwrap();
        assert_eq!(
            report.stats.parked_bodies_woken, 1,
            "ricochet must activate behind-origin target, ballistic={ballistic}"
        );
        assert!(world.box_by_id(BodyId(1)).unwrap().body().velocity().x < 0);
        assert_eq!(
            world.boxes().cloned().collect::<Vec<_>>(),
            reference.boxes().cloned().collect::<Vec<_>>()
        );
        assert_eq!(
            world.ballistic_spheres().copied().collect::<Vec<_>>(),
            reference.ballistic_spheres().copied().collect::<Vec<_>>(),
            "a rejected provisional impact must not partially advance the sphere"
        );
    }
}

#[test]
fn a_blocking_wall_does_not_wake_a_target_behind_it() {
    let mut world = world();
    world
        .add_box(wall(10, Vec3i::new(20, 0, 0), Vec3i::new(1, 10, 10)))
        .unwrap();
    world
        .add_box(body(1, Vec3i::new(30, 0, 0), Vec3i::new(1, 1, 1)))
        .unwrap();
    settle(&mut world);
    let before = world.box_by_id(BodyId(1)).unwrap().clone();
    world
        .add_ballistic_sphere(sphere(1000, Vec3i::ZERO, Vec3i::new(100, 0, 0)), true)
        .unwrap();
    let report = world.step(1, 1).unwrap();
    assert_eq!(world.ballistic_sphere_count(), 0);
    assert_eq!(report.stats.parked_bodies_woken, 0);
    assert!(world.is_sleeping(BodyId(1)));
    assert_eq!(world.box_by_id(BodyId(1)), Some(&before));
}

#[test]
fn collision_layers_and_disabled_wake_policy_remain_authoritative() {
    for disabled_layers in [false, true] {
        let mut world = world();
        let target = body(1, Vec3i::new(10, 0, 0), Vec3i::new(1, 1, 1))
            .with_collision_layers(CollisionLayers3d::new(1, 1));
        world.add_box(target).unwrap();
        settle(&mut world);
        if !disabled_layers {
            world.set_default_interaction_policy(
                InteractionPolicy3d::default().with_wake_propagation(WakePropagation3d::None),
            );
        }
        world
            .add_ballistic_sphere(
                sphere(1000, Vec3i::ZERO, Vec3i::new(20, 0, 0)).with_collision_layers(
                    if disabled_layers {
                        CollisionLayers3d::new(2, 2)
                    } else {
                        CollisionLayers3d::new(1, 1)
                    },
                ),
                true,
            )
            .unwrap();
        let report = world.step(1, 1).unwrap();
        assert!(world.is_sleeping(BodyId(1)));
        assert_eq!(report.stats.parked_bodies_woken, 0);
    }
}

#[test]
fn a_shared_fixed_floor_is_not_a_dynamic_island_connection() {
    let mut world = world();
    world
        .add_box(wall(10, Vec3i::new(0, -2, 0), Vec3i::new(100, 1, 100)))
        .unwrap();
    world
        .add_box(body(1, Vec3i::ZERO, Vec3i::new(1, 1, 1)))
        .unwrap();
    world
        .add_box(body(2, Vec3i::new(50, 0, 0), Vec3i::new(1, 1, 1)))
        .unwrap();
    settle(&mut world);
    world.remove_box(BodyId(1)).unwrap();
    assert!(
        world.is_sleeping(BodyId(2)),
        "removal cannot traverse the shared fixed floor"
    );
    world.set_default_interaction_policy(
        InteractionPolicy3d::default().with_wake_propagation(WakePropagation3d::None),
    );
    world.remove_box(BodyId(10)).unwrap();
    assert!(
        !world.is_sleeping(BodyId(2)),
        "removing the actual support must wake it"
    );
}

#[test]
fn independent_simultaneous_hits_retry_without_partially_committing_projectiles() {
    let mut world = world();
    for (id, y) in [(1, 0), (2, 20)] {
        world
            .add_box(body(id, Vec3i::new(10, y, 0), Vec3i::new(1, 1, 1)))
            .unwrap();
    }
    settle(&mut world);
    for (id, y) in [(1000, 0), (1001, 20)] {
        world
            .add_ballistic_sphere(sphere(id, Vec3i::new(0, y, 0), Vec3i::new(20, 0, 0)), true)
            .unwrap();
    }
    let mut reference = world.clone();
    for id in [1, 2] {
        reference
            .set_linear_velocity(BodyId(id), Vec3i::ZERO)
            .unwrap();
    }
    reference.step(1, 1).unwrap();
    let report = world.step(1, 1).unwrap();
    assert_eq!(report.stats.parked_bodies_woken, 2);
    assert_eq!(report.stats.parked_wake_retries, 2);
    assert!(
        report.stats.wake_probe_broad_phase_queries > 0,
        "discarded probes remain visible in work evidence"
    );
    assert_eq!(world.ballistic_sphere_count(), 0);
    assert_eq!(
        world.boxes().cloned().collect::<Vec<_>>(),
        reference.boxes().cloned().collect::<Vec<_>>()
    );
}
