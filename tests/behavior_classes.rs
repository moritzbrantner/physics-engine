use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, InteractionCategory3d, InteractionPolicy3d,
    Material, Orientation3d, RigidBody, RigidBox3d, RigidBoxFreeFlightConfig3d, RotatingWorld3d,
    RotatingWorldConfig3d, SolverParticipation3d, Vec3i, WakePropagation3d,
    resolve_obb_contact, rotational_sweep_candidate_pairs,
};

fn angular() -> AngularState3d {
    AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default())
}

fn dynamic(id: u64, position: Vec3i, velocity: Vec3i, half: Vec3i) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::dynamic(BodyId(id), position, velocity, half)
            .with_material(Material::new(0)),
        angular(),
    )
    .expect("valid dynamic box")
}

fn fixed(id: u64, position: Vec3i, half: Vec3i) -> RigidBox3d {
    RigidBox3d::new(RigidBody::fixed(BodyId(id), position, half), angular())
        .expect("valid fixed box")
}

fn world() -> RotatingWorld3d {
    RotatingWorld3d::new(RotatingWorldConfig3d {
        gravity: Vec3i::ZERO,
        sample_count: 8,
        refinement_steps: 2,
        solver_passes: 4,
        max_events: 8,
    })
}

#[test]
fn overlap_only_sensor_is_queryable_but_never_becomes_a_solver_candidate() {
    let sensor_id = BodyId(10);
    let solid_id = BodyId(11);
    let sensor = fixed(10, Vec3i::ZERO, Vec3i::new(3, 3, 3)).with_overlap_only();
    let solid = dynamic(11, Vec3i::ZERO, Vec3i::ZERO, Vec3i::new(1, 1, 1));
    let query = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1);

    assert_eq!(
        sensor.solver_participation(),
        SolverParticipation3d::OverlapOnly
    );
    assert!(
        rotational_sweep_candidate_pairs(&[sensor.clone(), solid.clone()], query)
            .expect("valid broad-phase query")
            .is_empty()
    );

    let mut world = world();
    world.add_box(sensor).expect("add sensor");
    world.add_box(solid).expect("add solid");

    assert_eq!(
        world.body_overlaps(sensor_id).expect("sensor overlap query"),
        vec![solid_id]
    );
    assert!(
        world
            .body_contacts(sensor_id)
            .expect("sensor rigid-contact query")
            .is_empty()
    );
}

#[test]
fn external_motion_is_one_sided_contact_authority() {
    let kinematic = dynamic(
        1,
        Vec3i::ZERO,
        Vec3i::new(20, 0, 0),
        Vec3i::new(2, 2, 2),
    )
    .with_external_motion();
    let dynamic = dynamic(
        2,
        Vec3i::new(3, 0, 0),
        Vec3i::ZERO,
        Vec3i::new(2, 2, 2),
    );
    let before = kinematic.clone();

    let response = resolve_obb_contact(kinematic, dynamic, false).expect("valid contact response");

    assert_eq!(response.left, before, "solver must not mutate external authority");
    assert!(response.contact.is_some());
    assert_ne!(
        response.right.body().velocity(),
        Vec3i::ZERO,
        "dynamic target should receive the one-sided response"
    );
}

#[test]
fn aggressive_debris_sleep_settles_before_normal_sleep() {
    let normal_id = BodyId(20);
    let debris_id = BodyId(21);
    let mut normal_world = world();
    let mut debris_world = world();
    normal_world
        .add_box(dynamic(
            normal_id.0,
            Vec3i::ZERO,
            Vec3i::ZERO,
            Vec3i::new(1, 1, 1),
        ))
        .expect("add normal body");
    debris_world
        .add_box(
            dynamic(
                debris_id.0,
                Vec3i::ZERO,
                Vec3i::ZERO,
                Vec3i::new(1, 1, 1),
            )
            .with_aggressive_sleep(),
        )
        .expect("add debris body");

    for _ in 0..4 {
        normal_world.step(1, 60).expect("step normal body");
        debris_world.step(1, 60).expect("step debris body");
    }

    assert!(!normal_world.is_sleeping(normal_id));
    assert!(debris_world.is_sleeping(debris_id));
}

#[test]
fn debris_pair_can_decline_wake_propagation_without_disabling_collision() {
    let mover_id = BodyId(30);
    let sleeper_id = BodyId(31);
    let mover_category = InteractionCategory3d::new(10);
    let debris_category = InteractionCategory3d::new(11);
    let mut world = world();

    world
        .add_box(
            dynamic(
                sleeper_id.0,
                Vec3i::ZERO,
                Vec3i::ZERO,
                Vec3i::new(1, 1, 1),
            )
            .with_aggressive_sleep(),
        )
        .expect("add debris sleeper");
    world
        .set_body_interaction_category(sleeper_id, debris_category)
        .expect("categorize debris");

    for _ in 0..4 {
        world.step(1, 60).expect("settle debris");
    }
    assert!(world.is_sleeping(sleeper_id));

    world
        .add_box(dynamic(
            mover_id.0,
            Vec3i::new(-4, 0, 0),
            Vec3i::new(240, 0, 0),
            Vec3i::new(1, 1, 1),
        ))
        .expect("add mover");
    world
        .set_body_interaction_category(mover_id, mover_category)
        .expect("categorize mover");
    world.set_directional_interaction_policy(
        mover_category,
        debris_category,
        InteractionPolicy3d::default().with_wake_propagation(WakePropagation3d::None),
    );

    world.step(1, 60).expect("step passive debris interaction");

    assert!(
        world.is_sleeping(sleeper_id),
        "no-wake pair policy should keep the parked target passive"
    );
    assert_ne!(
        world
            .box_by_id(mover_id)
            .expect("mover remains in world")
            .body()
            .velocity(),
        Vec3i::new(240, 0, 0),
        "parked collision geometry must still respond to the mover"
    );
}
