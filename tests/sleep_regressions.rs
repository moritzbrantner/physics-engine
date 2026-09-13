use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d,
    RotatingWorld3d, RotatingWorldConfig3d, Vec3i,
};

const TICKS_PER_SECOND: i32 = 60;

fn world() -> RotatingWorld3d {
    RotatingWorld3d::new(RotatingWorldConfig3d {
        gravity: Vec3i::ZERO,
        sample_count: 8,
        refinement_steps: 2,
        solver_passes: 4,
        max_events: 8,
    })
}

fn dynamic(id: u64, position: Vec3i, velocity: Vec3i) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::dynamic(BodyId(id), position, velocity, Vec3i::new(1, 1, 1)),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid dynamic body")
}

fn fixed(id: u64, position: Vec3i, half_extents: Vec3i) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::fixed(BodyId(id), position, half_extents),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid fixed body")
}

#[test]
fn unconstrained_low_speed_body_preserves_inertia_instead_of_sleeping() {
    let id = BodyId(1);
    let mut world = world();
    world
        .add_box(dynamic(id.0, Vec3i::ZERO, Vec3i::new(60, 0, 0)))
        .expect("add coasting body");

    for _ in 0..24 {
        world.step(1, TICKS_PER_SECOND).expect("coasting step");
    }

    let body = world.box_by_id(id).expect("coasting body").body();
    assert_eq!(body.position(), Vec3i::new(24, 0, 0));
    assert_eq!(body.velocity(), Vec3i::new(60, 0, 0));
    assert!(!world.is_sleeping(id));
}

#[test]
fn frictionless_tangential_contact_does_not_turn_sleep_into_drag() {
    let id = BodyId(3);
    let mut world = world();
    world
        .add_box(fixed(90, Vec3i::ZERO, Vec3i::new(100, 1, 100)))
        .expect("add floor");
    world
        .add_box(dynamic(id.0, Vec3i::new(0, 2, 0), Vec3i::new(60, 0, 0)))
        .expect("add sliding body");

    for _ in 0..24 {
        world.step(1, TICKS_PER_SECOND).expect("sliding step");
    }

    let body = world.box_by_id(id).expect("sliding body").body();
    assert_eq!(body.position(), Vec3i::new(24, 2, 0));
    assert_eq!(body.velocity(), Vec3i::new(60, 0, 0));
    assert!(!world.is_sleeping(id));
}

#[test]
fn adding_fixed_geometry_wakes_a_sleeping_dynamic_before_resolution() {
    let id = BodyId(4);
    let mut world = world();
    world
        .add_box(dynamic(id.0, Vec3i::ZERO, Vec3i::ZERO))
        .expect("add sleeper");
    for _ in 0..24 {
        world.step(1, TICKS_PER_SECOND).expect("settle sleeper");
    }
    assert!(world.is_sleeping(id));

    world
        .add_box(fixed(91, Vec3i::new(1, 0, 0), Vec3i::new(1, 1, 1)))
        .expect("add overlapping fixed body");
    assert!(
        !world.is_sleeping(id),
        "new fixed geometry must invalidate the sleeper's old constraint state"
    );

    world.step(1, TICKS_PER_SECOND).expect("resolve new fixed contact");
    assert_ne!(
        world.box_by_id(id).expect("woken body").body().position(),
        Vec3i::ZERO,
        "woken body did not resolve the newly introduced overlap"
    );
}

#[test]
fn truly_stationary_unconstrained_body_may_sleep_without_changing_state() {
    let id = BodyId(2);
    let mut world = world();
    world
        .add_box(dynamic(id.0, Vec3i::new(4, 5, 6), Vec3i::ZERO))
        .expect("add stationary body");

    for _ in 0..24 {
        world.step(1, TICKS_PER_SECOND).expect("stationary step");
    }

    let body = world.box_by_id(id).expect("stationary body").body();
    assert_eq!(body.position(), Vec3i::new(4, 5, 6));
    assert_eq!(body.velocity(), Vec3i::ZERO);
    assert!(world.is_sleeping(id));
}
