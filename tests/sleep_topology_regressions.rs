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

fn dynamic(id: u64, position: Vec3i) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::dynamic(BodyId(id), position, Vec3i::ZERO, Vec3i::new(1, 1, 1)),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid dynamic body")
}

#[test]
fn removing_an_awake_dynamic_contact_wakes_remaining_sleepers() {
    let removed = BodyId(1);
    let remaining = BodyId(2);
    let mut world = world();
    world
        .add_box(dynamic(removed.0, Vec3i::ZERO))
        .expect("add lower body");
    world
        .add_box(dynamic(remaining.0, Vec3i::new(0, 2, 0)))
        .expect("add touching upper body");

    for _ in 0..24 {
        world.step(1, TICKS_PER_SECOND).expect("settle pair");
    }
    assert!(world.is_sleeping(removed), "lower body never slept");
    assert!(world.is_sleeping(remaining), "upper body never slept");

    world
        .set_linear_velocity(removed, Vec3i::ZERO)
        .expect("wake one contact participant without moving it");
    assert!(!world.is_sleeping(removed));
    assert!(world.is_sleeping(remaining));

    world.remove_box(removed).expect("remove awake body");
    assert!(
        !world.is_sleeping(remaining),
        "any membership removal must invalidate the remaining sleep/contact topology"
    );
}
