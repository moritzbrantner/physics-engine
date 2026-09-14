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

#[test]
fn trace_two_crate_resting_stack() {
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
    for (id, y) in [(100, 18), (101, 54)] {
        world
            .add_box(rotating_box(
                RigidBody::dynamic(
                    BodyId(id),
                    Vec3i::new(0, y, 0),
                    Vec3i::ZERO,
                    Vec3i::new(18, 18, 18),
                )
                .with_mass(2)
                .with_material(material),
            ))
            .expect("crate");
    }

    let mut trace = Vec::new();
    for tick in 1..=20 {
        world.step(1, 60).expect("step");
        let lower = world.box_by_id(BodyId(100)).expect("lower");
        let upper = world.box_by_id(BodyId(101)).expect("upper");
        trace.push((
            tick,
            lower.body().position(),
            upper.body().position(),
            lower.body().velocity(),
            upper.body().velocity(),
            world.is_sleeping(BodyId(100)),
            world.is_sleeping(BodyId(101)),
            lower.angular().angular_velocity,
            upper.angular().angular_velocity,
        ));
    }

    panic!("resting stack trace: {trace:#?}");
}
