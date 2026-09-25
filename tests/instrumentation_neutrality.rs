use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d,
    RotatingWorld3d, RotatingWorldConfig3d, RotatingWorldStepStats3d, Vec3i,
};

fn box3d(body: RigidBody) -> RigidBox3d {
    RigidBox3d::new(
        body,
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid deterministic test body")
}

fn world() -> RotatingWorld3d {
    let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
        gravity: Vec3i::ZERO,
        sample_count: 8,
        refinement_steps: 2,
        solver_passes: 4,
        max_events: 8,
    });
    world
        .add_box(box3d(RigidBody::dynamic(
            BodyId(1),
            Vec3i::ZERO,
            Vec3i::new(120, 0, 0),
            Vec3i::new(1, 1, 1),
        )))
        .expect("moving body");
    world
        .add_box(box3d(RigidBody::fixed(
            BodyId(2),
            Vec3i::new(3, 0, 0),
            Vec3i::new(1, 4, 4),
        )))
        .expect("fixed obstacle");
    world
}

#[test]
fn production_world_semantics_do_not_depend_on_performance_counters() {
    let mut world = world();
    let report = world.step(1, 60).expect("deterministic production step");

    let moving = world.box_by_id(BodyId(1)).expect("moving body remains");
    assert_eq!(moving.body().position(), Vec3i::new(1, 0, 0));
    assert_eq!(moving.body().velocity(), Vec3i::ZERO);
    assert_eq!(
        world
            .box_by_id(BodyId(2))
            .expect("fixed body remains")
            .body()
            .position(),
        Vec3i::new(3, 0, 0)
    );
    assert!(report.changed_body_ids.contains(&BodyId(1)));

    #[cfg(feature = "performance-counters")]
    {
        assert!(report.stats.body_count >= 2);
        assert!(report.stats.broad_phase_queries > 0);
        assert!(report.stats.sampled_events > 0);
    }

    #[cfg(not(feature = "performance-counters"))]
    assert_eq!(report.stats, RotatingWorldStepStats3d::default());
}
