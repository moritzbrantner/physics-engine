use physics_engine::approximate::Real;
use physics_engine::{
    BodyId,
    approximate::{Body, CheckpointContext, Config, Error, Shape, SweepFailure, Vector, World},
};

#[test]
fn computed_nonfinite_sweep_input_returns_an_atomic_error_instead_of_panicking() {
    let mut world = World::new(Config {
        gravity: Vector::ZERO,
        substeps: 1,
        ..Config::default()
    })
    .unwrap();
    world
        .add_body(Body::new(BodyId(0), Shape::Sphere(1.0), Vector::ZERO, 0.0))
        .unwrap();
    world
        .add_body(Body::new(
            BodyId(1),
            Shape::capsule(0.0, 1.0),
            Vector(-4.0, 0.0, 0.0),
            0.1,
        ))
        .unwrap();
    world
        .add_body(Body::new(
            BodyId(2),
            Shape::Sphere(0.1),
            Vector(20.0, 0.0, 0.0),
            1.0,
        ))
        .unwrap();
    world.step(0.01).unwrap();
    world
        .add_force(BodyId(1), Vector(Real::MAX, 0.0, 0.0))
        .unwrap();
    world
        .apply_impulse(BodyId(2), Vector(1.0, 0.0, 0.0), Vector(20.0, 0.0, 0.0))
        .unwrap();
    let context = CheckpointContext {
        build: [17; 32],
        content: [29; 32],
    };
    let before = world.checkpoint(context).unwrap().to_bytes();
    let bodies = world.bodies().cloned().collect::<Vec<_>>();
    let prior_report = format!("{:?}", world.last_report);
    let mut control = world.clone();
    for _ in 0..3 {
        assert_eq!(
            world.step(0.1).unwrap_err(),
            Error::CollisionSearchFailed {
                bodies: [BodyId(0), BodyId(1)],
                reason: SweepFailure::InvalidGeometryInput,
            }
        );
        assert_eq!(world.checkpoint(context).unwrap().to_bytes(), before);
        assert_eq!(world.bodies().cloned().collect::<Vec<_>>(), bodies);
        assert_eq!(world.elapsed_seconds(), 0.01);
        assert_eq!(format!("{:?}", world.last_report), prior_report);
        let work = world.last_failed_step_work().unwrap();
        assert_eq!(work.attempted_substeps, 1);
        assert_eq!(work.geometry.sweep_queries, 1);
        assert_eq!(work.geometry.primitive_sweep_failures, 1);
        assert_eq!(work.geometry.primitive_sweep_iterations, 0);
        assert_eq!(work.integration_attempts, 0);
        assert_eq!(work.retirement_attempts, 0);
        assert_eq!(work.transaction.journaled_bodies, 2);
        assert!(work.transaction.rolled_back);
    }
    // Cancel the unsupported force through the public command API. The unrelated pending
    // impulse must still apply once, exactly as in a world that never attempted the failed step.
    for world in [&mut world, &mut control] {
        world
            .add_force(BodyId(1), Vector(-Real::MAX, 0.0, 0.0))
            .unwrap();
    }
    for _ in 0..16 {
        world.step(0.01).unwrap();
        control.step(0.01).unwrap();
        assert_eq!(
            world.checkpoint(context).unwrap().to_bytes(),
            control.checkpoint(context).unwrap().to_bytes()
        );
        assert_eq!(
            world.bodies().cloned().collect::<Vec<_>>(),
            control.bodies().cloned().collect::<Vec<_>>()
        );
    }
}

#[test]
fn finite_velocities_with_overflowing_relative_motion_fail_without_admitting_contact() {
    let mut world = World::new(Config {
        gravity: Vector::ZERO,
        substeps: 1,
        ..Config::default()
    })
    .unwrap();
    for (id, shape, x) in [
        (1, Shape::capsule(0.0, 1.0), -4.0),
        (2, Shape::Sphere(1.0), 4.0),
    ] {
        let position = Vector(x, 0.0, 0.0);
        world
            .add_body(Body::new(BodyId(id), shape, position, 1.0))
            .unwrap();
        world
            .apply_impulse(
                BodyId(id),
                Vector(if id == 1 { Real::MAX } else { -Real::MAX }, 0.0, 0.0),
                position,
            )
            .unwrap();
    }
    let context = CheckpointContext {
        build: [17; 32],
        content: [29; 32],
    };
    let before = world.checkpoint(context).unwrap().to_bytes();
    for _ in 0..3 {
        assert_eq!(
            world.step(0.1).unwrap_err(),
            Error::CollisionSearchFailed {
                bodies: [BodyId(1), BodyId(2)],
                reason: SweepFailure::NonFiniteComputation,
            }
        );
        assert_eq!(world.checkpoint(context).unwrap().to_bytes(), before);
        let work = world.last_failed_step_work().unwrap();
        assert_eq!(work.geometry.primitive_sweep_failures, 1);
        assert_eq!(work.geometry.primitive_sweep_iterations, 0);
        assert_eq!(work.contact_points, 0);
        assert_eq!(work.integration_attempts, 0);
        assert_eq!(work.woken_bodies, 0);
        assert!(work.transaction.rolled_back);
    }
}
