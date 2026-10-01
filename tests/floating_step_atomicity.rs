use physics_engine::{
    BodyId,
    approximate::{Body, Config, Error, Shape, Vector, World},
};
use physics_engine::{approximate::Real, numeric::Scalar};

fn bodies(world: &World) -> Vec<Body> {
    world.bodies().cloned().collect()
}

fn failing_body(id: u64, position: Real, speed: Real) -> Body {
    let mut body = Body::new(
        BodyId(id),
        Shape::Sphere(0.1),
        Vector(position, 0.0, 0.0),
        1.0,
    );
    body.velocity = Vector(speed, 0.0, 0.0);
    body
}

#[test]
fn failed_step_preserves_physical_state_and_pending_input() {
    let mut world = World::new(Config {
        gravity: Vector::ZERO,
        substeps: 1,
        ..Config::default()
    })
    .unwrap();
    let id = BodyId(1);
    let position = Vector(1e12 - 1.0, 0.0, 0.0);
    let mut body = Body::new(id, Shape::Sphere(1.0), position, 1.0);
    body.velocity = Vector(20.0, 0.0, 0.0);
    world.add_body(body).unwrap();
    world.add_force(id, Vector(1.0, 0.0, 0.0)).unwrap();
    world
        .apply_impulse(id, Vector(1.0, 0.0, 0.0), position)
        .unwrap();
    let before = world.body(id).unwrap().clone();
    assert!(matches!(
        world.step(0.1),
        Err(Error::NonFiniteState(BodyId(1)))
    ));
    assert_eq!(world.body(id), Some(&before));
    assert_eq!(world.elapsed_seconds(), 0.0);
    assert!(world.last_step_transaction().rolled_back);
}

#[test]
fn repeated_failures_restore_earlier_bodies_and_do_not_consume_input_twice() {
    let mut world = World::new(Config {
        gravity: Vector::ZERO,
        substeps: 4,
        ..Config::default()
    })
    .unwrap();
    world
        .add_body(Body::new(BodyId(1), Shape::Sphere(0.1), Vector::ZERO, 1.0))
        .unwrap();
    world
        .apply_impulse(BodyId(1), Vector(2.0, 0.0, 0.0), Vector::ZERO)
        .unwrap();
    world.add_force(BodyId(1), Vector(3.0, 0.0, 0.0)).unwrap();
    world.add_body(failing_body(99, 1e12 - 0.03, 1.0)).unwrap();
    let mut reference = world.clone();
    let before = bodies(&world);
    let report = format!("{:?}", world.last_report);
    for _ in 0..3 {
        assert!(matches!(
            world.step(0.1),
            Err(Error::NonFiniteState(BodyId(99)))
        ));
        assert_eq!(bodies(&world), before);
        assert_eq!(format!("{:?}", world.last_report), report);
        assert_eq!(world.elapsed_seconds(), 0.0);
    }
    world.remove_body(BodyId(99)).unwrap();
    reference.remove_body(BodyId(99)).unwrap();
    for _ in 0..20 {
        world.step(0.01).unwrap();
        reference.step(0.01).unwrap();
        assert_eq!(bodies(&world), bodies(&reference));
        assert_eq!(world.elapsed_seconds(), reference.elapsed_seconds());
    }
}

#[test]
fn position_correction_failure_restores_pose_and_new_contact_history() {
    let fixture = |iterations| {
        let mut world = World::new(Config {
            gravity: Vector::ZERO,
            substeps: 1,
            fixed_position_iterations: iterations,
            ..Config::default()
        })
        .unwrap();
        world
            .add_body(Body::new(
                BodyId(0),
                Shape::Box(Vector(10.0, 20.0, 20.0)),
                Vector(1e12 - 8.0, 0.0, 0.0),
                0.0,
            ))
            .unwrap();
        world
            .add_body(Body::new(
                BodyId(1),
                Shape::Box(Vector(2.0, 10.0, 10.0)),
                Vector(1e12 - 1.0, 0.0, 0.0),
                1.0,
            ))
            .unwrap();
        world
    };
    let mut control = fixture(0);
    control.step(0.01).unwrap();
    let mut world = fixture(1);
    let before = bodies(&world);
    assert!(matches!(
        world.step(0.01),
        Err(Error::NonFiniteState(BodyId(1)))
    ));
    assert_eq!(bodies(&world), before);
    assert_eq!(world.elapsed_seconds(), 0.0);
    assert!(world.last_step_transaction().journaled_pairs > 0);
}

#[test]
fn retirement_before_a_later_substep_error_restores_membership_and_identity() {
    let mut world = World::new(Config {
        gravity: Vector::ZERO,
        substeps: 2,
        ..Config::default()
    })
    .unwrap();
    world
        .add_body(Body::new(
            BodyId(0),
            Shape::Box(Vector(0.1, 5.0, 5.0)),
            Vector::ZERO,
            0.0,
        ))
        .unwrap();
    let mut projectile = Body::new(BodyId(1), Shape::Sphere(0.1), Vector(-1.0, 0.0, 0.0), 1.0);
    projectile.velocity = Vector(40.0, 0.0, 0.0);
    projectile.ccd = true;
    projectile.retire_on_impact = true;
    world.add_body(projectile).unwrap();
    world.add_body(failing_body(99, 1e12 - 0.075, 1.0)).unwrap();
    let mut reference = world.clone();
    reference.remove_body(BodyId(99)).unwrap();
    assert_eq!(reference.step(0.1).unwrap().retired, [BodyId(1)]);
    let before = bodies(&world);
    assert!(matches!(
        world.step(0.1),
        Err(Error::NonFiniteState(BodyId(99)))
    ));
    assert_eq!(bodies(&world), before);
    assert_eq!(world.last_step_transaction().retired_bodies, 1);
    assert_eq!(
        world.last_failed_step_work().unwrap().retirement_attempts,
        1
    );
    world.remove_body(BodyId(99)).unwrap();
    assert_eq!(world.step(0.1).unwrap().retired, [BodyId(1)]);
    assert_eq!(bodies(&world), bodies(&reference));
}

fn sleeping_world(count: u64) -> World {
    let mut world = World::new(Config {
        gravity: Vector(0.0, -10.0, 0.0),
        substeps: 2,
        sleep_speed: 0.1,
        sleep_seconds: 0.05,
        ..Config::default()
    })
    .unwrap();
    world
        .add_body(Body::new(
            BodyId(0),
            Shape::Box(Vector(500.0, 0.1, 10.0)),
            Vector(0.0, -0.1, 0.0),
            0.0,
        ))
        .unwrap();
    for index in 0..count {
        world
            .add_body(Body::new(
                BodyId(index + 1),
                Shape::Box(Vector(0.5, 0.5, 0.5)),
                Vector(index as Real * 3.0, 0.5, 0.0),
                1.0,
            ))
            .unwrap();
    }
    for _ in 0..100 {
        world.step(0.01).unwrap();
    }
    assert!(
        world
            .bodies()
            .filter(|b| b.mass > 0.0)
            .all(Body::is_sleeping)
    );
    world
}

#[test]
fn failed_wake_restores_sleep_timers_warm_starts_and_changed_timestep_continuation() {
    let mut world = sleeping_world(1);
    assert!(world.has_support(BodyId(1)));
    let mut projectile = Body::new(BodyId(2), Shape::Sphere(0.2), Vector(-2.0, 0.5, 0.0), 1.0);
    projectile.velocity = Vector(20.0, 0.0, 0.0);
    projectile.ccd = true;
    world.add_body(projectile).unwrap();
    world.add_body(failing_body(99, 1e12 - 0.075, 1.0)).unwrap();
    let mut reference = world.clone();
    let mut control = world.clone();
    control.remove_body(BodyId(99)).unwrap();
    assert!(control.step(0.1).unwrap().woken_bodies > 0);
    let before = bodies(&world);
    let elapsed = world.elapsed_seconds();
    let prior_report = format!("{:?}", world.last_report);
    assert!(matches!(
        world.step(0.1),
        Err(Error::NonFiniteState(BodyId(99)))
    ));
    assert_eq!(bodies(&world), before);
    assert_eq!(world.elapsed_seconds(), elapsed);
    assert_eq!(format!("{:?}", world.last_report), prior_report);
    assert!(world.last_step_transaction().copied_contact_points > 0);
    let work = world.last_failed_step_work().unwrap();
    assert!(work.woken_bodies > 0);
    assert!(work.geometry.sweep_queries > 0);
    assert!(work.constraint_visits > 0);
    assert!(work.transaction.rolled_back);
    world.remove_body(BodyId(99)).unwrap();
    reference.remove_body(BodyId(99)).unwrap();
    for tick in 0..100 {
        let dt = if tick % 2 == 0 { 0.02 } else { 0.01 };
        let actual = world.step(dt as Scalar).unwrap();
        let expected = reference.step(dt as Scalar).unwrap();
        assert_eq!(bodies(&world), bodies(&reference), "tick {tick}");
        assert_eq!(actual.retired, expected.retired);
        assert_eq!(actual.woken_bodies, expected.woken_bodies);
        assert_eq!(world.elapsed_seconds(), reference.elapsed_seconds());
    }
}

#[test]
fn local_step_does_not_journal_unrelated_sleepers() {
    let mut world = sleeping_world(128);
    let before = bodies(&world);
    world
        .set_velocity(BodyId(1), Vector(0.2, 0.0, 0.0))
        .unwrap();
    world.step(0.01).unwrap();
    let work = world.last_step_transaction();
    assert_eq!(work.journaled_bodies, 2); // one responding box and the floor
    assert_eq!(work.journaled_pairs, 1);
    assert!(!work.rolled_back);
    for body in before.iter().filter(|b| b.id.0 > 1) {
        assert_eq!(world.body(body.id), Some(body));
    }
}

#[test]
fn quiet_and_invalid_attempts_do_no_journaling_and_scratch_release_preserves_continuation() {
    let mut world = sleeping_world(8);
    let mut reference = world.clone();
    let before = bodies(&world);
    let time = world.elapsed_seconds();
    world.step(0.03).unwrap();
    reference.step(0.03).unwrap();
    let idle = world.last_step_transaction();
    assert_eq!(idle.journaled_bodies, 0);
    assert_eq!(idle.journaled_pairs, 0);
    assert_eq!(idle.vector_growths, 0);
    assert!(idle.vector_capacity_bytes > 0);
    assert_eq!(bodies(&world), before);
    assert_eq!(world.elapsed_seconds(), time + 0.03);
    let report = format!("{:?}", world.last_report);
    for dt in [f64::NAN, -0.1, 0.1001, f64::from_bits(1)] {
        assert!(matches!(world.step(dt as Scalar), Err(Error::InvalidInput)));
        assert_eq!(bodies(&world), before);
        assert_eq!(format!("{:?}", world.last_report), report);
        assert_eq!(world.last_step_transaction().journaled_bodies, 0);
    }
    world.release_transaction_scratch();
    assert_eq!(world.last_step_transaction().vector_capacity_bytes, 0);
    assert!(world.has_support(BodyId(1)));
    for step in 0..32 {
        if step == 0 || step == 16 {
            let speed = if step == 0 { 0.2 } else { -0.2 };
            world
                .set_velocity(BodyId(1), Vector(speed, 0.0, 0.0))
                .unwrap();
            reference
                .set_velocity(BodyId(1), Vector(speed, 0.0, 0.0))
                .unwrap();
        }
        world.step(0.01).unwrap();
        reference.step(0.01).unwrap();
        assert_eq!(bodies(&world), bodies(&reference));
        assert_eq!(world.elapsed_seconds(), reference.elapsed_seconds());
    }
}

#[cfg(feature = "experimental-soft-contact")]
#[test]
fn experimental_relaxation_history_is_also_rolled_back() {
    use physics_engine::approximate::SoftContact;
    let mut world = World::new(Config {
        gravity: Vector(0.0, -10.0, 0.0),
        substeps: 2,
        soft_contact: Some(SoftContact::default()),
        ..Config::default()
    })
    .unwrap();
    for (id, y, mass) in [(0, -0.5, 0.0), (1, 0.5, 1.0), (2, 1.45, 3.0)] {
        world
            .add_body(Body::new(
                BodyId(id),
                Shape::Box(Vector(0.5, 0.5, 0.5)),
                Vector(0.0, y, 0.0),
                mass,
            ))
            .unwrap();
    }
    world.add_body(failing_body(99, 1e12 - 0.075, 1.0)).unwrap();
    let mut reference = world.clone();
    let mut control = world.clone();
    control.remove_body(BodyId(99)).unwrap();
    assert!(control.step(0.1).unwrap().correction.softened_points > 0);
    let before = bodies(&world);
    assert!(world.step(0.1).is_err());
    assert_eq!(bodies(&world), before);
    world.remove_body(BodyId(99)).unwrap();
    reference.remove_body(BodyId(99)).unwrap();
    for _ in 0..32 {
        world.step(0.01).unwrap();
        reference.step(0.01).unwrap();
        assert_eq!(bodies(&world), bodies(&reference));
    }
}
