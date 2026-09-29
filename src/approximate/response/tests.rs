use super::*;
use crate::BodyId;
use crate::approximate::{Config, Quaternion, Shape, World};

fn vector_bits(v: Vector) -> [u64; 3] {
    [v.0.to_bits(), v.1.to_bits(), v.2.to_bits()]
}

#[test]
fn prepared_response_is_bit_identical_to_original_formula() {
    for shape in [Shape::Sphere(0.7), Shape::Box(Vector(0.3, 2.5, 7.0))] {
        for mass in [0.0, 0.01, 2.0, 10_000.0] {
            for q in [
                Quaternion::IDENTITY,
                Quaternion(0.2, -0.3, 0.4, 0.8).normalized(),
            ] {
                for mode in 0..4 {
                    let mut b = Body::new(BodyId(1), shape, Vector::ZERO, mass);
                    b.orientation = q;
                    b.rotation_locked = mode == 1;
                    b.external = mode == 2;
                    b.sleeping = mode == 3;
                    let mut report = Report::default();
                    let p = PreparedResponse::new(&b, &mut report);
                    assert_eq!(p.inverse_mass.to_bits(), b.inverse_mass().to_bits());
                    for v in [
                        Vector::ZERO,
                        Vector::X,
                        Vector(0.3, -4.25, 7.5),
                        Vector(1e4, -0.0, 1e-6),
                    ] {
                        assert_eq!(
                            vector_bits(p.inertia(&b, v, &mut report)),
                            vector_bits(b.inverse_inertia(v))
                        );
                    }
                    assert_eq!(
                        report.response_preparations,
                        u64::from(mass > 0.0 && mode < 2)
                    );
                    assert_eq!(
                        report.inertia_preparations,
                        u64::from(mass > 0.0 && mode == 0)
                    );
                }
            }
        }
    }
}

fn compare_step(prepared: &mut World, reference: &mut World, dt: Scalar) -> Report {
    let got = prepared.step(dt).unwrap();
    let want = reference.step_with_preparation::<false>(dt).unwrap();
    assert_eq!(
        prepared.bodies, reference.bodies,
        "full body state must match, not just visible pose"
    );
    assert_eq!(prepared.elapsed.to_bits(), reference.elapsed.to_bits());
    assert_eq!(got.substeps, want.substeps);
    assert_eq!(got.pair_tests, want.pair_tests);
    assert_eq!(got.narrow_tests, want.narrow_tests);
    assert_eq!(got.contact_points, want.contact_points);
    assert_eq!(got.impulse_iterations, want.impulse_iterations);
    assert_eq!(got.integrated_bodies, want.integrated_bodies);
    assert_eq!(got.woken_bodies, want.woken_bodies);
    assert_eq!(got.swept_contacts, want.swept_contacts);
    assert_eq!(got.retired, want.retired);
    assert_eq!(
        got.max_penetration.to_bits(),
        want.max_penetration.to_bits()
    );
    assert_eq!(got.inertia_applications, want.inertia_applications);
    got
}

#[test]
fn rotating_bodies_force_torque_and_impulses_match_unprepared_replay() {
    let mut w = World::new(Config {
        gravity: Vector(0.0, -10.0, 0.0),
        ..Config::default()
    })
    .unwrap();
    w.add_body(Body::new(
        BodyId(1),
        Shape::Box(Vector(20.0, 1.0, 20.0)),
        Vector(0.0, -1.0, 0.0),
        0.0,
    ))
    .unwrap();
    for id in 2..6 {
        let mut b = Body::new(
            BodyId(id),
            Shape::Box(Vector(0.8, 1.0, 1.2)),
            Vector((id - 2) as f64 * 2.1, 2.0, 0.0),
            id as f64,
        );
        if id == 2 {
            b.orientation = Quaternion(0.1, 0.2, -0.3, 1.0).normalized();
            b.angular_velocity = Vector(0.5, -0.8, 0.3);
        }
        b.rotation_locked = id == 3;
        b.external = id == 4;
        b.sensor = id == 5;
        w.add_body(b).unwrap();
    }
    let mut reference = w.clone();
    for tick in 0..100 {
        for world in [&mut w, &mut reference] {
            if tick % 17 == 0 {
                world.add_force(BodyId(2), Vector(2.0, 0.0, -0.4)).unwrap();
                let p = world.body(BodyId(2)).unwrap().position;
                world
                    .apply_impulse(BodyId(2), Vector(0.5, 0.2, 0.0), p + Vector(0.0, 0.6, 0.0))
                    .unwrap();
                // Torque has no public setter yet; exercise the same force integration field internally.
                let i = world.index(BodyId(2)).unwrap();
                world.bodies[i].torque = Vector(0.3, -0.2, 0.1);
            }
        }
        compare_step(
            &mut w,
            &mut reference,
            if tick % 2 == 0 {
                1.0 / 60.0
            } else {
                1.0 / 120.0
            },
        );
    }
}

#[test]
fn contact_wake_prepares_real_mass_before_response_and_retirement() {
    let mut w = World::new(Config {
        gravity: Vector::ZERO,
        ..Config::default()
    })
    .unwrap();
    let mut target = Body::new(
        BodyId(10),
        Shape::Box(Vector(1.0, 1.0, 1.0)),
        Vector::ZERO,
        2.0,
    );
    target.sleeping = true;
    w.add_body(target).unwrap();
    let mut p = Body::new(BodyId(20), Shape::Sphere(0.2), Vector(-4.0, 0.4, 0.0), 1.0);
    p.velocity = Vector(600.0, 0.0, 0.0);
    p.ccd = true;
    p.retire_on_impact = true;
    w.add_body(p).unwrap();
    let mut reference = w.clone();
    let r = compare_step(&mut w, &mut reference, 1.0 / 60.0);
    assert_eq!(r.woken_bodies, 1);
    assert_eq!(r.retired, vec![BodyId(20)]);
    assert!(w.body(BodyId(10)).unwrap().velocity.0 > 0.0);
    assert!(w.body(BodyId(10)).unwrap().angular_velocity.length() > 0.0);
    assert!(r.inertia_preparations >= 2);
    compare_step(&mut w, &mut reference, 1.0 / 60.0);
}

#[test]
fn lifecycle_rebuilds_response_indices_and_shape_mass_lock_state() {
    let mut w = World::new(Config {
        gravity: Vector::ZERO,
        ..Config::default()
    })
    .unwrap();
    for id in [20, 40] {
        w.add_body(Body::new(
            BodyId(id),
            Shape::Sphere(1.0),
            Vector(id as f64, 0.0, 0.0),
            2.0,
        ))
        .unwrap();
    }
    let mut reference = w.clone();
    compare_step(&mut w, &mut reference, 1.0 / 60.0);
    for world in [&mut w, &mut reference] {
        world.remove_body(BodyId(20)).unwrap();
        let mut replacement = Body::new(
            BodyId(20),
            Shape::Box(Vector(1.0, 2.0, 4.0)),
            Vector::ZERO,
            5.0,
        );
        replacement.orientation = Quaternion(0.2, 0.3, -0.4, 1.0).normalized();
        world.add_body(replacement).unwrap();
        let mut inserted = Body::new(
            BodyId(5),
            Shape::Box(Vector(1.0, 1.0, 1.0)),
            Vector(-10.0, 0.0, 0.0),
            3.0,
        );
        inserted.rotation_locked = true;
        world.add_body(inserted).unwrap();
        world
            .apply_impulse(BodyId(20), Vector(0.7, 0.2, -0.3), Vector(0.0, 1.0, 0.0))
            .unwrap();
        world
            .apply_impulse(BodyId(5), Vector(3.0, 0.0, 0.0), Vector(-10.0, 1.0, 0.0))
            .unwrap();
    }
    for _ in 0..10 {
        compare_step(&mut w, &mut reference, 1.0 / 60.0);
    }
    assert_eq!(w.body(BodyId(5)).unwrap().angular_velocity, Vector::ZERO);
}

#[test]
fn preparation_work_depends_on_bodies_not_iteration_count() {
    let mut applications = Vec::new();
    for iterations in [1, 8, 32] {
        let mut w = World::new(Config {
            gravity: Vector::ZERO,
            substeps: 1,
            velocity_iterations: iterations,
            // This test measures the fixed-work reference, independently of convergence.
            convergence: None,
            ..Config::default()
        })
        .unwrap();
        for (id, x, v) in [(1, -1.0, 2.0), (2, 1.0, -2.0)] {
            let mut b = Body::new(
                BodyId(id),
                Shape::Box(Vector(1.0, 1.0, 1.0)),
                Vector(x, 0.0, 0.0),
                1.0,
            );
            b.velocity = Vector(v, 0.0, 0.0);
            w.add_body(b).unwrap();
        }
        let mut reference = w.clone();
        let r = compare_step(&mut w, &mut reference, 1.0 / 60.0);
        assert_eq!(r.inertia_preparations, 2);
        assert_eq!(r.response_preparations, 2);
        assert!(reference.last_report.inertia_preparations > r.inertia_preparations);
        assert!(r.inertia_applications > r.inertia_preparations);
        applications.push(r.inertia_applications);
    }
    assert!(applications[0] < applications[1] && applications[1] < applications[2]);
}

#[test]
fn quiescent_and_zero_steps_do_not_prepare_response() {
    let mut w = World::new(Config::default()).unwrap();
    let mut b = Body::new(BodyId(1), Shape::Sphere(1.0), Vector::ZERO, 2.0);
    b.sleeping = true;
    w.add_body(b).unwrap();
    for dt in [0.0, 1.0 / 60.0] {
        let r = w.step(dt).unwrap();
        assert_eq!(r.response_preparations, 0);
        assert_eq!(r.inertia_preparations, 0);
        assert_eq!(r.inertia_applications, 0);
    }
}
