use super::*;
use crate::approximate::{Config, Quaternion, Shape, World};

fn vector_bits(v: Vector) -> [u64; 3] {
    [v.0, v.1, v.2].map(|x| crate::approximate::primitive::widen(x).to_bits())
}

#[test]
fn prepared_response_matches_the_quaternion_formula() {
    for shape in [Shape::Sphere(0.7), Shape::Box(Vector(0.3, 2.5, 7.0))] {
        for mass in [0.0, 0.01, 2.0, 10_000.0] {
            for q in [
                Quaternion::IDENTITY,
                Quaternion(0.2, -0.3, 0.4, 0.8).normalized(),
            ] {
                for mode in 0..4 {
                    let mut b = Body::fixture(1, shape, Vector::ZERO, mass);
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
                        let (got, want) = (p.inertia(v, &mut report), b.inverse_inertia(v));
                        if q == Quaternion::IDENTITY {
                            // Unrotated, the world matrix is the diagonal itself.
                            assert_eq!(vector_bits(got), vector_bits(want));
                        } else {
                            let scale = want.length().max(1e-30);
                            assert!((got - want).length() <= 1e-5 * scale, "{got:?} vs {want:?}");
                        }
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

#[test]
fn contact_wake_prepares_real_mass_before_response_and_retirement() {
    let mut w = World::new(Config {
        gravity: Vector::ZERO,
        ..Config::default()
    })
    .unwrap();
    let mut target = Body::fixture(10, Shape::Box(Vector(1.0, 1.0, 1.0)), Vector::ZERO, 2.0);
    target.sleeping = true;
    w.add_body(target).unwrap();
    let mut p = Body::fixture(20, Shape::Sphere(0.2), Vector(-4.0, 0.4, 0.0), 1.0);
    p.velocity = Vector(600.0, 0.0, 0.0);
    p.ccd = true;
    p.retire_on_impact = true;
    let projectile = w.add_body(p).unwrap();
    let r = w.step(1.0 / 60.0).unwrap();
    assert_eq!(r.woken_bodies, 1);
    assert_eq!(r.retired, vec![projectile]);
    assert!(w.body_id(10).and_then(|id| w.body(id)).unwrap().velocity.0 > 0.0);
    assert!(
        w.body_id(10)
            .and_then(|id| w.body(id))
            .unwrap()
            .angular_velocity
            .length()
            > 0.0
    );
    assert!(r.inertia_preparations >= 2);
    w.step(1.0 / 60.0).unwrap();
}

#[test]
fn slow_current_contact_keeps_sleeping_anchor_response_like_fixed_anchor() {
    for ids in [[10, 20], [20, 10]] {
        let fixture = |sleeping_anchor| {
            let mut w = World::new(Config {
                gravity: Vector::ZERO,
                substeps: 1,
                fixed_position_iterations: 0,
                convergence: None,
                ..Config::legacy()
            })
            .unwrap();
            let mut anchor = Body::fixture(
                ids[0],
                Shape::Sphere(18.0),
                Vector::ZERO,
                if sleeping_anchor { 2.0 } else { 0.0 },
            );
            anchor.sleeping = sleeping_anchor;
            w.add_body(anchor).unwrap();
            let mut moving =
                Body::fixture(ids[1], Shape::Sphere(18.0), Vector(0.0, 35.99, 0.0), 2.0);
            moving.velocity = Vector(0.25, -0.25, 0.0);
            moving.sleep_allowed = false;
            w.add_body(moving).unwrap();
            w
        };
        let mut sleeping = fixture(true);
        let mut fixed = fixture(false);
        for w in [&mut sleeping, &mut fixed] {
            let report = w.step(1.0 / 240.0).unwrap();
            assert_eq!(report.woken_bodies, 0);
            assert_eq!(report.swept_contacts, 0);
            assert_eq!(report.contact_points, 1);
        }
        assert!(
            sleeping
                .body_id(ids[0])
                .and_then(|id| sleeping.body(id))
                .unwrap()
                .sleeping
        );
        assert_eq!(
            sleeping.body_id(ids[1]).and_then(|id| sleeping.body(id)),
            fixed.body_id(ids[1]).and_then(|id| fixed.body(id)),
            "an anchor with inactive mass/inertia must retain the original witness"
        );
        assert!(
            sleeping
                .body(sleeping.body_id(ids[1]).unwrap())
                .unwrap()
                .angular_velocity
                .length()
                > 0.0
        );
    }
}

#[test]
fn lifecycle_rebuilds_response_indices_and_shape_mass_lock_state() {
    let mut w = World::new(Config {
        gravity: Vector::ZERO,
        ..Config::default()
    })
    .unwrap();
    for id in [20, 40] {
        w.add_body(Body::fixture(
            id,
            Shape::Sphere(1.0),
            Vector(id as Real, 0.0, 0.0),
            2.0,
        ))
        .unwrap();
    }
    w.step(1.0 / 60.0).unwrap();
    w.remove_body(w.body_id(20).unwrap()).unwrap();
    let mut replacement = Body::fixture(20, Shape::Box(Vector(1.0, 2.0, 4.0)), Vector::ZERO, 5.0);
    replacement.orientation = Quaternion(0.2, 0.3, -0.4, 1.0).normalized();
    w.add_body(replacement).unwrap();
    let mut inserted = Body::fixture(
        5,
        Shape::Box(Vector(1.0, 1.0, 1.0)),
        Vector(-10.0, 0.0, 0.0),
        3.0,
    );
    inserted.rotation_locked = true;
    w.add_body(inserted).unwrap();
    w.apply_impulse(
        w.body_id(20).unwrap(),
        Vector(0.7, 0.2, -0.3),
        Vector(0.0, 1.0, 0.0),
    )
    .unwrap();
    w.apply_impulse(
        w.body_id(5).unwrap(),
        Vector(3.0, 0.0, 0.0),
        Vector(-10.0, 1.0, 0.0),
    )
    .unwrap();
    for _ in 0..10 {
        w.step(1.0 / 60.0).unwrap();
    }
    assert_eq!(
        w.body_id(5)
            .and_then(|id| w.body(id))
            .unwrap()
            .angular_velocity,
        Vector::ZERO
    );
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
            let mut b = Body::fixture(
                id,
                Shape::Box(Vector(1.0, 1.0, 1.0)),
                Vector(x, 0.0, 0.0),
                1.0,
            );
            b.velocity = Vector(v, 0.0, 0.0);
            w.add_body(b).unwrap();
        }
        let r = w.step(1.0 / 60.0).unwrap();
        assert_eq!(r.inertia_preparations, 2);
        assert_eq!(r.response_preparations, 2);
        assert!(r.inertia_applications > r.inertia_preparations);
        applications.push(r.inertia_applications);
    }
    // Rows hold their angular Jacobians (#426): iterations apply no further inertia.
    assert!(applications[0] == applications[1] && applications[1] == applications[2]);
}

#[test]
fn quiescent_and_zero_steps_do_not_prepare_response() {
    let mut w = World::new(Config::default()).unwrap();
    let mut b = Body::fixture(1, Shape::Sphere(1.0), Vector::ZERO, 2.0);
    b.sleeping = true;
    w.add_body(b).unwrap();
    for dt in [0.0, 1.0 / 60.0] {
        let r = w.step(dt).unwrap();
        assert_eq!(r.response_preparations, 0);
        assert_eq!(r.inertia_preparations, 0);
        assert_eq!(r.inertia_applications, 0);
    }
}

#[test]
fn naturally_parked_contact_wake_applies_pending_load_once() {
    let mut w = World::new(Config {
        gravity: Vector(0.0, -10.0, 0.0),
        substeps: 1,
        convergence: None,
        ..Config::default()
    })
    .unwrap();
    let mut ground = Body::fixture(
        0,
        Shape::Box(Vector(4.0, 0.5, 4.0)),
        Vector(0.0, -0.5, 0.0),
        0.0,
    );
    // The ground's friction 0.6 applies to the frictionless target.
    ground.friction = 0.6;
    ground.friction_combine = crate::approximate::CombineRule::Max;
    w.add_body(ground).unwrap();
    let mut target = Body::fixture(10, Shape::Sphere(1.0), Vector(0.0, 1.0, 0.0), 2.0);
    target.friction = 0.0;
    w.add_body(target).unwrap();
    for _ in 0..32 {
        w.step(1.0 / 60.0).unwrap();
    }
    assert!(
        w.body_id(10)
            .and_then(|id| w.body(id))
            .unwrap()
            .is_sleeping()
    );
    for (id, half, position, velocity) in [
        (
            1,
            Vector(0.5, 4.0, 4.0),
            Vector(-1.5, 1.0, 0.0),
            Vector(3.0, 0.0, 0.0),
        ),
        (
            20,
            Vector(4.0, 4.0, 0.5),
            Vector(0.0, 1.0, -1.5),
            Vector(0.0, 0.0, 3.0),
        ),
    ] {
        let mut wall = Body::fixture(id, Shape::Box(half), position, 1.0);
        wall.external = true;
        wall.friction = 0.0;
        // The target's own friction, as when the pair took the larger coefficient.
        wall.friction_combine = crate::approximate::CombineRule::Max;
        wall.velocity = velocity;
        w.add_body(wall).unwrap();
    }
    // Exercise accepted queued history internally; the public force setter wakes immediately.
    let target = w.index(w.body_id(10).unwrap()).unwrap();
    w.bodies[target].force = Vector(0.0, -20.0, 0.0);
    w.bodies[target].torque = Vector(0.0, 0.8, 0.0);
    let h = 1.0 / 240.0;
    let report = w.step(h).unwrap();
    assert_eq!(report.woken_bodies, 1);
    assert_eq!(report.response_preparations, 1);
    let b = w.body_id(10).and_then(|id| w.body(id)).unwrap();
    assert_eq!(b.velocity, Vector(3.0, 0.0, 3.0));
    // Total downward acceleration20 supplies friction spin30h; torque/I supplies yaw h.
    let tolerance = crate::approximate::rounding_tolerance(1e-10, 16.0, 30.0 * h as Real);
    assert!((b.angular_velocity.1 - h as Real).abs() <= tolerance);
    assert!(
        (Vector(b.angular_velocity.0, 0.0, b.angular_velocity.2).length() - 30.0 * h as Real).abs()
            <= tolerance
    );
    assert_eq!(b.force, Vector::ZERO);
    assert_eq!(b.torque, Vector::ZERO);
}
