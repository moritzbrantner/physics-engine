//! Analytic public-API controls shared by native tests and the existing WASM driver.
use physics_engine::{
    BodyId,
    approximate::{Body, CheckpointContext, Config, Error, Shape, Vector as V, World},
};

const DT: f64 = 0.01;

fn close(actual: f64, expected: f64) {
    let tolerance = 1e-11 * (1.0 + expected.abs());
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual} != {expected} within {tolerance}"
    );
}

fn world(gravity: V, warm_start: bool) -> World {
    World::new(Config {
        gravity,
        substeps: 1,
        convergence: None,
        warm_start,
        ..Default::default()
    })
    .unwrap()
}

fn sphere(id: u64, position: V, mass: f64, velocity: V) -> Body {
    let mut body = Body::new(BodyId(id), Shape::Sphere(1.0), position, mass);
    body.velocity = velocity;
    body.friction = 0.0;
    body.rotation_locked = true;
    body.sleep_allowed = false;
    body
}

fn step(world: &mut World) {
    let before = world.elapsed_seconds();
    let report = world.step(DT).unwrap();
    close(world.elapsed_seconds() - before, DT);
    assert_eq!(report.substeps, 1);
    assert!(report.contact_points > 0);
    assert!(report.impulse_iterations <= 8);
    assert_eq!(report.position.dynamic_corrections, 0);
    assert!(report.retired.is_empty());
}

pub fn restitution_combination_threshold_and_mass_scaling() {
    for mass_scale in [1e-6, 1.0, 1e6] {
        for (left_id, right_id) in [(1, 2), (2, 1)] {
            for (left_e, right_e) in [(0.2, 0.8), (0.8, 0.2), (0.0, 1.0), (1.0, 1.0)] {
                for speed in [0.999, 1.0, 1.001, 3.0] {
                    let mut w = world(V::ZERO, true);
                    let (ma, mb) = (mass_scale, 3.0 * mass_scale);
                    let mut a = sphere(left_id, -V::X, ma, V::X * speed);
                    let mut b = sphere(right_id, V::X, mb, V::ZERO);
                    a.restitution = left_e;
                    b.restitution = right_e;
                    w.add_body(b).unwrap();
                    w.add_body(a).unwrap();
                    step(&mut w);
                    let a = w.body(BodyId(left_id)).unwrap();
                    let b = w.body(BodyId(right_id)).unwrap();
                    // Independent one-dimensional impulse/momentum solution.
                    let e = if speed > 1.0 {
                        left_e.min(right_e)
                    } else {
                        0.0
                    };
                    let impulse = (1.0 + e) * speed / (1.0 / ma + 1.0 / mb);
                    close(a.velocity.0, speed - impulse / ma);
                    close(b.velocity.0, impulse / mb);
                    close((a.velocity * ma + b.velocity * mb).0, ma * speed);
                    close(b.velocity.0 - a.velocity.0, e * speed);
                    let reduced_mass = ma * mb / (ma + mb);
                    let energy_before = 0.5 * ma * speed * speed;
                    let expected_loss = 0.5 * reduced_mass * (1.0 - e * e) * speed * speed;
                    close(
                        a.kinetic_energy() + b.kinetic_energy(),
                        energy_before - expected_loss,
                    );
                    assert_eq!(a.angular_velocity, V::ZERO);
                    assert_eq!(b.angular_velocity, V::ZERO);
                }
            }
        }
    }
}

pub fn fixed_target_restitution_and_speculative_contact() {
    for (dynamic_id, fixed_id) in [(1, 2), (2, 1)] {
        for gap in [0.0, 0.01, 0.03] {
            let mut w = world(V::ZERO, true);
            let mut fixed = sphere(fixed_id, V::X * (1.0 + gap), 0.0, V::ZERO);
            fixed.restitution = 0.75;
            let mut moving = sphere(dynamic_id, -V::X, 2.0, V::X * 4.0);
            moving.restitution = 0.5;
            moving.ccd = true;
            w.add_body(fixed).unwrap();
            w.add_body(moving).unwrap();
            let fixed = w.body(BodyId(fixed_id)).unwrap().clone();
            step(&mut w);
            let moving = w.body(BodyId(dynamic_id)).unwrap();
            // Separated speculative contacts permit arrival, never premature restitution.
            close(moving.velocity.0, if gap == 0.0 { -2.0 } else { gap / DT });
            assert_eq!(w.body(BodyId(fixed_id)), Some(&fixed));
        }
    }
}

pub fn persistent_contact_gates_restitution_even_without_warm_start() {
    for warm_start in [false, true] {
        let mut w = world(V::ZERO, warm_start);
        for (id, position) in [(1, -V::X), (2, V::X)] {
            let mut body = sphere(id, position, 1.0, V::ZERO);
            body.restitution = 1.0;
            w.add_body(body).unwrap();
        }
        step(&mut w); // Establish a touching pair with zero impulse.
        w.set_velocity(BodyId(1), V::X * 2.0).unwrap();
        w.set_velocity(BodyId(2), -V::X * 2.0).unwrap();
        step(&mut w);
        assert_eq!(w.body(BodyId(1)).unwrap().velocity, V::ZERO);
        assert_eq!(w.body(BodyId(2)).unwrap().velocity, V::ZERO);

        // An actual clear interval removes history; the next contact can bounce again.
        w.set_velocity(BodyId(1), -V::X * 2.0).unwrap();
        w.set_velocity(BodyId(2), V::X * 2.0).unwrap();
        for _ in 0..3 {
            w.step(DT).unwrap();
        }
        w.set_velocity(BodyId(1), V::X * 2.0).unwrap();
        w.set_velocity(BodyId(2), -V::X * 2.0).unwrap();
        w.step(DT).unwrap();
        w.step(DT).unwrap();
        w.step(DT).unwrap();
        step(&mut w);
        close(w.body(BodyId(1)).unwrap().velocity.0, -2.0);
        close(w.body(BodyId(2)).unwrap().velocity.0, 2.0);

        // Same-ID replacement removes the old pair's restitution history as well.
        let mut w = world(V::ZERO, warm_start);
        for (id, position) in [(1, -V::X), (2, V::X)] {
            let mut body = sphere(id, position, 1.0, V::ZERO);
            body.restitution = 1.0;
            w.add_body(body).unwrap();
        }
        step(&mut w);
        let removed = w.remove_body(BodyId(1)).unwrap();
        w.add_body(removed).unwrap();
        w.set_velocity(BodyId(1), V::X * 2.0).unwrap();
        w.set_velocity(BodyId(2), -V::X * 2.0).unwrap();
        step(&mut w);
        close(w.body(BodyId(1)).unwrap().velocity.0, -2.0);
        close(w.body(BodyId(2)).unwrap().velocity.0, 2.0);
    }
}

fn sliding_sphere(
    mass: f64,
    friction: [f64; 2],
    velocity: V,
    locked: bool,
    linear: bool,
    ids: [u64; 2],
) -> World {
    let mut w = world(-V::Y * 10.0, true);
    let mut floor = Body::new(BodyId(ids[0]), Shape::Box(V(100.0, 1.0, 100.0)), -V::Y, 0.0);
    floor.friction = friction[0];
    let mut body = sphere(ids[1], V::Y, mass, velocity);
    body.friction = friction[1];
    body.rotation_locked = locked;
    body.linear_support = linear.then_some(V::Y);
    w.add_body(body).unwrap();
    w.add_body(floor).unwrap();
    step(&mut w);
    w
}

pub fn friction_combination_disk_mass_scaling_and_rotation() {
    for mass in [1e-6, 1.0, 1e6] {
        for ids in [[1, 2], [2, 1]] {
            for friction in [[0.0, 0.0], [0.2, 0.8], [0.8, 0.2], [0.0, 0.8]] {
                let initial = V(3.0, 0.0, 4.0);
                let w = sliding_sphere(mass, friction, initial, true, false, ids);
                let body = w.body(BodyId(ids[1])).unwrap();
                let mu = friction[0].max(friction[1]);
                let speed = initial.length() - mu * 10.0 * DT;
                close(body.velocity.0, speed * 0.6);
                close(body.velocity.1, 0.0);
                close(body.velocity.2, speed * 0.8);
                // The vector impulse lies on the disk, not a per-axis square clamp.
                close(
                    (initial - body.velocity).length() * mass,
                    mu * mass * 10.0 * DT,
                );
                assert!(body.kinetic_energy() <= 0.5 * mass * initial.dot(initial));
            }
            let initial = V::X * 0.02;
            let locked = sliding_sphere(mass, [0.8, 0.0], initial, true, false, ids);
            close(locked.body(BodyId(ids[1])).unwrap().velocity.0, 0.0);
            let rolling = sliding_sphere(mass, [0.8, 0.0], initial, false, false, ids);
            let body = rolling.body(BodyId(ids[1])).unwrap();
            // Solid sphere I = 2/5 m r^2; the friction impulse stops contact slip.
            close(body.velocity.0, 5.0 / 7.0 * initial.0);
            close(body.angular_velocity.2, -5.0 / 7.0 * initial.0);
            close(
                (body.velocity + body.angular_velocity.cross(-V::Y)).length(),
                0.0,
            );
            close(
                body.kinetic_energy(),
                0.5 * mass * initial.dot(initial) * 5.0 / 7.0,
            );
            let linear = sliding_sphere(mass, [0.8, 0.8], V::X * 3.0, true, true, ids);
            close(
                (linear.body(BodyId(ids[1])).unwrap().velocity - V::X * 3.0).length(),
                0.0,
            );
        }
    }
}

pub fn invalid_materials_and_mass_leave_checkpoint_unchanged() {
    let mut w = world(V::ZERO, true);
    w.add_body(sphere(1, V::ZERO, 1.0, V::ZERO)).unwrap();
    w.add_force(BodyId(1), V::X).unwrap();
    let context = CheckpointContext {
        build: [22; 32],
        content: [1; 32],
    };
    let before = w.checkpoint(context).unwrap().to_bytes();
    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -0.01] {
        for field in 0..3 {
            let mut bad = sphere(2, V::X * 10.0, 1.0, V::ZERO);
            match field {
                0 => bad.friction = value,
                1 => bad.restitution = value,
                _ => bad.mass = value,
            }
            assert_eq!(w.add_body(bad), Err(Error::InvalidInput));
            assert_eq!(w.checkpoint(context).unwrap().to_bytes(), before);
        }
    }
    for (friction, restitution, mass) in [
        (10.01, 0.0, 1.0),
        (0.0, 1.01, 1.0),
        (0.0, 0.0, 1e-7),
        (0.0, 0.0, 1e12),
    ] {
        let mut bad = sphere(2, V::X * 10.0, mass, V::ZERO);
        bad.friction = friction;
        bad.restitution = restitution;
        assert_eq!(w.add_body(bad), Err(Error::InvalidInput));
        assert_eq!(w.checkpoint(context).unwrap().to_bytes(), before);
    }
    let mut endpoints = sphere(2, V::X * 10.0, 1e-6, V::ZERO);
    endpoints.friction = 10.0;
    endpoints.restitution = 1.0;
    w.add_body(endpoints).unwrap();
}

pub fn run() {
    restitution_combination_threshold_and_mass_scaling();
    fixed_target_restitution_and_speculative_contact();
    persistent_contact_gates_restitution_even_without_warm_start();
    friction_combination_disk_mass_scaling_and_rotation();
    invalid_materials_and_mass_leave_checkpoint_unchanged();
    penetration_bias_and_same_target_replay();
}

pub fn penetration_bias_and_same_target_replay() {
    fn trace() -> Vec<Vec<u8>> {
        let mut w = world(V::ZERO, true);
        let context = CheckpointContext {
            build: [22; 32],
            content: [1; 32],
        };
        // No incoming kinetic energy: only the declared penetration-bias source acts.
        w.add_body(sphere(1, -V::X * 0.975, 1.0, V::ZERO)).unwrap();
        w.add_body(sphere(2, V::X * 0.975, 3.0, V::ZERO)).unwrap();
        step(&mut w);
        let a = w.body(BodyId(1)).unwrap();
        let b = w.body(BodyId(2)).unwrap();
        let separating_speed = 0.2 * (0.05 - w.config().contact_slop) / DT;
        close(b.velocity.0 - a.velocity.0, separating_speed);
        close(a.velocity.0 + 3.0 * b.velocity.0, 0.0);
        close(
            a.kinetic_energy() + b.kinetic_energy(),
            0.5 * 0.75 * separating_speed * separating_speed,
        );
        let mut history = vec![w.checkpoint(context).unwrap().to_bytes()];
        for _ in 0..16 {
            w.step(DT).unwrap();
            history.push(w.checkpoint(context).unwrap().to_bytes());
        }
        history
    }
    assert_eq!(trace(), trace());
}
