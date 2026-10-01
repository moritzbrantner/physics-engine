//! First-substep force/load oracles for contact-awakened bodies. No sleep-policy changes.
use physics_engine::{
    BodyId,
    approximate::{
        Body, Checkpoint, CheckpointContext, CheckpointLimits, Config, Shape, Vector as V, World,
    },
};
const H: f64 = 1.0 / 240.0;
const CONTEXT: CheckpointContext = CheckpointContext {
    build: [0; 32],
    content: [253; 32],
};
#[derive(Clone, Copy, Debug)]
enum Case {
    ContactWake,
    RepeatedRoots,
    AwakeReference,
    RemovedSupport,
}
fn fixture(case: Case, ids: [u64; 2], substeps: u8) -> World {
    let mut w = World::new(Config {
        gravity: V(0.0, -10.0, 0.0),
        substeps,
        convergence: None,
        ..Config::default()
    })
    .unwrap();
    for (id, x) in [(0, 0.0), (40, 10.0)] {
        w.add_body(Body::new(
            BodyId(id),
            Shape::Box(V(4.0, 0.5, 4.0)),
            V(x, -0.5, 0.0),
            0.0,
        ))
        .unwrap();
    }
    for (id, x) in [(ids[1], 0.0), (30, 10.0)] {
        let mut b = Body::new(BodyId(id), Shape::Sphere(1.0), V(x, 1.0, 0.0), 2.0);
        b.friction = 0.0; // max mixing: the floor contributes .6; driven walls contribute0.
        if id == ids[1] && matches!(case, Case::AwakeReference) {
            b.sleep_allowed = false;
        }
        w.add_body(b).unwrap();
    }
    for _ in 0..32 {
        w.step(1.0 / 60.0).unwrap();
    }
    assert_eq!(
        w.body(BodyId(ids[1])).unwrap().is_sleeping(),
        !matches!(case, Case::AwakeReference)
    );
    assert!(w.body(BodyId(30)).unwrap().is_sleeping());
    if matches!(case, Case::RemovedSupport) {
        w.remove_body(BodyId(0)).unwrap();
        assert!(!w.body(BodyId(ids[1])).unwrap().is_sleeping());
        assert!(!w.has_support(BodyId(ids[1])));
        assert!(w.body(BodyId(30)).unwrap().is_sleeping());
    }
    let walls = if matches!(case, Case::RepeatedRoots) {
        2
    } else {
        1
    };
    for wall in 0..walls {
        let (id, half, position, velocity) = if wall == 0 {
            (
                ids[0],
                V(0.5, 4.0, 4.0),
                V(-1.5, 1.0, 0.0),
                V(3.0, 0.0, 0.0),
            )
        } else {
            (20, V(4.0, 4.0, 0.5), V(0.0, 1.0, -1.5), V(0.0, 0.0, 3.0))
        };
        let mut body = Body::new(BodyId(id), Shape::Box(half), position, 1.0);
        body.external = true;
        body.friction = 0.0;
        w.add_body(body).unwrap();
        w.set_velocity(BodyId(id), velocity).unwrap();
    }
    assert_eq!(
        w.body(BodyId(ids[1])).unwrap().is_sleeping(),
        matches!(case, Case::ContactWake | Case::RepeatedRoots)
    );
    w
}
fn observe(w: &World, case: Case, ids: [u64; 2], n: u32) -> f64 {
    let b = w.body(BodyId(ids[1])).unwrap();
    let t = H * f64::from(n);
    let double = matches!(case, Case::RepeatedRoots);
    let removed = matches!(case, Case::RemovedSupport);
    let z = if double { 3.0 } else { 0.0 };
    let expected_v = V(3.0, if removed { -10.0 * t } else { 0.0 }, z);
    let expected_y = if removed {
        1.0 - 5.0 * H * H * f64::from(n) * f64::from(n + 1)
    } else {
        1.0
    };
    let diagonal = if double {
        std::f64::consts::FRAC_1_SQRT_2
    } else {
        1.0
    };
    let expected_spin = if removed {
        V::ZERO
    } else {
        V(
            if double { 15.0 * t * diagonal } else { 0.0 },
            0.0,
            -15.0 * t * diagonal,
        )
    };
    let error = (b.position - V(3.0 * t, expected_y, z * t))
        .length()
        .max((b.velocity - expected_v).length())
        .max((b.angular_velocity - expected_spin).length());
    assert!(
        error <= 1e-10,
        "{case:?} ids={ids:?} n={n} error={error} body={b:?}"
    );
    assert!(b.position.finite() && b.velocity.finite() && b.angular_velocity.finite());
    assert!(!b.is_sleeping());
    assert_eq!(b.mass, 2.0);
    let neighbor = w.body(BodyId(30)).unwrap();
    assert!(neighbor.is_sleeping());
    assert_eq!(neighbor.position, V(10.0, 1.0, 0.0));
    assert_eq!(neighbor.velocity, V::ZERO);
    for (id, position, velocity) in [
        (ids[0], V(-1.5, 1.0, 0.0), V(3.0, 0.0, 0.0)),
        (20, V(0.0, 1.0, -1.5), V(0.0, 0.0, 3.0)),
    ] {
        if let Some(wall) = w.body(BodyId(id)) {
            assert!(wall.external && !wall.is_sleeping());
            assert_eq!(wall.velocity, velocity);
            assert!((wall.position - (position + velocity * t)).length() <= 1e-10);
        }
    }
    error
}
fn run_case(case: Case, ids: [u64; 2], substeps: u8) -> [f64; 17] {
    let mut w = fixture(case, ids, substeps);
    let mut repeated = fixture(case, ids, substeps);
    let bytes = w.checkpoint(CONTEXT).unwrap().to_bytes();
    let mut restored = Checkpoint::from_bytes(&bytes, CONTEXT, CheckpointLimits::default())
        .unwrap()
        .restore();
    let before = w.elapsed_seconds();
    let dt = H * f64::from(substeps);
    let mut first = None;
    let mut max_error: f64 = 0.0;
    for call in 1..=9 {
        let report = w.step(dt).unwrap();
        repeated.step(dt).unwrap();
        restored.step(dt).unwrap();
        for other in [&repeated, &restored] {
            assert_eq!(
                w.checkpoint(CONTEXT).unwrap().to_bytes(),
                other.checkpoint(CONTEXT).unwrap().to_bytes()
            );
        }
        max_error = max_error.max(observe(&w, case, ids, call * u32::from(substeps)));
        assert_eq!(report.substeps, u32::from(substeps));
        assert!(report.impulse_iterations <= 8 * u64::from(substeps));
        assert!(report.convergence.constraint_visits <= 8 * report.contact_points);
        assert_eq!(report.position.passes, 0);
        assert!(report.retired.is_empty());
        if call == 1 {
            let expected_wake = u64::from(matches!(case, Case::ContactWake | Case::RepeatedRoots));
            assert_eq!(report.woken_bodies, expected_wake);
            assert_eq!(report.response_preparations, u64::from(substeps));
            assert_eq!(report.inertia_preparations, u64::from(substeps));
            let b = w.body(BodyId(ids[1])).unwrap();
            let energy =
                b.velocity.dot(b.velocity) + 0.4 * b.angular_velocity.dot(b.angular_velocity);
            // Floor angular impulse comes only from tangential load, since wall friction is0.
            let floor_x = 0.8 * b.angular_velocity.2;
            let floor_z = -0.8 * b.angular_velocity.0;
            let work = 3.0 * (2.0 * b.velocity.0 - floor_x)
                + if matches!(case, Case::RepeatedRoots) {
                    3.0 * (2.0 * b.velocity.2 - floor_z)
                } else {
                    0.0
                };
            assert!(energy + 20.0 * (b.position.1 - 1.0) <= work + 1e-10);
            assert!(
                (w.elapsed_seconds() - before - dt).abs() <= 16.0 * f64::EPSILON * (before + dt)
            );
            first = Some([
                b.velocity.0,
                b.velocity.1,
                b.velocity.2,
                b.angular_velocity.length(),
                energy,
                work,
                report.woken_bodies as f64,
                report.response_preparations as f64,
                report.inertia_preparations as f64,
                report.inertia_applications as f64,
                report.geometry.current_queries as f64,
                report.geometry.sweep_queries as f64,
                report.contact_points as f64,
                report.convergence.constraint_visits as f64,
                w.elapsed_seconds() - before,
                0.0,
                f64::from(u8::from(w.body(BodyId(30)).unwrap().is_sleeping())),
            ]);
        }
    }
    assert!((w.elapsed_seconds() - before - 9.0 * dt).abs() <= 1e-12);
    let mut values = first.unwrap();
    values[15] = max_error;
    println!(
        "CONTACT_WAKE_FORCES {{\"case\":\"{case:?}\",\"ids\":{ids:?},\"substeps\":{substeps},\"values\":{values:?}}}"
    );
    values
}
pub fn run() -> [[f64; 17]; 16] {
    std::array::from_fn(|index| {
        run_case(
            [
                Case::ContactWake,
                Case::RepeatedRoots,
                Case::AwakeReference,
                Case::RemovedSupport,
            ][index / 4],
            if index % 4 < 2 { [1, 10] } else { [10, 1] },
            if index % 2 == 0 { 1 } else { 4 },
        )
    })
}
