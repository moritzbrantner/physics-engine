//! Bounded axial-reference contacts, not a general geometry or rotation oracle.
use physics_engine::{
    BodyId,
    approximate::{Body, CheckpointContext, Config, Quaternion, Report, Shape, Vector as V, World},
};
use physics_engine::{approximate::Real, numeric::Scalar};

const DT: Real = 1.0 / 240.0;
const STEPS: u32 = 64;

fn origin(shape: Shape, z: Real) -> V {
    // Put the rounded shapes' z-facing contact inside the wedge's triangular face.
    if matches!(shape, Shape::Wedge(_)) {
        V(0.5, 0.5, z)
    } else {
        V(0.0, 0.0, z)
    }
}

fn fixture(
    shapes: [Shape; 2],
    ids: [u64; 2],
    mass: Real,
    e: Real,
    depth: Real,
    substeps: u8,
) -> World {
    let mut w = World::new(Config {
        gravity: V::ZERO,
        substeps,
        convergence: None,
        ..Config::default()
    })
    .unwrap();
    for i in 0..2 {
        let mut b = Body::new(
            BodyId(ids[i]),
            shapes[i],
            origin(shapes[i], if i == 0 { 0.0 } else { 2.0 - depth }),
            if i == 0 { 1.0 } else { mass },
        );
        b.rotation_locked = true;
        b.sleep_allowed = false;
        b.friction = 0.0;
        b.restitution = e;
        if i == 0 {
            b.velocity = V(0.0, 0.0, 3.0);
        }
        w.add_body(b).unwrap();
    }
    w
}

fn advance(w: &mut World, substeps: u32) -> Report {
    let before = w.elapsed_seconds();
    let dt = DT * substeps as Real;
    let r = w.step(dt as Scalar).unwrap();
    let tolerance = 16.0 * f64::EPSILON * (before.abs() + dt as Scalar);
    assert!((w.elapsed_seconds() - before - dt as Scalar).abs() <= tolerance);
    assert_eq!(r.substeps, substeps);
    assert!(r.impulse_iterations <= 8 * u64::from(substeps));
    assert!(r.contact_points <= 4 * u64::from(substeps));
    assert!(r.convergence.constraint_visits <= 8 * r.contact_points);
    assert_eq!(r.position.passes, 0);
    assert_eq!(r.swept_contacts, 0);
    assert_eq!(r.woken_bodies, 0);
    assert!(r.retired.is_empty());
    r
}

fn error(actual: V, expected: V) -> Real {
    (actual - expected).length() / (1.0 + expected.length())
}

/// Returns peak/residual overlap, velocity/momentum errors, energy ratio, cases,
/// continuous-contact samples, total solver visits across all three worlds, and elapsed bounds.
pub fn run() -> [Real; 10] {
    let shapes = [
        Shape::Sphere(1.0),
        Shape::Box(V(1.0, 1.0, 1.0)),
        Shape::capsule(0.5, 1.0),
        Shape::wedge(V(1.0, 1.0, 1.0)),
    ];
    let context = CheckpointContext {
        build: [0; 32],
        content: [241; 32],
    };
    let mut values = [(0.0 as Real); 10];
    values[8] = Real::INFINITY;
    let mut cases = 0_u32;
    let mut continuous = 0_u32;
    let mut visits = 0_u64;
    for a in shapes {
        for b in shapes {
            for ids in [[1, 2], [2, 1]] {
                for mass in [0.0, 3.0] {
                    for restitution in [0.0, 0.5, 1.0] {
                        for depth in [0.0, 0.01] {
                            let make =
                                |substeps| fixture([a, b], ids, mass, restitution, depth, substeps);
                            let mut w = make(1);
                            let mut replay = make(1);
                            let mut ticks = make(4);
                            let inv = if mass == 0.0 { 0.0 } else { 1.0 / mass };
                            let impulse = (1.0 + restitution) * 3.0 / (1.0 + inv);
                            let expected = [V(0.0, 0.0, 3.0 - impulse), V(0.0, 0.0, impulse * inv)];
                            let expected_momentum = expected[0] + expected[1] * mass;
                            let observe = |world: &World, step: u32, values: &mut [Real; 10]| {
                                let bodies = ids.map(|id| world.body(BodyId(id)).unwrap());
                                for i in 0..2 {
                                    let body = bodies[i];
                                    assert_eq!(body.shape, [a, b][i]);
                                    assert_eq!(body.mass, if i == 0 { 1.0 } else { mass });
                                    assert!(body.position.finite() && body.velocity.finite());
                                    assert_eq!(body.orientation, Quaternion::IDENTITY);
                                    assert_eq!(body.angular_velocity, V::ZERO);
                                    assert!(!body.is_sleeping());
                                    let v_error = error(body.velocity, expected[i]);
                                    values[2] = values[2].max(v_error);
                                    assert!(
                                        v_error <= 1e-10,
                                        "{a:?}/{b:?} mass={mass} e={restitution} depth={depth} ids={ids:?} step={step} velocity={:?}",
                                        body.velocity
                                    );
                                    let initial =
                                        origin(body.shape, if i == 0 { 0.0 } else { 2.0 - depth });
                                    let position = initial + expected[i] * (step as Real * DT);
                                    assert!(error(body.position, position) <= 1e-10);
                                }
                                // Every fixture has z half-width 1 and a common interior x/y
                                // cross-section. In this locked axial trace the face gap gives
                                // the shallow penetration independently of engine manifolds.
                                let overlap =
                                    (2.0 - (bodies[1].position.2 - bodies[0].position.2)).max(0.0);
                                values[0] = values[0].max(overlap);
                                assert!(overlap <= 0.02 + 1e-10);
                                if step == STEPS {
                                    values[1] = values[1].max(overlap);
                                }
                                let momentum_error = error(
                                    bodies[0].velocity + bodies[1].velocity * mass,
                                    expected_momentum,
                                );
                                values[3] = values[3].max(momentum_error);
                                assert!(momentum_error <= 1e-10);
                                if mass > 0.0 {
                                    // Dynamic pair conserves linear momentum; a fixed target
                                    // intentionally exchanges it with its external authority.
                                    assert!(error(expected_momentum, V(0.0, 0.0, 3.0)) <= 1e-10);
                                }
                                let energy = 0.5
                                    * (bodies[0].velocity.dot(bodies[0].velocity)
                                        + mass * bodies[1].velocity.dot(bodies[1].velocity));
                                let ratio = energy / 4.5;
                                values[4] = values[4].max(ratio);
                                assert!(ratio.is_finite() && ratio <= 1.0 + 1e-10);
                                if restitution > 0.0 && step == STEPS {
                                    assert_eq!(overlap, 0.0);
                                }
                            };
                            for step in 1..=STEPS {
                                let report = advance(&mut w, 1);
                                let repeated = advance(&mut replay, 1);
                                visits += report.convergence.constraint_visits
                                    + repeated.convergence.constraint_visits;
                                if step == 1 || restitution == 0.0 {
                                    assert!(report.contact_points > 0, "{a:?}/{b:?} step={step}");
                                    if restitution == 0.0 {
                                        continuous += 1;
                                    }
                                }
                                observe(&w, step, &mut values);
                                assert_eq!(
                                    w.checkpoint(context).unwrap().to_bytes(),
                                    replay.checkpoint(context).unwrap().to_bytes()
                                );
                                if step % 4 == 0 {
                                    let report = advance(&mut ticks, 4);
                                    visits += report.convergence.constraint_visits;
                                    observe(&ticks, step, &mut values);
                                    assert_eq!(
                                        w.bodies().collect::<Vec<_>>(),
                                        ticks.bodies().collect::<Vec<_>>()
                                    );
                                }
                            }
                            let elapsed = w.elapsed_seconds();
                            assert!((elapsed - STEPS as Scalar * DT as Scalar).abs() <= 1e-12);
                            assert!((ticks.elapsed_seconds() - elapsed).abs() <= 1e-12);
                            values[8] = values[8].min(elapsed as Real);
                            values[9] = values[9].max(elapsed as Real);
                            cases += 1;
                        }
                    }
                }
            }
        }
    }
    values[5] = cases as Real;
    values[6] = continuous as Real;
    // This explicitly bounded fixture cannot reach the f64 exact-integer boundary.
    assert!(visits < (1_u64 << 53));
    values[7] = visits as Real;
    println!("PRIMITIVE_CONTACT {{\"values\":{values:?}}}");
    values
}
