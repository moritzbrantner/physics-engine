//! #477: independent Rapier evidence for simultaneous dynamic/dynamic CCD contacts.
//!
//! The blocking paired fixture has an analytic answer. Radial convergence is a
//! deliberately adversarial stress diagnostic, *not* a per-ball parity gate:
//! near-simultaneous many-body contacts can have solver-dependent outcomes.
//!
//! ```text
//! cargo test --locked -p physics-oracle --test ccd_dense_reference
//! cargo test --release --locked -p physics-oracle --test ccd_dense_reference -- --ignored --nocapture
//! ```

use physics_engine::approximate::{Body, BodyStorage, Config, Shape, Vector as V, World};
use rapier3d::prelude::{
    ColliderBuilder, PhysicsWorld, RigidBodyBuilder, RigidBodyHandle, Vector as RV,
};

const DT: f32 = 1.0 / 60.0;
const RADIUS: f32 = 0.1;
const MASS: f32 = 1.0;
const DENSITY: f32 = MASS / (4.0 / 3.0 * std::f32::consts::PI * RADIUS * RADIUS * RADIUS);
const IRRELEVANT_STATICS: usize = 128;

#[derive(Clone, Copy)]
struct Initial {
    position: [f32; 3],
    velocity: [f32; 3],
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct State {
    position: [f32; 3],
    velocity: [f32; 3],
}

struct EngineResult {
    states: Vec<State>,
    impacts: u64,
    fallbacks: u64,
    pairs: u64,
    narrows: u64,
    sweeps: u64,
    single_tick_us: f64,
}

fn paired_lanes(pairs: usize) -> Vec<Initial> {
    // Independent, equal-mass, elastic head-on sphere pairs. Their TOIs are
    // identical, exercising the shared-clock event scheduler. One impact per
    // pair has a unique solution; at 60 Hz both balls end at +/- 0.5 m.
    (0..pairs)
        .flat_map(|lane| {
            let y = lane as f32 * 0.55;
            [
                Initial {
                    position: [-0.45, y, 0.0],
                    velocity: [45.0, 0.0, 0.0],
                },
                Initial {
                    position: [0.45, y, 0.0],
                    velocity: [-45.0, 0.0, 0.0],
                },
            ]
        })
        .collect()
}

fn radial_convergence(count: usize) -> Vec<Initial> {
    // Exactly the construction in tests/ccd_dense_scaling.rs.
    let ring_radius = 0.75 + count as f32 * 0.04;
    let speed = (ring_radius + 0.5) * 60.0;
    (0..count)
        .map(|i| {
            let angle = std::f32::consts::TAU * i as f32 / count as f32;
            let (sin, cos) = angle.sin_cos();
            Initial {
                position: [ring_radius * cos, ring_radius * sin, 0.0],
                velocity: [-speed * cos, -speed * sin, 0.0],
            }
        })
        .collect()
}

fn engine(input: &[Initial], iterations: u8, storage: BodyStorage) -> EngineResult {
    let mut world = World::new(Config {
        gravity: V::ZERO,
        substeps: 1,
        velocity_iterations: iterations,
        body_storage: storage,
        ..Config::default()
    })
    .unwrap();
    // Keep the same irrelevant-static control as the root dense benchmark.
    for i in 0..IRRELEVANT_STATICS {
        world
            .add_body(Body::keyed(
                2 + i as u64,
                Shape::Box(V(0.2, 0.2, 0.2)),
                V(10_000.0 + i as f32, 0.0, 0.0),
                0.0,
            ))
            .unwrap();
    }
    for (i, initial) in input.iter().enumerate() {
        let p = initial.position;
        let v = initial.velocity;
        let mut body = Body::keyed(
            10_000 + i as u64,
            Shape::Sphere(RADIUS),
            V(p[0], p[1], p[2]),
            MASS,
        );
        body.velocity = V(v[0], v[1], v[2]);
        body.restitution = 1.0;
        body.friction = 0.0;
        body.ccd = true;
        body.sleep_allowed = false;
        world.add_body(body).unwrap();
    }
    let start = std::time::Instant::now();
    let r = world.step(f64::from(DT)).unwrap();
    let single_tick_us = start.elapsed().as_secs_f64() * 1e6;
    let states = (0..input.len())
        .map(|i| {
            let body = world
                .body(world.body_id(10_000 + i as u64).unwrap())
                .unwrap();
            State {
                position: [body.position.0, body.position.1, body.position.2],
                velocity: [body.velocity.0, body.velocity.1, body.velocity.2],
            }
        })
        .collect();
    EngineResult {
        states,
        impacts: r.swept_contacts,
        fallbacks: r.geometry.ccd_budget_fallbacks,
        pairs: r.pair_tests,
        narrows: r.narrow_tests,
        sweeps: r.geometry.sweep_queries,
        single_tick_us,
    }
}

fn rapier(input: &[Initial], steps: usize) -> Vec<State> {
    let mut world = PhysicsWorld::new();
    world.gravity = RV::ZERO;
    world.integration_parameters.dt = DT / steps as f32;
    for i in 0..IRRELEVANT_STATICS {
        world.colliders.insert(
            ColliderBuilder::cuboid(0.2, 0.2, 0.2)
                .translation(RV::new(10_000.0 + i as f32, 0.0, 0.0))
                .build(),
        );
    }
    let mut handles: Vec<RigidBodyHandle> = Vec::with_capacity(input.len());
    for initial in input {
        let [x, y, z] = initial.position;
        let [vx, vy, vz] = initial.velocity;
        let handle = world.bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(RV::new(x, y, z))
                .linvel(RV::new(vx, vy, vz))
                .ccd_enabled(true)
                .can_sleep(false)
                .build(),
        );
        world.colliders.insert_with_parent(
            ColliderBuilder::ball(RADIUS)
                .density(DENSITY)
                .restitution(1.0)
                .friction(0.0)
                .build(),
            handle,
            &mut world.bodies,
        );
        handles.push(handle);
    }
    for _ in 0..steps {
        world.step();
    }
    handles
        .into_iter()
        .map(|handle| {
            let body = &world.bodies[handle];
            let p = body.translation();
            let v = body.linvel();
            State {
                position: [p.x, p.y, p.z],
                velocity: [v.x, v.y, v.z],
            }
        })
        .collect()
}

fn assert_near(actual: f32, expected: f32, tolerance: f32, label: &str) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{label}: expected {expected:.6}, got {actual:.6} (tolerance {tolerance})"
    );
}

#[test]
fn eight_simultaneous_sphere_pair_impacts_match_analytic_and_rapier() {
    let input = paired_lanes(8);
    let ordered = engine(&input, 8, BodyStorage::Ordered);
    let indexed = engine(&input, 8, BodyStorage::Indexed);
    let reference = rapier(&input, 64);
    assert_eq!(
        ordered.states, indexed.states,
        "CCD storage order divergence"
    );
    assert_eq!(ordered.impacts, 8, "one real impact per head-on pair");
    assert_eq!(ordered.impacts, indexed.impacts);
    assert_eq!(
        ordered.fallbacks, 0,
        "isolated pairs must not exhaust CCD budget"
    );
    assert_eq!(indexed.fallbacks, 0);
    for (i, (observed, rapier_state)) in ordered.states.iter().zip(&reference).enumerate() {
        let side = if i.is_multiple_of(2) { -1.0 } else { 1.0 };
        let y = (i / 2) as f32 * 0.55;
        assert_near(observed.position[0], side * 0.5, 0.025, "engine final x");
        assert_near(observed.velocity[0], side * 45.0, 0.15, "engine final vx");
        assert_near(observed.position[1], y, 0.001, "engine final y");
        assert_near(observed.velocity[1], 0.0, 0.001, "engine final vy");
        assert_near(
            observed.position[0],
            rapier_state.position[0],
            0.05,
            "Rapier x",
        );
        assert_near(
            observed.velocity[0],
            rapier_state.velocity[0],
            1.0,
            "Rapier vx",
        );
    }
}

#[derive(Debug)]
struct Metrics {
    energy_pct: f64,
    momentum: f64,
    mean_radius: f64,
    max_overlap: f64,
}

fn metrics(input: &[Initial], states: &[State]) -> Metrics {
    let before_energy: f64 = input
        .iter()
        .map(|s| {
            s.velocity
                .iter()
                .map(|v| f64::from(*v).powi(2))
                .sum::<f64>()
        })
        .sum();
    let after_energy: f64 = states
        .iter()
        .map(|s| {
            s.velocity
                .iter()
                .map(|v| f64::from(*v).powi(2))
                .sum::<f64>()
        })
        .sum();
    let momentum = (0..3)
        .map(|axis| {
            let sum: f64 = states.iter().map(|s| f64::from(s.velocity[axis])).sum();
            sum * sum
        })
        .sum::<f64>()
        .sqrt();
    let mean_radius = states
        .iter()
        .map(|s| f64::from(s.position[0]).hypot(f64::from(s.position[1])))
        .sum::<f64>()
        / states.len() as f64;
    let mut max_overlap: f64 = 0.0;
    for (i, a) in states.iter().enumerate() {
        for b in &states[i + 1..] {
            let dist = (0..3)
                .map(|axis| f64::from(a.position[axis] - b.position[axis]).powi(2))
                .sum::<f64>()
                .sqrt();
            max_overlap = max_overlap.max((2.0 * f64::from(RADIUS) - dist).max(0.0));
        }
    }
    Metrics {
        energy_pct: (after_energy / before_energy - 1.0) * 100.0,
        momentum,
        mean_radius,
        max_overlap,
    }
}

/// Non-blocking sensitivity and high-resolution reference. `velocity_iterations`
/// also controls the ordinary solver, so changing it does *not* isolate just
/// the CCD event cap. Compare work/fallback counts and state invariants; avoid
/// per-ball parity assertions for a maximally symmetric, chaotic pileup. The
/// single-tick timings have no warmup and are not stable performance evidence.
#[test]
#[ignore = "diagnostic radial multi-impact reference; run explicitly with --ignored --nocapture"]
fn print_radial_budget_sensitivity_against_rapier() {
    for count in [8, 16, 32, 64] {
        let input = radial_convergence(count);
        let reference = rapier(&input, 128);
        let reference_metrics = metrics(&input, &reference);
        println!(
            "ccd-dense-rapier count={count} steps=128 energy_delta_pct={:.4} momentum={:.5} mean_radius={:.5} max_overlap={:.5}",
            reference_metrics.energy_pct,
            reference_metrics.momentum,
            reference_metrics.mean_radius,
            reference_metrics.max_overlap,
        );
        for iterations in [4, 8, 16, 32] {
            let result = engine(&input, iterations, BodyStorage::Ordered);
            assert!(result.states.iter().all(|state| {
                state
                    .position
                    .iter()
                    .chain(state.velocity.iter())
                    .all(|x| x.is_finite())
            }));
            let state_metrics = metrics(&input, &result.states);
            let mean_position_gap = result
                .states
                .iter()
                .zip(&reference)
                .map(|(a, b)| {
                    (0..3)
                        .map(|axis| f64::from(a.position[axis] - b.position[axis]).powi(2))
                        .sum::<f64>()
                        .sqrt()
                })
                .sum::<f64>()
                / count as f64;
            println!(
                "ccd-dense-budget count={count} iterations={iterations} impacts={} fallbacks={} pairs={} narrows={} sweeps={} single_tick_us={:.0} mean_rapier_position_gap={mean_position_gap:.4} energy_delta_pct={:.4} momentum={:.5} mean_radius={:.5} max_overlap={:.5}",
                result.impacts,
                result.fallbacks,
                result.pairs,
                result.narrows,
                result.sweeps,
                result.single_tick_us,
                state_metrics.energy_pct,
                state_metrics.momentum,
                state_metrics.mean_radius,
                state_metrics.max_overlap,
            );
        }
    }
}

/// #481 isolation search: find the smallest radial group exhibiting a deep
/// final overlap at each budget. Not a correctness pass; an independent
/// minimized regression must replace this diagnostic before an engine fix.
#[test]
#[ignore = "diagnostic failing-size search for #481; run explicitly with --ignored --nocapture"]
fn print_first_deep_overlap_by_budget() {
    for iterations in [4, 8, 12, 16, 24, 32] {
        for count in 8..=64 {
            let initial = radial_convergence(count);
            let run = engine(&initial, iterations, BodyStorage::Ordered);
            let mut worst = (0.0_f64, 0, 0);
            for (i, a) in run.states.iter().enumerate() {
                for (j, b) in run.states.iter().enumerate().skip(i + 1) {
                    let distance = (0..3)
                        .map(|axis| f64::from(a.position[axis] - b.position[axis]).powi(2))
                        .sum::<f64>()
                        .sqrt();
                    let overlap = (2.0 * f64::from(RADIUS) - distance).max(0.0);
                    if overlap > worst.0 {
                        worst = (overlap, i, j);
                    }
                }
            }
            if worst.0 > 0.025 {
                let (depth, i, j) = worst;
                println!(
                    "ccd-overlap-first iterations={iterations} count={count} depth={depth:.5} pair=({i},{j}) impacts={} fallbacks={} a={:?} b={:?}",
                    run.impacts,
                    run.fallbacks,
                    run.states[i],
                    run.states[j],
                );
                break;
            }
        }
    }
}
