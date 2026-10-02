//! Public physical controls; all geometry is observed independently of engine manifolds.
use super::box_fixture_oracle;
use physics_engine::{approximate::Real, numeric::Scalar};

use physics_engine::{
    BodyId,
    approximate::{Body, CheckpointContext, Config, PositionCorrection, Shape, Vector as V, World},
};

const DT: Scalar = 1.0 / 60.0;
const TICKS: usize = 600;
const BOX: BodyId = BodyId(10);
const SUPPORT: BodyId = BodyId(2);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Case {
    Balanced,
    OffCenter,
    Removal,
}

fn fixture(case: Case, substeps: u8) -> World {
    let mut world = World::new(Config {
        substeps,
        velocity_iterations: 8,
        fixed_position_iterations: 2,
        position_correction: PositionCorrection::AdmittedContacts,
        ..Config::default()
    })
    .unwrap();
    for (id, half, position) in [
        (BodyId(1), V(500.0, 16.0, 500.0), V(0.0, -116.0, 0.0)),
        (SUPPORT, V(4.0, 16.0, 30.0), V(0.0, -16.0, 0.0)),
    ] {
        let mut body = Body::new(id, Shape::Box(half), position, 0.0);
        body.friction = 0.6;
        body.restitution = 0.0;
        world.add_body(body).unwrap();
    }
    let x = if case == Case::OffCenter { 10.0 } else { 0.0 };
    let mut body = Body::new(BOX, Shape::Box(V(18.0, 18.0, 18.0)), V(x, 18.0, 0.0), 2.0);
    body.friction = 0.6;
    body.restitution = 0.0;
    body.ccd = true;
    world.add_body(body).unwrap();
    world
}

#[derive(Debug, Default, PartialEq)]
struct Metrics {
    peak_penetration: Real,
    peak_speed: Real,
    peak_angular_speed: Real,
    peak_energy: Real,
    tail_speed: Real,
    tail_angular_speed: Real,
    support_departure: bool,
    sleep_transitions: u64,
    wake_transitions: u64,
    sleeping: bool,
    last_sleep_seconds: Real,
    contact_points: u64,
    constraint_visits: u64,
    position_tests: u64,
    active_substeps: u64,
}

fn observe(world: &World, case: Case, tail: bool, metrics: &mut Metrics) {
    let body = world.body(BOX).unwrap();
    assert!(body.position.finite());
    assert!(body.velocity.finite());
    assert!(body.angular_velocity.finite());
    assert!(body.orientation.finite());
    if body.is_sleeping() != metrics.sleeping {
        if body.is_sleeping() {
            metrics.sleep_transitions += 1;
            metrics.last_sleep_seconds = world.elapsed_seconds() as Real;
        } else {
            metrics.wake_transitions += 1;
        }
        metrics.sleeping = body.is_sleeping();
    }
    let speed = body.velocity.length();
    let angular = body.angular_velocity.length();
    let energy = body.kinetic_energy();
    assert!(energy.is_finite());
    metrics.peak_speed = metrics.peak_speed.max(speed);
    metrics.peak_angular_speed = metrics.peak_angular_speed.max(angular);
    metrics.peak_energy = metrics.peak_energy.max(energy);
    if tail {
        metrics.tail_speed = metrics.tail_speed.max(speed);
        metrics.tail_angular_speed = metrics.tail_angular_speed.max(angular);
    }
    for fixed in world.bodies().filter(|b| b.mass == 0.0) {
        metrics.peak_penetration = metrics
            .peak_penetration
            .max(box_fixture_oracle::box_overlap_depth(body, fixed));
    }
    metrics.support_departure |= body.position.1 < -36.0;
    if case == Case::Balanced {
        // Continuous support: sleeping or a late recovery cannot hide a departure.
        assert!((body.position - V(0.0, 18.0, 0.0)).length() <= 0.5);
    }
}

fn advance(world: &mut World, dt: Scalar, maximum_substeps: u32, metrics: &mut Metrics) {
    let before = world.elapsed_seconds();
    let report = world.step(dt).unwrap();
    let tolerance = 16.0 * f64::EPSILON * (before.abs() + dt);
    assert!((world.elapsed_seconds() - before - dt).abs() <= tolerance);
    assert!(report.substeps <= maximum_substeps);
    assert!(report.impulse_iterations <= 8 * u64::from(report.substeps));
    assert!(report.position.passes <= 2 * u64::from(report.substeps));
    assert!(report.contact_points <= 8 * u64::from(report.substeps));
    assert!(report.retired.is_empty());
    metrics.contact_points += report.contact_points;
    metrics.constraint_visits += report.convergence.constraint_visits;
    metrics.position_tests += report.position.contact_tests;
    metrics.active_substeps += u64::from(report.substeps);
}

fn run_case(case: Case) -> Measurements {
    let mut world = fixture(case, 4);
    let mut replay = fixture(case, 4);
    let mut observed = fixture(case, 1);
    let context = CheckpointContext {
        build: [0; 32],
        content: [231; 32],
    };
    let mut metrics = Metrics::default();
    let mut replay_metrics = Metrics::default();
    let mut substep_metrics = Metrics::default();
    for tick in 0..TICKS {
        if case == Case::Removal && tick == 120 {
            assert!(world.body(BOX).unwrap().is_sleeping());
            for w in [&mut world, &mut replay, &mut observed] {
                assert!(w.remove_body(SUPPORT).is_some());
                assert!(!w.body(BOX).unwrap().is_sleeping());
            }
        }
        advance(&mut world, DT, 4, &mut metrics);
        advance(&mut replay, DT, 4, &mut replay_metrics);
        let tail = tick >= 480;
        observe(&world, case, tail, &mut metrics);
        observe(&replay, case, tail, &mut replay_metrics);
        for _ in 0..4 {
            advance(&mut observed, DT / 4.0, 1, &mut substep_metrics);
            observe(&observed, case, tail, &mut substep_metrics);
        }
        assert_eq!(
            world.bodies().collect::<Vec<_>>(),
            observed.bodies().collect::<Vec<_>>(),
            "{case:?} physical-substep parity at tick {tick}"
        );
        assert_eq!(
            world.checkpoint(context).unwrap().to_bytes(),
            replay.checkpoint(context).unwrap().to_bytes(),
            "{case:?} same-target continuation replay at tick {tick}"
        );
    }
    assert_eq!(metrics, replay_metrics);
    assert!((world.elapsed_seconds() - 10.0).abs() < 1e-9);
    assert!((observed.elapsed_seconds() - 10.0).abs() < 1e-9);
    assert!(metrics.peak_penetration <= 0.5, "{case:?}: {metrics:?}");
    assert!(
        substep_metrics.peak_penetration <= 0.5,
        "{case:?}: {substep_metrics:?}"
    );
    assert!(metrics.contact_points > 0);
    assert!(
        metrics.sleeping,
        "{case:?} did not naturally settle: {metrics:?}"
    );
    assert!(metrics.tail_speed <= 1.0, "{case:?}: {metrics:?}");
    assert!(
        substep_metrics.tail_speed <= 1.0,
        "{case:?}: {substep_metrics:?}"
    );
    assert!(
        metrics.tail_angular_speed * 54.0 <= 1.0,
        "{case:?}: {metrics:?}"
    );
    assert!(
        substep_metrics.tail_angular_speed * 54.0 <= 1.0,
        "{case:?}: {substep_metrics:?}"
    );
    let body = world.body(BOX).unwrap();
    match case {
        Case::Balanced => {
            assert!(!metrics.support_departure);
            assert!(body.position.0.abs() <= 0.5);
            assert!((body.position.1 - 18.0).abs() <= 0.5);
        }
        Case::OffCenter => {
            assert!(metrics.support_departure);
            assert!(metrics.peak_angular_speed > 0.1);
            assert!(body.position.1 < -60.0);
        }
        Case::Removal => {
            assert!(metrics.support_departure);
            assert_eq!(metrics.wake_transitions, 1);
            assert!(metrics.sleep_transitions >= 2);
            assert!(body.position.1 < -60.0);
        }
    }
    let measurements = Measurements {
        ticks: metrics,
        substeps: substep_metrics,
    };
    println!(
        "NARROW_SUPPORT {{\"case\":\"{case:?}\",\"ticks\":{:?},\"physical_substeps\":{:?}}}",
        measurements.values(0).unwrap(),
        measurements.values(1).unwrap()
    );
    measurements
}

pub struct Measurements {
    ticks: Metrics,
    substeps: Metrics,
}

impl Measurements {
    /// Diagnostic bridge only. The bounded ten-second fixture keeps integer counters
    /// below 2^53, so their f64 transport is exact. No physics state crosses this seam.
    pub fn values(&self, cadence: u32) -> Option<[Real; 15]> {
        let m = match cadence {
            0 => &self.ticks,
            1 => &self.substeps,
            _ => return None,
        };
        Some([
            m.peak_penetration,
            m.peak_speed,
            m.peak_angular_speed,
            m.peak_energy,
            m.tail_speed,
            m.tail_angular_speed,
            Real::from(u8::from(m.support_departure)),
            m.sleep_transitions as Real,
            m.wake_transitions as Real,
            Real::from(u8::from(m.sleeping)),
            m.last_sleep_seconds,
            m.contact_points as Real,
            m.constraint_visits as Real,
            m.position_tests as Real,
            m.active_substeps as Real,
        ])
    }
}

pub fn run() -> [Measurements; 3] {
    [Case::Balanced, Case::OffCenter, Case::Removal].map(run_case)
}
