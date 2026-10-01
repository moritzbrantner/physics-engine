//! Analytic translating-support controls; no character or rotational platform policy.
use physics_engine::{
    BodyId,
    approximate::{Body, CheckpointContext, Config, Quaternion, Shape, Vector as V, World},
};
const H: f64 = 1.0 / 240.0;
const COMMAND_STEP: u32 = 480;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Case {
    Carry,
    Departure,
    Removal,
}

fn fixture(ids: [u64; 2], substeps: u8) -> World {
    let mut w = World::new(Config {
        gravity: V(0.0, -10.0, 0.0),
        substeps,
        convergence: None,
        ..Config::default()
    })
    .unwrap();
    let mut platform = Body::new(BodyId(ids[0]), Shape::Box(V(4.0, 0.5, 4.0)), V::ZERO, 1.0);
    platform.external = true;
    platform.velocity = V(2.0, 0.0, 0.0);
    platform.rotation_locked = true;
    platform.sleep_allowed = false;
    w.add_body(platform).unwrap();
    let mut rider = Body::new(
        BodyId(ids[1]),
        Shape::Box(V(1.0, 1.0, 1.0)),
        V(0.0, 1.5, 0.0),
        2.0,
    );
    rider.rotation_locked = true;
    w.add_body(rider).unwrap();
    w
}

fn command(w: &mut World, case: Case, ids: [u64; 2], m: &mut Metrics) {
    match case {
        Case::Carry => (),
        Case::Departure => {
            w.set_velocity(BodyId(ids[1]), V(2.0, 3.0, 0.0)).unwrap();
            let rider = w.body(BodyId(ids[1])).unwrap();
            // Capture the instantaneous imposed input as well as later physical steps.
            let energy = rider.velocity.dot(rider.velocity);
            m.peak_energy = m.peak_energy.max(energy);
            m.peak_speed = m.peak_speed.max(rider.velocity.length());
            assert!(energy + 20.0 * rider.position.1 <= 39.0 + m.external_work + 1e-9);
        }
        Case::Removal => {
            assert!(w.remove_body(BodyId(ids[0])).unwrap().external);
            assert!(!w.has_support(BodyId(ids[1])));
        }
    }
}

#[derive(Debug, Default, PartialEq)]
struct Metrics {
    peak_penetration: f64,
    peak_speed: f64,
    peak_energy: f64,
    external_work: f64,
    max_motion_error: f64,
    peak_departure_gap: f64,
    max_ballistic_error: f64,
    support_samples: u64,
    departure_samples: u64,
    contact_points: u64,
    constraint_visits: u64,
    woken_bodies: u64,
    sleep_transitions: u64,
    wake_transitions: u64,
    last_velocity: V,
    last_sleeping: bool,
    final_y: f64,
    final_vy: f64,
}

fn advance(w: &mut World, substeps: u32, m: &mut Metrics) {
    let before = w.elapsed_seconds();
    let dt = H * f64::from(substeps);
    let r = w.step(dt).unwrap();
    assert!((w.elapsed_seconds() - before - dt).abs() <= 16.0 * f64::EPSILON * (before.abs() + dt));
    assert_eq!(r.substeps, substeps);
    assert!(r.impulse_iterations <= 8 * u64::from(substeps));
    assert!(r.contact_points <= 4 * u64::from(substeps));
    assert!(r.convergence.constraint_visits <= 8 * r.contact_points);
    assert_eq!(r.position.passes, 0);
    assert!(r.retired.is_empty());
    m.contact_points += r.contact_points;
    m.constraint_visits += r.convergence.constraint_visits;
    m.woken_bodies += r.woken_bodies;
}

fn observe(w: &World, case: Case, ids: [u64; 2], step: u32, m: &mut Metrics) {
    let rider = w.body(BodyId(ids[1])).unwrap();
    assert_eq!(rider.mass, 2.0);
    assert_eq!(rider.shape, Shape::Box(V(1.0, 1.0, 1.0)));
    assert!(rider.position.finite() && rider.velocity.finite());
    assert_eq!(rider.orientation, Quaternion::IDENTITY);
    assert_eq!(rider.angular_velocity, V::ZERO);
    let n = f64::from(step);
    let accelerated = n.min(80.0);
    let expected_x = 3.0 * H * H * accelerated * (accelerated + 1.0) + 2.0 * H * (n - accelerated);
    let expected_vx = (6.0 * H * n).min(2.0);
    let mut expected_y = 1.5;
    let mut expected_vy = 0.0;
    if step > COMMAND_STEP && case != Case::Carry {
        let flight = f64::from(step - COMMAND_STEP);
        let vy = if case == Case::Departure { 3.0 } else { 0.0 };
        expected_y += vy * flight * H - 5.0 * H * H * flight * (flight + 1.0);
        expected_vy = vy - 10.0 * H * flight;
        if case == Case::Departure && flight >= 144.0 {
            expected_y = 1.5;
            expected_vy = 0.0;
        }
        let error = (rider.position.1 - expected_y)
            .abs()
            .max((rider.velocity.1 - expected_vy).abs());
        m.max_ballistic_error = m.max_ballistic_error.max(error);
        assert!(
            error <= 1e-9,
            "{case:?} ids={ids:?} step={step} y/vy {:?}, expected {expected_y}/{expected_vy}",
            (rider.position.1, rider.velocity.1)
        );
    }
    let motion_error = (rider.position - V(expected_x, expected_y, 0.0))
        .length()
        .max((rider.velocity - V(expected_vx, expected_vy, 0.0)).length());
    m.max_motion_error = m.max_motion_error.max(motion_error);
    assert!(
        motion_error <= 1e-9,
        "{case:?} ids={ids:?} step={step}: {motion_error}"
    );
    m.external_work += 4.0 * (rider.velocity.0 - m.last_velocity.0);
    let energy = rider.velocity.dot(rider.velocity);
    m.peak_energy = m.peak_energy.max(energy);
    m.peak_speed = m.peak_speed.max(rider.velocity.length());
    let departure_input = if case == Case::Departure && step > COMMAND_STEP {
        9.0
    } else {
        0.0
    };
    let mechanical = energy + 20.0 * rider.position.1;
    assert!(
        mechanical.is_finite() && mechanical <= 30.0 + m.external_work + departure_input + 1e-9
    );
    let supported = if let Some(platform) = w.body(BodyId(ids[0])) {
        assert!(platform.external && !platform.is_sleeping());
        assert_eq!(platform.mass, 1.0);
        assert_eq!(platform.shape, Shape::Box(V(4.0, 0.5, 4.0)));
        assert_eq!(platform.velocity, V(2.0, 0.0, 0.0));
        assert_eq!(platform.orientation, Quaternion::IDENTITY);
        assert_eq!(platform.angular_velocity, V::ZERO);
        let error = (platform.position - V(2.0 * H * n, 0.0, 0.0)).length();
        m.max_motion_error = m.max_motion_error.max(error);
        assert!(error <= 1e-9);
        let gap = rider.position.1 - platform.position.1 - 1.5;
        let horizontal = (rider.position.0 - platform.position.0).abs() < 5.0
            && (rider.position.2 - platform.position.2).abs() < 5.0;
        if horizontal {
            m.peak_penetration = m.peak_penetration.max((-gap).max(0.0));
            m.peak_departure_gap = m.peak_departure_gap.max(gap.max(0.0));
            assert!((-gap).max(0.0) <= 0.02 + 1e-9);
        }
        horizontal && gap.abs() <= 1e-9
    } else {
        false
    };
    if supported {
        m.support_samples += 1;
    } else {
        m.departure_samples += 1;
    }
    if case == Case::Carry || step <= COMMAND_STEP {
        assert!(supported && w.has_support(BodyId(ids[1])));
    }
    if case == Case::Departure && step > COMMAND_STEP {
        let gap = rider.position.1 - 1.5;
        // The query reads admitted pre-integration rows. Exclude their two-slop
        // margin and one physical step of the maximum imposed upward speed.
        let airborne = 2.0 * w.config().contact_slop + 3.0 * H + 1e-9;
        if gap > airborne {
            assert!(
                !w.has_support(BodyId(ids[1])),
                "stale airborne support at step {step}"
            );
        }
        if supported && rider.velocity.1.abs() <= 1e-9 {
            assert!(
                w.has_support(BodyId(ids[1])),
                "missing settled recontact at step {step}"
            );
        }
    }
    let sleeping = rider.is_sleeping();
    m.sleep_transitions += u64::from(!m.last_sleeping && sleeping);
    m.wake_transitions += u64::from(m.last_sleeping && !sleeping);
    assert!(
        !sleeping,
        "acceleration to speed 2 prevents a complete natural quiet window"
    );
    m.last_sleeping = sleeping;
    m.last_velocity = rider.velocity;
    m.final_y = rider.position.1;
    m.final_vy = rider.velocity.1;
}

pub struct Measurements {
    ticks: Metrics,
    physical: Metrics,
    total_visits: u64,
    elapsed: [f64; 2],
}
impl Measurements {
    /// Counts remain integers internally and are below 2^53 in this bounded fixture.
    pub fn values(&self, cadence: u32) -> Option<[f64; 19]> {
        let m = match cadence {
            0 => &self.ticks,
            1 => &self.physical,
            _ => return None,
        };
        Some([
            m.peak_penetration,
            m.peak_speed,
            m.peak_energy,
            m.external_work,
            m.max_motion_error,
            m.peak_departure_gap,
            m.max_ballistic_error,
            m.support_samples as f64,
            m.departure_samples as f64,
            m.contact_points as f64,
            m.constraint_visits as f64,
            m.woken_bodies as f64,
            m.sleep_transitions as f64,
            m.wake_transitions as f64,
            m.final_y,
            m.final_vy,
            f64::from(u8::from(m.last_sleeping)),
            self.total_visits as f64,
            self.elapsed[cadence as usize],
        ])
    }
}

fn run_case(case: Case, ids: [u64; 2]) -> Measurements {
    let mut w = fixture(ids, 1);
    let mut replay = fixture(ids, 1);
    let mut ticks = fixture(ids, 4);
    let mut physical = Metrics::default();
    let mut repeated = Metrics::default();
    let mut tick_metrics = Metrics::default();
    let context = CheckpointContext {
        build: [0; 32],
        content: [242; 32],
    };
    for tick in 0..240 {
        if tick == 120 {
            for (world, metrics) in [
                (&mut w, &mut physical),
                (&mut replay, &mut repeated),
                (&mut ticks, &mut tick_metrics),
            ] {
                command(world, case, ids, metrics);
            }
        }
        for part in 1..=4 {
            let step = tick * 4 + part;
            advance(&mut w, 1, &mut physical);
            advance(&mut replay, 1, &mut repeated);
            observe(&w, case, ids, step, &mut physical);
            observe(&replay, case, ids, step, &mut repeated);
            assert_eq!(
                w.checkpoint(context).unwrap().to_bytes(),
                replay.checkpoint(context).unwrap().to_bytes()
            );
        }
        advance(&mut ticks, 4, &mut tick_metrics);
        observe(&ticks, case, ids, (tick + 1) * 4, &mut tick_metrics);
        assert_eq!(
            w.bodies().collect::<Vec<_>>(),
            ticks.bodies().collect::<Vec<_>>()
        );
    }
    assert_eq!(physical, repeated);
    for world in [&w, &replay, &ticks] {
        assert!((world.elapsed_seconds() - 4.0).abs() <= 1e-9);
    }
    assert!((physical.external_work - 8.0).abs() <= 1e-9);
    match case {
        Case::Carry => assert_eq!(physical.departure_samples, 0),
        Case::Departure => {
            assert!(physical.departure_samples > 0);
            assert!(physical.peak_departure_gap > 0.3);
            assert!((physical.final_y - 1.5).abs() <= 1e-9);
        }
        Case::Removal => {
            assert_eq!(physical.support_samples, 480);
            assert_eq!(physical.departure_samples, 480);
            assert!(physical.final_y < 0.0);
        }
    }
    let total_visits =
        physical.constraint_visits + repeated.constraint_visits + tick_metrics.constraint_visits;
    let result = Measurements {
        ticks: tick_metrics,
        physical,
        total_visits,
        elapsed: [ticks.elapsed_seconds(), w.elapsed_seconds()],
    };
    println!(
        "MOVING_SUPPORT {{\"case\":\"{case:?}\",\"ids\":{ids:?},\"ticks\":{:?},\"physical_substeps\":{:?}}}",
        result.values(0).unwrap(),
        result.values(1).unwrap()
    );
    result
}

pub fn run() -> [Measurements; 6] {
    std::array::from_fn(|index| {
        run_case(
            [Case::Carry, Case::Departure, Case::Removal][index / 2],
            if index % 2 == 0 { [1, 2] } else { [2, 1] },
        )
    })
}
