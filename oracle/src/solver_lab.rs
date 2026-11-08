//! Timed workloads of the #545 solver comparison: the named oracle scenarios and engine-only
//! piles. Shared by `bin/solver_lab.rs` and the WASM exports.

use physics_engine::approximate::{Body, Config, Quaternion, Shape, Vector, World};

use crate::{Rng, Side, Trace, all};

const NAMED: [&str; 5] = [
    "rubble-wake",
    "debris",
    "debris-town-tumble",
    "mmorpg-tick",
    "crowd",
];
const PILES: [(&str, usize); 4] = [
    ("pile-16", 16),
    ("pile-128", 128),
    ("pile-1024", 1024),
    ("pile-4096", 4096),
];
const SPARSE_STATIC: usize = 2000;
const PILE_TICKS: u32 = 120;
const DT: f64 = 1.0 / 30.0;

pub struct Sample {
    pub us_per_tick: f64,
    /// Microseconds per tick spent in rows, the whole solve and a variant's prepare.
    pub phases: [f64; 3],
}

pub fn names() -> Vec<String> {
    NAMED
        .iter()
        .chain(PILES.iter().map(|(name, _)| name))
        .chain(std::iter::once(&"sparse-4"))
        .map(|name| name.to_string())
        .collect()
}

fn config() -> Config {
    // MMORPG's exact-continuation debris settings, as `rubble-wake`.
    Config {
        gravity: Vector(0.0, -9.0, 0.0),
        substeps: 2,
        velocity_iterations: 4,
        warm_start: false,
        ..Config::default()
    }
}

fn floor(world: &mut World) {
    let mut floor = Body::keyed(1, Shape::HalfSpace, Vector::ZERO, 0.0);
    floor.friction = 0.8;
    world.add_body(floor).unwrap();
}

fn dynamic_box(key: u64, position: Vector, rng: &mut Rng) -> Body {
    let half = Vector(
        rng.range(0.18, 0.3),
        rng.range(0.12, 0.24),
        rng.range(0.18, 0.3),
    );
    let mass = 600.0 * 8.0 * half.0 * half.1 * half.2;
    let mut body = Body::keyed(key, Shape::Box(half), position, mass);
    let (x, z, w) = (rng.range(-0.1, 0.1), rng.range(-0.1, 0.1), 1.0);
    let n = (x * x + z * z + w * w).sqrt();
    body.orientation = Quaternion(x / n, 0.0, z / n, w / n);
    body.friction = 0.8;
    body.linear_damping = 0.2;
    body.angular_damping = 0.8;
    body.sleep_allowed = false;
    body
}

/// `count` boxes on a tight jittered lattice collapse into one connected heap.
fn pile(count: usize) -> World {
    let mut world = World::new(config()).unwrap();
    floor(&mut world);
    let mut rng = Rng(0x9E37_79B9_7F4A_7C15);
    let side = (count as f32).cbrt().ceil() as usize;
    let offset = side as f32 * 0.35;
    for k in 0..count {
        let (x, y, z) = (k % side, k / (side * side), (k / side) % side);
        let position = Vector(
            x as f32 * 0.7 - offset + rng.range(-0.04, 0.04),
            y as f32 * 0.55 + 0.4,
            z as f32 * 0.7 - offset + rng.range(-0.04, 0.04),
        );
        world
            .add_body(dynamic_box(1000 + k as u64, position, &mut rng))
            .unwrap();
    }
    world
}

/// Four boxes solve while the world holds many bodies that make no rows.
fn sparse() -> World {
    let mut world = World::new(config()).unwrap();
    floor(&mut world);
    let mut rng = Rng(0x2545_F491_4F6C_DD1D);
    for k in 0..SPARSE_STATIC {
        let position = Vector(
            40.0 + (k % 50) as f32 * 2.0,
            1.0 + (k / 50) as f32 * 2.0,
            40.0,
        );
        let body = Body::keyed(
            100_000 + k as u64,
            Shape::Box(Vector(0.5, 0.5, 0.5)),
            position,
            0.0,
        );
        world.add_body(body).unwrap();
    }
    for k in 0..4 {
        let position = Vector(k as f32 * 1.5, 0.5, 0.0);
        world
            .add_body(dynamic_box(1000 + k as u64, position, &mut rng))
            .unwrap();
    }
    world
}

/// One timed run of workload `index`, built untimed; `now` is the host clock in microseconds.
pub fn run(index: usize, now: fn() -> f64) -> Sample {
    #[cfg(feature = "phase-timing")]
    {
        physics_engine::approximate::phase_timing::set_clock(now);
        physics_engine::approximate::phase_timing::take();
    }
    let (elapsed, ticks) = if index < NAMED.len() {
        let scenarios = all();
        let scenario = scenarios
            .iter()
            .find(|s| s.name == NAMED[index])
            .expect("named scenario is registered");
        let mut sim = scenario.build(Side::Engine);
        let mut trace = Trace::default();
        trace.0.reserve(1 << 16);
        let start = now();
        scenario.run(&mut *sim, &mut trace);
        let elapsed = now() - start;
        core::hint::black_box(&trace);
        (elapsed, scenario.ticks)
    } else {
        let mut world = match PILES.get(index - NAMED.len()) {
            Some(&(_, count)) => pile(count),
            None => sparse(),
        };
        let start = now();
        for _ in 0..PILE_TICKS {
            world.step(DT).unwrap();
        }
        let elapsed = now() - start;
        core::hint::black_box(&world);
        (elapsed, PILE_TICKS)
    };
    let ticks = f64::from(ticks);
    #[cfg(feature = "phase-timing")]
    let phases = physics_engine::approximate::phase_timing::take().map(|p| p / ticks);
    #[cfg(not(feature = "phase-timing"))]
    let phases = [0.0; 3];
    Sample {
        us_per_tick: elapsed / ticks,
        phases,
    }
}
