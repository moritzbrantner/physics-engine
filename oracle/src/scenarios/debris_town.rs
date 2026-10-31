//! C18 debris in a town (#412, #424, #426, mmorpg#288, mmorpg#442): MMORPG-zone-sized static
//! geometry with live debris. A town of 1,200 fixed 2 m house boxes on a 4 m grid stands on the
//! floor plane (a halfspace, as MMORPG's floor over removed terrain cells). One world per side,
//! stepped once per 1/30 s tick with the MMORPG adapter's crate step (two substeps of one pass)
//! and sleeping on.
//!
//! `debris-town`: 32 upright debris crates (rotations locked, as MMORPG's crates) rest on roofs
//! and in the lanes across the town; every 4 ticks one is kicked (2 m/s along its own heading,
//! as a hit or a push would), slides and falls asleep again, so a few crates are awake on most
//! ticks among 1,200 resting boxes. 16 free-rotating debris pieces (linear damping 0.2, angular
//! 0.8) drop a few centimetres onto the plane in the lanes, sliding and slowly tipping, and
//! must settle and fall asleep there as they do on a box (#424).
//!
//! `debris-town-tumble` (#426): no crates; 48 free-rotating pieces are thrown from 0.4–1.2 m
//! above the lanes at up to 2.25 m/s, spinning at up to 6 rad/s. They land on the plane at
//! speed, tumble, settle and fall asleep: the landing path of free-rotating boxes (rotating
//! box–plane contacts, the solve, integration) against Rapier.
//!
//! The town makes the step's cost follow its static size unless the broadphase visits only
//! what awake bodies reach, as Rapier's does. Rapier keeps one persistent world, the cheapest
//! Rapier pattern, as the perf baseline. Debris–debris contacts and heightfields are `debris`'s.

use crate::{Obs, Scenario, Trace};

const TICKS: u32 = 180;
const DT: f32 = 1.0 / 30.0;
const GRAVITY: f32 = 9.0;
/// Houses: `COLS × ROWS` boxes of half extent `HOUSE`, `SPACING` apart.
const COLS: usize = 40;
const ROWS: usize = 30;
const HOUSE: f32 = 1.0;
const SPACING: f32 = 4.0;
const DENSITY: f32 = 600.0;
const KICK: f32 = 2.0;
/// Ticks between kicks: piece `k` is kicked at tick `KICK_EVERY * k`.
const KICK_EVERY: u32 = 4;
const LINEAR_DAMPING: f32 = 0.2;
const ANGULAR_DAMPING: f32 = 0.8;
/// Every piece rests within this distance of where Rapier's does.
const REST_TOLERANCE: f32 = 0.05;
/// The engine's metre contact slop: a settler resting within it of Rapier's height rests at
/// Rapier's depth.
const SLOP: f32 = 0.005;
/// Every settler stops moving within this many ticks (1 s) after Rapier's does.
const SLEEP_GRACE: usize = 30;
/// Tumbling pieces land chaotically, so their rests are compared as `debris`'s are: at least
/// this share within `TUMBLE_TOLERANCE` of Rapier's, every one within `TUMBLE_CAP`.
const TUMBLE_SHARE: f32 = 0.9;
const TUMBLE_TOLERANCE: f32 = 0.5;
const TUMBLE_CAP: f32 = 1.5;

/// One variant of the scenario.
pub struct Variant {
    name: &'static str,
    /// Kicked rotation-locked crates.
    pieces: usize,
    /// Free-rotating pieces dropped onto the plane.
    settlers: usize,
    /// Settlers are thrown from height and tumble rather than dropped a few centimetres.
    tumble: bool,
}

const TOWN: Variant = Variant {
    name: "debris-town",
    pieces: 32,
    settlers: 16,
    tumble: false,
};
const TUMBLE: Variant = Variant {
    name: "debris-town-tumble",
    pieces: 0,
    settlers: 48,
    tumble: true,
};

impl Variant {
    /// Observations per tick: crates, then settlers.
    fn stride(&self) -> usize {
        self.pieces + self.settlers
    }
}

fn house(i: usize, j: usize) -> [f32; 3] {
    [i as f32 * SPACING, HOUSE, j as f32 * SPACING]
}

/// Piece `k`: half extents, resting start position and kick velocity. Even pieces rest on a
/// roof, odd ones in the lane east of a house.
fn piece(k: usize) -> ([f32; 3], [f32; 3], [f32; 3]) {
    let half = if k.is_multiple_of(3) {
        [0.3, 0.15, 0.2]
    } else {
        [0.2, 0.1, 0.25]
    };
    let h = house((k * 7) % COLS, (k * 11) % ROWS);
    let (lane, ground) = if k.is_multiple_of(2) {
        (0.0, 2.0 * HOUSE)
    } else {
        (SPACING * 0.5, 0.0)
    };
    let start = [h[0] + lane, ground + half[1], h[2]];
    let heading = 0.7 * k as f32;
    (
        half,
        start,
        [KICK * heading.cos(), 0.0, KICK * heading.sin()],
    )
}

/// Settler `k`: half extents, start position (in the lane north of a house), velocity and
/// spin. Dropped, it starts 2 to 6 cm above the plane, sliding and tipping about a horizontal
/// axis across the slide; the second size is #424's box (half height 0.1 m). Tumbling, it is
/// thrown from 0.4 to 1.2 m, faster, spinning about a tilted axis.
fn settler(variant: &Variant, k: usize) -> ([f32; 3], [f32; 3], [f32; 3], [f32; 3]) {
    let half = [[0.3, 0.15, 0.2], [0.2, 0.1, 0.25], [0.33, 0.03, 0.33]][k % 3];
    let h = house((k * 5 + 3) % COLS, (k * 13 + 1) % ROWS);
    let heading = 0.7 * k as f32;
    let (drop, speed, tip, yaw) = if variant.tumble {
        (
            0.4 + 0.2 * (k % 5) as f32,
            0.75 + 0.5 * (k % 4) as f32,
            2.0 + 2.0 * (k % 3) as f32,
            0.25 + 0.25 * (k % 2) as f32,
        )
    } else {
        (
            0.02 + 0.01 * (k % 5) as f32,
            0.2 + 0.05 * (k % 4) as f32,
            0.3 + 0.1 * (k % 3) as f32,
            0.0,
        )
    };
    let start = [h[0], half[1] + drop, h[2] + SPACING * 0.5];
    (
        half,
        start,
        [speed * heading.cos(), 0.0, speed * heading.sin()],
        [tip * heading.sin(), tip * yaw, -tip * heading.cos()],
    )
}

pub fn scenario() -> Scenario {
    Scenario {
        name: TOWN.name,
        capabilities: "C18",
        ticks: TICKS,
        #[cfg(feature = "engine")]
        engine: |storage| Box::new(engine::Sim::new(&TOWN, storage)),
        #[cfg(feature = "rapier")]
        rapier: || Box::new(rapier::Sim::new(&TOWN)),
        #[cfg(feature = "rapier")]
        rapier_raw: None,
        check: |e, r| check(&TOWN, e, r),
    }
}

pub fn scenario_tumble() -> Scenario {
    Scenario {
        name: TUMBLE.name,
        capabilities: "C18",
        ticks: TICKS,
        #[cfg(feature = "engine")]
        engine: |storage| Box::new(engine::Sim::new(&TUMBLE, storage)),
        #[cfg(feature = "rapier")]
        rapier: || Box::new(rapier::Sim::new(&TUMBLE)),
        #[cfg(feature = "rapier")]
        rapier_raw: None,
        check: |e, r| check(&TUMBLE, e, r),
    }
}

fn at(variant: &Variant, t: &Trace, tick: usize, k: usize) -> [f32; 3] {
    match t.0[tick * variant.stride() + k] {
        Obs::Position(p) => p,
        _ => [f32::NAN; 3],
    }
}

/// The first tick from which observation `k` no longer moves to the end of the run.
fn still_from(variant: &Variant, t: &Trace, k: usize) -> usize {
    (1..TICKS as usize)
        .rev()
        .find(|&tick| {
            (0..3)
                .any(|a| (at(variant, t, tick, k)[a] - at(variant, t, tick - 1, k)[a]).abs() > 1e-6)
        })
        .unwrap_or(0)
}

/// Per tick: every crate's position, then every settler's.
fn check(variant: &Variant, engine: &Trace, rapier: &Trace) -> Result<(), String> {
    if engine.0.len() != rapier.0.len() || engine.0.len() != variant.stride() * TICKS as usize {
        return Err("trace lengths differ".into());
    }
    let at = |t: &Trace, tick: usize, k: usize| at(variant, t, tick, k);
    let end = TICKS as usize - 1;
    for k in 0..variant.pieces {
        let (_, start, kick) = piece(k);
        for tick in 0..TICKS as usize {
            let p = at(engine, tick, k);
            // Flat on its roof or the ground, never through it.
            if (p[1] - start[1]).abs() > 0.01 {
                return Err(format!("piece {k} left its support at tick {tick}: {p:?}"));
            }
        }
        let (p, q, r) = (
            at(engine, end, k),
            at(engine, end - 5, k),
            at(rapier, end, k),
        );
        if (0..3).any(|a| (p[a] - q[a]).abs() > 1e-4) {
            return Err(format!("piece {k} still moving: {q:?} -> {p:?}"));
        }
        // Slid along its kick, and rests where Rapier's does.
        let slide = [p[0] - start[0], p[2] - start[2]];
        if slide[0] * kick[0] + slide[1] * kick[2] <= 0.0 {
            return Err(format!("piece {k} did not slide: {start:?} -> {p:?}"));
        }
        let d = (0..3).map(|a| (p[a] - r[a]).powi(2)).sum::<f32>().sqrt();
        if d > REST_TOLERANCE {
            return Err(format!(
                "piece {k} rests {d:.3} m from Rapier's: {p:?} vs {r:?}"
            ));
        }
    }
    // Settlers fall asleep (stop moving) no later than a second after Rapier's. Dropped, they
    // rest at Rapier's depth within 5 cm of Rapier's; tumbling (some hit a house wall), above
    // the plane and, as `debris`, mostly near Rapier's.
    // A tumble's landing is chaotic (a piece balanced on an edge may rock a while before it
    // tips either way), so tumbling settlers stop within a second of Rapier's last one.
    let last = if variant.tumble {
        (0..variant.settlers)
            .map(|s| still_from(variant, rapier, variant.pieces + s))
            .max()
            .unwrap_or(0)
    } else {
        0
    };
    let mut near = 0;
    for s in 0..variant.settlers {
        let k = variant.pieces + s;
        let (p, r) = (at(engine, end, k), at(rapier, end, k));
        let (still, rapier_still) = (
            still_from(variant, engine, k),
            still_from(variant, rapier, k).max(last),
        );
        if rapier_still + SLEEP_GRACE >= end {
            return Err(format!(
                "Rapier's settler {s} did not settle (tick {rapier_still})"
            ));
        }
        if still > rapier_still + SLEEP_GRACE {
            return Err(format!(
                "settler {s} still moving at tick {still}, Rapier's still from {rapier_still}: {p:?}"
            ));
        }
        let d = ((p[0] - r[0]).powi(2) + (p[2] - r[2]).powi(2)).sqrt();
        if variant.tumble {
            // On a face, or leaning on a house wall, but never sunk into the plane.
            let (half, ..) = settler(variant, s);
            let thinnest = half.iter().copied().fold(f32::INFINITY, f32::min);
            if p[1] < thinnest - SLOP {
                return Err(format!(
                    "settler {s} sinks into the plane: {p:?} (half extents {half:?})"
                ));
            }
            if d > TUMBLE_CAP {
                return Err(format!(
                    "settler {s} rests {d:.3} m from Rapier's: {p:?} vs {r:?}"
                ));
            }
            near += usize::from(d <= TUMBLE_TOLERANCE);
            continue;
        }
        if (p[1] - r[1]).abs() > SLOP {
            return Err(format!(
                "settler {s} rests {:.4} m from Rapier's height: {p:?} vs {r:?}",
                p[1] - r[1]
            ));
        }
        if d > REST_TOLERANCE {
            return Err(format!(
                "settler {s} rests {d:.3} m from Rapier's: {p:?} vs {r:?}"
            ));
        }
    }
    if variant.tumble && (near as f32) < TUMBLE_SHARE * variant.settlers as f32 {
        return Err(format!(
            "only {near} of {} settlers rest within {TUMBLE_TOLERANCE} m of Rapier's",
            variant.settlers
        ));
    }
    Ok(())
}

#[cfg(feature = "engine")]
mod engine {
    use super::*;
    use physics_engine::{
        BodyId,
        approximate::{Body, Config, Shape, Vector, World},
    };

    use crate::kcc::engine::v;

    pub struct Sim {
        world: World,
        pieces: Vec<BodyId>,
        settlers: Vec<BodyId>,
    }
    impl Sim {
        pub fn new(variant: &Variant, storage: physics_engine::approximate::BodyStorage) -> Self {
            let mut world = World::new(Config {
                body_storage: storage,
                gravity: Vector(0.0, -GRAVITY, 0.0),
                // As in `props`: two substeps of one pass, the MMORPG adapter's crate step.
                substeps: 2,
                velocity_iterations: 1,
                length_unit: 1.0,
                ..Config::default()
            })
            .unwrap();
            world
                .add_body(Body::new(Shape::HalfSpace, Vector::ZERO, 0.0))
                .unwrap();
            for j in 0..ROWS {
                for i in 0..COLS {
                    world
                        .add_body(Body::new(Shape::Box(v([HOUSE; 3])), v(house(i, j)), 0.0))
                        .unwrap();
                }
            }
            let pieces = (0..variant.pieces)
                .map(|k| {
                    let (half, start, _) = piece(k);
                    let mass = DENSITY * 8.0 * half[0] * half[1] * half[2];
                    let mut body = Body::new(Shape::Box(v(half)), v(start), mass);
                    body.rotation_locked = true;
                    world.add_body(body).unwrap()
                })
                .collect();
            let settlers = (0..variant.settlers)
                .map(|k| {
                    let (half, start, velocity, spin) = settler(variant, k);
                    let mass = DENSITY * 8.0 * half[0] * half[1] * half[2];
                    let mut body = Body::new(Shape::Box(v(half)), v(start), mass);
                    body.velocity = v(velocity);
                    body.angular_velocity = v(spin);
                    body.linear_damping = LINEAR_DAMPING;
                    body.angular_damping = ANGULAR_DAMPING;
                    world.add_body(body).unwrap()
                })
                .collect();
            Self {
                world,
                pieces,
                settlers,
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            if tick.is_multiple_of(KICK_EVERY) && ((tick / KICK_EVERY) as usize) < self.pieces.len()
            {
                let k = (tick / KICK_EVERY) as usize;
                self.world
                    .set_velocity(self.pieces[k], v(piece(k).2))
                    .unwrap();
            }
            self.world.step(f64::from(DT)).unwrap();
            for &id in self.pieces.iter().chain(&self.settlers) {
                let p = self.world.body(id).unwrap().position;
                trace.0.push(Obs::Position([p.0, p.1, p.2]));
            }
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::*;
    use rapier3d::prelude::*;

    use crate::kcc::rapier::v;

    pub struct Sim {
        world: PhysicsWorld,
        pieces: Vec<RigidBodyHandle>,
        settlers: Vec<RigidBodyHandle>,
    }

    impl Sim {
        pub fn new(variant: &Variant) -> Self {
            let mut world = PhysicsWorld::new();
            world.gravity = Vector::new(0.0, -GRAVITY, 0.0);
            world.integration_parameters.dt = DT;
            world
                .colliders
                .insert(crate::kcc::rapier::halfspace().build());
            for j in 0..ROWS {
                for i in 0..COLS {
                    world.colliders.insert(
                        ColliderBuilder::cuboid(HOUSE, HOUSE, HOUSE)
                            .translation(v(house(i, j)))
                            .build(),
                    );
                }
            }
            let mut dynamic = |body: RigidBodyBuilder, half: [f32; 3]| {
                let body = world.bodies.insert(body.build());
                world.colliders.insert_with_parent(
                    ColliderBuilder::cuboid(half[0], half[1], half[2])
                        .density(DENSITY)
                        .build(),
                    body,
                    &mut world.bodies,
                );
                body
            };
            let pieces = (0..variant.pieces)
                .map(|k| {
                    let (half, start, _) = piece(k);
                    dynamic(
                        RigidBodyBuilder::dynamic()
                            .translation(v(start))
                            .lock_rotations(),
                        half,
                    )
                })
                .collect();
            let settlers = (0..variant.settlers)
                .map(|k| {
                    let (half, start, velocity, spin) = settler(variant, k);
                    dynamic(
                        RigidBodyBuilder::dynamic()
                            .translation(v(start))
                            .linvel(v(velocity))
                            .angvel(v(spin))
                            .linear_damping(LINEAR_DAMPING)
                            .angular_damping(ANGULAR_DAMPING),
                        half,
                    )
                })
                .collect();
            Self {
                world,
                pieces,
                settlers,
            }
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            if tick.is_multiple_of(KICK_EVERY) && ((tick / KICK_EVERY) as usize) < self.pieces.len()
            {
                let k = (tick / KICK_EVERY) as usize;
                self.world.bodies[self.pieces[k]].set_linvel(v(piece(k).2), true);
            }
            self.world.step();
            for &h in self.pieces.iter().chain(&self.settlers) {
                let p = self.world.bodies[h].translation();
                trace.0.push(Obs::Position([p.x, p.y, p.z]));
            }
        }
    }
}
