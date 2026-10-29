//! C18 debris in a town (#412, mmorpg#288, mmorpg#442): MMORPG-zone-sized static geometry with
//! live debris. A town of 1,200 fixed 2 m house boxes on a 4 m grid stands on a fixed ground
//! slab. 32 upright debris crates (rotations locked, as MMORPG's crates) rest on roofs and in
//! the lanes across the town; every 4 ticks one is kicked (2 m/s along its own heading, as a hit
//! or a push would), slides and falls asleep again, so a few crates are awake on most ticks
//! among 1,200 resting boxes. One world per side, stepped once per 1/30 s tick with the MMORPG
//! adapter's crate step (two substeps of one pass) and sleeping on.
//!
//! The town makes the step's cost follow its static size unless the broadphase visits only
//! what awake bodies reach, as Rapier's does. Rapier keeps one persistent world, the cheapest
//! Rapier pattern, as the perf baseline. Landing and tumbling are the `debris` scenario's. The
//! ground is a box, not the floor plane: boxes resting on a halfspace never fall asleep in the
//! engine (#424).

use crate::{Obs, Scenario, Trace};

const TICKS: u32 = 180;
const DT: f32 = 1.0 / 30.0;
const GRAVITY: f32 = 9.0;
/// Houses: `COLS × ROWS` boxes of half extent `HOUSE`, `SPACING` apart.
const COLS: usize = 40;
const ROWS: usize = 30;
const HOUSE: f32 = 1.0;
const SPACING: f32 = 4.0;
const PIECES: usize = 32;
const DENSITY: f32 = 600.0;
const KICK: f32 = 2.0;
/// Ticks between kicks: piece `k` is kicked at tick `KICK_EVERY * k`.
const KICK_EVERY: u32 = 4;
/// Every piece rests within this distance of where Rapier's does.
const REST_TOLERANCE: f32 = 0.05;

fn house(i: usize, j: usize) -> [f32; 3] {
    [i as f32 * SPACING, HOUSE, j as f32 * SPACING]
}

/// The ground slab (centre, half extents): 1 m thick, top at 0, under the whole town.
const GROUND: ([f32; 3], [f32; 3]) = (
    [
        (COLS - 1) as f32 * SPACING * 0.5,
        -0.5,
        (ROWS - 1) as f32 * SPACING * 0.5,
    ],
    [
        COLS as f32 * SPACING * 0.5 + 4.0,
        0.5,
        ROWS as f32 * SPACING * 0.5 + 4.0,
    ],
);

/// Piece `k`: half extents, resting start position and kick velocity. Even pieces rest on a
/// roof, odd ones in the lane east of a house.
fn piece(k: usize) -> ([f32; 3], [f32; 3], [f32; 3]) {
    let half = if k % 3 == 0 {
        [0.3, 0.15, 0.2]
    } else {
        [0.2, 0.1, 0.25]
    };
    let h = house((k * 7) % COLS, (k * 11) % ROWS);
    let (lane, ground) = if k % 2 == 0 {
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

pub fn scenario() -> Scenario {
    Scenario {
        name: "debris-town",
        capabilities: "C18",
        ticks: TICKS,
        #[cfg(feature = "engine")]
        engine: || Box::new(engine::Sim::new()),
        #[cfg(feature = "rapier")]
        rapier: || Box::new(rapier::Sim::new()),
        #[cfg(feature = "rapier")]
        rapier_raw: None,
        check,
    }
}

/// Per tick and piece: its position.
fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    if engine.0.len() != rapier.0.len() || engine.0.len() != PIECES * TICKS as usize {
        return Err("trace lengths differ".into());
    }
    let at = |t: &Trace, tick: usize, k: usize| match t.0[tick * PIECES + k] {
        Obs::Position(p) => p,
        _ => [f32::NAN; 3],
    };
    for k in 0..PIECES {
        let (_, start, kick) = piece(k);
        for tick in 0..TICKS as usize {
            let p = at(engine, tick, k);
            // Flat on its roof or the ground, never through it.
            if (p[1] - start[1]).abs() > 0.01 {
                return Err(format!("piece {k} left its support at tick {tick}: {p:?}"));
            }
        }
        let end = TICKS as usize - 1;
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
    Ok(())
}

#[cfg(feature = "engine")]
mod engine {
    use super::*;
    use physics_engine::{
        BodyId,
        approximate::{Body, Config, Shape, Vector, World},
    };

    fn v(a: [f32; 3]) -> Vector {
        Vector(a[0], a[1], a[2])
    }

    pub struct Sim {
        world: World,
        pieces: Vec<BodyId>,
    }
    impl Sim {
        pub fn new() -> Self {
            let mut world = World::new(Config {
                body_storage: crate::engine_storage(),
                gravity: Vector(0.0, -GRAVITY, 0.0),
                // As in `props`: two substeps of one pass, the MMORPG adapter's crate step.
                substeps: 2,
                velocity_iterations: 1,
                length_unit: 1.0,
                ..Config::default()
            })
            .unwrap();
            let (centre, half) = GROUND;
            world
                .add_body(Body::new(Shape::Box(v(half)), v(centre), 0.0))
                .unwrap();
            for j in 0..ROWS {
                for i in 0..COLS {
                    world
                        .add_body(Body::new(Shape::Box(v([HOUSE; 3])), v(house(i, j)), 0.0))
                        .unwrap();
                }
            }
            let pieces = (0..PIECES)
                .map(|k| {
                    let (half, start, _) = piece(k);
                    let mass = DENSITY * 8.0 * half[0] * half[1] * half[2];
                    let mut body = Body::new(Shape::Box(v(half)), v(start), mass);
                    body.rotation_locked = true;
                    world.add_body(body).unwrap()
                })
                .collect();
            Self { world, pieces }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            if tick % KICK_EVERY == 0 && ((tick / KICK_EVERY) as usize) < PIECES {
                let k = (tick / KICK_EVERY) as usize;
                self.world
                    .set_velocity(self.pieces[k], v(piece(k).2))
                    .unwrap();
            }
            self.world.step(f64::from(DT)).unwrap();
            for &id in &self.pieces {
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

    fn v(a: [f32; 3]) -> Vector {
        Vector::new(a[0], a[1], a[2])
    }

    pub struct Sim {
        world: PhysicsWorld,
        pieces: Vec<RigidBodyHandle>,
    }

    impl Sim {
        pub fn new() -> Self {
            let mut world = PhysicsWorld::new();
            world.gravity = Vector::new(0.0, -GRAVITY, 0.0);
            world.integration_parameters.dt = DT;
            let (centre, half) = GROUND;
            world.colliders.insert(
                ColliderBuilder::cuboid(half[0], half[1], half[2])
                    .translation(v(centre))
                    .build(),
            );
            for j in 0..ROWS {
                for i in 0..COLS {
                    world.colliders.insert(
                        ColliderBuilder::cuboid(HOUSE, HOUSE, HOUSE)
                            .translation(v(house(i, j)))
                            .build(),
                    );
                }
            }
            let pieces = (0..PIECES)
                .map(|k| {
                    let (half, start, _) = piece(k);
                    let body = world.bodies.insert(
                        RigidBodyBuilder::dynamic()
                            .translation(v(start))
                            .lock_rotations()
                            .build(),
                    );
                    world.colliders.insert_with_parent(
                        ColliderBuilder::cuboid(half[0], half[1], half[2])
                            .density(DENSITY)
                            .build(),
                        body,
                        &mut world.bodies,
                    );
                    body
                })
                .collect();
            Self { world, pieces }
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            if tick % KICK_EVERY == 0 && ((tick / KICK_EVERY) as usize) < PIECES {
                let k = (tick / KICK_EVERY) as usize;
                self.world.bodies[self.pieces[k]].set_linvel(v(piece(k).2), true);
            }
            self.world.step();
            for &h in &self.pieces {
                let p = self.world.bodies[h].translation();
                trace.0.push(Obs::Position([p.x, p.y, p.z]));
            }
        }
    }
}
