//! C18 slice B (MMORPG crate debris, mmorpg#288, #366): eight crates break at once over
//! MMORPG's terrain pattern (the `terrain-holes` hills, flat cells removed over a floor plane)
//! into six tumbling planks each, thrown as MMORPG throws them, and settle on hills, the floor,
//! each other and, for one crate, against a wall. MMORPG's settings: density 600, friction 0.8,
//! restitution 0.15, linear damping 0.2, angular damping 0.8, gravity 9 m/s², one 1/30 s step
//! per tick. Rapier rebuilds the world per tick as MMORPG does, with settled pieces fixed;
//! the raw variant keeps one world. Restitution combines as each side does (Rapier averages it
//! with the terrain's zero; the engine takes the smaller, and does not bounce speculative
//! contacts), so tumbling outcomes are compared per crate, not per piece.

use super::terrain::ground;
use crate::{Obs, Scenario, Trace};

const TICKS: u32 = 180;
const DT: f32 = 1.0 / 30.0;
const GRAVITY: f32 = 9.0;
/// MMORPG units (centimetres) per tick to metres per second.
const PER_TICK: f32 = 30.0 / 100.0;
const DENSITY: f32 = 600.0;
const FRICTION: f32 = 0.8;
const RESTITUTION: f32 = 0.15;
const LINEAR_DAMPING: f32 = 0.2;
const ANGULAR_DAMPING: f32 = 0.8;
/// MMORPG parks a piece below 0.6 units/tick and 0.03 rad/tick.
const REST_SPEED: f32 = 0.6 * PER_TICK;
const REST_SPIN: f32 = 0.03 * 30.0;
const CRATES: usize = 8;
/// A crate's pieces in MMORPG units: offset and half extents (`CRATE_PIECES`).
const PIECES: [([f32; 3], [f32; 3]); 6] = [
    ([0.0, 32.0, 0.0], [33.0, 3.0, 33.0]),
    ([0.0, -32.0, 0.0], [33.0, 3.0, 33.0]),
    ([32.0, 0.0, 0.0], [3.0, 29.0, 33.0]),
    ([-32.0, 0.0, 0.0], [3.0, 29.0, 33.0]),
    ([0.0, 0.0, 32.0], [29.0, 29.0, 3.0]),
    ([0.0, 0.0, -32.0], [29.0, 29.0, 3.0]),
];
const WALL_CRATE: usize = 3;
const WALL_HALF: [f32; 3] = [0.5, 1.5, 2.0];
/// The wall face's distance from its crate's centre, toward +X.
const WALL_GAP: f32 = 0.9;

fn crate_centre(c: usize) -> [f32; 3] {
    let (x, z) = (-38.0 + 10.0 * c as f32, 4.0 + 3.0 * (c % 3) as f32);
    [x, ground(x, z, true) + 0.35, z]
}

fn wall_centre() -> [f32; 3] {
    let c = crate_centre(WALL_CRATE);
    [
        c[0] + WALL_GAP + WALL_HALF[0],
        c[1] + WALL_HALF[1] - 0.6,
        c[2],
    ]
}

/// A piece at the break, as MMORPG's `throw_debris` makes it (facing 0, no attacker).
struct Piece {
    position: [f32; 3],
    half: [f32; 3],
    velocity: [f32; 3],
    spin: [f32; 3],
}

fn piece(c: usize, index: usize) -> Piece {
    let (offset, half) = PIECES[index];
    let centre = crate_centre(c);
    let [ox, oy, oz] = offset;
    let length = ox.hypot(oz);
    let (dx, dz) = if length > 0.0 {
        (ox / length, oz / length)
    } else {
        (0.0, 0.0)
    };
    let i = index as f32;
    let lift = if oy > 0.0 { 1.4 } else { 1.0 };
    Piece {
        position: [
            centre[0] + ox / 100.0,
            centre[1] + oy / 100.0,
            centre[2] + oz / 100.0,
        ],
        half: half.map(|h| h / 100.0),
        velocity: [
            dx * 4.0 * PER_TICK,
            (5.0 * lift + i % 3.0) * PER_TICK,
            dz * 4.0 * PER_TICK,
        ],
        spin: [
            (0.12 * ((i % 3.0) - 1.0) + 0.05) * 30.0,
            0.08 * ((i % 2.0) * 2.0 - 1.0) * 30.0,
            0.12 * (((i + 1.0) % 3.0) - 1.0) * 30.0,
        ],
    }
}

fn mass(half: [f32; 3]) -> f32 {
    DENSITY * 8.0 * half[0] * half[1] * half[2]
}

/// Lowest clearance of a box's corners above the ground under each.
fn clearance(corners: impl Iterator<Item = [f32; 3]>) -> f32 {
    corners
        .map(|c| c[1] - ground(c[0], c[2], true))
        .fold(f32::INFINITY, f32::min)
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "debris",
        capabilities: "C18",
        ticks: TICKS,
        #[cfg(feature = "engine")]
        engine: || Box::new(engine::Sim::new()),
        #[cfg(feature = "rapier")]
        rapier: || Box::new(rapier::Sim::new(true)),
        #[cfg(feature = "rapier")]
        rapier_raw: Some(|| Box::new(rapier::Sim::new(false))),
        check,
    }
}

/// Per tick and piece: its centre, then its clearance.
fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    let n = CRATES * PIECES.len();
    if engine.0.len() != rapier.0.len() || engine.0.len() != 2 * n * TICKS as usize {
        return Err("trace lengths differ".into());
    }
    let at = |t: &Trace, tick: usize, k: usize| {
        let base = (tick * n + k) * 2;
        match (t.0[base], t.0[base + 1]) {
            (Obs::Position(p), Obs::Clearance(c)) => (p, c),
            _ => ([f32::NAN; 3], f32::NAN),
        }
    };
    let wall_face = wall_centre()[0] - WALL_HALF[0];
    let last = TICKS as usize - 1;
    for k in 0..n {
        for tick in 0..TICKS as usize {
            let (p, c) = at(engine, tick, k);
            if c < -0.02 {
                return Err(format!(
                    "piece {k} below the ground at tick {tick}: {c} at {p:?}"
                ));
            }
            if k / PIECES.len() == WALL_CRATE && p[0] > wall_face - 0.02 {
                return Err(format!("piece {k} inside the wall at tick {tick}: {p:?}"));
            }
        }
        // Settled: still, and on the ground or on other planks.
        let ((p, c), (q, _)) = (at(engine, last, k), at(engine, last - 5, k));
        if (0..3).any(|a| (p[a] - q[a]).abs() > 1e-3) {
            return Err(format!("piece {k} still moving at the end: {q:?} -> {p:?}"));
        }
        if c > 0.25 {
            return Err(format!("piece {k} rests {c} m above the ground at {p:?}"));
        }
    }
    // Each crate's pieces land around the same spot on both sides.
    for crate_ in 0..CRATES {
        let centroid = |t: &Trace| {
            let mut sum = [0.0; 3];
            for k in crate_ * PIECES.len()..(crate_ + 1) * PIECES.len() {
                let (p, _) = at(t, last, k);
                for a in 0..3 {
                    sum[a] += p[a] / PIECES.len() as f32;
                }
            }
            sum
        };
        let (e, r) = (centroid(engine), centroid(rapier));
        let d = ((e[0] - r[0]).powi(2) + (e[2] - r[2]).powi(2)).sqrt();
        if d > 0.3 || (e[1] - r[1]).abs() > 0.15 {
            return Err(format!(
                "crate {crate_} settles around {e:?}, Rapier's around {r:?}"
            ));
        }
    }
    Ok(())
}

#[cfg(feature = "engine")]
mod engine {
    use super::*;
    use crate::scenarios::terrain::{CELL, N, ORIGIN, heights, removed_cells};
    use physics_engine::{
        BodyId,
        approximate::{Body, Config, HeightfieldDesc, Shape, Vector, World},
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
                // The adapter steps once per tick, as MMORPG does.
                substeps: 1,
                velocity_iterations: 4,
                ..Config::default()
            })
            .unwrap();
            world
                .add_heightfield(
                    HeightfieldDesc {
                        origin: Vector(ORIGIN, 0.0, ORIGIN),
                        cell_size: (CELL, CELL),
                        columns: N,
                        rows: N,
                        heights: heights(true),
                        removed_cells: Some(removed_cells()),
                    },
                    None,
                )
                .unwrap();
            let mut fixed = |shape, at| {
                let mut body = Body::new(shape, v(at), 0.0);
                body.friction = FRICTION;
                world.add_body(body).unwrap();
            };
            fixed(Shape::HalfSpace, [0.0; 3]);
            fixed(Shape::Box(v(WALL_HALF)), wall_centre());
            let mut pieces = Vec::new();
            for c in 0..CRATES {
                for index in 0..PIECES.len() {
                    let p = piece(c, index);
                    let mut body = Body::new(Shape::Box(v(p.half)), v(p.position), mass(p.half));
                    body.velocity = v(p.velocity);
                    body.angular_velocity = v(p.spin);
                    body.friction = FRICTION;
                    body.restitution = RESTITUTION;
                    body.linear_damping = LINEAR_DAMPING;
                    body.angular_damping = ANGULAR_DAMPING;
                    pieces.push(world.add_body(body).unwrap());
                }
            }
            Self { world, pieces }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, _tick: u32, trace: &mut Trace) {
            self.world.step(f64::from(DT)).unwrap();
            for &id in &self.pieces {
                let b = self.world.body(id).unwrap();
                let Shape::Box(h) = b.shape else {
                    unreachable!()
                };
                let corners = (0..8).map(|k| {
                    let sign = |bit: usize| if k & bit == 0 { -1.0 } else { 1.0 };
                    let c = b.position
                        + b.orientation
                            .rotate(Vector(sign(1) * h.0, sign(2) * h.1, sign(4) * h.2));
                    [c.0, c.1, c.2]
                });
                trace
                    .0
                    .push(Obs::Position([b.position.0, b.position.1, b.position.2]));
                trace.0.push(Obs::Clearance(clearance(corners)));
            }
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::*;
    use crate::scenarios::terrain::rapier::{field, terrain_position};
    use rapier3d::prelude::*;

    fn v(a: [f32; 3]) -> Vector {
        Vector::new(a[0], a[1], a[2])
    }

    struct Debris {
        pose: Pose,
        velocity: Vector,
        spin: Vector,
        half: [f32; 3],
        resting: bool,
        handle: Option<RigidBodyHandle>,
    }

    pub struct Sim {
        world: PhysicsWorld,
        terrain: SharedShape,
        pieces: Vec<Debris>,
        rebuild: bool,
    }

    fn statics(world: &mut PhysicsWorld, terrain: &SharedShape) {
        world.gravity = Vector::new(0.0, -GRAVITY, 0.0);
        world.integration_parameters.dt = DT;
        world.colliders.insert(
            ColliderBuilder::new(SharedShape::halfspace(Vector::Y))
                .friction(FRICTION)
                .build(),
        );
        world.colliders.insert(
            ColliderBuilder::new(terrain.clone())
                .translation(terrain_position())
                .friction(FRICTION)
                .build(),
        );
        world.colliders.insert(
            ColliderBuilder::cuboid(WALL_HALF[0], WALL_HALF[1], WALL_HALF[2])
                .translation(v(wall_centre()))
                .friction(FRICTION)
                .build(),
        );
    }

    fn insert(world: &mut PhysicsWorld, d: &Debris) -> Option<RigidBodyHandle> {
        let collider = ColliderBuilder::cuboid(d.half[0], d.half[1], d.half[2])
            .friction(FRICTION)
            .restitution(RESTITUTION)
            .density(DENSITY);
        if d.resting {
            world.colliders.insert(collider.position(d.pose).build());
            return None;
        }
        let body = world.bodies.insert(
            RigidBodyBuilder::dynamic()
                .pose(d.pose)
                .linvel(d.velocity)
                .angvel(d.spin)
                .linear_damping(LINEAR_DAMPING)
                .angular_damping(ANGULAR_DAMPING)
                .build(),
        );
        world
            .colliders
            .insert_with_parent(collider.build(), body, &mut world.bodies);
        Some(body)
    }

    impl Sim {
        pub fn new(rebuild: bool) -> Self {
            let terrain = field(true);
            let mut world = PhysicsWorld::new();
            statics(&mut world, &terrain);
            let mut pieces: Vec<_> = (0..CRATES)
                .flat_map(|c| (0..PIECES.len()).map(move |i| piece(c, i)))
                .map(|p| Debris {
                    pose: Pose::from_translation(v(p.position)),
                    velocity: v(p.velocity),
                    spin: v(p.spin),
                    half: p.half,
                    resting: false,
                    handle: None,
                })
                .collect();
            if !rebuild {
                for d in &mut pieces {
                    d.handle = insert(&mut world, d);
                }
            }
            Self {
                world,
                terrain,
                pieces,
                rebuild,
            }
        }

        /// MMORPG's pattern: a fresh world per tick with the moving pieces dynamic and the
        /// settled ones fixed, stepped once; a piece below the rest speed and spin is parked.
        fn rebuilt_tick(&mut self) {
            if self.pieces.iter().all(|d| d.resting) {
                return;
            }
            let mut world = PhysicsWorld::new();
            statics(&mut world, &self.terrain);
            for d in &mut self.pieces {
                d.handle = insert(&mut world, d);
            }
            world.step();
            for d in &mut self.pieces {
                let Some(body) = d.handle.and_then(|h| world.bodies.get(h)) else {
                    continue;
                };
                d.pose = *body.position();
                d.velocity = body.linvel();
                d.spin = body.angvel();
                if d.velocity.length() < REST_SPEED && d.spin.length() < REST_SPIN {
                    d.resting = true;
                    d.velocity = Vector::ZERO;
                    d.spin = Vector::ZERO;
                }
            }
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, _tick: u32, trace: &mut Trace) {
            if self.rebuild {
                self.rebuilt_tick();
            } else {
                self.world.step();
                for d in &mut self.pieces {
                    d.pose = *self.world.bodies[d.handle.unwrap()].position();
                }
            }
            for d in &self.pieces {
                let h = d.half;
                let corners = (0..8).map(|k| {
                    let sign = |bit: usize| if k & bit == 0 { -1.0 } else { 1.0 };
                    let c = d.pose * Vector::new(sign(1) * h[0], sign(2) * h[1], sign(4) * h[2]);
                    [c.x, c.y, c.z]
                });
                let p = d.pose.translation;
                trace.0.push(Obs::Position([p.x, p.y, p.z]));
                trace.0.push(Obs::Clearance(clearance(corners)));
            }
        }
    }
}
