//! C18 slice A (MMORPG movable crates, mmorpg#58, #366): upright dynamic boxes on a floor
//! plane. A Shield Bash launches each crate at 5.91 m/s (19.7 units/tick at 30 Hz); it slides
//! under friction and linear damping and stops on its own, at a wall or at a unit's capsule.
//! At tick 120 every crate is pushed back, waking the resting ones. MMORPG's settings: 0.7 m
//! crates of density 600, rotations locked, friction 0.8, linear damping 0.2, gravity
//! 9 m/s², one 1/30 s step per tick. Both sides use friction 0.8 on every collider, so
//! Rapier's averaged and the engine's larger coefficient agree; restitution is left to the
//! debris slice. Rapier rebuilds the world per tick as MMORPG does, with resting crates
//! fixed; the raw variant keeps one world. The engine steps once per tick, as MMORPG does.

use crate::{Obs, Scenario, Trace};

const TICKS: u32 = 240;
const DT: f32 = 1.0 / 30.0;
const GRAVITY: f32 = 9.0;
const HALF: f32 = 0.35;
const MASS: f32 = 600.0 * 8.0 * HALF * HALF * HALF;
const FRICTION: f32 = 0.8;
const DAMPING: f32 = 0.2;
const LAUNCH: f32 = 5.91;
const PUSH_BACK: f32 = -3.0;
const PUSH_BACK_TICK: u32 = 120;
/// MMORPG treats a crate slower than this as resting.
const REST_SPEED: f32 = 0.05;
const LANES: usize = 24;
const WALL_FACE: f32 = 1.5;
const WALL_HALF: [f32; 3] = [0.5, 1.0, 1.0];
const UNIT_X: f32 = 1.6;
const UNIT_HALF: f32 = 0.5;
const UNIT_RADIUS: f32 = 0.3;

#[derive(Clone, Copy, PartialEq)]
enum Lane {
    Free,
    Wall,
    Unit,
}

fn lane(l: usize) -> Lane {
    [Lane::Free, Lane::Wall, Lane::Unit][l % 3]
}

fn crate_start(l: usize) -> [f32; 3] {
    [0.0, HALF, 3.0 * l as f32]
}

fn wall_centre(l: usize) -> [f32; 3] {
    [WALL_FACE + WALL_HALF[0], WALL_HALF[1], 3.0 * l as f32]
}

fn unit_centre(l: usize) -> [f32; 3] {
    [UNIT_X, UNIT_HALF + UNIT_RADIUS, 3.0 * l as f32]
}

/// The crate's velocity change this tick, if it is pushed.
fn push(tick: u32) -> Option<f32> {
    match tick {
        0 => Some(LAUNCH),
        PUSH_BACK_TICK => Some(PUSH_BACK),
        _ => None,
    }
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "props",
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

/// Per tick and lane: the crate's position.
fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    if engine.0.len() != rapier.0.len() || engine.0.len() != LANES * TICKS as usize {
        return Err("trace lengths differ".into());
    }
    let at = |t: &Trace, tick: usize, l: usize| match t.0[tick * LANES + l] {
        Obs::Position(p) => p,
        _ => [f32::NAN; 3],
    };
    for l in 0..LANES {
        let start = crate_start(l);
        for tick in 0..TICKS as usize {
            let p = at(engine, tick, l);
            // Upright on the floor, in its lane; never into the wall or the unit.
            if (p[1] - HALF).abs() > 0.01 || (p[2] - start[2]).abs() > 1e-4 {
                return Err(format!("lane {l} left the floor at tick {tick}: {p:?}"));
            }
            let limit = match lane(l) {
                Lane::Free => f32::INFINITY,
                Lane::Wall => WALL_FACE - HALF,
                Lane::Unit => UNIT_X - UNIT_RADIUS - HALF,
            };
            if p[0] > limit + 0.01 {
                return Err(format!(
                    "lane {l} entered its obstacle at tick {tick}: {p:?}"
                ));
            }
        }
        // At rest before each push, within 5% of Rapier's slide of where Rapier's crate rests
        // (#375: two substeps track Rapier's internal substeps).
        for (from, end) in [
            (0, PUSH_BACK_TICK as usize - 1),
            (PUSH_BACK_TICK as usize - 1, TICKS as usize - 1),
        ] {
            let (p, q) = (at(engine, end, l), at(engine, end - 5, l));
            if (p[0] - q[0]).abs() > 1e-4 {
                return Err(format!(
                    "lane {l} still moving at tick {end}: {q:?} -> {p:?}"
                ));
            }
            let r = at(rapier, end, l);
            let slide = (r[0] - at(rapier, from, l)[0]).abs();
            if (p[0] - r[0]).abs() > 0.05 * slide + 0.01 {
                return Err(format!(
                    "lane {l} rests at {:.4} m at tick {end}, Rapier at {:.4} m",
                    p[0], r[0]
                ));
            }
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
        crates: Vec<BodyId>,
    }
    impl Sim {
        pub fn new() -> Self {
            let mut world = World::new(Config {
                body_storage: crate::engine_storage(),
                gravity: Vector(0.0, -GRAVITY, 0.0),
                // The adapter steps once per tick, as MMORPG does: two substeps of one pass
                // track Rapier's rest positions within 5% of each slide (#375).
                substeps: 2,
                velocity_iterations: 1,
                ..Config::default()
            })
            .unwrap();
            let mut fixed = |shape, at| {
                let mut body = Body::new(shape, v(at), 0.0);
                body.friction = FRICTION;
                world.add_body(body).unwrap();
            };
            fixed(Shape::HalfSpace, [0.0; 3]);
            for l in 0..LANES {
                match lane(l) {
                    Lane::Free => {}
                    Lane::Wall => fixed(Shape::Box(v(WALL_HALF)), wall_centre(l)),
                    Lane::Unit => fixed(Shape::capsule(UNIT_HALF, UNIT_RADIUS), unit_centre(l)),
                }
            }
            let crates = (0..LANES)
                .map(|l| {
                    let mut body = Body::new(Shape::Box(v([HALF; 3])), v(crate_start(l)), MASS);
                    body.rotation_locked = true;
                    body.friction = FRICTION;
                    body.linear_damping = DAMPING;
                    world.add_body(body).unwrap()
                })
                .collect();
            Self { world, crates }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            if let Some(dv) = push(tick) {
                for &id in &self.crates {
                    let v = self.world.body(id).unwrap().velocity;
                    self.world
                        .set_velocity(id, Vector(v.0 + dv, v.1, v.2))
                        .unwrap();
                }
            }
            self.world.step(f64::from(DT)).unwrap();
            for &id in &self.crates {
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

    struct Crate {
        position: Vector,
        velocity: Vector,
        resting: bool,
        handle: Option<RigidBodyHandle>,
    }

    pub struct Sim {
        world: PhysicsWorld,
        crates: Vec<Crate>,
        rebuild: bool,
    }

    fn statics(world: &mut PhysicsWorld) {
        world.gravity = Vector::new(0.0, -GRAVITY, 0.0);
        world.integration_parameters.dt = DT;
        world.colliders.insert(
            ColliderBuilder::new(SharedShape::halfspace(Vector::Y))
                .friction(FRICTION)
                .build(),
        );
        for l in 0..LANES {
            let collider = match lane(l) {
                Lane::Free => continue,
                Lane::Wall => ColliderBuilder::cuboid(WALL_HALF[0], WALL_HALF[1], WALL_HALF[2])
                    .translation(v(wall_centre(l))),
                Lane::Unit => ColliderBuilder::capsule_y(UNIT_HALF, UNIT_RADIUS)
                    .translation(v(unit_centre(l))),
            };
            world.colliders.insert(collider.friction(FRICTION).build());
        }
    }

    fn insert(world: &mut PhysicsWorld, c: &Crate) -> Option<RigidBodyHandle> {
        let collider = ColliderBuilder::cuboid(HALF, HALF, HALF)
            .friction(FRICTION)
            .density(600.0);
        if c.resting {
            world
                .colliders
                .insert(collider.translation(c.position).build());
            return None;
        }
        let body = world.bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(c.position)
                .linvel(c.velocity)
                .linear_damping(DAMPING)
                .lock_rotations()
                .build(),
        );
        world
            .colliders
            .insert_with_parent(collider.build(), body, &mut world.bodies);
        Some(body)
    }

    impl Sim {
        pub fn new(rebuild: bool) -> Self {
            let mut world = PhysicsWorld::new();
            statics(&mut world);
            let mut crates: Vec<_> = (0..LANES)
                .map(|l| Crate {
                    position: v(crate_start(l)),
                    velocity: Vector::ZERO,
                    resting: rebuild,
                    handle: None,
                })
                .collect();
            if !rebuild {
                for c in &mut crates {
                    c.handle = insert(&mut world, c);
                }
            }
            Self {
                world,
                crates,
                rebuild,
            }
        }

        /// MMORPG's pattern: a fresh world per tick with the moving crates dynamic and the
        /// resting ones fixed, stepped once; a crate below the rest speed is parked.
        fn rebuilt_tick(&mut self, tick: u32) {
            if let Some(dv) = push(tick) {
                for c in &mut self.crates {
                    c.velocity.x += dv;
                    c.resting = false;
                }
            }
            if self.crates.iter().all(|c| c.resting) {
                return;
            }
            let mut world = PhysicsWorld::new();
            statics(&mut world);
            for c in &mut self.crates {
                c.handle = insert(&mut world, c);
            }
            world.step();
            for c in &mut self.crates {
                let Some(body) = c.handle.and_then(|h| world.bodies.get(h)) else {
                    continue;
                };
                c.position = body.translation();
                c.velocity = body.linvel();
                if c.velocity.length() < REST_SPEED {
                    c.resting = true;
                    c.velocity = Vector::ZERO;
                }
            }
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            if self.rebuild {
                self.rebuilt_tick(tick);
            } else {
                if let Some(dv) = push(tick) {
                    for c in &self.crates {
                        let body = &mut self.world.bodies[c.handle.unwrap()];
                        let mut linvel = body.linvel();
                        linvel.x += dv;
                        body.set_linvel(linvel, true);
                    }
                }
                self.world.step();
                for c in &mut self.crates {
                    c.position = self.world.bodies[c.handle.unwrap()].translation();
                }
            }
            for c in &self.crates {
                trace
                    .0
                    .push(Obs::Position([c.position.x, c.position.y, c.position.z]));
            }
        }
    }
}
