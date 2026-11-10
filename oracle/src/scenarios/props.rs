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

pub(crate) const TICKS: u32 = 240;
pub(crate) const DT: f32 = 1.0 / 30.0;
pub(crate) const GRAVITY: f32 = 9.0;
const HALF: f32 = 0.35;
const MASS: f32 = 600.0 * 8.0 * HALF * HALF * HALF;
const FRICTION: f32 = 0.8;
const DAMPING: f32 = 0.2;
const LAUNCH: f32 = 5.91;
const PUSH_BACK: f32 = -3.0;
const PUSH_BACK_TICK: u32 = 120;
/// MMORPG treats a crate slower than this as resting.
const REST_SPEED: f32 = 0.05;
pub(crate) const LANES: usize = 24;
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

/// Lane `l`'s crate start, wall and unit, with the lanes starting at `z0` along Z.
fn crate_start(l: usize, z0: f32) -> [f32; 3] {
    [0.0, HALF, z0 + 3.0 * l as f32]
}

fn wall_centre(l: usize, z0: f32) -> [f32; 3] {
    [WALL_FACE + WALL_HALF[0], WALL_HALF[1], z0 + 3.0 * l as f32]
}

fn unit_centre(l: usize, z0: f32) -> [f32; 3] {
    [UNIT_X, UNIT_HALF + UNIT_RADIUS, z0 + 3.0 * l as f32]
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
        capabilities: "C13 C18",
        ticks: TICKS,
        #[cfg(feature = "engine")]
        engine: |storage| Box::new(engine::Sim::new(storage)),
        #[cfg(feature = "rapier")]
        rapier: || Box::new(rapier::Sim::new(true)),
        #[cfg(feature = "rapier")]
        rapier_raw: Some(|| Box::new(rapier::Sim::new(false))),
        #[cfg(feature = "rapier")]
        bare_gated: true,
        check,
    }
}

fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    check_at(engine, rapier, 0.0)
}

/// Per tick and lane: the crate's position, the lanes starting at `z0`.
pub(crate) fn check_at(engine: &Trace, rapier: &Trace, z0: f32) -> Result<(), String> {
    if engine.0.len() != rapier.0.len() || engine.0.len() != LANES * TICKS as usize {
        return Err("trace lengths differ".into());
    }
    let at = |t: &Trace, tick: usize, l: usize| match t.0[tick * LANES + l] {
        Obs::Position(p) => p,
        _ => [f32::NAN; 3],
    };
    for l in 0..LANES {
        let start = crate_start(l, z0);
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
pub(crate) mod engine {
    use super::*;
    use physics_engine::{
        BodyId,
        approximate::{Body, Config, Shape, Vector, World},
    };

    use crate::kcc::engine::v;

    pub struct Sim {
        world: World,
        crates: Vec<BodyId>,
    }

    /// The walls and units of the lanes starting at `z0`, without the floor.
    pub(crate) fn add_obstacles(world: &mut World, z0: f32) {
        let mut fixed = |shape, at| {
            let mut body = Body::new(shape, v(at), 0.0);
            body.friction = FRICTION;
            world.add_body(body).unwrap();
        };
        for l in 0..LANES {
            match lane(l) {
                Lane::Free => {}
                Lane::Wall => fixed(Shape::Box(v(WALL_HALF)), wall_centre(l, z0)),
                Lane::Unit => fixed(Shape::capsule(UNIT_HALF, UNIT_RADIUS), unit_centre(l, z0)),
            }
        }
    }

    /// MMORPG's upright crates, one per lane.
    pub(crate) fn add_crates(world: &mut World, z0: f32) -> Vec<BodyId> {
        (0..LANES)
            .map(|l| {
                let mut body = Body::new(Shape::Box(v([HALF; 3])), v(crate_start(l, z0)), MASS);
                body.rotation_locked = true;
                body.friction = FRICTION;
                body.linear_damping = DAMPING;
                world.add_body(body).unwrap()
            })
            .collect()
    }

    /// This tick's push, if any.
    pub(crate) fn push_crates(world: &mut World, crates: &[BodyId], tick: u32) {
        if let Some(dv) = push(tick) {
            for &id in crates {
                let v = world.body(id).unwrap().velocity;
                world.set_velocity(id, Vector(v.0 + dv, v.1, v.2)).unwrap();
            }
        }
    }

    pub(crate) fn record(world: &World, crates: &[BodyId], trace: &mut Trace) {
        for &id in crates {
            let p = world.body(id).unwrap().position;
            trace.0.push(Obs::Position([p.0, p.1, p.2]));
        }
    }

    impl Sim {
        pub fn new(storage: physics_engine::approximate::BodyStorage) -> Self {
            let mut world = World::new(Config {
                body_storage: storage,
                gravity: Vector(0.0, -GRAVITY, 0.0),
                // The adapter steps once per tick, as MMORPG does: two substeps of one pass
                // track Rapier's rest positions within 5% of each slide (#375).
                substeps: 2,
                velocity_iterations: 1,
                // Slop and sleep are the metre defaults (#318).
                length_unit: 1.0,
                ..Config::default()
            })
            .unwrap();
            let mut floor = Body::new(Shape::HalfSpace, Vector::ZERO, 0.0);
            floor.friction = FRICTION;
            world.add_body(floor).unwrap();
            add_obstacles(&mut world, 0.0);
            let crates = add_crates(&mut world, 0.0);
            Self { world, crates }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            push_crates(&mut self.world, &self.crates, tick);
            self.world.step(f64::from(DT)).unwrap();
            record(&self.world, &self.crates, trace);
        }
    }
}

#[cfg(feature = "rapier")]
pub(crate) mod rapier {
    use super::*;
    use rapier3d::prelude::*;

    use crate::kcc::rapier::v;

    pub(crate) struct Crate {
        position: Vector,
        velocity: Vector,
        resting: bool,
        handle: Option<RigidBodyHandle>,
    }

    /// The crates of the lanes starting at `z0`, resting (fixed) until pushed when `rebuild`.
    pub(crate) fn crates(z0: f32, rebuild: bool) -> Vec<Crate> {
        (0..LANES)
            .map(|l| Crate {
                position: v(crate_start(l, z0)),
                velocity: Vector::ZERO,
                resting: rebuild,
                handle: None,
            })
            .collect()
    }

    /// The walls and units of the lanes starting at `z0`, without the floor.
    pub(crate) fn obstacles(world: &mut PhysicsWorld, z0: f32) {
        for l in 0..LANES {
            let collider = match lane(l) {
                Lane::Free => continue,
                Lane::Wall => ColliderBuilder::cuboid(WALL_HALF[0], WALL_HALF[1], WALL_HALF[2])
                    .translation(v(wall_centre(l, z0))),
                Lane::Unit => ColliderBuilder::capsule_y(UNIT_HALF, UNIT_RADIUS)
                    .translation(v(unit_centre(l, z0))),
            };
            world.colliders.insert(collider.friction(FRICTION).build());
        }
    }

    /// This tick's push wakes every crate.
    pub(crate) fn push_crates(crates: &mut [Crate], tick: u32) {
        if let Some(dv) = push(tick) {
            for c in crates {
                c.velocity.x += dv;
                c.resting = false;
            }
        }
    }

    /// Inserts every crate into a fresh world: moving ones dynamic, resting ones fixed.
    pub(crate) fn insert_all(world: &mut PhysicsWorld, crates: &mut [Crate]) {
        for c in crates {
            c.handle = insert(world, c);
        }
    }

    /// Reads the moving crates back after the step; a crate below the rest speed is parked.
    pub(crate) fn read_back(world: &PhysicsWorld, crates: &mut [Crate]) {
        for c in crates {
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

    /// This tick's push in a retained world.
    pub(crate) fn push_retained(world: &mut PhysicsWorld, crates: &[Crate], tick: u32) {
        if let Some(dv) = push(tick) {
            for c in crates {
                let body = &mut world.bodies[c.handle.unwrap()];
                let mut linvel = body.linvel();
                linvel.x += dv;
                body.set_linvel(linvel, true);
            }
        }
    }

    /// The crates' positions after a step of a retained world.
    pub(crate) fn read_positions(world: &PhysicsWorld, crates: &mut [Crate]) {
        for c in crates {
            c.position = world.bodies[c.handle.unwrap()].translation();
        }
    }

    pub(crate) fn record(crates: &[Crate], trace: &mut Trace) {
        for c in crates {
            trace
                .0
                .push(Obs::Position([c.position.x, c.position.y, c.position.z]));
        }
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
        obstacles(world, 0.0);
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
            let mut crates = crates(0.0, rebuild);
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
            push_crates(&mut self.crates, tick);
            if self.crates.iter().all(|c| c.resting) {
                return;
            }
            let mut world = PhysicsWorld::new();
            statics(&mut world);
            insert_all(&mut world, &mut self.crates);
            world.step();
            read_back(&world, &mut self.crates);
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            if self.rebuild {
                self.rebuilt_tick(tick);
            } else {
                push_retained(&mut self.world, &self.crates, tick);
                self.world.step();
                read_positions(&self.world, &mut self.crates);
            }
            record(&self.crates, trace);
        }
    }
}
