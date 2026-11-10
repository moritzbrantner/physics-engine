//! C18 restitution (#458; mmorpg#288 debris, mmorpg#290 bouncing thrown weapons): spheres and
//! rotation-locked boxes thrown at fixed walls at 2–40 m/s and dropped onto the floor plane from
//! 1–4 m, at restitutions 0.15, 0.5 and 0.8 on both sides. Everything is frictionless, so a
//! rebound keeps its speed along the floor. Rapier bounces a new contact once the step is solved
//! (its end-of-step restitution pass), speculative contacts included; before #458 the engine
//! stopped a fast body dead at the surface. 60 Hz, one world on both sides.

use crate::{Obs, Scenario, Trace};

const TICKS: u32 = 120;
const DT: f32 = 1.0 / 60.0;
const GRAVITY: f32 = 9.81;
const RADIUS: f32 = 0.1;
const HALF: f32 = 0.1;
const DENSITY: f32 = 600.0;
const RESTITUTIONS: [f32; 3] = [0.15, 0.5, 0.8];
const SPEEDS: [f32; 4] = [2.0, 5.0, 20.0, 40.0];
const DROPS: [f32; 3] = [1.0, 2.0, 4.0];
const WALL_HALF: [f32; 3] = [0.5, 3.0, 0.5];
const WALL_X: f32 = 3.0;
const THROW_Y: f32 = 1.5;
/// Acceptance: rebound speeds and bounce heights within 5% of Rapier's (plus 2 cm/s, 1 cm), no
/// body past its surface by more than 1 cm.
const SHARE: f32 = 0.05;

#[derive(Clone, Copy)]
enum Throw {
    Wall { speed: f32 },
    Drop { height: f32 },
}

#[derive(Clone, Copy)]
struct Lane {
    sphere: bool,
    restitution: f32,
    throw: Throw,
    z: f32,
}

fn lanes() -> Vec<Lane> {
    let mut out = Vec::new();
    for sphere in [true, false] {
        for restitution in RESTITUTIONS {
            let throws = SPEEDS
                .map(|speed| Throw::Wall { speed })
                .into_iter()
                .chain(DROPS.map(|height| Throw::Drop { height }));
            for throw in throws {
                let z = 2.0 * out.len() as f32;
                out.push(Lane {
                    sphere,
                    restitution,
                    throw,
                    z,
                });
            }
        }
    }
    out
}

fn half(lane: &Lane) -> f32 {
    if lane.sphere { RADIUS } else { HALF }
}
fn start(lane: &Lane) -> ([f32; 3], [f32; 3]) {
    match lane.throw {
        Throw::Wall { speed } => ([0.0, THROW_Y, lane.z], [speed, 0.0, 0.0]),
        Throw::Drop { height } => ([0.0, half(lane) + height, lane.z], [0.0; 3]),
    }
}
fn mass(lane: &Lane) -> f32 {
    if lane.sphere {
        DENSITY * 4.0 / 3.0 * std::f32::consts::PI * RADIUS.powi(3)
    } else {
        DENSITY * 8.0 * HALF.powi(3)
    }
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "bounce",
        capabilities: "C18",
        ticks: TICKS,
        #[cfg(feature = "engine")]
        engine: |storage| Box::new(engine::Sim::new(storage)),
        #[cfg(feature = "rapier")]
        rapier: || Box::new(rapier::Sim::new()),
        #[cfg(feature = "rapier")]
        rapier_raw: None,
        #[cfg(feature = "rapier")]
        bare_gated: true,
        check,
    }
}

fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    let lanes = lanes();
    let n = lanes.len();
    if engine.0.len() != rapier.0.len() || engine.0.len() != n * TICKS as usize {
        return Err("trace lengths differ".into());
    }
    let at = |t: &Trace, tick: usize, l: usize| match t.0[tick * n + l] {
        Obs::Position(p) => p,
        _ => [f32::NAN; 3],
    };
    for (l, lane) in lanes.iter().enumerate() {
        let r = half(lane);
        for tick in 0..TICKS as usize {
            let p = at(engine, tick, l);
            if p[1] < r - 0.01 || p[0] > WALL_X - WALL_HALF[0] - r + 0.01 {
                return Err(format!("lane {l} passed its surface at tick {tick}: {p:?}"));
            }
        }
        match lane.throw {
            Throw::Wall { speed } => {
                // The rebound's speed along x over the last ticks (frictionless floor).
                let end = TICKS as usize - 1;
                let vx = |t: &Trace| (at(t, end, l)[0] - at(t, end - 10, l)[0]) / (10.0 * DT);
                let (e, q) = (vx(engine), vx(rapier));
                if e >= 0.0 || (e - q).abs() > SHARE * q.abs() + 0.02 {
                    return Err(format!(
                        "lane {l} ({speed} m/s, e {}) rebounds at {e:.3} m/s, Rapier {q:.3}",
                        lane.restitution
                    ));
                }
            }
            Throw::Drop { height } => {
                // The first bounce's apex: the highest point after the first landing.
                let apex = |t: &Trace| {
                    let land = (1..TICKS as usize)
                        .find(|&k| at(t, k, l)[1] > at(t, k - 1, l)[1])
                        .unwrap_or(TICKS as usize - 1);
                    (land..TICKS as usize)
                        .map(|k| at(t, k, l)[1])
                        .fold(0.0, f32::max)
                        - r
                };
                let (e, q) = (apex(engine), apex(rapier));
                if (e - q).abs() > SHARE * q.max(height * 0.01) + 0.01 {
                    return Err(format!(
                        "lane {l} (drop {height} m, e {}) bounces {e:.3} m high, Rapier {q:.3} m",
                        lane.restitution
                    ));
                }
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
        approximate::{Body, BodyStorage, Config, Shape, Vector, World},
    };

    use crate::kcc::engine::v;

    pub struct Sim {
        world: World,
        bodies: Vec<BodyId>,
    }
    impl Sim {
        pub fn new(storage: BodyStorage) -> Self {
            let mut world = World::new(Config {
                body_storage: storage,
                gravity: Vector(0.0, -GRAVITY, 0.0),
                substeps: 2,
                velocity_iterations: 4,
                length_unit: 1.0,
                ..Config::default()
            })
            .unwrap();
            let fixed = |shape, at: [f32; 3], restitution| {
                let mut body = Body::new(shape, v(at), 0.0);
                body.friction = 0.0;
                body.restitution = restitution;
                body
            };
            let lanes = lanes();
            // One floor plane per restitution, on its own layer (a plane is unbounded); a wall
            // per lane.
            for (k, &e) in RESTITUTIONS.iter().enumerate() {
                let mut floor = fixed(Shape::HalfSpace, [0.0; 3], e);
                floor.layers = physics_engine::CollisionLayers3d::new(1 << k, 1 << k);
                world.add_body(floor).unwrap();
            }
            for lane in &lanes {
                world
                    .add_body(fixed(
                        Shape::Box(v(WALL_HALF)),
                        [WALL_X, WALL_HALF[1], lane.z],
                        lane.restitution,
                    ))
                    .unwrap();
            }
            let bodies = lanes
                .iter()
                .map(|lane| {
                    let (p, vel) = start(lane);
                    let shape = if lane.sphere {
                        Shape::Sphere(RADIUS)
                    } else {
                        Shape::Box(Vector(HALF, HALF, HALF))
                    };
                    let mut body = Body::new(shape, v(p), mass(lane));
                    body.velocity = v(vel);
                    body.rotation_locked = !lane.sphere;
                    body.friction = 0.0;
                    body.restitution = lane.restitution;
                    body.layers = layer(lane);
                    world.add_body(body).unwrap()
                })
                .collect();
            Self { world, bodies }
        }
    }
    /// A layer per restitution keeps each body on its own floor plane.
    fn layer(lane: &Lane) -> physics_engine::CollisionLayers3d {
        let bit = 1
            << RESTITUTIONS
                .iter()
                .position(|&e| e == lane.restitution)
                .unwrap();
        physics_engine::CollisionLayers3d::new(bit, bit)
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, _tick: u32, trace: &mut Trace) {
            self.world.step(f64::from(DT)).unwrap();
            for &id in &self.bodies {
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
        bodies: Vec<RigidBodyHandle>,
    }
    fn groups(lane: &Lane) -> InteractionGroups {
        let bit = 1
            << RESTITUTIONS
                .iter()
                .position(|&e| e == lane.restitution)
                .unwrap();
        let group = Group::from_bits_truncate(bit);
        InteractionGroups::new(group, group, InteractionTestMode::And)
    }
    impl Sim {
        pub fn new() -> Self {
            let mut world = PhysicsWorld::new();
            world.gravity = Vector::new(0.0, -GRAVITY, 0.0);
            world.integration_parameters.dt = DT;
            let lanes = lanes();
            for (k, &e) in RESTITUTIONS.iter().enumerate() {
                let group = Group::from_bits_truncate(1 << k);
                world.colliders.insert(
                    ColliderBuilder::new(SharedShape::halfspace(Vector::Y))
                        .friction(0.0)
                        .restitution(e)
                        .collision_groups(InteractionGroups::new(
                            group,
                            group,
                            InteractionTestMode::And,
                        ))
                        .build(),
                );
            }
            for lane in &lanes {
                world.colliders.insert(
                    ColliderBuilder::cuboid(WALL_HALF[0], WALL_HALF[1], WALL_HALF[2])
                        .translation(Vector::new(WALL_X, WALL_HALF[1], lane.z))
                        .friction(0.0)
                        .restitution(lane.restitution)
                        .build(),
                );
            }
            let bodies = lanes
                .iter()
                .map(|lane| {
                    let (p, vel) = start(lane);
                    let mut body = RigidBodyBuilder::dynamic().translation(v(p)).linvel(v(vel));
                    if !lane.sphere {
                        body = body.lock_rotations();
                    }
                    let handle = world.bodies.insert(body.build());
                    let collider = if lane.sphere {
                        ColliderBuilder::ball(RADIUS)
                    } else {
                        ColliderBuilder::cuboid(HALF, HALF, HALF)
                    };
                    world.colliders.insert_with_parent(
                        collider
                            .density(DENSITY)
                            .friction(0.0)
                            .restitution(lane.restitution)
                            .collision_groups(groups(lane))
                            .build(),
                        handle,
                        &mut world.bodies,
                    );
                    handle
                })
                .collect();
            Self { world, bodies }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, _tick: u32, trace: &mut Trace) {
            self.world.step();
            for &h in &self.bodies {
                let p = self.world.bodies[h].translation();
                trace.0.push(Obs::Position([p.x, p.y, p.z]));
            }
        }
    }
}
