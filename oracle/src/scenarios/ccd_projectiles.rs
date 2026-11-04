//! Q38 / MMORPG #290: a persistent CCD sphere hits a thin wall, rebounds into another
//! obstruction in the *same* 60 Hz tick, and consumes the remainder of the requested time.
//! Separate glancing and moving-wall fixtures test tangential velocity and relative time of
//! impact without conflating their cost with simultaneous multi-projectile scheduling.
//! Rapier's single 60 Hz CCD step stops at its first impact; the parity side
//! subdivides the *same* 60 Hz interval into 64 Rapier steps to observe subsequent collisions.
//! A bare single-step Rapier run remains the performance baseline (and is expected to have a
//! different first-tick position).

use crate::{Obs, Scenario, Trace};

const DT: f32 = 1.0 / 60.0;
const TICKS: u32 = 60;
const RADIUS: f32 = 0.1;
const DENSITY: f32 = 600.0;
#[cfg(feature = "engine")]
const MASS: f32 = 4.0 / 3.0 * std::f32::consts::PI * RADIUS * RADIUS * RADIUS * DENSITY;
const HALF: [f32; 3] = [0.05, 1.0, 0.5];
const RIGHT: f32 = 0.45;
const LEFT: f32 = -0.25;
const SPEED: f32 = 45.0;
const TOLERANCE: f32 = 0.05;
#[cfg(feature = "rapier")]
const RAPIER_STEPS: usize = 64;

#[derive(Clone, Copy)]
struct Lane {
    second_wall: bool,
    moving_wall: bool,
    tangent: f32,
}

const LANES: [Lane; 3] = [
    Lane {
        second_wall: true,
        moving_wall: false,
        tangent: 0.0,
    },
    Lane {
        second_wall: false,
        moving_wall: false,
        tangent: 9.0,
    },
    Lane {
        second_wall: true,
        moving_wall: true,
        tangent: 0.0,
    },
];

pub fn ricochet() -> Scenario {
    Scenario {
        name: "ccd-ricochet",
        capabilities: "C20",
        ticks: TICKS,
        #[cfg(feature = "engine")]
        engine: |storage| Box::new(engine::Sim::new(storage, 0)),
        #[cfg(feature = "rapier")]
        rapier: || Box::new(rapier::Sim::new(0, RAPIER_STEPS)),
        #[cfg(feature = "rapier")]
        rapier_raw: Some(|| Box::new(rapier::Sim::new(0, 1))),
        check: |engine, rapier| check(0, engine, rapier),
    }
}

pub fn glancing() -> Scenario {
    Scenario {
        name: "ccd-glancing",
        capabilities: "C20",
        ticks: TICKS,
        #[cfg(feature = "engine")]
        engine: |storage| Box::new(engine::Sim::new(storage, 1)),
        #[cfg(feature = "rapier")]
        rapier: || Box::new(rapier::Sim::new(1, RAPIER_STEPS)),
        #[cfg(feature = "rapier")]
        rapier_raw: Some(|| Box::new(rapier::Sim::new(1, 1))),
        check: |engine, rapier| check(1, engine, rapier),
    }
}

pub fn moving() -> Scenario {
    Scenario {
        name: "ccd-moving-wall",
        capabilities: "C20",
        ticks: TICKS,
        #[cfg(feature = "engine")]
        engine: |storage| Box::new(engine::Sim::new(storage, 2)),
        #[cfg(feature = "rapier")]
        rapier: || Box::new(rapier::Sim::new(2, RAPIER_STEPS)),
        #[cfg(feature = "rapier")]
        rapier_raw: Some(|| Box::new(rapier::Sim::new(2, 1))),
        check: |engine, rapier| check(2, engine, rapier),
    }
}

fn state(trace: &Trace, tick: usize) -> Result<([f32; 3], [f32; 3]), String> {
    let i = tick * 2;
    match (trace.0.get(i), trace.0.get(i + 1)) {
        (Some(Obs::Position(p)), Some(Obs::Velocity(v))) => Ok((*p, *v)),
        other => Err(format!("bad projectile observations at {i}: {other:?}")),
    }
}

fn check(lane: usize, engine: &Trace, rapier: &Trace) -> Result<(), String> {
    let expected = TICKS as usize * 2;
    if engine.0.len() != expected || rapier.0.len() != expected {
        return Err(format!(
            "unexpected trace lengths: {}, {}",
            engine.0.len(),
            rapier.0.len()
        ));
    }
    let setup = LANES[lane];
    // Both wall contacts fit inside the first tick, with some remaining flight time.
    // These are independent elastic-trajectory checks, not merely trace equality.
    let (position, velocity) = state(engine, 0)?;
    let expected_x = if setup.second_wall {
        if setup.moving_wall { 0.116 } else { -0.05 }
    } else {
        -0.15
    };
    let expected_speed = if setup.moving_wall { 55.0 } else { SPEED };
    let expected_direction = if setup.second_wall { 1.0 } else { -1.0 };
    if (position[0] - expected_x).abs() > 0.035
        || (velocity[0] - expected_direction * expected_speed).abs() > 1.0
        || (velocity[2] - setup.tangent).abs() > 0.1
    {
        return Err(format!(
            "lane {lane}: two-hit/tangent outcome {position:?}, {velocity:?}; expected x={expected_x}, vx={}, vz={}",
            expected_direction * expected_speed,
            setup.tangent
        ));
    }
    let (reference_position, reference_velocity) = state(rapier, 0)?;
    for axis in [0, 2] {
        if (position[axis] - reference_position[axis]).abs() > TOLERANCE
            || (velocity[axis] - reference_velocity[axis]).abs()
                > 0.05 * reference_velocity[axis].abs() + 0.5
        {
            return Err(format!(
                "lane {lane} axis {axis}: engine {position:?}/{velocity:?} vs Rapier {reference_position:?}/{reference_velocity:?}"
            ));
        }
    }
    // Once the moving obstacle has played its part in the first tick, it parks. All later
    // samples must remain inside the resulting corridor, with no slow tunnelling and no
    // energy loss at frictionless elastic walls. The glancing sphere travels freely after
    // its first wall strike and preserves its tangential speed.
    let right = RIGHT - if setup.moving_wall { 5.0 * DT } else { 0.0 };
    for tick in 0..TICKS as usize {
        let (p, v) = state(engine, tick)?;
        if p[0] > right - HALF[0] - RADIUS + 0.01
            || (setup.second_wall && p[0] < LEFT + HALF[0] + RADIUS - 0.01)
            || (v[0].abs() - expected_speed).abs() > 1.0
            || (v[2] - setup.tangent).abs() > 0.1
        {
            return Err(format!(
                "lane {lane} tick {tick}: tunnelled or lost energy: {p:?}/{v:?}"
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
        approximate::{Body, BodyStorage, Config, Shape, Vector, World},
    };

    pub struct Sim {
        world: World,
        projectile: BodyId,
        moving_wall: Option<BodyId>,
    }
    impl Sim {
        pub fn new(storage: BodyStorage, lane: usize) -> Self {
            let mut world = World::new(Config {
                body_storage: storage,
                gravity: Vector::ZERO,
                substeps: 1,
                velocity_iterations: 8,
                ..Config::default()
            })
            .unwrap();
            let setup = LANES[lane];
            let mut moving_wall = None;
            for (x, moving) in [(RIGHT, setup.moving_wall), (LEFT, false)]
                .into_iter()
                .take(if setup.second_wall { 2 } else { 1 })
            {
                let mut wall = Body::new(
                    Shape::Box(Vector(HALF[0], HALF[1], HALF[2])),
                    Vector(x, 0.0, 0.0),
                    if moving { 1.0 } else { 0.0 },
                );
                wall.external = moving;
                wall.velocity = if moving {
                    Vector(-5.0, 0.0, 0.0)
                } else {
                    Vector::ZERO
                };
                wall.restitution = 1.0;
                wall.friction = 0.0;
                let id = world.add_body(wall).unwrap();
                if moving {
                    moving_wall = Some(id);
                }
            }
            let mut projectile = Body::new(Shape::Sphere(RADIUS), Vector::ZERO, MASS);
            projectile.velocity = Vector(SPEED, 0.0, setup.tangent);
            projectile.restitution = 1.0;
            projectile.friction = 0.0;
            projectile.ccd = true;
            projectile.sleep_allowed = false;
            let projectile = world.add_body(projectile).unwrap();
            Self {
                world,
                projectile,
                moving_wall,
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            if tick == 1
                && let Some(id) = self.moving_wall
            {
                self.world.set_velocity(id, Vector::ZERO).unwrap();
            }
            self.world.step(f64::from(DT)).unwrap();
            let b = self.world.body(self.projectile).unwrap();
            trace
                .0
                .push(Obs::Position([b.position.0, b.position.1, b.position.2]));
            trace
                .0
                .push(Obs::Velocity([b.velocity.0, b.velocity.1, b.velocity.2]));
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::*;
    use rapier3d::prelude::*;

    pub struct Sim {
        world: PhysicsWorld,
        projectile: RigidBodyHandle,
        moving_wall: Option<RigidBodyHandle>,
        steps: usize,
    }
    impl Sim {
        pub fn new(lane: usize, steps: usize) -> Self {
            let mut world = PhysicsWorld::new();
            world.gravity = Vector::ZERO;
            world.integration_parameters.dt = DT / steps as f32;
            let setup = LANES[lane];
            let mut moving_wall = None;
            for (x, moving) in [(RIGHT, setup.moving_wall), (LEFT, false)]
                .into_iter()
                .take(if setup.second_wall { 2 } else { 1 })
            {
                let collider = ColliderBuilder::cuboid(HALF[0], HALF[1], HALF[2])
                    .restitution(1.0)
                    .friction(0.0);
                if moving {
                    let body = world.bodies.insert(
                        RigidBodyBuilder::kinematic_velocity_based()
                            .translation(Vector::new(x, 0.0, 0.0))
                            .linvel(Vector::new(-5.0, 0.0, 0.0))
                            .build(),
                    );
                    world
                        .colliders
                        .insert_with_parent(collider.build(), body, &mut world.bodies);
                    moving_wall = Some(body);
                } else {
                    world
                        .colliders
                        .insert(collider.translation(Vector::new(x, 0.0, 0.0)).build());
                }
            }
            let projectile = world.bodies.insert(
                RigidBodyBuilder::dynamic()
                    .translation(Vector::ZERO)
                    .linvel(Vector::new(SPEED, 0.0, setup.tangent))
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
                projectile,
                &mut world.bodies,
            );
            Self {
                world,
                projectile,
                moving_wall,
                steps,
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            if tick == 1
                && let Some(id) = self.moving_wall
            {
                self.world.bodies[id].set_linvel(Vector::ZERO, true);
            }
            for _ in 0..self.steps {
                self.world.step();
            }
            let body = &self.world.bodies[self.projectile];
            let p = body.translation();
            let v = body.linvel();
            trace.0.push(Obs::Position([p.x, p.y, p.z]));
            trace.0.push(Obs::Velocity([v.x, v.y, v.z]));
        }
    }
}
