//! Eight independent high-speed spheres strike one wall in the same 60 Hz tick.
//! Processing their TOIs chronologically must retain every equal-time impact. This fixture
//! caught six spheres tunnelling past the wall under the original #464 group scheduler.

use crate::{Obs, Scenario, Trace};

const DT: f32 = 1.0 / 60.0;
const TICKS: u32 = 60;
const COUNT: usize = 8;
const SPEED: f32 = 45.0;
const RADIUS: f32 = 0.1;
const DENSITY: f32 = 600.0;
#[cfg(feature = "engine")]
const MASS: f32 = 4.0 / 3.0 * std::f32::consts::PI * RADIUS * RADIUS * RADIUS * DENSITY;

pub fn scenario() -> Scenario {
    Scenario {
        name: "ccd-synchronized",
        capabilities: "C20",
        ticks: TICKS,
        #[cfg(feature = "engine")]
        engine: |storage| Box::new(engine::Sim::new(storage)),
        #[cfg(feature = "rapier")]
        rapier: || Box::new(rapier::Sim::new(64)),
        #[cfg(feature = "rapier")]
        rapier_raw: Some(|| Box::new(rapier::Sim::new(1))),
        #[cfg(feature = "rapier")]
        bare_gated: true,
        check,
    }
}

fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    let expected = TICKS as usize * COUNT * 2;
    if engine.0.len() != expected || rapier.0.len() != expected {
        return Err(format!(
            "synchronized observations: expected {expected}, got {} and {}",
            engine.0.len(),
            rapier.0.len()
        ));
    }
    for i in 0..COUNT {
        let at = |trace: &Trace, tick: usize| {
            let offset = (tick * COUNT + i) * 2;
            match (&trace.0[offset], &trace.0[offset + 1]) {
                (Obs::Position(p), Obs::Velocity(v)) => Ok((*p, *v)),
                _ => Err(format!("bad observations for projectile {i} tick {tick}")),
            }
        };
        let (p, v) = at(engine, 0)?;
        let (rp, rv) = at(rapier, 0)?;
        if (p[0] + 0.15).abs() > 0.01
            || (v[0] + SPEED).abs() > 0.1
            || (p[0] - rp[0]).abs() > 0.05
            || (v[0] - rv[0]).abs() > 0.05 * rv[0].abs() + 0.5
        {
            return Err(format!(
                "projectile {i}: engine {p:?}/{v:?}, Rapier {rp:?}/{rv:?} at first impact"
            ));
        }
        for tick in 1..TICKS as usize {
            let (p, v) = at(engine, tick)?;
            if p[0] > -0.14 || (v[0] + SPEED).abs() > 0.1 {
                return Err(format!(
                    "projectile {i} tick {tick}: bounced trajectory lost {p:?}/{v:?}"
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
        approximate::{Body, BodyStorage, Config, Shape, Vector, World},
    };

    pub(super) struct Sim {
        world: World,
        projectiles: Vec<BodyId>,
    }
    impl Sim {
        pub fn new(storage: BodyStorage) -> Self {
            let mut world = World::new(Config {
                body_storage: storage,
                gravity: Vector::ZERO,
                substeps: 1,
                velocity_iterations: 8,
                ..Config::default()
            })
            .unwrap();
            let mut wall = Body::new(
                Shape::Box(Vector(0.05, 64.0, 2.0)),
                Vector(0.45, 0.0, 0.0),
                0.0,
            );
            wall.restitution = 1.0;
            wall.friction = 0.0;
            world.add_body(wall).unwrap();
            let mut projectiles = Vec::new();
            for k in 0..COUNT {
                let mut ball = Body::new(
                    Shape::Sphere(RADIUS),
                    Vector(0.0, k as f32 * 0.5, 0.0),
                    MASS,
                );
                ball.velocity = Vector(SPEED, 0.0, 0.0);
                ball.restitution = 1.0;
                ball.friction = 0.0;
                ball.ccd = true;
                ball.sleep_allowed = false;
                projectiles.push(world.add_body(ball).unwrap());
            }
            Self { world, projectiles }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, _tick: u32, trace: &mut Trace) {
            self.world.step(f64::from(DT)).unwrap();
            for &id in &self.projectiles {
                let b = self.world.body(id).unwrap();
                trace
                    .0
                    .push(Obs::Position([b.position.0, b.position.1, b.position.2]));
                trace
                    .0
                    .push(Obs::Velocity([b.velocity.0, b.velocity.1, b.velocity.2]));
            }
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::*;
    use rapier3d::prelude::*;

    pub(super) struct Sim {
        world: PhysicsWorld,
        projectiles: Vec<RigidBodyHandle>,
        steps: usize,
    }
    impl Sim {
        pub fn new(steps: usize) -> Self {
            let mut world = PhysicsWorld::new();
            world.gravity = Vector::ZERO;
            world.integration_parameters.dt = DT / steps as f32;
            world.colliders.insert(
                ColliderBuilder::cuboid(0.05, 64.0, 2.0)
                    .translation(Vector::new(0.45, 0.0, 0.0))
                    .restitution(1.0)
                    .friction(0.0)
                    .build(),
            );
            let mut projectiles = Vec::new();
            for k in 0..COUNT {
                let id = world.bodies.insert(
                    RigidBodyBuilder::dynamic()
                        .translation(Vector::new(0.0, k as f32 * 0.5, 0.0))
                        .linvel(Vector::new(SPEED, 0.0, 0.0))
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
                    id,
                    &mut world.bodies,
                );
                projectiles.push(id);
            }
            Self {
                world,
                projectiles,
                steps,
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, _tick: u32, trace: &mut Trace) {
            for _ in 0..self.steps {
                self.world.step();
            }
            for &id in &self.projectiles {
                let b = &self.world.bodies[id];
                let p = b.translation();
                let v = b.linvel();
                trace.0.push(Obs::Position([p.x, p.y, p.z]));
                trace.0.push(Obs::Velocity([v.x, v.y, v.z]));
            }
        }
    }
}
