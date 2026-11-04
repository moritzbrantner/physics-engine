//! Fast parallel CCD spheres have overlapping swept *world* bounds but no relative approach.
//! Neither engine may create a collision or change their common velocity.
use crate::{Obs, Scenario, Trace};

const COUNT: usize = 8;
const TICKS: u32 = 32;
const DT: f32 = 1.0 / 60.0;
const RADIUS: f32 = 0.1;
const SPACING: f32 = 0.3;
const SPEED: f32 = (COUNT as f32 * SPACING + 1.0) * 60.0;
const DENSITY: f32 = 600.0;
#[cfg(feature = "engine")]
const MASS: f32 = 4.0 / 3.0 * std::f32::consts::PI * RADIUS * RADIUS * RADIUS * DENSITY;

pub fn scenario() -> Scenario {
    Scenario {
        name: "ccd-parallel",
        capabilities: "C20",
        ticks: TICKS,
        #[cfg(feature = "engine")]
        engine: |storage| Box::new(engine::Sim::new(storage)),
        #[cfg(feature = "rapier")]
        rapier: || Box::new(rapier::Sim::new()),
        #[cfg(feature = "rapier")]
        rapier_raw: None,
        check,
    }
}

fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    let expected = COUNT * TICKS as usize * 2;
    if engine.0.len() != expected || rapier.0.len() != expected {
        return Err(format!(
            "parallel CCD observations: expected {expected}, got {} and {}",
            engine.0.len(),
            rapier.0.len()
        ));
    }
    for tick in 0..TICKS as usize {
        for body in 0..COUNT {
            let index = (tick * COUNT + body) * 2;
            let expected_x = body as f32 * SPACING + SPEED * DT * (tick + 1) as f32;
            for (name, trace) in [("engine", engine), ("Rapier", rapier)] {
                let (Obs::Position(p), Obs::Velocity(v)) =
                    (&trace.0[index], &trace.0[index + 1])
                else {
                    return Err(format!("{name}: invalid observation {index}"));
                };
                if (p[0] - expected_x).abs() > 0.01
                    || p[1].abs() > 0.001
                    || p[2].abs() > 0.001
                    || (v[0] - SPEED).abs() > 0.01
                    || v[1].abs() > 0.001
                    || v[2].abs() > 0.001
                {
                    return Err(format!(
                        "{name} body {body} tick {tick}: {p:?}/{v:?}, expected x={expected_x} vx={SPEED}"
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

    pub(super) struct Sim {
        world: World,
        ids: Vec<BodyId>,
    }
    impl Sim {
        pub fn new(storage: BodyStorage) -> Self {
            let mut world = World::new(Config {
                body_storage: storage,
                gravity: Vector::ZERO,
                substeps: 1,
                ..Config::default()
            })
            .unwrap();
            let mut ids = Vec::new();
            for index in 0..COUNT {
                let mut body = Body::new(
                    Shape::Sphere(RADIUS),
                    Vector(index as f32 * SPACING, 0.0, 0.0),
                    MASS,
                );
                body.velocity = Vector(SPEED, 0.0, 0.0);
                body.ccd = true;
                body.sleep_allowed = false;
                ids.push(world.add_body(body).unwrap());
            }
            Self { world, ids }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, _tick: u32, trace: &mut Trace) {
            let report = self.world.step(f64::from(DT)).unwrap();
            assert_eq!(report.swept_contacts, 0);
            for &id in &self.ids {
                let body = self.world.body(id).unwrap();
                trace.0.push(Obs::Position([
                    body.position.0,
                    body.position.1,
                    body.position.2,
                ]));
                trace.0.push(Obs::Velocity([
                    body.velocity.0,
                    body.velocity.1,
                    body.velocity.2,
                ]));
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
        ids: Vec<RigidBodyHandle>,
    }
    impl Sim {
        pub fn new() -> Self {
            let mut world = PhysicsWorld::new();
            world.gravity = Vector::ZERO;
            world.integration_parameters.dt = DT;
            let mut ids = Vec::new();
            for index in 0..COUNT {
                let handle = world.bodies.insert(
                    RigidBodyBuilder::dynamic()
                        .translation(Vector::new(index as f32 * SPACING, 0.0, 0.0))
                        .linvel(Vector::new(SPEED, 0.0, 0.0))
                        .ccd_enabled(true)
                        .can_sleep(false)
                        .build(),
                );
                world.colliders.insert_with_parent(
                    ColliderBuilder::ball(RADIUS).density(DENSITY).build(),
                    handle,
                    &mut world.bodies,
                );
                ids.push(handle);
            }
            Self { world, ids }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, _tick: u32, trace: &mut Trace) {
            self.world.step();
            for &id in &self.ids {
                let body = &self.world.bodies[id];
                let p = body.translation();
                let v = body.linvel();
                trace.0.push(Obs::Position([p.x, p.y, p.z]));
                trace.0.push(Obs::Velocity([v.x, v.y, v.z]));
            }
        }
    }
}
