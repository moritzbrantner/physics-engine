//! CCD spheres each starting a few millimetres or less from an elastic wall they are closing
//! on. A CCD body that starts a tick inside the contact margin must still consume the whole
//! tick (impact at its time of impact, then the remaining time along the response), instead of
//! stopping at the wall on a speculative row and losing the rest of the tick's travel (found
//! by the #465 corridor experiment; `tests/ccd_margin_start.rs` covers the multi-bounce
//! corridor analytically).

use crate::{Obs, Scenario, Trace};

const DT: f32 = 1.0 / 60.0;
const TICKS: u32 = 60;
const SPEED: f32 = 30.0;
const RADIUS: f32 = 0.196;
/// The wall's face is at `x = FACE`; it is `2 * HALF` thick.
const FACE: f32 = 0.4;
const HALF: f32 = 0.05;
/// Start gaps to the +x wall's face, metres; the first two start outside the contact margin.
const GAPS: [f32; 4] = [0.1, 0.02, 0.003, 0.0005];
const DENSITY: f32 = 600.0;
#[cfg(feature = "engine")]
const MASS: f32 = 4.0 / 3.0 * std::f32::consts::PI * RADIUS * RADIUS * RADIUS * DENSITY;

pub fn scenario() -> Scenario {
    Scenario {
        name: "ccd-margin-start",
        capabilities: "C20",
        ticks: TICKS,
        #[cfg(feature = "engine")]
        engine: |storage| Box::new(engine::Sim::new(storage)),
        #[cfg(feature = "rapier")]
        rapier: || Box::new(rapier::Sim::new(64)),
        #[cfg(feature = "rapier")]
        rapier_raw: Some(|| Box::new(rapier::Sim::new(1))),
        check,
    }
}

fn start(gap: f32) -> f32 {
    FACE - RADIUS - gap
}

fn lane_z(k: usize) -> f32 {
    k as f32 * 2.0
}

/// One elastic reflection off the wall face, after `gap / SPEED` seconds.
fn analytic(gap: f32, t: f64) -> (f32, f32) {
    let limit = f64::from(FACE - RADIUS);
    let x = f64::from(start(gap)) + f64::from(SPEED) * t;
    if x <= limit {
        (x as f32, SPEED)
    } else {
        ((2.0 * limit - x) as f32, -SPEED)
    }
}

fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    let lanes = GAPS.len();
    let expected = TICKS as usize * lanes * 2;
    if engine.0.len() != expected || rapier.0.len() != expected {
        return Err(format!(
            "margin-start observations: expected {expected}, got {} and {}",
            engine.0.len(),
            rapier.0.len()
        ));
    }
    let at = |trace: &Trace, tick: usize, k: usize| {
        let offset = (tick * lanes + k) * 2;
        match (&trace.0[offset], &trace.0[offset + 1]) {
            (Obs::Position(p), Obs::Velocity(v)) => Ok((*p, *v)),
            _ => Err(format!("bad observations for lane {k} tick {tick}")),
        }
    };
    for (k, &gap) in GAPS.iter().enumerate() {
        // First tick: high-resolution Rapier advances the same interval in 64 steps.
        let (p, v) = at(engine, 0, k)?;
        let (rp, rv) = at(rapier, 0, k)?;
        if (p[0] - rp[0]).abs() > 0.05 || (v[0] - rv[0]).abs() > 0.05 * rv[0].abs() + 0.5 {
            return Err(format!(
                "lane {k} (gap {gap}): engine {p:?}/{v:?}, Rapier {rp:?}/{rv:?} after tick 0"
            ));
        }
        // Every tick: the analytic reflected path.
        for tick in 0..TICKS as usize {
            let (p, v) = at(engine, tick, k)?;
            let (x, vx) = analytic(gap, f64::from(DT) * (tick + 1) as f64);
            if (p[0] - x).abs() > 0.01 || (v[0] - vx).abs() > 0.1 || p[0] > FACE - RADIUS + 0.005 {
                return Err(format!(
                    "lane {k} (gap {gap}) tick {tick}: engine {p:?}/{v:?}, analytic x {x} vx {vx}"
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
        balls: Vec<BodyId>,
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
                Shape::Box(Vector(HALF, 2.0, 8.0)),
                Vector(FACE + HALF, 0.0, 3.0),
                0.0,
            );
            wall.restitution = 1.0;
            wall.friction = 0.0;
            world.add_body(wall).unwrap();
            let balls = GAPS
                .iter()
                .enumerate()
                .map(|(k, &gap)| {
                    let mut ball = Body::new(
                        Shape::Sphere(RADIUS),
                        Vector(start(gap), 0.0, lane_z(k)),
                        MASS,
                    );
                    ball.velocity = Vector(SPEED, 0.0, 0.0);
                    ball.restitution = 1.0;
                    ball.friction = 0.0;
                    ball.ccd = true;
                    ball.sleep_allowed = false;
                    world.add_body(ball).unwrap()
                })
                .collect();
            Self { world, balls }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, _tick: u32, trace: &mut Trace) {
            self.world.step(f64::from(DT)).unwrap();
            for &id in &self.balls {
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
        balls: Vec<RigidBodyHandle>,
        steps: usize,
    }
    impl Sim {
        pub fn new(steps: usize) -> Self {
            let mut world = PhysicsWorld::new();
            world.gravity = Vector::ZERO;
            world.integration_parameters.dt = DT / steps as f32;
            world.colliders.insert(
                ColliderBuilder::cuboid(HALF, 2.0, 8.0)
                    .translation(Vector::new(FACE + HALF, 0.0, 3.0))
                    .restitution(1.0)
                    .friction(0.0)
                    .build(),
            );
            let balls = GAPS
                .iter()
                .enumerate()
                .map(|(k, &gap)| {
                    let id = world.bodies.insert(
                        RigidBodyBuilder::dynamic()
                            .translation(Vector::new(start(gap), 0.0, lane_z(k)))
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
                    id
                })
                .collect();
            Self {
                world,
                balls,
                steps,
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, _tick: u32, trace: &mut Trace) {
            for _ in 0..self.steps {
                self.world.step();
            }
            for &id in &self.balls {
                let b = &self.world.bodies[id];
                let p = b.translation();
                let v = b.linvel();
                trace.0.push(Obs::Position([p.x, p.y, p.z]));
                trace.0.push(Obs::Velocity([v.x, v.y, v.z]));
            }
        }
    }
}
