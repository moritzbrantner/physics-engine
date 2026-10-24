//! C18 mixed materials (physics-engine #391, mmorpg#442): MMORPG's crates (friction 0.8,
//! restitution 0.15) slide against geometry left at Rapier's collider defaults (0.5, 0), against
//! each other and against non-default combine rules. Rapier averages a pair's coefficients by
//! default. Under the engine's former larger-friction rule lane 0 would slide like lane 1, about
//! 19% shorter than Rapier. Upright 0.7 m crates of density 600 with linear damping 0.2 launched
//! at 5.91 m/s, gravity 9 m/s², one 1/30 s step per tick, one world per side.
//!
//! Lanes, 3 m apart along Z:
//! - 0: on the default floor plane (friction 0.65);
//! - 1: on a fixed crate slab (crate on crate, 0.8);
//! - 2: on a slab of friction 0.5 with `Multiply` (0.4);
//! - 3: on a slab of friction 1.0 with `Max` (1.0).
//!
//! Restitution combining is covered by the analytic contact-material contract: at 30 Hz the
//! tick-sampled rebound of a dropped crate depends on the landing phase, not on the coefficient,
//! on both engines.

use crate::{Obs, Scenario, Trace};

const TICKS: u32 = 120;
const DT: f32 = 1.0 / 30.0;
const GRAVITY: f32 = 9.0;
const HALF: f32 = 0.35;
const MASS: f32 = 600.0 * 8.0 * HALF * HALF * HALF;
const CRATE_FRICTION: f32 = 0.8;
const CRATE_RESTITUTION: f32 = 0.15;
const DAMPING: f32 = 0.2;
const LAUNCH: f32 = 5.91;
const LANES: usize = 4;
/// Slab half extents: 16 m long, as high as a crate.
const SLAB_HALF: [f32; 3] = [8.0, HALF, 1.0];
const SLAB_X: f32 = 6.0;

#[derive(Clone, Copy)]
enum Rule {
    Average,
    Multiply,
    Max,
}

/// Each lane's slab material (friction, restitution, rule), or `None` for the floor plane.
fn slab(l: usize) -> Option<(f32, f32, Rule)> {
    match l {
        1 => Some((CRATE_FRICTION, CRATE_RESTITUTION, Rule::Average)),
        2 => Some((0.5, 0.0, Rule::Multiply)),
        3 => Some((1.0, 0.0, Rule::Max)),
        _ => None,
    }
}

fn ground(l: usize) -> f32 {
    if slab(l).is_some() { 2.0 * HALF } else { 0.0 }
}

fn crate_start(l: usize) -> [f32; 3] {
    [0.0, ground(l) + HALF, 3.0 * l as f32]
}

fn slab_centre(l: usize) -> [f32; 3] {
    [SLAB_X, HALF, 3.0 * l as f32]
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "materials",
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

/// Per tick and lane: the crate's position.
fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    if engine.0.len() != rapier.0.len() || engine.0.len() != LANES * TICKS as usize {
        return Err("trace lengths differ".into());
    }
    let at = |t: &Trace, tick: usize, l: usize| match t.0[tick * LANES + l] {
        Obs::Position(p) => p,
        _ => [f32::NAN; 3],
    };
    let end = TICKS as usize - 1;
    for l in 0..LANES {
        let start = crate_start(l);
        for tick in 0..TICKS as usize {
            let p = at(engine, tick, l);
            // Upright on its ground, in its lane.
            if (p[1] - start[1]).abs() > 0.01 || (p[2] - start[2]).abs() > 1e-4 {
                return Err(format!("lane {l} left its ground at tick {tick}: {p:?}"));
            }
        }
        let (p, q, r) = (
            at(engine, end, l),
            at(engine, end - 5, l),
            at(rapier, end, l),
        );
        if (p[0] - q[0]).abs() > 1e-4 {
            return Err(format!("lane {l} still moving: {q:?} -> {p:?}"));
        }
        // Rests within 5% of Rapier's slide of where Rapier's crate rests (the `props` bound).
        let slide = r[0] - start[0];
        if (p[0] - r[0]).abs() > 0.05 * slide + 0.01 {
            return Err(format!(
                "lane {l} slides {:.4} m, Rapier {slide:.4} m",
                p[0] - start[0]
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
        approximate::{Body, CombineRule, Config, Shape, Vector, World},
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
                // As in `props`: two substeps of one pass, the MMORPG adapter's crate step.
                substeps: 2,
                velocity_iterations: 1,
                length_unit: 1.0,
                ..Config::default()
            })
            .unwrap();
            // The floor plane keeps the defaults: friction 0.5, restitution 0, Average.
            world
                .add_body(Body::new(Shape::HalfSpace, Vector::ZERO, 0.0))
                .unwrap();
            for l in 0..LANES {
                let Some((friction, restitution, rule)) = slab(l) else {
                    continue;
                };
                let mut body = Body::new(Shape::Box(v(SLAB_HALF)), v(slab_centre(l)), 0.0);
                body.friction = friction;
                body.restitution = restitution;
                body.friction_combine = match rule {
                    Rule::Average => CombineRule::Average,
                    Rule::Multiply => CombineRule::Multiply,
                    Rule::Max => CombineRule::Max,
                };
                world.add_body(body).unwrap();
            }
            let crates = (0..LANES)
                .map(|l| {
                    let mut body = Body::new(Shape::Box(v([HALF; 3])), v(crate_start(l)), MASS);
                    body.velocity = Vector(LAUNCH, 0.0, 0.0);
                    body.rotation_locked = true;
                    body.friction = CRATE_FRICTION;
                    body.restitution = CRATE_RESTITUTION;
                    body.linear_damping = DAMPING;
                    world.add_body(body).unwrap()
                })
                .collect();
            Self { world, crates }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, _tick: u32, trace: &mut Trace) {
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

    pub struct Sim {
        world: PhysicsWorld,
        crates: Vec<RigidBodyHandle>,
    }

    impl Sim {
        pub fn new() -> Self {
            let mut world = PhysicsWorld::new();
            world.gravity = Vector::new(0.0, -GRAVITY, 0.0);
            world.integration_parameters.dt = DT;
            world
                .colliders
                .insert(ColliderBuilder::new(SharedShape::halfspace(Vector::Y)).build());
            for l in 0..LANES {
                let Some((friction, restitution, rule)) = slab(l) else {
                    continue;
                };
                let rule = match rule {
                    Rule::Average => CoefficientCombineRule::Average,
                    Rule::Multiply => CoefficientCombineRule::Multiply,
                    Rule::Max => CoefficientCombineRule::Max,
                };
                world.colliders.insert(
                    ColliderBuilder::cuboid(SLAB_HALF[0], SLAB_HALF[1], SLAB_HALF[2])
                        .translation(v(slab_centre(l)))
                        .friction(friction)
                        .restitution(restitution)
                        .friction_combine_rule(rule)
                        .build(),
                );
            }
            let crates = (0..LANES)
                .map(|l| {
                    let body = world.bodies.insert(
                        RigidBodyBuilder::dynamic()
                            .translation(v(crate_start(l)))
                            .linvel(Vector::new(LAUNCH, 0.0, 0.0))
                            .linear_damping(DAMPING)
                            .lock_rotations()
                            .build(),
                    );
                    world.colliders.insert_with_parent(
                        ColliderBuilder::cuboid(HALF, HALF, HALF)
                            .friction(CRATE_FRICTION)
                            .restitution(CRATE_RESTITUTION)
                            .density(600.0)
                            .build(),
                        body,
                        &mut world.bodies,
                    );
                    body
                })
                .collect();
            Self { world, crates }
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, _tick: u32, trace: &mut Trace) {
            self.world.step();
            for &h in &self.crates {
                let p = self.world.bodies[h].translation();
                trace.0.push(Obs::Position([p.x, p.y, p.z]));
            }
        }
    }
}
