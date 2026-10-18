//! C7/C16 (MMORPG): a crowd on an open plaza. 32 pairs of characters walk head-on into each
//! other and 16 more cross their paths; all moves of a tick form one batch. Rapier mirrors
//! MMORPG: a world rebuild per tick, `move_shape` per character in id order and
//! `detect_collisions` after each move (head-on Rapier characters block each other).

use crate::{Obs, Scenario, Trace};

const TICKS: u32 = 300;
const DT: f32 = 1.0 / 60.0;
const OFFSET: f32 = 0.0025;
const HALF: f32 = 0.5;
const RADIUS: f32 = 0.3;
const REST: f32 = HALF + RADIUS + OFFSET;
const PAIRS: usize = 32;
const CROSSERS: usize = 16;
const SPEED: f32 = 1.5;

/// Start position and walking direction (unit, horizontal) of every character, in id order.
fn characters() -> Vec<([f32; 3], [f32; 2])> {
    let mut out = Vec::new();
    for k in 0..PAIRS {
        let x = -31.0 + 2.0 * k as f32;
        out.push(([x, REST, -4.0], [0.0, 1.0]));
        out.push(([x, REST, 4.0], [0.0, -1.0]));
    }
    for k in 0..CROSSERS {
        let z = -7.5 + k as f32;
        out.push(([-40.0, REST, z], [1.0, 0.0]));
    }
    out
}

fn desired(direction: [f32; 2]) -> [f32; 3] {
    [direction[0] * SPEED * DT, 0.0, direction[1] * SPEED * DT]
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "crowd",
        capabilities: "C2 C7 C16",
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

fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    let chars = characters();
    let n = chars.len();
    if engine.0.len() != rapier.0.len() || engine.0.len() != n * TICKS as usize {
        return Err("trace lengths differ".into());
    }
    let at = |tick: usize, i: usize| match engine.0[tick * n + i] {
        Obs::Position(p) => p,
        _ => [f32::NAN; 3],
    };
    for tick in (0..TICKS as usize).step_by(5) {
        for i in 0..n {
            let a = at(tick, i);
            if a[1] != REST {
                return Err(format!(
                    "character {i} left the ground plane at tick {tick}"
                ));
            }
            for j in i + 1..n {
                let b = at(tick, j);
                let d = ((a[0] - b[0]).powi(2) + (a[2] - b[2]).powi(2)).sqrt();
                if d < 2.0 * RADIUS - 1e-3 {
                    return Err(format!(
                        "characters {i} and {j} overlap by {} at tick {tick}",
                        2.0 * RADIUS - d
                    ));
                }
            }
        }
    }
    let last = TICKS as usize - 1;
    for k in 0..PAIRS {
        let (a, b) = (at(last, 2 * k), at(last, 2 * k + 1));
        if a[2] < 2.0 || b[2] > -2.0 {
            return Err(format!("head-on pair {k} did not pass: {a:?} {b:?}"));
        }
    }
    Ok(())
}

#[cfg(feature = "engine")]
mod engine {
    use super::*;
    use physics_engine::{
        BodyId,
        approximate::{
            Body, CharacterMove, CharacterMoveResult, CharacterSettings, Config, Shape, Vector,
            World,
        },
    };

    fn v(a: [f32; 3]) -> Vector {
        Vector(a[0], a[1], a[2])
    }

    pub struct Sim {
        world: World,
        ids: Vec<BodyId>,
        directions: Vec<[f32; 2]>,
        moves: Vec<CharacterMove>,
        results: Vec<CharacterMoveResult>,
    }
    impl Sim {
        pub fn new() -> Self {
            let mut world = World::new(Config {
                body_storage: crate::engine_storage(),
                gravity: Vector::ZERO,
                ..Config::default()
            })
            .unwrap();
            world
                .add_body(Body::keyed(1, Shape::HalfSpace, Vector::ZERO, 0.0))
                .unwrap();
            let settings = CharacterSettings {
                offset: OFFSET,
                plane: Some(REST),
                ..CharacterSettings::default()
            };
            let mut ids = Vec::new();
            let mut directions = Vec::new();
            for (i, (p, d)) in characters().into_iter().enumerate() {
                let mut body =
                    Body::keyed(1_000 + i as u64, Shape::capsule(HALF, RADIUS), v(p), 1.0);
                body.external = true;
                let id = world.add_body(body).unwrap();
                world.add_character(id, settings).unwrap();
                ids.push(id);
                directions.push(d);
            }
            Self {
                world,
                ids,
                directions,
                moves: Vec::new(),
                results: Vec::new(),
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, _tick: u32, trace: &mut Trace) {
            self.moves.clear();
            for (i, &body) in self.ids.iter().enumerate() {
                self.moves.push(CharacterMove {
                    body,
                    desired: v(desired(self.directions[i])),
                });
            }
            self.world
                .move_characters(&self.moves, &mut self.results)
                .unwrap();
            for r in &self.results {
                let p = self.world.body(r.body).unwrap().position;
                trace.0.push(Obs::Position([p.0, p.1, p.2]));
            }
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::*;
    use rapier3d::control::{CharacterLength, KinematicCharacterController};
    use rapier3d::parry::query::DefaultQueryDispatcher;
    use rapier3d::prelude::*;

    fn v(a: [f32; 3]) -> Vector {
        Vector::new(a[0], a[1], a[2])
    }

    pub struct Sim {
        world: PhysicsWorld,
        floor: ColliderHandle,
        controller: KinematicCharacterController,
        bodies: Vec<RigidBodyHandle>,
        positions: Vec<Vector>,
        directions: Vec<[f32; 2]>,
        shape: Capsule,
        rebuild: bool,
    }

    fn build(
        positions: &[Vector],
        shape: Capsule,
    ) -> (PhysicsWorld, ColliderHandle, Vec<RigidBodyHandle>) {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::ZERO;
        world.integration_parameters.dt = DT;
        let floor = world
            .colliders
            .insert(ColliderBuilder::new(SharedShape::halfspace(Vector::Y)).build());
        let bodies = positions
            .iter()
            .map(|&p| {
                let body = world.bodies.insert(
                    RigidBodyBuilder::kinematic_position_based()
                        .translation(p)
                        .build(),
                );
                world.colliders.insert_with_parent(
                    ColliderBuilder::new(SharedShape::new(shape)).build(),
                    body,
                    &mut world.bodies,
                );
                body
            })
            .collect();
        world.detect_collisions(&(), &());
        (world, floor, bodies)
    }

    impl Sim {
        pub fn new(rebuild: bool) -> Self {
            let shape = Capsule::new_y(HALF, RADIUS);
            let (positions, directions): (Vec<_>, Vec<_>) =
                characters().into_iter().map(|(p, d)| (v(p), d)).unzip();
            let (world, floor, bodies) = build(&positions, shape);
            Self {
                world,
                floor,
                controller: KinematicCharacterController {
                    up: Vector::Y,
                    offset: CharacterLength::Absolute(OFFSET),
                    slide: true,
                    autostep: None,
                    max_slope_climb_angle: core::f32::consts::FRAC_PI_4,
                    min_slope_slide_angle: core::f32::consts::FRAC_PI_4,
                    snap_to_ground: None,
                    normal_nudge_factor: KinematicCharacterController::default()
                        .normal_nudge_factor,
                },
                bodies,
                positions,
                directions,
                shape,
                rebuild,
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, _tick: u32, trace: &mut Trace) {
            if self.rebuild {
                (self.world, self.floor, self.bodies) = build(&self.positions, self.shape);
            }
            let dispatcher = DefaultQueryDispatcher;
            for i in 0..self.bodies.len() {
                let body = self.bodies[i];
                let from = self.positions[i];
                let w = &self.world;
                let filter = QueryFilter::default()
                    .exclude_rigid_body(body)
                    .exclude_collider(self.floor);
                let queries =
                    w.broad_phase
                        .as_query_pipeline(&dispatcher, &w.bodies, &w.colliders, filter);
                let movement = self.controller.move_shape(
                    DT,
                    &queries,
                    &self.shape,
                    &Pose::from_translation(from),
                    v(desired(self.directions[i])),
                    |_| {},
                );
                let to = from + Vector::new(movement.translation.x, 0.0, movement.translation.z);
                self.positions[i] = to;
                self.world.bodies[body].set_translation(to, false);
                self.world.detect_collisions(&(), &());
                trace.0.push(Obs::Position([to.x, to.y, to.z]));
            }
        }
    }
}
