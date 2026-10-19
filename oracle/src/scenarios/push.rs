//! C16 (MMORPG): bounded pushing. 24 lanes each hold a pusher walking towards a wall and a
//! resting character in its way; the pusher pushes it up to the wall (`max_push` = one step per
//! tick), then sidesteps
//! around it. Rapier mirrors MMORPG: a world rebuild per tick, `move_shape` per
//! character in id order (Rapier characters never push: the pusher sidesteps or blocks).

use crate::{Obs, Scenario, Trace};

const TICKS: u32 = 240;
const DT: f32 = 1.0 / 60.0;
const OFFSET: f32 = 0.0025;
const HALF: f32 = 0.5;
const RADIUS: f32 = 0.3;
const REST: f32 = HALF + RADIUS + OFFSET;
const LANES: usize = 24;
const SPEED: f32 = 1.5;
/// Face of the wall the lanes run into, and the wall's half extents and centre.
const WALL_X: f32 = 4.0;
const WALL_HALF: [f32; 3] = [0.25, 2.0, 30.0];
const WALL: [f32; 3] = [WALL_X + 0.25, 2.0, 0.0];
const GAP: f32 = 2.0 * RADIUS + OFFSET;

/// Start position and desired motion per tick of every character, in id order: pushers on
/// even ids, the characters they push on odd ids.
fn characters() -> Vec<([f32; 3], [f32; 3])> {
    let mut out = Vec::new();
    for k in 0..LANES {
        let z = -23.0 + 2.0 * k as f32;
        out.push(([-2.0, REST, z], [SPEED * DT, 0.0, 0.0]));
        out.push(([-2.0 + GAP, REST, z], [0.0; 3]));
    }
    out
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "push",
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
    let step = SPEED * DT;
    for tick in 0..TICKS as usize {
        for k in 0..LANES {
            let (a, b) = (at(tick, 2 * k), at(tick, 2 * k + 1));
            let z = chars[2 * k].0[2];
            // The pushed character stays in its lane; once it is pinned at the wall the
            // pusher sidesteps around it (head-on rule).
            if a[1] != REST || b[1] != REST || b[2] != z {
                return Err(format!(
                    "lane {k} left its line at tick {tick}: {a:?} {b:?}"
                ));
            }
            if ((b[0] - a[0]).powi(2) + (b[2] - a[2]).powi(2)).sqrt() < GAP - 1e-4 {
                return Err(format!("lane {k} overlaps at tick {tick}: {a:?} {b:?}"));
            }
            if b[0] + RADIUS + OFFSET > WALL_X + 1e-4 {
                return Err(format!(
                    "lane {k} pushed into the wall at tick {tick}: {b:?}"
                ));
            }
            let before = if tick == 0 {
                chars[2 * k + 1].0[0]
            } else {
                at(tick - 1, 2 * k + 1)[0]
            };
            if b[0] - before > step + 1e-4 || b[0] < before {
                return Err(format!("lane {k} pushed beyond the ceiling at tick {tick}"));
            }
        }
    }
    let last = TICKS as usize - 1;
    for k in 0..LANES {
        let (a, b) = (at(last, 2 * k), at(last, 2 * k + 1));
        if (b[0] + RADIUS + OFFSET - WALL_X).abs() > 1e-4 || a[2] == b[2] {
            return Err(format!("lane {k} did not push up to the wall: {a:?} {b:?}"));
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
        directions: Vec<[f32; 3]>,
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
            world
                .add_body(Body::keyed(2, Shape::Box(v(WALL_HALF)), v(WALL), 0.0))
                .unwrap();
            let settings = CharacterSettings {
                offset: OFFSET,
                plane: Some(REST),
                max_push: Some(SPEED * DT),
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
                    desired: v(self.directions[i]),
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
        directions: Vec<[f32; 3]>,
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
        world.colliders.insert(
            ColliderBuilder::cuboid(WALL_HALF[0], WALL_HALF[1], WALL_HALF[2])
                .translation(v(WALL))
                .build(),
        );
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
                    v(self.directions[i]),
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
