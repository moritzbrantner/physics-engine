//! C6/C8 (ARPG): planar monsters steering through the dungeon of `static-queries`, some
//! placed into pillars (as when a door closes or a spawn overlaps). Rapier mirrors ARPG's
//! call pattern: a world rebuild per tick (#292), the planar `KinematicCharacterController`
//! (no snapping) per monster with the halfspace floor excluded, `set_translation` and
//! `detect_collisions` after each move.

use super::character_move::capsule_box_gap;
use super::static_queries::{BoxSpec, FLOOR, layout};
use crate::{Obs, Rng, Scenario, Trace};

const TICKS: u32 = 240;
const DT: f32 = 1.0 / 60.0;
const OFFSET: f32 = 0.0025;
const HALF: f32 = 0.5;
const RADIUS: f32 = 0.3;
/// Monster centre height: resting one offset above the floor.
const HEIGHT: f32 = HALF + RADIUS + OFFSET;
const MONSTERS: usize = 48;
/// Monsters placed into a pillar.
const OVERLAPPING: usize = 8;

/// Static boxes (floor first) and monster start positions.
fn scene() -> (Vec<BoxSpec>, Vec<[f32; 3]>) {
    let (walls, doors) = layout();
    let mut boxes = vec![FLOOR];
    boxes.extend(walls.iter().copied());
    boxes.extend(doors);
    // Pillars with free space around them, so leaving the overlap has a clear way out.
    let pillars: Vec<BoxSpec> = walls
        .iter()
        .copied()
        .filter(|(c, h)| {
            h[0] == 0.5
                && boxes[1..]
                    .iter()
                    .filter(|(o, _)| o != c)
                    .all(|b| capsule_box_gap([c[0], HEIGHT, c[2]], *b) > 1.5)
        })
        .collect();
    assert!(pillars.len() >= OVERLAPPING);
    let mut rng = Rng(0xa1b2_c3d4_e5f6_0718);
    let mut starts = Vec::new();
    for (c, _) in pillars.iter().take(OVERLAPPING) {
        // 0.25 into the pillar's side: penetrating by 0.55.
        starts.push([c[0] + 0.25, HEIGHT, c[2]]);
    }
    while starts.len() < MONSTERS {
        let p = [rng.range(-29.0, 29.0), HEIGHT, rng.range(-29.0, 29.0)];
        if boxes[1..].iter().all(|b| capsule_box_gap(p, *b) > 0.2) {
            starts.push(p);
        }
    }
    (boxes, starts)
}

/// Steering intent shared by both engines: a new heading every 30 ticks, 3 m/s.
fn desired(index: usize, tick: u32) -> [f32; 3] {
    let mut rng = Rng(0x51 + index as u64 * 7_919 + u64::from(tick / 30));
    let a = rng.range(0.0, core::f32::consts::TAU);
    [3.0 * a.cos() * DT, 0.0, 3.0 * a.sin() * DT]
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "arpg-planar",
        capabilities: "C2 C6 C8",
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
    let (boxes, _) = scene();
    if engine.0.len() != rapier.0.len() || engine.0.len() != 2 * MONSTERS * TICKS as usize {
        return Err("trace lengths differ".into());
    }
    for tick in 0..TICKS as usize {
        for i in 0..MONSTERS {
            let base = (tick * MONSTERS + i) * 2;
            let (Obs::Position(p), Obs::Depenetration(d)) = (engine.0[base], engine.0[base + 1])
            else {
                return Err("unexpected observation".into());
            };
            if p[1] != HEIGHT {
                return Err(format!("monster {i} left its plane at tick {tick}: {p:?}"));
            }
            let pushed = d != [0.0; 3];
            if pushed != (tick == 0 && i < OVERLAPPING) {
                return Err(format!("monster {i} depenetration {d:?} at tick {tick}"));
            }
            if d[1] != 0.0 {
                return Err(format!("monster {i} depenetrated off its plane: {d:?}"));
            }
            if tick % 20 == 0 || tick + 1 == TICKS as usize {
                for b in &boxes[1..] {
                    if capsule_box_gap(p, *b) < -1e-3 {
                        return Err(format!("monster {i} penetrates a box at tick {tick}"));
                    }
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
            let (boxes, starts) = scene();
            for (i, (c, h)) in boxes.into_iter().enumerate() {
                world
                    .add_body(if i == 0 {
                        // The consumers' ground is a halfspace through the floor box's top.
                        Body::keyed(1, Shape::HalfSpace, Vector::ZERO, 0.0)
                    } else {
                        Body::keyed(1 + i as u64, Shape::Box(v(h)), v(c), 0.0)
                    })
                    .unwrap();
            }
            let settings = CharacterSettings {
                offset: OFFSET,
                snap_to_ground: None,
                plane: Some(HEIGHT),
                ..CharacterSettings::default()
            };
            let ids = starts
                .into_iter()
                .enumerate()
                .map(|(i, p)| {
                    let mut body =
                        Body::keyed(10_000 + i as u64, Shape::capsule(HALF, RADIUS), v(p), 1.0);
                    body.external = true;
                    let id = world.add_body(body).unwrap();
                    world.add_character(id, settings).unwrap();
                    id
                })
                .collect();
            Self {
                world,
                ids,
                moves: Vec::with_capacity(MONSTERS),
                results: Vec::with_capacity(MONSTERS),
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            self.moves.clear();
            for (i, &body) in self.ids.iter().enumerate() {
                self.moves.push(CharacterMove {
                    body,
                    desired: v(desired(i, tick)),
                    exclude: None,
                    autostep: None,
                });
            }
            self.world
                .move_characters(&self.moves, &mut self.results)
                .unwrap();
            for r in &self.results {
                let p = self.world.body(r.body).unwrap().position;
                let d = r.depenetration;
                trace.0.push(Obs::Position([p.0, p.1, p.2]));
                trace.0.push(Obs::Depenetration([d.0, d.1, d.2]));
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
        shape: Capsule,
        rebuild: bool,
    }

    /// Halfspace floor, static boxes, then monsters at `positions`, in key order.
    fn build(
        positions: &[Vector],
        shape: Capsule,
    ) -> (PhysicsWorld, ColliderHandle, Vec<RigidBodyHandle>) {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::ZERO;
        world.integration_parameters.dt = DT;
        let (boxes, _) = scene();
        for (c, h) in boxes.into_iter().skip(1) {
            world.colliders.insert(
                ColliderBuilder::cuboid(h[0], h[1], h[2])
                    .translation(v(c))
                    .build(),
            );
        }
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
        // ARPG inserts the floor last (its insertion changes how ties break).
        let floor = world
            .colliders
            .insert(ColliderBuilder::new(SharedShape::halfspace(Vector::Y)).build());
        world.detect_collisions(&(), &());
        (world, floor, bodies)
    }

    impl Sim {
        pub fn new(rebuild: bool) -> Self {
            let shape = Capsule::new_y(HALF, RADIUS);
            let (_, starts) = scene();
            let positions: Vec<Vector> = starts.into_iter().map(v).collect();
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
                shape,
                rebuild,
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
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
                    v(desired(i, tick)),
                    |_| {},
                );
                // ARPG keeps monsters on the plane.
                let to = from + Vector::new(movement.translation.x, 0.0, movement.translation.z);
                self.positions[i] = to;
                self.world.bodies[body].set_translation(to, false);
                self.world.detect_collisions(&(), &());
                trace.0.push(Obs::Position([to.x, to.y, to.z]));
                trace.0.push(Obs::Depenetration([0.0; 3]));
            }
        }
    }
}
