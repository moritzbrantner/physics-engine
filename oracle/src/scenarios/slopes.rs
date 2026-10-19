//! C4/C5 (MMORPG ramps, #59): characters walking under gravity into wedge ramps on a
//! halfspace ground. Ramps up to `max_slope` (45°) are climbed; steeper ones are walls, and the
//! character stays at their foot instead of climbing and sliding back. Rapier mirrors MMORPG:
//! wedges as convex hulls, a world rebuild per tick, `move_shape` per character in id order.

use crate::{Obs, Scenario, Trace};

const TICKS: u32 = 240;
const DT: f32 = 1.0 / 60.0;
const GRAVITY: f32 = 9.81;
const OFFSET: f32 = 0.0025;
const SNAP: f32 = 0.05;
const HALF: f32 = 0.5;
const RADIUS: f32 = 0.3;
const REST: f32 = HALF + RADIUS + OFFSET;
const SPEED: f32 = 2.0;
const LANES: usize = 24;
/// Every ramp's foot (its +X end); ramps rise toward -X.
const FOOT_X: f32 = 10.0;
const HEIGHT: f32 = 1.5;
/// Positions of the two engines' steep-lane characters may differ by Rapier's normal nudge.
const RAPIER_TOLERANCE: f32 = 0.02;

/// Ramp angle (radians) of each lane: 12 climbable (15°–42°), 12 too steep (50°–72°).
fn angle(lane: usize) -> f32 {
    let deg = if lane < 12 {
        15.0 + 2.5 * lane as f32
    } else {
        50.0 + 2.0 * (lane - 12) as f32
    };
    deg.to_radians()
}

/// Centre and half extents of lane `lane`'s wedge.
fn ramp(lane: usize) -> ([f32; 3], [f32; 3]) {
    let hx = HEIGHT / angle(lane).tan() / 2.0;
    let z = -30.0 + 2.5 * lane as f32;
    ([FOOT_X - hx, HEIGHT / 2.0, z], [hx, HEIGHT / 2.0, 0.8])
}

fn start(lane: usize) -> [f32; 3] {
    [FOOT_X + 1.0, REST, ramp(lane).0[2]]
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "slopes",
        capabilities: "C2 C4 C5",
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
    if engine.0.len() != rapier.0.len() || engine.0.len() != LANES * TICKS as usize {
        return Err("trace lengths differ".into());
    }
    let at = |t: &Trace, tick: usize, lane: usize| match t.0[tick * LANES + lane] {
        Obs::Position(p) => p,
        _ => [f32::NAN; 3],
    };
    let last = TICKS as usize - 1;
    for lane in 0..LANES {
        let (c, h) = ramp(lane);
        let z = start(lane)[2];
        for tick in 0..TICKS as usize {
            let p = at(engine, tick, lane);
            if p[2] != z || !p.iter().all(|a| a.is_finite()) {
                return Err(format!("lane {lane} left its line at tick {tick}: {p:?}"));
            }
            // Never into the floor. Landing at a shallow angle may still end inside the
            // offset (#349).
            if p[1] < REST - OFFSET - 1e-4 {
                return Err(format!("lane {lane} sank to {} at tick {tick}", p[1]));
            }
            if lane >= 12 && p[1] > REST + 1e-4 {
                return Err(format!(
                    "lane {lane} climbed the too-steep ramp to {} at tick {tick}",
                    p[1]
                ));
            }
        }
        let (e, r) = (at(engine, last, lane), at(rapier, last, lane));
        if lane < 12 {
            // Both walked up and over the top.
            let top = c[0] - h[0];
            if e[0] > top || r[0] > top {
                return Err(format!(
                    "lane {lane} did not cross the ramp: engine {e:?}, rapier {r:?}"
                ));
            }
        } else if (0..3).any(|a| (e[a] - r[a]).abs() > RAPIER_TOLERANCE) {
            return Err(format!(
                "lane {lane} rests apart at the steep foot: engine {e:?}, rapier {r:?}"
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
        vy: Vec<f32>,
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
            for lane in 0..LANES {
                let (c, h) = ramp(lane);
                world
                    .add_body(Body::keyed(2 + lane as u64, Shape::wedge(v(h)), v(c), 0.0))
                    .unwrap();
            }
            let settings = CharacterSettings {
                offset: OFFSET,
                snap_to_ground: Some(SNAP),
                ..CharacterSettings::default()
            };
            let ids = (0..LANES)
                .map(|lane| {
                    let mut body = Body::keyed(
                        1_000 + lane as u64,
                        Shape::capsule(HALF, RADIUS),
                        v(start(lane)),
                        1.0,
                    );
                    body.external = true;
                    let id = world.add_body(body).unwrap();
                    world.add_character(id, settings).unwrap();
                    id
                })
                .collect();
            Self {
                world,
                ids,
                vy: vec![0.0; LANES],
                moves: Vec::with_capacity(LANES),
                results: Vec::with_capacity(LANES),
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, _tick: u32, trace: &mut Trace) {
            self.moves.clear();
            for (lane, &body) in self.ids.iter().enumerate() {
                self.vy[lane] -= GRAVITY * DT;
                self.moves.push(CharacterMove {
                    body,
                    desired: Vector(-SPEED * DT, self.vy[lane] * DT, 0.0),
                });
            }
            self.world
                .move_characters(&self.moves, &mut self.results)
                .unwrap();
            for (lane, r) in self.results.iter().enumerate() {
                if r.grounded {
                    self.vy[lane] = 0.0;
                }
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

    /// The wedge as MMORPG builds it: the box's bottom rectangle and its top edge at -X.
    fn wedge(h: [f32; 3]) -> SharedShape {
        let mut points = Vec::with_capacity(6);
        for sx in [-1.0, 1.0] {
            for sz in [-1.0, 1.0] {
                points.push(Vector::new(sx * h[0], -h[1], sz * h[2]));
                if sx < 0.0 {
                    points.push(Vector::new(sx * h[0], h[1], sz * h[2]));
                }
            }
        }
        SharedShape::convex_hull(&points).expect("solid wedge")
    }

    pub struct Sim {
        world: PhysicsWorld,
        wedges: Vec<SharedShape>,
        controller: KinematicCharacterController,
        bodies: Vec<RigidBodyHandle>,
        positions: Vec<Vector>,
        vy: Vec<f32>,
        shape: Capsule,
        rebuild: bool,
    }

    fn build(
        positions: &[Vector],
        wedges: &[SharedShape],
        shape: Capsule,
    ) -> (PhysicsWorld, Vec<RigidBodyHandle>) {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::ZERO;
        world.integration_parameters.dt = DT;
        world
            .colliders
            .insert(ColliderBuilder::new(SharedShape::halfspace(Vector::Y)).build());
        for (lane, w) in wedges.iter().enumerate() {
            world.colliders.insert(
                ColliderBuilder::new(w.clone())
                    .translation(v(ramp(lane).0))
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
        world.detect_collisions(&(), &());
        (world, bodies)
    }

    impl Sim {
        pub fn new(rebuild: bool) -> Self {
            let shape = Capsule::new_y(HALF, RADIUS);
            let wedges: Vec<_> = (0..LANES).map(|lane| wedge(ramp(lane).1)).collect();
            let positions: Vec<_> = (0..LANES).map(|lane| v(start(lane))).collect();
            let (world, bodies) = build(&positions, &wedges, shape);
            Self {
                world,
                wedges,
                controller: KinematicCharacterController {
                    up: Vector::Y,
                    offset: CharacterLength::Absolute(OFFSET),
                    slide: true,
                    autostep: None,
                    max_slope_climb_angle: core::f32::consts::FRAC_PI_4,
                    min_slope_slide_angle: core::f32::consts::FRAC_PI_4,
                    snap_to_ground: Some(CharacterLength::Absolute(OFFSET + SNAP)),
                    normal_nudge_factor: KinematicCharacterController::default()
                        .normal_nudge_factor,
                },
                bodies,
                positions,
                vy: vec![0.0; LANES],
                shape,
                rebuild,
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, _tick: u32, trace: &mut Trace) {
            if self.rebuild {
                (self.world, self.bodies) = build(&self.positions, &self.wedges, self.shape);
            }
            let dispatcher = DefaultQueryDispatcher;
            for lane in 0..LANES {
                let body = self.bodies[lane];
                let from = self.positions[lane];
                self.vy[lane] -= GRAVITY * DT;
                let w = &self.world;
                let queries = w.broad_phase.as_query_pipeline(
                    &dispatcher,
                    &w.bodies,
                    &w.colliders,
                    QueryFilter::default().exclude_rigid_body(body),
                );
                let movement = self.controller.move_shape(
                    DT,
                    &queries,
                    &self.shape,
                    &Pose::from_translation(from),
                    Vector::new(-SPEED * DT, self.vy[lane] * DT, 0.0),
                    |_| {},
                );
                if movement.grounded {
                    self.vy[lane] = 0.0;
                }
                let to = from + movement.translation;
                self.positions[lane] = to;
                self.world.bodies[body].set_translation(to, false);
                self.world.detect_collisions(&(), &());
                trace.0.push(Obs::Position([to.x, to.y, to.z]));
            }
        }
    }
}
