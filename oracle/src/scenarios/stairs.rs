//! C4/C5 (ARPG stairs, #353): characters walking under gravity into stairs with Rapier's
//! autostep (0.25 m, 0.15 m free width, dynamic bodies excluded). Steps up to the autostep
//! height are climbed onto the landing; higher ones stop the character at the first riser.
//! Rapier mirrors the consumer call pattern: a world rebuild per tick, `move_shape` per
//! character in id order.

use crate::{Obs, Scenario, Trace};

const TICKS: u32 = 360;
const DT: f32 = 1.0 / 60.0;
const GRAVITY: f32 = 9.81;
const OFFSET: f32 = 0.0025;
const SNAP: f32 = 0.05;
const HALF: f32 = 0.5;
const RADIUS: f32 = 0.3;
const REST: f32 = HALF + RADIUS + OFFSET;
const SPEED: f32 = 2.0;
const LANES: usize = 24;
/// Every staircase's first riser; stairs rise toward -X.
const FOOT_X: f32 = 10.0;
const STEPS: usize = 4;
const TREAD: f32 = 0.4;
const LANDING: f32 = 14.0;
const MAX_STEP: f32 = 0.25;
const MIN_WIDTH: f32 = 0.15;
/// Positions of the two engines may differ by Rapier's normal nudge.
const RAPIER_TOLERANCE: f32 = 0.02;
/// How far apart the two engines may stop at a too-high riser (the capsule rides its edge).
const RISER_TOLERANCE: f32 = 0.05;

/// Rise of each lane's steps: 12 climbed by autostep (0.10–0.20 m), 12 too high (0.27–0.49
/// m). Lower steps are not autostep: a capsule slides over their edge as over a walkable slope.
/// Near the autostep height a capsule climbs a step's rounded edge only a little faster than
/// gravity pulls it back, on Rapier as on the engine, so whether it gets up is fragile there.
fn rise(lane: usize) -> f32 {
    if lane < 12 {
        0.10 + 0.0091 * lane as f32
    } else {
        0.27 + 0.02 * (lane - 12) as f32
    }
}

fn lane_z(lane: usize) -> f32 {
    -30.0 + 2.5 * lane as f32
}

/// Centres and half extents of lane `lane`'s steps, the last one its landing.
fn steps(lane: usize) -> Vec<([f32; 3], [f32; 3])> {
    (0..=STEPS)
        .map(|k| {
            let top = rise(lane) * (k + 1) as f32;
            let half = if k == STEPS {
                LANDING / 2.0
            } else {
                TREAD / 2.0
            };
            let x = FOOT_X - TREAD * k as f32 - half;
            ([x, top / 2.0, lane_z(lane)], [half, top / 2.0, 0.8])
        })
        .collect()
}

fn start(lane: usize) -> [f32; 3] {
    [FOOT_X + 1.0, REST, lane_z(lane)]
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "stairs",
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
        let z = lane_z(lane);
        for tick in 0..TICKS as usize {
            let p = at(engine, tick, lane);
            if p[2] != z || !p.iter().all(|a| a.is_finite()) {
                return Err(format!("lane {lane} left its line at tick {tick}: {p:?}"));
            }
            // A too-high step is never climbed: the character may ride up its rounded edge a
            // little, as on Rapier, but never reaches its top.
            if lane >= 12 && p[1] > REST + rise(lane) - RADIUS / 2.0 {
                return Err(format!(
                    "lane {lane} stepped up a too-high step to {} at tick {tick}",
                    p[1]
                ));
            }
        }
        let (e, r) = (at(engine, last, lane), at(rapier, last, lane));
        if lane < 12 {
            // Both climbed onto the landing. The pace differs: with ARPG's walking rule Rapier
            // can stall at a step for a few ticks, its normal nudge leaving the player just off
            // support so that it does not step; the engine does not stall.
            let top = rise(lane) * (STEPS + 1) as f32 + REST;
            let landing = FOOT_X - TREAD * STEPS as f32;
            for (side, p) in [("engine", e), ("rapier", r)] {
                if (p[1] - top).abs() > RAPIER_TOLERANCE || p[0] > landing {
                    return Err(format!(
                        "lane {lane}: {side} did not reach the landing: {p:?}"
                    ));
                }
            }
            if (e[1] - top).abs() > 1e-3 {
                return Err(format!("lane {lane} does not rest on the landing: {e:?}"));
            }
        } else {
            // Both stop at the first riser. A rounded capsule rides a little up a too-high
            // step's edge and slips back, on Rapier as on the engine, so heights differ by
            // phase.
            let top = REST + rise(lane) - RADIUS / 2.0;
            if (e[0] - r[0]).abs() > RISER_TOLERANCE || (e[2] - r[2]).abs() > RAPIER_TOLERANCE {
                return Err(format!(
                    "lane {lane} stops apart at the riser: engine {e:?}, rapier {r:?}"
                ));
            }
            if r[1] > top {
                return Err(format!(
                    "lane {lane}: rapier climbed a too-high step: {r:?}"
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
        approximate::{
            Autostep, Body, CharacterMove, CharacterMoveResult, CharacterSettings, Config, Shape,
            Vector, World,
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
                for (k, (c, h)) in steps(lane).into_iter().enumerate() {
                    let key = 2 + (lane * (STEPS + 1) + k) as u64;
                    world
                        .add_body(Body::keyed(key, Shape::Box(v(h)), v(c), 0.0))
                        .unwrap();
                }
            }
            let settings = CharacterSettings {
                offset: OFFSET,
                snap_to_ground: Some(SNAP),
                autostep: Some(Autostep {
                    max_height: MAX_STEP,
                    min_width: MIN_WIDTH,
                }),
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
    use rapier3d::control::{CharacterAutostep, CharacterLength, KinematicCharacterController};
    use rapier3d::parry::query::{DefaultQueryDispatcher, ShapeCastOptions};
    use rapier3d::prelude::*;

    fn v(a: [f32; 3]) -> Vector {
        Vector::new(a[0], a[1], a[2])
    }

    pub struct Sim {
        world: PhysicsWorld,
        /// ARPG's controllers: walking players step, jumps and falls do not. A player walks
        /// when Rapier found it grounded last tick (a step's rounded edge included).
        walking: KinematicCharacterController,
        vertical: KinematicCharacterController,
        on_ground: Vec<bool>,
        bodies: Vec<RigidBodyHandle>,
        positions: Vec<Vector>,
        vy: Vec<f32>,
        shape: Capsule,
        rebuild: bool,
    }

    fn build(positions: &[Vector], shape: Capsule) -> (PhysicsWorld, Vec<RigidBodyHandle>) {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::ZERO;
        world.integration_parameters.dt = DT;
        world
            .colliders
            .insert(ColliderBuilder::new(SharedShape::halfspace(Vector::Y)).build());
        for lane in 0..LANES {
            for (c, h) in steps(lane) {
                world.colliders.insert(
                    ColliderBuilder::cuboid(h[0], h[1], h[2])
                        .translation(v(c))
                        .build(),
                );
            }
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
            let positions: Vec<_> = (0..LANES).map(|lane| v(start(lane))).collect();
            let (world, bodies) = build(&positions, shape);
            let vertical = KinematicCharacterController {
                up: Vector::Y,
                offset: CharacterLength::Absolute(OFFSET),
                slide: true,
                autostep: None,
                max_slope_climb_angle: core::f32::consts::FRAC_PI_4,
                min_slope_slide_angle: core::f32::consts::FRAC_PI_4,
                snap_to_ground: Some(CharacterLength::Absolute(OFFSET + SNAP)),
                normal_nudge_factor: KinematicCharacterController::default().normal_nudge_factor,
            };
            Self {
                world,
                walking: KinematicCharacterController {
                    autostep: Some(CharacterAutostep {
                        max_height: CharacterLength::Absolute(MAX_STEP),
                        min_width: CharacterLength::Absolute(MIN_WIDTH),
                        include_dynamic_bodies: false,
                    }),
                    ..vertical
                },
                vertical,
                on_ground: vec![true; LANES],
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
                (self.world, self.bodies) = build(&self.positions, self.shape);
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
                let controller = if self.on_ground[lane] {
                    &self.walking
                } else {
                    &self.vertical
                };
                let movement = controller.move_shape(
                    DT,
                    &queries,
                    &self.shape,
                    &Pose::from_translation(from),
                    Vector::new(-SPEED * DT, self.vy[lane] * DT, 0.0),
                    |_| {},
                );
                let to = from + movement.translation;
                // The consumers' grounded: Rapier's, and walkable static support below.
                let fixed = w.broad_phase.as_query_pipeline(
                    &dispatcher,
                    &w.bodies,
                    &w.colliders,
                    QueryFilter::only_fixed(),
                );
                let support = movement.grounded
                    && fixed
                        .cast_shape(
                            &Pose::from_translation(to),
                            -Vector::Y,
                            &self.shape,
                            ShapeCastOptions {
                                target_distance: OFFSET,
                                stop_at_penetration: false,
                                max_time_of_impact: OFFSET + SNAP,
                                compute_impact_geometry_on_penetration: true,
                            },
                        )
                        .is_some_and(|(_, hit)| {
                            hit.normal1.dot(Vector::Y) >= core::f32::consts::FRAC_PI_4.cos()
                        });
                if support {
                    self.vy[lane] = 0.0;
                }
                self.on_ground[lane] = movement.grounded;
                self.positions[lane] = to;
                self.world.bodies[body].set_translation(to, false);
                self.world.detect_collisions(&(), &());
                trace.0.push(Obs::Position([to.x, to.y, to.z]));
            }
        }
    }
}
