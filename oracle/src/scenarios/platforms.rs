//! C1/C4/C5 (MMORPG moving platforms, #56/#352): 24 static platforms move on a schedule (up,
//! sideways, down, back) and carry the character standing on each. Both sides use MMORPG's
//! adapter pattern: a character found on a platform by a downward cast filtered to that
//! platform gets the platform's displacement added to its move; a sinking platform moves
//! before the characters and a rising one after them. Rapier rebuilds the world per tick as
//! MMORPG does and moves the fixed collider in place; the engine moves the platform with
//! `set_static_position`.

use crate::{Obs, Scenario, Trace};

const TICKS: u32 = 240;
const DT: f32 = 1.0 / 60.0;
const GRAVITY: f32 = 9.81;
const OFFSET: f32 = 0.0025;
const SNAP: f32 = 0.05;
/// Rapier's ground prediction beyond `offset`.
const PREDICT: f32 = OFFSET + 0.05;
const HALF: f32 = 0.5;
const RADIUS: f32 = 0.3;
const FOOT: f32 = HALF + RADIUS;
const LANES: usize = 24;
const PLATFORM_HALF: [f32; 3] = [1.0, 0.25, 1.0];
const STEP: f32 = 0.01;

fn platform_start(lane: usize) -> [f32; 3] {
    [
        -30.0 + 5.0 * (lane % 12) as f32,
        0.5,
        -5.0 + 10.0 * (lane / 12) as f32,
    ]
}

fn rider_start(lane: usize) -> [f32; 3] {
    let p = platform_start(lane);
    [p[0], p[1] + PLATFORM_HALF[1] + FOOT + OFFSET, p[2]]
}

/// Platform `lane`'s displacement this tick: up, +X, down and -X for 30 ticks each, shifted
/// by lane.
fn displacement(lane: usize, tick: u32) -> [f32; 3] {
    match ((tick as usize + lane * 7) / 30) % 4 {
        0 => [0.0, STEP, 0.0],
        1 => [STEP, 0.0, 0.0],
        2 => [0.0, -STEP, 0.0],
        _ => [-STEP, 0.0, 0.0],
    }
}

fn add(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "platforms",
        capabilities: "C1 C4 C5",
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

/// Per tick and lane: the platform's position, then the rider's.
fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    if engine.0.len() != rapier.0.len() || engine.0.len() != 2 * LANES * TICKS as usize {
        return Err("trace lengths differ".into());
    }
    let at = |t: &Trace, tick: usize, lane: usize| {
        let base = (tick * LANES + lane) * 2;
        match (t.0[base], t.0[base + 1]) {
            (Obs::Position(p), Obs::Position(r)) => (p, r),
            _ => ([f32::NAN; 3], [f32::NAN; 3]),
        }
    };
    for tick in 0..TICKS as usize {
        for lane in 0..LANES {
            let (p, r) = at(engine, tick, lane);
            let (_, rr) = at(rapier, tick, lane);
            // Carried exactly: centred over the platform, resting on its top.
            let top = p[1] + PLATFORM_HALF[1] + FOOT + OFFSET;
            if (r[0] - p[0]).abs() > 1e-3 || r[2] != p[2] || (r[1] - top).abs() > 2e-3 {
                return Err(format!(
                    "lane {lane} rider left its platform at tick {tick}: {r:?} on {p:?}"
                ));
            }
            if (0..3).any(|a| (r[a] - rr[a]).abs() > 0.01) {
                return Err(format!(
                    "lane {lane} at tick {tick}: engine {r:?}, rapier {rr:?}"
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
            Body, CharacterMove, CharacterMoveResult, CharacterSettings, Config, QueryFilter,
            QueryHit, QueryPose, Shape, ShapeCast, Vector, World,
        },
    };

    fn v(a: [f32; 3]) -> Vector {
        Vector(a[0], a[1], a[2])
    }

    pub struct Sim {
        world: World,
        platforms: Vec<BodyId>,
        riders: Vec<BodyId>,
        vy: Vec<f32>,
        carry: Vec<Option<[f32; 3]>>,
        hits: Vec<QueryHit>,
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
            let platforms = (0..LANES)
                .map(|lane| {
                    world
                        .add_body(Body::keyed(
                            10 + lane as u64,
                            Shape::Box(v(PLATFORM_HALF)),
                            v(platform_start(lane)),
                            0.0,
                        ))
                        .unwrap()
                })
                .collect();
            let settings = CharacterSettings {
                offset: OFFSET,
                snap_to_ground: Some(SNAP),
                ..CharacterSettings::default()
            };
            let riders = (0..LANES)
                .map(|lane| {
                    let mut body = Body::keyed(
                        1_000 + lane as u64,
                        Shape::capsule(HALF, RADIUS),
                        v(rider_start(lane)),
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
                platforms,
                riders,
                vy: vec![0.0; LANES],
                carry: vec![None; LANES],
                hits: Vec::new(),
                moves: Vec::with_capacity(LANES),
                results: Vec::with_capacity(LANES),
            }
        }

        /// Whether `rider` stands on `platform`: walkable top within the ground prediction.
        fn stands_on(&mut self, rider: BodyId, platform: BodyId) -> bool {
            let reach = PREDICT + OFFSET;
            let p = self.world.body(rider).unwrap().position;
            let cast = ShapeCast::new(
                QueryPose::new(Shape::capsule(HALF, RADIUS), p),
                Vector(0.0, -reach, 0.0),
            );
            let filter = QueryFilter {
                exclude: Some(rider),
                ..QueryFilter::default()
            };
            self.world.cast_shape(cast, filter, &mut self.hits).unwrap();
            let min_up = core::f32::consts::FRAC_PI_4.cos();
            self.hits.iter().any(|h| {
                h.body == platform && h.normal.1 >= min_up && h.fraction * reach <= PREDICT
            })
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            for lane in 0..LANES {
                let d = displacement(lane, tick);
                self.carry[lane] = (self.vy[lane] <= 0.0
                    && self.stands_on(self.riders[lane], self.platforms[lane]))
                .then_some(d);
            }
            for lane in 0..LANES {
                let d = displacement(lane, tick);
                if d[1] <= 0.0 {
                    let p = self.platforms[lane];
                    let to = self.world.body(p).unwrap().position + v(d);
                    self.world.set_static_position(p, to).unwrap();
                }
            }
            self.moves.clear();
            for lane in 0..LANES {
                let carry = self.carry[lane].unwrap_or([0.0; 3]);
                if self.carry[lane].is_none() {
                    self.vy[lane] -= GRAVITY * DT;
                }
                self.moves.push(CharacterMove {
                    body: self.riders[lane],
                    desired: v(add(carry, [0.0, self.vy[lane] * DT, 0.0])),
                    exclude: None,
                });
            }
            self.world
                .move_characters(&self.moves, &mut self.results)
                .unwrap();
            for lane in 0..LANES {
                if self.results[lane].grounded {
                    self.vy[lane] = 0.0;
                }
                let d = displacement(lane, tick);
                if d[1] > 0.0 {
                    let p = self.platforms[lane];
                    let to = self.world.body(p).unwrap().position + v(d);
                    self.world.set_static_position(p, to).unwrap();
                }
            }
            for lane in 0..LANES {
                let p = self.world.body(self.platforms[lane]).unwrap().position;
                let r = self.world.body(self.riders[lane]).unwrap().position;
                trace.0.push(Obs::Position([p.0, p.1, p.2]));
                trace.0.push(Obs::Position([r.0, r.1, r.2]));
            }
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::*;
    use rapier3d::control::{CharacterLength, KinematicCharacterController};
    use rapier3d::parry::query::{DefaultQueryDispatcher, ShapeCastOptions};
    use rapier3d::prelude::*;

    fn v(a: [f32; 3]) -> Vector {
        Vector::new(a[0], a[1], a[2])
    }

    pub struct Sim {
        world: PhysicsWorld,
        controller: KinematicCharacterController,
        platforms: Vec<ColliderHandle>,
        platform_positions: Vec<Vector>,
        riders: Vec<RigidBodyHandle>,
        positions: Vec<Vector>,
        vy: Vec<f32>,
        carry: Vec<Option<[f32; 3]>>,
        shape: Capsule,
        rebuild: bool,
    }

    fn build(
        platforms: &[Vector],
        positions: &[Vector],
        shape: Capsule,
    ) -> (PhysicsWorld, Vec<ColliderHandle>, Vec<RigidBodyHandle>) {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::ZERO;
        world.integration_parameters.dt = DT;
        world
            .colliders
            .insert(ColliderBuilder::new(SharedShape::halfspace(Vector::Y)).build());
        let h = PLATFORM_HALF;
        let colliders = platforms
            .iter()
            .map(|&p| {
                world.colliders.insert(
                    ColliderBuilder::cuboid(h[0], h[1], h[2])
                        .translation(p)
                        .build(),
                )
            })
            .collect();
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
        (world, colliders, bodies)
    }

    impl Sim {
        pub fn new(rebuild: bool) -> Self {
            let shape = Capsule::new_y(HALF, RADIUS);
            let platform_positions: Vec<_> = (0..LANES).map(|l| v(platform_start(l))).collect();
            let positions: Vec<_> = (0..LANES).map(|l| v(rider_start(l))).collect();
            let (world, platforms, riders) = build(&platform_positions, &positions, shape);
            Self {
                world,
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
                platforms,
                platform_positions,
                riders,
                positions,
                vy: vec![0.0; LANES],
                carry: vec![None; LANES],
                shape,
                rebuild,
            }
        }

        fn shift_platform(&mut self, lane: usize, d: [f32; 3]) {
            self.platform_positions[lane] += v(d);
            let c = self.platforms[lane];
            self.world.colliders[c].set_translation(self.platform_positions[lane]);
            self.world.detect_collisions(&(), &());
        }

        fn stands_on(&self, lane: usize) -> bool {
            let w = &self.world;
            let platform = self.platforms[lane];
            let predicate = |handle: ColliderHandle, _: &Collider| handle == platform;
            let filter = QueryFilter::default().predicate(&predicate);
            let queries = w.broad_phase.as_query_pipeline(
                &DefaultQueryDispatcher,
                &w.bodies,
                &w.colliders,
                filter,
            );
            queries
                .cast_shape(
                    &Pose::from_translation(self.positions[lane]),
                    -Vector::Y,
                    &self.shape,
                    ShapeCastOptions {
                        target_distance: 0.0,
                        stop_at_penetration: false,
                        max_time_of_impact: PREDICT + OFFSET,
                        compute_impact_geometry_on_penetration: true,
                    },
                )
                .is_some_and(|(_, hit)| {
                    hit.time_of_impact <= PREDICT
                        && hit.normal1.dot(Vector::Y) >= core::f32::consts::FRAC_PI_4.cos()
                })
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            if self.rebuild {
                (self.world, self.platforms, self.riders) =
                    build(&self.platform_positions, &self.positions, self.shape);
            }
            for lane in 0..LANES {
                let d = displacement(lane, tick);
                self.carry[lane] = (self.vy[lane] <= 0.0 && self.stands_on(lane)).then_some(d);
            }
            for lane in 0..LANES {
                let d = displacement(lane, tick);
                if d[1] <= 0.0 {
                    self.shift_platform(lane, d);
                }
            }
            for lane in 0..LANES {
                let carry = self.carry[lane].unwrap_or([0.0; 3]);
                if self.carry[lane].is_none() {
                    self.vy[lane] -= GRAVITY * DT;
                }
                let body = self.riders[lane];
                let from = self.positions[lane];
                let w = &self.world;
                let queries = w.broad_phase.as_query_pipeline(
                    &DefaultQueryDispatcher,
                    &w.bodies,
                    &w.colliders,
                    QueryFilter::default().exclude_rigid_body(body),
                );
                let movement = self.controller.move_shape(
                    DT,
                    &queries,
                    &self.shape,
                    &Pose::from_translation(from),
                    v(add(carry, [0.0, self.vy[lane] * DT, 0.0])),
                    |_| {},
                );
                if movement.grounded {
                    self.vy[lane] = 0.0;
                }
                let to = from + movement.translation;
                self.positions[lane] = to;
                self.world.bodies[body].set_translation(to, false);
                self.world.detect_collisions(&(), &());
            }
            for lane in 0..LANES {
                let d = displacement(lane, tick);
                if d[1] > 0.0 {
                    self.shift_platform(lane, d);
                }
            }
            for lane in 0..LANES {
                let p = self.platform_positions[lane];
                let r = self.positions[lane];
                trace.0.push(Obs::Position([p.x, p.y, p.z]));
                trace.0.push(Obs::Position([r.x, r.y, r.z]));
            }
        }
    }
}
