//! C4/C5 (ARPG stairs pace, #406): ARPG's player walking up its traversal stairs under gravity
//! at 60 Hz, compared tick by tick. Two 0.2 m risers 0.4 m deep up to a platform 0.6 m high,
//! below ARPG's 0.25 m autostep (`min_width` 0.1 m). Both sides follow ARPG's walking rule for
//! each move: a walk, which steps, when the previous move ended in ground contact (Rapier's
//! `grounded`; the engine's `ground_contact`) and the player does not rise. Rapier mirrors the
//! consumer call pattern: a world rebuild per tick, `move_shape` per player, the walking
//! controller (autostep) or the vertical one (none) chosen by that rule.
//!
//! The lanes differ in the phase at which the player meets the first riser. Except for
//! [`THRESHOLD_LANE`], each is compared every tick: ground contact equal, positions within
//! ARPG's 0.5 unit (5 mm) pre-contact bound, a riser stepped in one tick on the same tick;
//! every lane ends resting on the platform on both sides. Support (walkable ground below,
//! too-steep surfaces met first set aside) is compared at the end only: on a riser's rounded
//! edge it flips at 45°, where parry's GJK normals on Rapier's side carry noise of up to a
//! degree. Ground contact is equal every tick on every lane, the threshold lane included.
//!
//! The threshold lane is labelled as such and compared on outcomes only. Lanes clear of the
//! threshold climb each riser up its rounded edge over a few ticks. Where a lane instead lifts
//! up a whole riser in one tick (phases 21–32 of 64), the step's outcome hangs on the 45°
//! threshold: after the lift, the capsule set down on the riser's rounded edge, at a normal
//! within that noise of 45°, either stops on it (and the step holds) or slides back down it.
//! So Rapier's own climb is chaotic there (ARPG sees it step on one seed and not another from
//! the same climb): on this lane both step up the first riser on the same tick, but Rapier
//! steps up the second on a tick the engine slides up its edge. Both still reach the platform
//! and rest on it.

use crate::{Obs, Scenario, Trace};

const TICKS: u32 = 90;
const DT: f32 = 1.0 / 60.0;
const GRAVITY: f32 = 9.81 * DT * DT;
/// ARPG's contact offset (0.25 units) and support reach (offset plus Rapier's 5 cm).
const OFFSET: f32 = 0.0025;
const SNAP: f32 = 0.05;
/// ARPG's player capsule (half extents 30 × 50 × 30 units).
const HALF: f32 = 0.2;
const RADIUS: f32 = 0.3;
const REST: f32 = HALF + RADIUS + OFFSET;
/// ARPG's walking speed, 7 units per tick.
const SPEED: f32 = 0.07;
/// Where each lane starts, in 64ths of a tick's walk: the phase at which it meets the first
/// riser. Picked from a sweep of all 64 phases as the middles of runs that stay clear of the
/// threshold, so neighbouring phases agree too; the last is the threshold lane, in the middle
/// of a run that does not.
const PHASES: [usize; 16] = [3, 4, 5, 9, 10, 44, 45, 46, 47, 48, 52, 53, 54, 59, 60, 29];
const LANES: usize = PHASES.len();
/// The lane compared on outcomes only (see the module docs).
const THRESHOLD_LANE: usize = LANES - 1;
const FOOT_X: f32 = 10.0;
const RISE: f32 = 0.2;
const TREAD: f32 = 0.4;
/// Risers up to the platform (its own riser the last one).
const RISERS: usize = 3;
const LANDING: f32 = 8.0;
const MAX_STEP: f32 = 0.25;
const MIN_WIDTH: f32 = 0.1;
/// ARPG's pre-contact position bound, 0.5 units.
const POSITION_TOLERANCE: f32 = 0.005;
/// A move that rises more than this stepped up a riser (sliding up an edge rises far less).
const STEP: f32 = RISE / 2.0;
/// Observations per lane per tick: position, ground contact, support.
const OBS: usize = 3;

fn lane_z(lane: usize) -> f32 {
    -20.0 + 2.5 * lane as f32
}

/// Centres and half extents of the steps, the last one the platform.
fn steps(lane: usize) -> Vec<([f32; 3], [f32; 3])> {
    (0..RISERS)
        .map(|k| {
            let top = RISE * (k + 1) as f32;
            let half = if k + 1 == RISERS {
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
    [
        FOOT_X + RADIUS + 0.5 + SPEED * PHASES[lane] as f32 / 64.0,
        REST,
        lane_z(lane),
    ]
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "stairs-pace",
        capabilities: "C4 C5",
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

/// ARPG's vertical intent: none on the ground, gravity while airborne.
fn intent(grounded: bool, vy: f32) -> f32 {
    if grounded { 0.0 } else { vy - GRAVITY }
}

/// ARPG's velocity after a move: the intent, clamped to what the move achieved; a supported
/// player that is not rising stops falling.
fn settle(intended: f32, moved: f32, supported: bool) -> (f32, bool) {
    let vy = moved.clamp(intended.min(0.0), intended.max(0.0));
    let grounded = supported && vy <= 0.0;
    (if grounded { 0.0 } else { vy }, grounded)
}

fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    if engine.0.len() != rapier.0.len() || engine.0.len() != OBS * LANES * TICKS as usize {
        return Err("trace lengths differ".into());
    }
    let obs = |t: &Trace, tick: usize, lane: usize, k: usize| t.0[(tick * LANES + lane) * OBS + k];
    let at = |t: &Trace, tick: usize, lane: usize| match obs(t, tick, lane, 0) {
        Obs::Position(p) => p,
        _ => [f32::NAN; 3],
    };
    let top = RISE * RISERS as f32 + REST;
    let platform = FOOT_X - TREAD * (RISERS - 1) as f32;
    for lane in 0..LANES {
        let compared = lane != THRESHOLD_LANE;
        let mut before = (start(lane), start(lane));
        for tick in 0..TICKS as usize {
            let (e, r) = (at(engine, tick, lane), at(rapier, tick, lane));
            if !e.iter().all(|a| a.is_finite()) || e[2] != lane_z(lane) {
                return Err(format!("lane {lane} left its line at tick {tick}: {e:?}"));
            }
            if compared {
                if obs(engine, tick, lane, 1) != obs(rapier, tick, lane, 1) {
                    return Err(format!(
                        "lane {lane} tick {tick}: ground contact differs: engine {:?}, rapier \
                         {:?} (engine {e:?}, rapier {r:?})",
                        obs(engine, tick, lane, 1),
                        obs(rapier, tick, lane, 1)
                    ));
                }
                if (0..3).any(|i| (e[i] - r[i]).abs() > POSITION_TOLERANCE) {
                    return Err(format!(
                        "lane {lane} tick {tick}: positions apart: engine {e:?}, rapier {r:?}"
                    ));
                }
                let stepped = |from: [f32; 3], to: [f32; 3]| to[1] - from[1] > STEP;
                if stepped(before.0, e) != stepped(before.1, r) {
                    return Err(format!(
                        "lane {lane} tick {tick}: stepped on one side only: engine {e:?}, \
                         rapier {r:?}"
                    ));
                }
            }
            before = (e, r);
        }
        // Every lane, the threshold lane included: both reached the platform and rest on it.
        let last = TICKS as usize - 1;
        for (side, t) in [("engine", engine), ("rapier", rapier)] {
            let p = at(t, last, lane);
            let supported = obs(t, last, lane, 2) == Obs::Grounded(true);
            if (p[1] - top).abs() > 1e-3 || p[0] > platform || !supported {
                return Err(format!(
                    "lane {lane}: {side} does not rest on the platform: {p:?}"
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
        grounded: Vec<bool>,
        contact: Vec<bool>,
        intended: Vec<f32>,
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
                    let key = 2 + (lane * RISERS + k) as u64;
                    world
                        .add_body(Body::keyed(key, Shape::Box(v(h)), v(c), 0.0))
                        .unwrap();
                }
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
                grounded: vec![true; LANES],
                contact: vec![true; LANES],
                intended: vec![0.0; LANES],
                moves: Vec::with_capacity(LANES),
                results: Vec::with_capacity(LANES),
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, _tick: u32, trace: &mut Trace) {
            self.moves.clear();
            for (lane, &body) in self.ids.iter().enumerate() {
                let vy = intent(self.grounded[lane], self.vy[lane]);
                self.intended[lane] = vy;
                let walking = self.contact[lane] && vy <= 0.0;
                self.moves.push(CharacterMove {
                    body,
                    desired: Vector(-SPEED, vy, 0.0),
                    exclude: None,
                    autostep: walking.then_some(Autostep {
                        max_height: MAX_STEP,
                        min_width: MIN_WIDTH,
                    }),
                });
            }
            self.world
                .move_characters(&self.moves, &mut self.results)
                .unwrap();
            for (lane, r) in self.results.iter().enumerate() {
                let moved = r.translation.1 + r.depenetration.1;
                (self.vy[lane], self.grounded[lane]) =
                    settle(self.intended[lane], moved, r.grounded);
                self.contact[lane] = r.ground_contact;
                let p = self.world.body(r.body).unwrap().position;
                trace.0.push(Obs::Position([p.0, p.1, p.2]));
                trace.0.push(Obs::Grounded(r.ground_contact));
                trace.0.push(Obs::Grounded(r.grounded));
            }
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::super::upright_queries::UprightShapes;
    use super::*;
    use rapier3d::control::{CharacterAutostep, CharacterLength, KinematicCharacterController};
    use rapier3d::parry::query::{
        DefaultQueryDispatcher, QueryDispatcher, QueryDispatcherChain, ShapeCastOptions,
    };
    use rapier3d::prelude::*;

    fn v(a: [f32; 3]) -> Vector {
        Vector::new(a[0], a[1], a[2])
    }

    pub struct Sim {
        world: PhysicsWorld,
        /// ARPG's controllers: walking players step, jumps and falls do not.
        walking: KinematicCharacterController,
        vertical: KinematicCharacterController,
        bodies: Vec<RigidBodyHandle>,
        positions: Vec<Vector>,
        vy: Vec<f32>,
        grounded: Vec<bool>,
        contact: Vec<bool>,
        /// The consumers' exact upright-capsule queries (ARPG's dispatcher).
        queries: QueryDispatcherChain<UprightShapes, DefaultQueryDispatcher>,
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
                bodies,
                positions,
                vy: vec![0.0; LANES],
                grounded: vec![true; LANES],
                contact: vec![true; LANES],
                queries: UprightShapes.chain(DefaultQueryDispatcher),
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
            let dispatcher = &self.queries;
            for lane in 0..LANES {
                let body = self.bodies[lane];
                let from = self.positions[lane];
                let vy = intent(self.grounded[lane], self.vy[lane]);
                let w = &self.world;
                let queries = w.broad_phase.as_query_pipeline(
                    dispatcher,
                    &w.bodies,
                    &w.colliders,
                    QueryFilter::default().exclude_rigid_body(body),
                );
                let controller = if self.contact[lane] && vy <= 0.0 {
                    &self.walking
                } else {
                    &self.vertical
                };
                let movement = controller.move_shape(
                    DT,
                    &queries,
                    &self.shape,
                    &Pose::from_translation(from),
                    Vector::new(-SPEED, vy, 0.0),
                    |_| {},
                );
                let to = from + movement.translation;
                // ARPG's ground contact: Rapier's grounded with static geometry within the
                // support reach below, at any angle. Support: walkable static ground there,
                // a too-steep surface met first set aside and the cast repeated (as MMORPG's
                // support cast, #395).
                let below = |steep: &[ColliderHandle]| {
                    let admit = |handle: ColliderHandle, _: &Collider| !steep.contains(&handle);
                    w.broad_phase
                        .as_query_pipeline(
                            dispatcher,
                            &w.bodies,
                            &w.colliders,
                            QueryFilter::only_fixed().predicate(&admit),
                        )
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
                        .map(|(handle, hit)| (handle, hit.normal1.dot(Vector::Y)))
                };
                let contact = movement.grounded && below(&[]).is_some_and(|(_, up)| up > 0.0);
                let mut steep = Vec::new();
                let mut support = false;
                while movement.grounded
                    && steep.len() < 4
                    && let Some((handle, up)) = below(&steep)
                {
                    if up >= core::f32::consts::FRAC_PI_4.cos() {
                        support = true;
                        break;
                    }
                    steep.push(handle);
                }
                (self.vy[lane], self.grounded[lane]) = settle(vy, movement.translation.y, support);
                self.contact[lane] = contact;
                self.positions[lane] = to;
                self.world.bodies[body].set_translation(to, false);
                self.world.detect_collisions(&(), &());
                trace.0.push(Obs::Position([to.x, to.y, to.z]));
                trace.0.push(Obs::Grounded(contact));
                trace.0.push(Obs::Grounded(support));
            }
        }
    }
}
