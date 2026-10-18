//! C15 (#297): ARPG's whole physics tick in the dungeon of `static-queries`. Two players walk,
//! slide along walls and jump; 24 monsters steer on the plane; door locks toggle; every tick
//! each monster checks its line of sight to the first player, six projectiles advance with a
//! first-hit ray, and every 20 ticks four spawn placements are tested for penetration.
//!
//! The Rapier side runs ARPG's `Physics` call pattern (main at d4ff0ab): `rebuild` (static
//! boxes, then characters, each in key order, then the halfspace floor), `move_player` with the
//! snapping controller and a static support cast, `move_character` on the plane with the floor
//! excluded, `set_translation` plus `detect_collisions` after each move, `add_fixed` and
//! `remove_fixed` with `detect_collisions`, and its queries through ARPG's dispatcher. The
//! engine side is one `move_characters` batch (players and monsters, each with its settings).

use super::character_move::capsule_box_gap;
use super::static_queries::{BoxSpec, layout};
use crate::{Obs, Rng, Scenario, Trace};

const TICKS: u32 = 240;
const DT: f32 = 1.0 / 60.0;
const OFFSET: f32 = 0.0025;
const SNAP: f32 = 0.05;
const SUPPORT_REACH: f32 = OFFSET + SNAP;
const HALF: f32 = 0.5;
const RADIUS: f32 = 0.3;
/// Resting centre height: one offset above the floor.
const HEIGHT: f32 = HALF + RADIUS + OFFSET;
const GRAVITY: f32 = 9.81 * DT * DT;
const TAKEOFF: f32 = 5.0 * DT;
const PLAYERS: usize = 2;
const MONSTERS: usize = 24;
const PROJECTILES: usize = 6;
const PLACEMENTS: usize = 4;
const DOOR_PERIOD: u32 = 40;
/// ARPG's penetration tolerance (1e-3 world units), in metres here.
const PENETRATION_TOLERANCE: f32 = 1e-5;
/// Keys: floor 1, walls and pillars from 2, doors from `DOOR_BASE`, players then monsters.
const DOOR_BASE: u64 = 10_000;
const CHARACTER_BASE: u64 = 100_000;

/// Static boxes (walls and pillars, then doors) and the free start positions.
fn scene() -> (Vec<BoxSpec>, Vec<BoxSpec>, Vec<[f32; 3]>) {
    let (walls, doors) = layout();
    let mut rng = Rng(0x7a2c_9e1d_44b3_0f68);
    let mut starts = Vec::new();
    while starts.len() < PLAYERS + MONSTERS {
        let p = [rng.range(-29.0, 29.0), HEIGHT, rng.range(-29.0, 29.0)];
        if walls
            .iter()
            .chain(&doors)
            .all(|b| capsule_box_gap(p, *b) > 0.3)
            && starts
                .iter()
                .all(|q: &[f32; 3]| (p[0] - q[0]).hypot(p[2] - q[2]) > 1.0)
        {
            starts.push(p);
        }
    }
    (walls, doors, starts)
}

/// Player `p`'s horizontal intent (metres per tick) and whether it presses jump.
fn player_intent(p: usize, tick: u32) -> ([f32; 2], bool) {
    let speed = 4.0 * DT;
    let heading = match p {
        // A square walk, a quarter turn every 60 ticks.
        0 => [[1.0, 0.0], [0.0, 1.0], [-1.0, 0.0], [0.0, -1.0]][(tick / 60) as usize % 4],
        // Diagonally into walls, sliding along them.
        _ => [core::f32::consts::FRAC_1_SQRT_2; 2],
    };
    let jump = if p == 0 {
        tick % 50 == 20
    } else {
        tick % 70 == 35
    };
    ([heading[0] * speed, heading[1] * speed], jump)
}

/// Monster `m`'s steering (metres per tick): a new heading every 30 ticks, 3 m/s.
fn monster_intent(m: usize, tick: u32) -> [f32; 3] {
    let mut rng = Rng(0x93 + m as u64 * 7_919 + u64::from(tick / 30));
    let a = rng.range(0.0, core::f32::consts::TAU);
    [3.0 * a.cos() * DT, 0.0, 3.0 * a.sin() * DT]
}

/// Projectile `k` at `tick`: origin and this tick's translation.
fn projectile(k: usize, tick: u32) -> ([f32; 3], [f32; 3]) {
    let a = k as f32 * 1.1;
    let (dx, dz) = (a.cos(), a.sin());
    let speed = 12.0 * DT;
    let t = (tick % 120) as f32 * speed;
    (
        [-2.0 + dx * t, 1.0, 3.0 + dz * t],
        [dx * speed, 0.0, dz * speed],
    )
}

/// Spawn placements tested for penetration every 20 ticks.
fn placements(tick: u32) -> Vec<[f32; 3]> {
    let mut rng = Rng(0x5eed + u64::from(tick));
    (0..PLACEMENTS)
        .map(|_| [rng.range(-29.0, 29.0), HEIGHT, rng.range(-29.0, 29.0)])
        .collect()
}

/// The door whose lock toggles at `tick`, if any.
fn toggle(tick: u32, doors: usize) -> Option<usize> {
    (tick % DOOR_PERIOD == 5).then_some((tick / DOOR_PERIOD) as usize % doors)
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "arpg-tick",
        capabilities: "C15",
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

/// Per tick: every character's position and grounded state, each monster's line of sight,
/// each projectile's first hit, and on placement ticks each placement's penetration.
fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    if engine.0.len() != rapier.0.len() {
        return Err("trace lengths differ".into());
    }
    let (walls, doors, _) = scene();
    let mut at = 0;
    for tick in 0..TICKS {
        for c in 0..PLAYERS + MONSTERS {
            let (Obs::Position(p), Obs::Grounded(g)) = (engine.0[at], engine.0[at + 1]) else {
                return Err("unexpected observation".into());
            };
            at += 2;
            if c >= PLAYERS && p[1] != HEIGHT {
                return Err(format!("monster {c} left its plane at tick {tick}: {p:?}"));
            }
            if p[1] < HEIGHT - 1e-3 {
                return Err(format!("character {c} sank into the floor at tick {tick}"));
            }
            if c < PLAYERS && g && (p[1] - HEIGHT).abs() > 1e-3 {
                return Err(format!(
                    "player {c} grounded in the air at tick {tick}: {p:?}"
                ));
            }
            if tick % 20 == 0 {
                for b in &walls {
                    if capsule_box_gap(p, *b) < -1e-3 {
                        return Err(format!("character {c} penetrates a wall at tick {tick}"));
                    }
                }
            }
        }
        // Lines of sight depend on where each engine moved the monsters: not compared.
        at += MONSTERS;
        // Projectiles and placements are fixed inputs: both engines agree exactly.
        let compared = PROJECTILES + if tick % 20 == 0 { PLACEMENTS } else { 0 };
        for k in 0..compared {
            if engine.0[at + k] != rapier.0[at + k] {
                return Err(format!(
                    "query {k} disagrees at tick {tick}: {:?} vs {:?}",
                    engine.0[at + k],
                    rapier.0[at + k]
                ));
            }
        }
        at += compared;
    }
    let _ = doors;
    // Players jumped and landed again.
    let airborne = (0..TICKS as usize)
        .filter(|&t| {
            let base = t * (2 * (PLAYERS + MONSTERS));
            matches!(engine.0.get(base), Some(Obs::Position(p)) if p[1] > HEIGHT + 0.2)
        })
        .count();
    if airborne == 0 {
        return Err("the first player never left the ground".into());
    }
    Ok(())
}

#[cfg(feature = "engine")]
mod engine {
    use super::*;
    use physics_engine::{
        BodyId, CollisionLayers3d,
        approximate::{
            Body, CharacterMove, CharacterMoveResult, CharacterSettings, Config, QueryFilter,
            QueryHit, QueryPose, RayCast, Shape, Vector, World,
        },
    };

    fn v(a: [f32; 3]) -> Vector {
        Vector(a[0], a[1], a[2])
    }

    const CHARACTERS: CollisionLayers3d = CollisionLayers3d::new(2, u32::MAX);

    pub struct Sim {
        world: World,
        floor: BodyId,
        ids: Vec<BodyId>,
        vy: Vec<f32>,
        grounded: Vec<bool>,
        doors: Vec<BoxSpec>,
        door_ids: Vec<Option<BodyId>>,
        moves: Vec<CharacterMove>,
        results: Vec<CharacterMoveResult>,
        hits: Vec<QueryHit>,
    }

    impl Sim {
        pub fn new() -> Self {
            let mut world = World::new(Config {
                body_storage: crate::engine_storage(),
                gravity: Vector::ZERO,
                ..Config::default()
            })
            .unwrap();
            let (walls, doors, starts) = scene();
            let floor = world
                .add_body(Body::keyed(1, Shape::HalfSpace, Vector::ZERO, 0.0))
                .unwrap();
            for (i, (c, h)) in walls.iter().enumerate() {
                world
                    .add_body(Body::keyed(2 + i as u64, Shape::Box(v(*h)), v(*c), 0.0))
                    .unwrap();
            }
            let door_ids = doors
                .iter()
                .enumerate()
                .map(|(i, (c, h))| {
                    let body = Body::keyed(DOOR_BASE + i as u64, Shape::Box(v(*h)), v(*c), 0.0);
                    Some(world.add_body(body).unwrap())
                })
                .collect();
            let player = CharacterSettings {
                offset: OFFSET,
                snap_to_ground: Some(SNAP),
                ..CharacterSettings::default()
            };
            let monster = CharacterSettings {
                offset: OFFSET,
                snap_to_ground: None,
                plane: Some(HEIGHT),
                ..CharacterSettings::default()
            };
            let ids = starts
                .iter()
                .enumerate()
                .map(|(c, p)| {
                    let mut body = Body::keyed(
                        CHARACTER_BASE + c as u64,
                        Shape::capsule(HALF, RADIUS),
                        v(*p),
                        1.0,
                    );
                    body.external = true;
                    body.layers = CHARACTERS;
                    let id = world.add_body(body).unwrap();
                    world
                        .add_character(id, if c < PLAYERS { player } else { monster })
                        .unwrap();
                    id
                })
                .collect::<Vec<_>>();
            let n = ids.len();
            Self {
                world,
                floor,
                ids,
                vy: vec![0.0; PLAYERS],
                grounded: vec![true; PLAYERS],
                doors,
                door_ids,
                moves: Vec::with_capacity(n),
                results: Vec::with_capacity(n),
                hits: Vec::new(),
            }
        }

        /// Static geometry only, the floor excluded (ARPG's `first_fixed_hit`).
        fn walls_only(&self) -> QueryFilter {
            QueryFilter {
                layers: CollisionLayers3d::new(1, 1),
                exclude: Some(self.floor),
                include_sensors: false,
            }
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            if let Some(d) = toggle(tick, self.doors.len()) {
                self.door_ids[d] = match self.door_ids[d].take() {
                    Some(id) => {
                        self.world.remove_body(id);
                        None
                    }
                    None => {
                        let (c, h) = self.doors[d];
                        let body = Body::keyed(DOOR_BASE + d as u64, Shape::Box(v(h)), v(c), 0.0);
                        Some(self.world.add_body(body).unwrap())
                    }
                };
            }
            self.moves.clear();
            for p in 0..PLAYERS {
                let ([x, z], jump) = player_intent(p, tick);
                self.vy[p] = if self.grounded[p] && jump {
                    TAKEOFF
                } else if self.grounded[p] {
                    0.0
                } else {
                    self.vy[p] - GRAVITY
                };
                self.moves.push(CharacterMove {
                    body: self.ids[p],
                    desired: Vector(x, self.vy[p], z),
                });
            }
            for m in 0..MONSTERS {
                self.moves.push(CharacterMove {
                    body: self.ids[PLAYERS + m],
                    desired: v(monster_intent(m, tick)),
                });
            }
            self.world
                .move_characters(&self.moves, &mut self.results)
                .unwrap();
            for (c, r) in self.results.iter().enumerate() {
                if c < PLAYERS {
                    let intended = self.vy[c];
                    self.vy[c] = r.translation.1.clamp(intended.min(0.0), intended.max(0.0));
                    self.grounded[c] = r.grounded && self.vy[c] <= 0.0;
                    if self.grounded[c] {
                        self.vy[c] = 0.0;
                    }
                }
                let p = self.world.body(r.body).unwrap().position;
                trace.0.push(Obs::Position([p.0, p.1, p.2]));
                trace.0.push(Obs::Grounded(c < PLAYERS && self.grounded[c]));
            }
            let filter = self.walls_only();
            let target = self.world.body(self.ids[0]).unwrap().position;
            for m in 0..MONSTERS {
                let from = self.world.body(self.ids[PLAYERS + m]).unwrap().position;
                let hit = self
                    .world
                    .cast_ray_first(RayCast::new(from, target - from), filter)
                    .unwrap();
                trace
                    .0
                    .push(Obs::Blocked(hit.is_some_and(|h| h.fraction < 1.0)));
            }
            for k in 0..PROJECTILES {
                let (origin, translation) = projectile(k, tick);
                let hit = self
                    .world
                    .cast_ray_first(RayCast::new(v(origin), v(translation)), filter)
                    .unwrap();
                trace.0.push(Obs::Blocked(hit.is_some()));
            }
            if tick % 20 == 0 {
                let statics = QueryFilter {
                    layers: CollisionLayers3d::new(1, 1),
                    ..QueryFilter::default()
                };
                for p in placements(tick) {
                    self.world
                        .overlap_shape(
                            QueryPose::new(Shape::capsule(HALF, RADIUS), v(p)),
                            statics,
                            &mut self.hits,
                        )
                        .unwrap();
                    let deep = self
                        .hits
                        .iter()
                        .any(|h| h.separation < -PENETRATION_TOLERANCE);
                    trace.0.push(Obs::Penetrates(deep));
                }
            }
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::super::upright_queries::UprightShapes;
    use super::*;
    use rapier3d::control::{CharacterLength, KinematicCharacterController};
    use rapier3d::parry::query::{
        DefaultQueryDispatcher, QueryDispatcher, QueryDispatcherChain, ShapeCastOptions,
    };
    use rapier3d::prelude::*;

    fn v(a: [f32; 3]) -> Vector {
        Vector::new(a[0], a[1], a[2])
    }

    pub struct Sim {
        world: PhysicsWorld,
        queries: QueryDispatcherChain<UprightShapes, DefaultQueryDispatcher>,
        planar: KinematicCharacterController,
        vertical: KinematicCharacterController,
        /// Static boxes in key order: walls and pillars, then locked doors (`None` unlocked).
        walls: Vec<BoxSpec>,
        doors: Vec<(BoxSpec, Option<ColliderHandle>)>,
        positions: Vec<[f32; 3]>,
        bodies: Vec<RigidBodyHandle>,
        floor: ColliderHandle,
        vy: Vec<f32>,
        grounded: Vec<bool>,
        shape: Capsule,
        rebuild: bool,
    }

    fn insert_fixed(world: &mut PhysicsWorld, (c, h): BoxSpec) -> ColliderHandle {
        world.colliders.insert(
            ColliderBuilder::cuboid(h[0], h[1], h[2])
                .translation(v(c))
                .build(),
        )
    }

    impl Sim {
        pub fn new(rebuild: bool) -> Self {
            let (walls, doors, starts) = scene();
            let planar = KinematicCharacterController {
                up: Vector::Y,
                offset: CharacterLength::Absolute(OFFSET),
                slide: true,
                autostep: None,
                max_slope_climb_angle: core::f32::consts::FRAC_PI_4,
                min_slope_slide_angle: core::f32::consts::FRAC_PI_4,
                snap_to_ground: None,
                normal_nudge_factor: KinematicCharacterController::default().normal_nudge_factor,
            };
            let mut sim = Self {
                world: PhysicsWorld::new(),
                queries: UprightShapes.chain(DefaultQueryDispatcher),
                planar,
                vertical: KinematicCharacterController {
                    snap_to_ground: Some(CharacterLength::Absolute(SUPPORT_REACH)),
                    ..planar
                },
                walls,
                doors: doors.into_iter().map(|d| (d, None)).collect(),
                positions: starts,
                bodies: Vec::new(),
                floor: ColliderHandle::invalid(),
                vy: vec![0.0; PLAYERS],
                grounded: vec![true; PLAYERS],
                shape: Capsule::new_y(HALF, RADIUS),
                rebuild,
            };
            sim.build();
            sim
        }

        /// ARPG's `rebuild`: static boxes, then characters, each in key order, then the floor.
        fn build(&mut self) {
            let mut world = PhysicsWorld::new();
            world.gravity = Vector::ZERO;
            world.integration_parameters.dt = DT;
            for &wall in &self.walls {
                insert_fixed(&mut world, wall);
            }
            // Doors start locked: the first build inserts every one.
            let first = self.bodies.is_empty();
            for (door, handle) in &mut self.doors {
                if handle.is_some() || first {
                    *handle = Some(insert_fixed(&mut world, *door));
                }
            }
            self.bodies = self
                .positions
                .iter()
                .map(|&p| {
                    let body = world.bodies.insert(
                        RigidBodyBuilder::kinematic_position_based()
                            .translation(v(p))
                            .build(),
                    );
                    world.colliders.insert_with_parent(
                        ColliderBuilder::new(SharedShape::new(self.shape)).build(),
                        body,
                        &mut world.bodies,
                    );
                    body
                })
                .collect();
            self.floor = world
                .colliders
                .insert(ColliderBuilder::new(SharedShape::halfspace(Vector::Y)).build());
            world.detect_collisions(&(), &());
            self.world = world;
        }

        fn pipeline<'a>(&'a self, filter: QueryFilter<'a>) -> QueryPipeline<'a> {
            let w = &self.world;
            w.broad_phase
                .as_query_pipeline(&self.queries, &w.bodies, &w.colliders, filter)
        }

        /// ARPG's `ground_below`: static ground within the support reach.
        fn ground_below(&self, position: [f32; 3]) -> bool {
            self.pipeline(QueryFilter::only_fixed())
                .cast_shape(
                    &Pose::from_translation(v(position)),
                    -Vector::Y,
                    &self.shape,
                    ShapeCastOptions {
                        target_distance: OFFSET,
                        stop_at_penetration: false,
                        max_time_of_impact: SUPPORT_REACH,
                        compute_impact_geometry_on_penetration: true,
                    },
                )
                .is_some_and(|(_, hit)| {
                    hit.normal1.dot(Vector::Y) >= self.vertical.max_slope_climb_angle.cos()
                })
        }

        /// ARPG's `set_position`: mirror the body there and `detect_collisions`.
        fn set_position(&mut self, c: usize, to: [f32; 3]) {
            if self.positions[c] == to {
                return;
            }
            self.positions[c] = to;
            self.world.bodies[self.bodies[c]].set_translation(v(to), false);
            self.world.detect_collisions(&(), &());
        }

        /// ARPG's `first_fixed_hit`: static geometry, the floor excluded.
        fn first_fixed_hit(&self, origin: [f32; 3], translation: [f32; 3]) -> Option<f32> {
            self.pipeline(QueryFilter::only_fixed().exclude_collider(self.floor))
                .cast_ray(&Ray::new(v(origin), v(translation)), 1.0, true)
                .map(|(_, toi)| toi)
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            if let Some(d) = toggle(tick, self.doors.len()) {
                // ARPG's `add_fixed` / `remove_fixed`, each with `detect_collisions`.
                let (door, handle) = &mut self.doors[d];
                match handle.take() {
                    Some(h) => {
                        self.world.remove_collider(h);
                    }
                    None => *handle = Some(insert_fixed(&mut self.world, *door)),
                }
                self.world.detect_collisions(&(), &());
            }
            if self.rebuild {
                self.build();
            }
            for p in 0..PLAYERS {
                let ([x, z], jump) = player_intent(p, tick);
                let intended_y = if self.grounded[p] && jump {
                    TAKEOFF
                } else if self.grounded[p] {
                    0.0
                } else {
                    self.vy[p] - GRAVITY
                };
                let from = self.positions[p];
                let movement = self.vertical.move_shape(
                    DT,
                    &self.pipeline(QueryFilter::default().exclude_rigid_body(self.bodies[p])),
                    &self.shape,
                    &Pose::from_translation(v(from)),
                    Vector::new(x, intended_y, z),
                    |_| {},
                );
                let moved = movement.translation;
                let to = [from[0] + moved.x, from[1] + moved.y, from[2] + moved.z];
                let supported = movement.grounded && self.ground_below(to);
                self.vy[p] = moved.y.clamp(intended_y.min(0.0), intended_y.max(0.0));
                self.grounded[p] = supported && self.vy[p] <= 0.0;
                if self.grounded[p] {
                    self.vy[p] = 0.0;
                }
                self.set_position(p, to);
            }
            for m in 0..MONSTERS {
                let c = PLAYERS + m;
                let from = self.positions[c];
                let movement = self.planar.move_shape(
                    DT,
                    &self.pipeline(
                        QueryFilter::default()
                            .exclude_rigid_body(self.bodies[c])
                            .exclude_collider(self.floor),
                    ),
                    &self.shape,
                    &Pose::from_translation(v(from)),
                    v(monster_intent(m, tick)),
                    |_| {},
                );
                let t = movement.translation;
                let to = [from[0] + t.x, from[1], from[2] + t.z];
                self.set_position(c, to);
            }
            for c in 0..PLAYERS + MONSTERS {
                trace.0.push(Obs::Position(self.positions[c]));
                trace.0.push(Obs::Grounded(c < PLAYERS && self.grounded[c]));
            }
            let target = self.positions[0];
            for m in 0..MONSTERS {
                let from = self.positions[PLAYERS + m];
                let translation = [
                    target[0] - from[0],
                    target[1] - from[1],
                    target[2] - from[2],
                ];
                let blocked = self
                    .first_fixed_hit(from, translation)
                    .is_some_and(|toi| toi < 1.0);
                trace.0.push(Obs::Blocked(blocked));
            }
            for k in 0..PROJECTILES {
                let (origin, translation) = projectile(k, tick);
                trace.0.push(Obs::Blocked(
                    self.first_fixed_hit(origin, translation).is_some(),
                ));
            }
            if tick % 20 == 0 {
                // ARPG's `penetrates_fixed`: static geometry, the floor included.
                let queries = self.pipeline(QueryFilter::only_fixed());
                for p in placements(tick) {
                    let pose = Pose::from_translation(v(p));
                    let deep = queries
                        .intersect_shape(pose, &self.shape)
                        .any(|(_, collider)| {
                            queries
                                .dispatcher
                                .contact(
                                    &pose.inv_mul(collider.position()),
                                    &self.shape,
                                    collider.shape(),
                                    0.0,
                                )
                                .ok()
                                .flatten()
                                .is_some_and(|contact| contact.dist < -PENETRATION_TOLERANCE)
                        });
                    trace.0.push(Obs::Penetrates(deep));
                }
            }
        }
    }
}
