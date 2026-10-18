//! C15 (#310): MMORPG's whole physics tick in a Greyhaven-sized town (~1,150 static boxes and a
//! halfspace ground). 64 characters idle, push into walls, wander, stand on platforms and
//! jump into ceilings, 16 pairs meet head-on in the lanes, door leaves toggle (an overlap probe,
//! then a remove and an add), and 48 line-of-sight segments are tested against static geometry
//! every tick.
//!
//! The Rapier side runs MMORPG's `World::step` call pattern (main at 0782d4b): a world rebuild
//! in body-id order, then per moving body a static-only ground cast, `slide_around` over every
//! body, `move_shape` with MMORPG's query dispatcher, a support cast and `set_translation` plus
//! `detect_collisions`. The engine side is how MMORPG would use physics-engine: one
//! `move_characters` batch, native head-on sidestep, and the batch's grounded results.

use super::character_move::{
    BoxSpec, CEILING_UNDERSIDE, FOOT, HALF, OFFSET, PLATFORM_TOP, RADIUS, Role, SNAP,
    capsule_box_gap, cell, characters as town_characters, layout, walk,
};
use crate::{Obs, Scenario, Trace};

const TICKS: u32 = 240;
/// MMORPG's zone tick.
const DT: f32 = 1.0 / 30.0;
/// Gravity, metres per tick².
const G: f32 = -9.81 * DT * DT;
/// Jump speed, metres per tick.
const JUMP: f32 = 6.0 * DT;
/// Head-on walking speed, metres per tick.
const HEAD_ON: f32 = 3.0 * DT;
const PAIRS: usize = 16;
const DOORS: usize = 8;
const SEGMENTS: usize = 48;
/// Body keys: ground 1, boxes 2.., doors from `DOOR_BASE`, characters from `PLAYER_BASE`.
const DOOR_BASE: u64 = 50_000;
const PLAYER_BASE: u64 = 100_000;
/// A ground gap below this counts as standing on it (MMORPG's 1e-3 units).
const GAP_TOLERANCE: f32 = 1.0e-5;
/// The support reach of MMORPG's ground casts and snapping.
const SUPPORT_REACH: f32 = OFFSET + SNAP;

#[derive(Clone, Copy, Debug, PartialEq)]
enum Who {
    Town(Role),
    /// Walks along x at the given direction (+1 or -1).
    HeadOn(f32),
}

/// Start position (game position: feet on the support) and behaviour of every character.
fn characters() -> Vec<([f32; 3], Who)> {
    let mut out = town_characters()
        .into_iter()
        .map(|(p, role)| ([p[0], p[1] - OFFSET, p[2]], Who::Town(role)))
        .collect::<Vec<_>>();
    for k in 0..PAIRS {
        // Lanes between building rows 8..11, centred between two columns.
        let (x, z) = cell(8 * 24 + (k / 4) * 24 + 4 * (k % 4) + 1);
        let (x, z) = (x + 5.0, z + 5.0);
        out.push(([x - 3.0, FOOT, z], Who::HeadOn(1.0)));
        out.push(([x + 3.0, FOOT, z], Who::HeadOn(-1.0)));
    }
    out
}

/// Door `d`'s leaf, closed or open: in the lane between two buildings of row 14.
fn door(d: usize, open: bool) -> BoxSpec {
    let (x, z) = cell(14 * 24 + 2 + 2 * d);
    if open {
        ([x + 5.9, 1.0, z - 1.1], [1.0, 1.0, 0.1])
    } else {
        ([x + 5.0, 1.0, z], [0.1, 1.0, 1.0])
    }
}

/// The door toggled at `tick`, if any.
fn toggle(tick: u32) -> Option<usize> {
    (tick % 15 == 7).then_some((tick / 15) as usize % DOORS)
}

/// Line-of-sight segments: origin and translation, at head height between lane crossings.
fn segments() -> Vec<([f32; 3], [f32; 3])> {
    (0..SEGMENTS)
        .map(|k| {
            let (ax, az) = cell((k * 11) % 576);
            let (bx, bz) = cell((k * 37 + 100) % 576);
            ([ax + 5.0, 1.5, az + 5.0], [bx - ax, 0.0, bz - az])
        })
        .collect()
}

/// Horizontal velocity (metres per tick) of character `i`.
fn horizontal(who: Who, i: usize, tick: u32) -> (f32, f32) {
    match who {
        Who::Town(role) => {
            let (x, z) = walk(role, i, tick);
            (x * DT, z * DT)
        }
        Who::HeadOn(direction) => (direction * HEAD_ON, 0.0),
    }
}

fn jumps(who: Who, tick: u32) -> bool {
    who == Who::Town(Role::Jumper) && tick % 40 == 10
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "mmorpg-tick",
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

/// Per tick: every character's game position and grounded state, then every segment's
/// blocked state.
fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    let chars = characters();
    let n = chars.len();
    let per_tick = 2 * n + SEGMENTS;
    if engine.0.len() != rapier.0.len() || engine.0.len() != per_tick * TICKS as usize {
        return Err("trace lengths differ".into());
    }
    let at = |t: &Trace, tick: usize, i: usize| -> ([f32; 3], bool) {
        let base = tick * per_tick + 2 * i;
        match (t.0[base], t.0[base + 1]) {
            (Obs::Position(p), Obs::Grounded(g)) => (p, g),
            _ => ([f32::NAN; 3], false),
        }
    };
    // Line of sight agrees exactly: it is static geometry and door state only.
    for tick in 0..TICKS as usize {
        let base = tick * per_tick + 2 * n;
        for s in 0..SEGMENTS {
            if engine.0[base + s] != rapier.0[base + s] {
                return Err(format!("segment {s} disagrees at tick {tick}"));
            }
        }
    }
    let boxes = layout();
    for tick in (0..TICKS as usize).step_by(30).chain([TICKS as usize - 1]) {
        for i in 0..n {
            let (p, _) = at(engine, tick, i);
            // Capsule centre: the game position (feet on the support) plus the offset.
            let centre = [p[0], p[1] + OFFSET, p[2]];
            for b in &boxes[1..] {
                if capsule_box_gap(centre, *b) < -1e-3 {
                    return Err(format!("character {i} penetrates a box at tick {tick}"));
                }
            }
        }
    }
    let settle = 120;
    for (i, &(start, who)) in chars.iter().enumerate() {
        let (rest, grounded) = at(engine, settle, i);
        match who {
            Who::Town(Role::Idle | Role::Platform | Role::Wall) => {
                for tick in settle..TICKS as usize {
                    let (p, g) = at(engine, tick, i);
                    if p != rest || !g || !grounded {
                        return Err(format!("{who:?} character {i} moved at tick {tick}"));
                    }
                }
                let support = if who == Who::Town(Role::Platform) {
                    PLATFORM_TOP
                } else {
                    0.0
                };
                if who != Who::Town(Role::Wall) && (rest[1] - (support + FOOT)).abs() > 1e-3 {
                    return Err(format!("{who:?} character {i} rests at {}", rest[1]));
                }
            }
            Who::Town(Role::Jumper) => {
                for tick in 0..TICKS as usize {
                    let (p, _) = at(engine, tick, i);
                    if p[1] + FOOT + OFFSET > CEILING_UNDERSIDE - OFFSET + 1e-3 {
                        return Err(format!("jumper {i} passed the ceiling at tick {tick}"));
                    }
                }
            }
            Who::HeadOn(direction) => {
                // Head-on pairs pass each other and keep walking. (On Rapier, MMORPG's
                // `slide_around` frees exactly head-on pairs, but pairs that meet slightly off
                // axis lock: the controller stops tangential motion against a touching capsule.)
                let (end, _) = at(engine, TICKS as usize - 1, i);
                if (end[0] - start[0]) * direction < 6.5 {
                    return Err(format!("head-on character {i} stuck at {end:?}"));
                }
            }
            Who::Town(Role::Wander) => {}
        }
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
            QueryHit, QueryPose, RayCast, Shape, Vector,
        },
    };

    fn v(a: [f32; 3]) -> Vector {
        Vector(a[0], a[1], a[2])
    }

    /// Characters on layer 2; line of sight only sees static geometry (layer 1).
    const CHARACTERS: CollisionLayers3d = CollisionLayers3d::new(2, u32::MAX);
    const STATIC_ONLY: QueryFilter = QueryFilter {
        layers: CollisionLayers3d::new(1, 1),
        exclude: None,
        include_sensors: false,
    };

    pub struct Sim {
        world: physics_engine::approximate::World,
        ids: Vec<BodyId>,
        who: Vec<Who>,
        vy: Vec<f32>,
        grounded: Vec<bool>,
        doors: Vec<(BodyId, bool)>,
        segments: Vec<([f32; 3], [f32; 3])>,
        moves: Vec<CharacterMove>,
        results: Vec<CharacterMoveResult>,
        hits: Vec<QueryHit>,
    }
    impl Sim {
        pub fn new() -> Self {
            let mut world = physics_engine::approximate::World::new(Config {
                body_storage: crate::engine_storage(),
                gravity: Vector::ZERO,
                ..Config::default()
            })
            .unwrap();
            for (i, (c, h)) in layout().into_iter().enumerate() {
                world
                    .add_body(if i == 0 {
                        Body::keyed(1, Shape::HalfSpace, Vector::ZERO, 0.0)
                    } else {
                        Body::keyed(1 + i as u64, Shape::Box(v(h)), v(c), 0.0)
                    })
                    .unwrap();
            }
            let doors = (0..DOORS)
                .map(|d| {
                    let (c, h) = door(d, false);
                    let body = Body::keyed(DOOR_BASE + d as u64, Shape::Box(v(h)), v(c), 0.0);
                    (world.add_body(body).unwrap(), false)
                })
                .collect();
            let settings = CharacterSettings {
                offset: OFFSET,
                snap_to_ground: Some(SNAP),
                ..CharacterSettings::default()
            };
            let (mut ids, mut who) = (Vec::new(), Vec::new());
            for (i, (p, w)) in characters().into_iter().enumerate() {
                let mut body = Body::keyed(
                    PLAYER_BASE + i as u64,
                    Shape::capsule(HALF, RADIUS),
                    v([p[0], p[1] + OFFSET, p[2]]),
                    1.0,
                );
                body.external = true;
                body.layers = CHARACTERS;
                let id = world.add_body(body).unwrap();
                world.add_character(id, settings).unwrap();
                ids.push(id);
                who.push(w);
            }
            let n = ids.len();
            Self {
                world,
                ids,
                who,
                vy: vec![0.0; n],
                grounded: vec![false; n],
                doors,
                segments: segments(),
                moves: Vec::with_capacity(n),
                results: Vec::with_capacity(n),
                hits: Vec::new(),
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            if let Some(d) = toggle(tick) {
                let (id, open) = self.doors[d];
                let (c, h) = door(d, !open);
                // A unit standing in the leaf's way keeps the door as it is.
                self.world
                    .overlap_shape(
                        QueryPose::new(Shape::Box(v(h)), v(c)),
                        QueryFilter::default(),
                        &mut self.hits,
                    )
                    .unwrap();
                let blocked = self.hits.iter().any(|hit| {
                    self.world
                        .body(hit.body)
                        .is_some_and(|b| b.key.is_some_and(|k| k >= PLAYER_BASE))
                });
                if !blocked {
                    self.world.remove_body(id);
                    let body = Body::keyed(DOOR_BASE + d as u64, Shape::Box(v(h)), v(c), 0.0);
                    self.doors[d] = (self.world.add_body(body).unwrap(), !open);
                }
            }
            self.moves.clear();
            for i in 0..self.ids.len() {
                let who = self.who[i];
                if self.grounded[i] && jumps(who, tick) {
                    self.vy[i] = JUMP;
                } else if self.grounded[i] && self.vy[i] <= 0.0 {
                    self.vy[i] = 0.0;
                } else {
                    self.vy[i] += G;
                }
                let (x, z) = horizontal(who, i, tick);
                self.moves.push(CharacterMove {
                    body: self.ids[i],
                    desired: Vector(x, self.vy[i], z),
                });
            }
            self.world
                .move_characters(&self.moves, &mut self.results)
                .unwrap();
            for (i, r) in self.results.iter().enumerate() {
                let vy = self.vy[i];
                if (r.grounded && vy <= 0.0) || (vy > 0.0 && r.translation.1 < vy - 1e-5) {
                    self.vy[i] = 0.0;
                }
                self.grounded[i] = r.grounded;
                let p = self.world.body(r.body).unwrap().position;
                trace.0.push(Obs::Position([p.0, p.1 - OFFSET, p.2]));
                trace.0.push(Obs::Grounded(r.grounded));
            }
            for &(origin, translation) in &self.segments {
                let hit = self
                    .world
                    .cast_ray_first(RayCast::new(v(origin), v(translation)), STATIC_ONLY)
                    .unwrap();
                trace
                    .0
                    .push(Obs::Blocked(hit.is_some_and(|h| h.fraction < 1.0)));
            }
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::super::mmorpg_queries::UprightShapes;
    use super::*;
    use rapier3d::control::{CharacterLength, KinematicCharacterController};
    use rapier3d::parry::query::{DefaultQueryDispatcher, QueryDispatcher, ShapeCastOptions};
    use rapier3d::prelude::*;

    fn v(a: [f32; 3]) -> Vector {
        Vector::new(a[0], a[1], a[2])
    }

    /// MMORPG's body list entry: what `World::rebuild` inserts, in id order.
    #[derive(Clone, Copy)]
    struct Entry {
        key: u64,
        /// `Some` for a moving capsule (game position), `None` for a static box.
        moving: Option<[f32; 3]>,
        shape: BoxSpec,
    }

    type Dispatcher =
        rapier3d::parry::query::QueryDispatcherChain<UprightShapes, DefaultQueryDispatcher>;

    pub struct Sim {
        world: PhysicsWorld,
        queries: Dispatcher,
        controller: KinematicCharacterController,
        /// Every body in key order (MMORPG's `BTreeMap<BodyId, RigidBody>`).
        entries: Vec<Entry>,
        handles: Vec<Option<RigidBodyHandle>>,
        who: Vec<Who>,
        velocity: Vec<[f32; 3]>,
        /// Grounded after the step, as MMORPG's `grounded` would answer it.
        grounded: Vec<bool>,
        doors: Vec<bool>,
        segments: Vec<([f32; 3], [f32; 3])>,
        shape: Capsule,
        rebuild: bool,
    }

    fn lifted(p: [f32; 3]) -> Vector {
        Vector::new(p[0], p[1] + OFFSET, p[2])
    }

    impl Sim {
        pub fn new(rebuild: bool) -> Self {
            let mut entries = layout()
                .into_iter()
                .enumerate()
                .map(|(i, shape)| Entry {
                    key: 1 + i as u64,
                    moving: None,
                    shape,
                })
                .collect::<Vec<_>>();
            entries.extend((0..DOORS).map(|d| Entry {
                key: DOOR_BASE + d as u64,
                moving: None,
                shape: door(d, false),
            }));
            let chars = characters();
            entries.extend(chars.iter().enumerate().map(|(i, &(p, _))| Entry {
                key: PLAYER_BASE + i as u64,
                moving: Some(p),
                shape: ([0.0; 3], [0.0; 3]),
            }));
            let mut sim = Self {
                world: PhysicsWorld::new(),
                queries: UprightShapes.chain(DefaultQueryDispatcher),
                controller: KinematicCharacterController {
                    up: Vector::Y,
                    offset: CharacterLength::Absolute(OFFSET),
                    slide: true,
                    autostep: None,
                    max_slope_climb_angle: core::f32::consts::FRAC_PI_4,
                    min_slope_slide_angle: core::f32::consts::FRAC_PI_4,
                    snap_to_ground: Some(CharacterLength::Absolute(SUPPORT_REACH)),
                    normal_nudge_factor: KinematicCharacterController::default()
                        .normal_nudge_factor,
                },
                entries,
                handles: Vec::new(),
                who: chars.iter().map(|&(_, w)| w).collect(),
                velocity: vec![[0.0; 3]; chars.len()],
                grounded: vec![false; chars.len()],
                doors: vec![false; DOORS],
                segments: segments(),
                shape: Capsule::new_y(HALF, RADIUS),
                rebuild,
            };
            sim.build();
            sim
        }

        /// MMORPG's `World::rebuild`: a fresh world from the bodies in id order, then
        /// `detect_collisions`.
        fn build(&mut self) {
            let mut world = PhysicsWorld::new();
            world.gravity = Vector::ZERO;
            world.integration_parameters.dt = DT;
            self.handles.clear();
            for e in &self.entries {
                let user_data = u128::from(e.key);
                let handle = if let Some(p) = e.moving {
                    let handle = world.bodies.insert(
                        RigidBodyBuilder::kinematic_position_based()
                            .translation(lifted(p))
                            .build(),
                    );
                    world.colliders.insert_with_parent(
                        ColliderBuilder::new(SharedShape::new(self.shape))
                            .user_data(user_data)
                            .build(),
                        handle,
                        &mut world.bodies,
                    );
                    Some(handle)
                } else if e.key == 1 {
                    world.colliders.insert(
                        ColliderBuilder::new(SharedShape::halfspace(Vector::Y))
                            .user_data(user_data)
                            .build(),
                    );
                    None
                } else {
                    let (c, h) = e.shape;
                    world.colliders.insert(
                        ColliderBuilder::cuboid(h[0], h[1], h[2])
                            .translation(v(c))
                            .user_data(user_data)
                            .build(),
                    );
                    None
                };
                self.handles.push(handle);
            }
            world.detect_collisions(&(), &());
            self.world = world;
        }

        fn first_character(&self) -> usize {
            self.entries.len() - self.who.len()
        }

        /// MMORPG's `ground_gap`: static ground within the support reach below `position`.
        fn ground_gap(&self, position: [f32; 3]) -> Option<f32> {
            let w = &self.world;
            let fixed = w.broad_phase.as_query_pipeline(
                &self.queries,
                &w.bodies,
                &w.colliders,
                QueryFilter::only_fixed(),
            );
            fixed
                .cast_shape(
                    &Pose::from_translation(lifted(position)),
                    -Vector::Y,
                    &self.shape,
                    ShapeCastOptions {
                        target_distance: OFFSET,
                        stop_at_penetration: false,
                        max_time_of_impact: SUPPORT_REACH,
                        compute_impact_geometry_on_penetration: true,
                    },
                )
                .filter(|(_, hit)| {
                    hit.normal1.dot(Vector::Y) >= self.controller.max_slope_climb_angle.cos()
                })
                .map(|(_, hit)| hit.time_of_impact)
        }

        /// MMORPG's `slide_around`: motion into a touching moving capsule turned along it.
        fn slide_around(&self, me: usize, desired: [f32; 3]) -> [f32; 3] {
            let [mut vx, vy, mut vz] = desired;
            let speed = vx.hypot(vz);
            if speed == 0.0 {
                return desired;
            }
            let p = self.entries[me].moving.unwrap();
            let top = HALF + RADIUS;
            let (low, high) = (p[1] - top, p[1] + top);
            for (j, other) in self.entries.iter().enumerate() {
                let Some(q) = other.moving else {
                    continue;
                };
                if j == me || q[1] + top < low || q[1] - top > high {
                    continue;
                }
                let (dx, dz) = (p[0] - q[0], p[2] - q[2]);
                let distance = dx.hypot(dz);
                // MMORPG: both radii, the contact offset and one unit (1 cm).
                let reach = 2.0 * RADIUS + OFFSET + 0.01;
                if distance == 0.0 || distance > reach {
                    continue;
                }
                let (nx, nz) = (dx / distance, dz / distance);
                let into = vx * nx + vz * nz;
                if into >= 0.0 {
                    continue;
                }
                vx -= into * nx;
                vz -= into * nz;
                if vx.hypot(vz) < 0.25 * speed {
                    vx = -nz * speed;
                    vz = nx * speed;
                }
            }
            [vx, vy, vz]
        }

        fn toggle_door(&mut self, d: usize) {
            let (c, h) = door(d, !self.doors[d]);
            let w = &self.world;
            let all = w.broad_phase.as_query_pipeline(
                &self.queries,
                &w.bodies,
                &w.colliders,
                QueryFilter::default(),
            );
            let blocked = all
                .intersect_shape(Pose::from_translation(v(c)), &Cuboid::new(v(h)))
                .any(|(_, collider)| collider.user_data >= u128::from(PLAYER_BASE));
            if blocked {
                return;
            }
            self.doors[d] = !self.doors[d];
            let index = self.first_character() - DOORS + d;
            self.entries[index].shape = (c, h);
            // MMORPG's `replace_body`: remove (refresh), then add (refresh).
            let key = u128::from(DOOR_BASE + d as u64);
            let old = self
                .world
                .colliders
                .iter()
                .find(|(_, collider)| collider.user_data == key)
                .map(|(handle, _)| handle)
                .unwrap();
            self.world.remove_collider(old);
            self.world.detect_collisions(&(), &());
            self.world.colliders.insert(
                ColliderBuilder::cuboid(h[0], h[1], h[2])
                    .translation(v(c))
                    .user_data(key)
                    .build(),
            );
            self.world.detect_collisions(&(), &());
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            if let Some(d) = toggle(tick) {
                self.toggle_door(d);
            }
            // The zone sets velocities (grounded jumps) before the step.
            let first = self.first_character();
            for i in 0..self.who.len() {
                let (x, z) = horizontal(self.who[i], i, tick);
                self.velocity[i][0] = x;
                self.velocity[i][2] = z;
                let p = self.entries[first + i].moving.unwrap();
                if jumps(self.who[i], tick) && self.ground_gap(p).is_some() {
                    self.velocity[i][1] = JUMP;
                }
            }
            if self.rebuild {
                self.build();
            }
            for i in 0..self.who.len() {
                let me = first + i;
                let p = self.entries[me].moving.unwrap();
                let mut velocity = self.velocity[i];
                let mut settle = 0.0;
                let gap = self.ground_gap(p);
                self.grounded[i] = gap.is_some();
                match gap {
                    Some(gap) if velocity[1] <= 0.0 => {
                        velocity[1] = 0.0;
                        if gap > GAP_TOLERANCE {
                            settle = -gap;
                        }
                    }
                    _ => velocity[1] += G,
                }
                let desired = [velocity[0], velocity[1] + settle, velocity[2]];
                if desired == [0.0; 3] {
                    self.velocity[i] = velocity;
                } else {
                    let desired = self.slide_around(me, desired);
                    let handle = self.handles[me].unwrap();
                    let w = &self.world;
                    let queries = w.broad_phase.as_query_pipeline(
                        &self.queries,
                        &w.bodies,
                        &w.colliders,
                        QueryFilter::default().exclude_rigid_body(handle),
                    );
                    let movement = self.controller.move_shape(
                        DT,
                        &queries,
                        &self.shape,
                        &Pose::from_translation(lifted(p)),
                        v(desired),
                        |_| {},
                    );
                    let moved = movement.translation;
                    let to = [p[0] + moved.x, p[1] + moved.y, p[2] + moved.z];
                    let supported = movement.grounded && self.ground_gap(to).is_some();
                    self.grounded[i] = supported;
                    if (supported && velocity[1] <= 0.0)
                        || (velocity[1] > 0.0 && moved.y < velocity[1] - 1e-5)
                    {
                        velocity[1] = 0.0;
                    }
                    velocity[0] = moved.x;
                    velocity[2] = moved.z;
                    self.entries[me].moving = Some(to);
                    self.velocity[i] = velocity;
                    self.world.bodies[handle].set_translation(lifted(to), false);
                    self.world.detect_collisions(&(), &());
                }
            }
            for i in 0..self.who.len() {
                let p = self.entries[first + i].moving.unwrap();
                trace.0.push(Obs::Position(p));
                trace.0.push(Obs::Grounded(self.grounded[i]));
            }
            for &(origin, translation) in &self.segments {
                let w = &self.world;
                let predicate =
                    |_, collider: &Collider| collider.user_data < u128::from(PLAYER_BASE);
                let queries = w.broad_phase.as_query_pipeline(
                    &self.queries,
                    &w.bodies,
                    &w.colliders,
                    QueryFilter::default().predicate(&predicate),
                );
                let blocked = queries
                    .cast_ray(&Ray::new(v(origin), v(translation)), 1.0, true)
                    .is_some_and(|(_, toi)| toi < 1.0);
                trace.0.push(Obs::Blocked(blocked));
            }
        }
    }
}
