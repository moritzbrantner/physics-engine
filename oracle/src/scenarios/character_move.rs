//! C4/C5 (MMORPG first): kinematic capsule characters in a Greyhaven-sized static town
//! (~1,150 boxes): idle on a halfspace ground, pushing into walls, wandering through streets,
//! standing on platform tops and jumping into ceilings, with game-side gravity. Rapier mirrors
//! MMORPG's call pattern: a full world rebuild in body-id order every tick (#292), then
//! `KinematicCharacterController::move_shape` per character in id order, `set_translation` and `detect_collisions` after each move, a downward static-only
//! support cast for grounded. Both engines stand on a halfspace ground, as MMORPG does (C2).

use crate::{Obs, Rng, Scenario, Trace};

const TICKS: u32 = 240;
pub(crate) const DT: f32 = 1.0 / 60.0;
pub(crate) const GRAVITY: f32 = 9.81;
pub(crate) const OFFSET: f32 = 0.0025;
pub(crate) const SNAP: f32 = 0.05;
pub(crate) const HALF: f32 = 0.5;
pub(crate) const RADIUS: f32 = 0.3;
pub(crate) const FOOT: f32 = HALF + RADIUS;
pub(crate) const CEILING_UNDERSIDE: f32 = 2.5;
pub(crate) const PLATFORM_TOP: f32 = 1.0;
/// Rapier sinks into box tops (#305) and nudges resting characters (#303): its resting
/// positions may differ by this much.
const RAPIER_REST_TOLERANCE: f32 = 0.05;

pub(crate) type BoxSpec = ([f32; 3], [f32; 3]);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Role {
    Idle,
    Wall,
    Wander,
    Platform,
    Jumper,
}

const ROLES: [(Role, usize); 5] = [
    (Role::Idle, 16),
    (Role::Wall, 16),
    (Role::Wander, 16),
    (Role::Platform, 8),
    (Role::Jumper, 8),
];

pub(crate) fn cell(index: usize) -> (f32, f32) {
    (
        -115.0 + 10.0 * (index % 24) as f32,
        -115.0 + 10.0 * (index / 24) as f32,
    )
}

/// Floor, 576 buildings, 400 pillars, 150 platforms and 24 ceiling slabs.
pub(crate) fn layout() -> Vec<BoxSpec> {
    let mut rng = Rng(0x5eed_6eed_1234_5678);
    let mut boxes = vec![([0.0, -0.5, 0.0], [130.0, 0.5, 130.0])];
    for i in 0..576 {
        let (x, z) = cell(i);
        let h = rng.range(1.0, 4.0);
        boxes.push(([x, h, z], [3.0, h, 3.0]));
    }
    for i in 0..400 {
        let (x, z) = cell(i);
        boxes.push(([x + 4.2, 1.5, z + 4.2], [0.3, 1.5, 0.3]));
    }
    for i in 0..150 {
        let (x, z) = cell(i);
        boxes.push(([x + 5.0, PLATFORM_TOP - 0.25, z], [1.0, 0.25, 1.0]));
    }
    for i in 0..24 {
        let (x, z) = cell(i);
        boxes.push(([x, CEILING_UNDERSIDE + 0.25, z + 5.0], [1.5, 0.25, 1.5]));
    }
    boxes
}

/// Start position and role of every character, in id order.
pub(crate) fn characters() -> Vec<([f32; 3], Role)> {
    let mut out = Vec::new();
    let rest = FOOT + OFFSET;
    for (role, count) in ROLES {
        for k in 0..count {
            let p = match role {
                Role::Idle => {
                    let (x, z) = cell(300 + k * 7);
                    [x + 5.0, rest + 0.5, z + 5.0]
                }
                Role::Wall => {
                    let (x, z) = cell(400 + k * 9);
                    [x + 5.0, rest, z]
                }
                Role::Wander => {
                    let (x, z) = cell(450 + k * 5);
                    [x + 5.0, rest, z + 5.0]
                }
                Role::Platform => {
                    let (x, z) = cell(k * 3);
                    [x + 5.0, PLATFORM_TOP + rest, z]
                }
                Role::Jumper => {
                    let (x, z) = cell(k * 2);
                    [x, rest, z + 5.0]
                }
            };
            out.push((p, role));
        }
    }
    out
}

/// Game-side intent shared by both engines: horizontal walking per role, jumps and gravity.
#[derive(Clone, Copy)]
struct Intent {
    vy: f32,
}

fn desired(role: Role, index: usize, tick: u32, grounded: bool, intent: &mut Intent) -> [f32; 3] {
    if grounded {
        intent.vy = 0.0;
        if role == Role::Jumper && tick % 40 == 10 {
            intent.vy = 6.0;
        }
    }
    intent.vy -= GRAVITY * DT;
    let (vx, vz) = walk(role, index, tick);
    [vx * DT, intent.vy * DT, vz * DT]
}

/// Horizontal walking velocity (m/s) of character `index` with `role` at `tick`.
pub(crate) fn walk(role: Role, index: usize, tick: u32) -> (f32, f32) {
    match role {
        Role::Wall => (2.0, 0.0),
        Role::Wander => {
            let mut rng = Rng(0x77 + index as u64 * 1_000_003 + u64::from(tick / 60));
            let a = rng.range(0.0, core::f32::consts::TAU);
            (3.0 * a.cos(), 3.0 * a.sin())
        }
        _ => (0.0, 0.0),
    }
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "character-move",
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

/// Distance from a capsule (upright, centre `p`) to a box, minus the capsule radius.
pub(crate) fn capsule_box_gap(p: [f32; 3], (c, h): BoxSpec) -> f32 {
    // The segment point nearest the box: the y ranges' overlap, else the nearer endpoint.
    let (lo, hi) = (p[1] - HALF, p[1] + HALF);
    let (bl, bh) = (c[1] - h[1], c[1] + h[1]);
    let y = if hi < bl {
        hi
    } else if lo > bh {
        lo
    } else {
        c[1].clamp(lo.max(bl), hi.min(bh))
    };
    let q = [p[0], y, p[2]];
    let d: f32 = (0..3)
        .map(|a| {
            let e = (q[a] - c[a]).abs() - h[a];
            e.max(0.0).powi(2)
        })
        .sum::<f32>()
        .sqrt();
    d - RADIUS
}

fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    let chars = characters();
    let n = chars.len();
    if engine.0.len() != rapier.0.len() || engine.0.len() != 2 * n * TICKS as usize {
        return Err("trace lengths differ".into());
    }
    let at = |t: &Trace, tick: usize, i: usize| -> ([f32; 3], bool) {
        let base = (tick * n + i) * 2;
        match (t.0[base], t.0[base + 1]) {
            (Obs::Position(p), Obs::Grounded(g)) => (p, g),
            _ => ([f32::NAN; 3], false),
        }
    };
    let boxes = layout();
    for tick in (0..TICKS as usize).step_by(30).chain([TICKS as usize - 1]) {
        for i in 0..n {
            let (p, _) = at(engine, tick, i);
            for b in &boxes {
                let gap = capsule_box_gap(p, *b);
                if gap < -1e-3 {
                    return Err(format!(
                        "character {i} penetrates a box by {} at tick {tick}",
                        -gap
                    ));
                }
            }
        }
    }
    for (i, &(_, role)) in chars.iter().enumerate() {
        let settle = 120;
        let (rest, grounded) = at(engine, settle, i);
        for tick in settle..TICKS as usize {
            let (p, g) = at(engine, tick, i);
            match role {
                Role::Idle | Role::Platform | Role::Wall => {
                    if p != rest || !g || !grounded {
                        return Err(format!(
                            "{role:?} character {i} moved or lost support at tick {tick}: {p:?} vs {rest:?}"
                        ));
                    }
                }
                Role::Jumper => {
                    if p[1] + FOOT > CEILING_UNDERSIDE - OFFSET + 1e-3 {
                        return Err(format!("jumper {i} passed the ceiling at tick {tick}"));
                    }
                }
                Role::Wander => {}
            }
        }
        if matches!(role, Role::Idle | Role::Platform) {
            let support = if role == Role::Platform {
                PLATFORM_TOP
            } else {
                0.0
            };
            if (rest[1] - (support + FOOT + OFFSET)).abs() > 1e-3 {
                return Err(format!(
                    "{role:?} character {i} rests at {} not on its support",
                    rest[1]
                ));
            }
            let (r, rg) = at(rapier, TICKS as usize - 1, i);
            if !rg || (0..3).any(|a| (r[a] - rest[a]).abs() > RAPIER_REST_TOLERANCE) {
                return Err(format!(
                    "{role:?} character {i}: engine {rest:?}, rapier {r:?} grounded {rg}"
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
            Body, CharacterMove, CharacterMoveResult, CharacterSettings, Config, Shape, Vector,
            World,
        },
    };

    const CHARACTER_BASE: u64 = 100_000;

    fn v(a: [f32; 3]) -> Vector {
        Vector(a[0], a[1], a[2])
    }

    pub struct Sim {
        world: World,
        ids: Vec<BodyId>,
        roles: Vec<Role>,
        intents: Vec<Intent>,
        grounded: Vec<bool>,
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
            for (i, (c, h)) in layout().into_iter().enumerate() {
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
                snap_to_ground: Some(SNAP),
                ..CharacterSettings::default()
            };
            let mut roles = Vec::new();
            let mut ids = Vec::new();
            for (i, (p, role)) in characters().into_iter().enumerate() {
                let mut body = Body::keyed(
                    CHARACTER_BASE + i as u64,
                    Shape::capsule(HALF, RADIUS),
                    v(p),
                    1.0,
                );
                body.external = true;
                let id = world.add_body(body).unwrap();
                world.add_character(id, settings).unwrap();
                ids.push(id);
                roles.push(role);
            }
            let n = roles.len();
            Self {
                world,
                ids,
                roles,
                intents: vec![Intent { vy: 0.0 }; n],
                grounded: vec![false; n],
                moves: Vec::with_capacity(n),
                results: Vec::with_capacity(n),
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            self.moves.clear();
            for i in 0..self.roles.len() {
                let d = desired(
                    self.roles[i],
                    i,
                    tick,
                    self.grounded[i],
                    &mut self.intents[i],
                );
                self.moves.push(CharacterMove {
                    body: self.ids[i],
                    desired: v(d),
                });
            }
            self.world
                .move_characters(&self.moves, &mut self.results)
                .unwrap();
            for (i, r) in self.results.iter().enumerate() {
                self.grounded[i] = r.grounded;
                let p = self.world.body(r.body).unwrap().position;
                trace.0.push(Obs::Position([p.0, p.1, p.2]));
                trace.0.push(Obs::Grounded(r.grounded));
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
        bodies: Vec<RigidBodyHandle>,
        roles: Vec<Role>,
        intents: Vec<Intent>,
        grounded: Vec<bool>,
        shape: Capsule,
        /// MMORPG rebuilds the whole world in body-id order every tick (#292).
        rebuild: bool,
        positions: Vec<Vector>,
    }

    /// Ground halfspace, static boxes and character bodies at `positions`, in id order.
    fn build(positions: &[Vector], shape: Capsule) -> (PhysicsWorld, Vec<RigidBodyHandle>) {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::ZERO;
        world.integration_parameters.dt = DT;
        // MMORPG's ground is a halfspace: Rapier falls through a large thin floor box.
        world
            .colliders
            .insert(ColliderBuilder::new(SharedShape::halfspace(Vector::Y)).build());
        for (c, h) in layout().into_iter().skip(1) {
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
        world.detect_collisions(&(), &());
        (world, bodies)
    }

    impl Sim {
        pub fn new(rebuild: bool) -> Self {
            let shape = Capsule::new_y(HALF, RADIUS);
            let (positions, roles): (Vec<_>, Vec<_>) = characters()
                .into_iter()
                .map(|(p, role)| (v(p), role))
                .unzip();
            let (world, bodies) = build(&positions, shape);
            let controller = KinematicCharacterController {
                up: Vector::Y,
                offset: CharacterLength::Absolute(OFFSET),
                slide: true,
                autostep: None,
                max_slope_climb_angle: core::f32::consts::FRAC_PI_4,
                min_slope_slide_angle: core::f32::consts::FRAC_PI_4,
                snap_to_ground: Some(CharacterLength::Absolute(OFFSET + SNAP)),
                normal_nudge_factor: KinematicCharacterController::default().normal_nudge_factor,
            };
            let n = roles.len();
            Self {
                world,
                controller,
                bodies,
                roles,
                intents: vec![Intent { vy: 0.0 }; n],
                grounded: vec![false; n],
                shape,
                rebuild,
                positions,
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            if self.rebuild {
                (self.world, self.bodies) = build(&self.positions, self.shape);
            }
            let dispatcher = DefaultQueryDispatcher;
            for i in 0..self.roles.len() {
                let d = desired(
                    self.roles[i],
                    i,
                    tick,
                    self.grounded[i],
                    &mut self.intents[i],
                );
                let body = self.bodies[i];
                let from = self.world.bodies[body].translation();
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
                    v(d),
                    |_| {},
                );
                let to = from + movement.translation;
                // MMORPG/ARPG pattern: grounded only on static support below.
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
                self.grounded[i] = support;
                self.positions[i] = to;
                self.world.bodies[body].set_translation(to, false);
                self.world.detect_collisions(&(), &());
                trace.0.push(Obs::Position([to.x, to.y, to.z]));
                trace.0.push(Obs::Grounded(support));
            }
        }
    }
}
