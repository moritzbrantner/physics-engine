//! C4/C5 (MMORPG first): kinematic capsule characters in a Greyhaven-sized static town
//! (~1,150 boxes): idle on a halfspace ground, pushing into walls, wandering through streets,
//! standing on platform tops and jumping into ceilings, with game-side gravity. Rapier mirrors
//! MMORPG's call pattern: a full world rebuild in body-id order every tick (#292), then
//! `KinematicCharacterController::move_shape` per character in id order, `set_translation` and `detect_collisions` after each move, a downward static-only
//! support cast for grounded. Both engines stand on a halfspace ground, as MMORPG does (C2).

use crate::kcc::{DT, GRAVITY, HALF, Lanes, OFFSET, RADIUS, SNAP};
use crate::{Rng, Scenario, Trace};

const TICKS: u32 = 240;
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
        engine: |storage| Box::new(engine::Sim::new(storage)),
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
    let engine = Lanes::new(engine, TICKS, n, 2, 0)?;
    let rapier = Lanes::new(rapier, TICKS, n, 2, 0)?;
    let at = |t: &Lanes, tick: usize, i: usize| (t.position(tick, i), t.grounded(tick, i, 1));
    let boxes = layout();
    for tick in (0..TICKS as usize).step_by(30).chain([TICKS as usize - 1]) {
        for i in 0..n {
            let (p, _) = at(&engine, tick, i);
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
        let (rest, grounded) = at(&engine, settle, i);
        for tick in settle..TICKS as usize {
            let (p, g) = at(&engine, tick, i);
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
            let (r, rg) = at(&rapier, TICKS as usize - 1, i);
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
    use crate::Obs;
    use crate::kcc::engine::{Characters, capsules, fixed, v, world};
    use physics_engine::approximate::{BodyStorage, CharacterSettings, Shape};

    const CHARACTER_BASE: u64 = 100_000;

    pub struct Sim {
        characters: Characters,
        roles: Vec<Role>,
        intents: Vec<Intent>,
        grounded: Vec<bool>,
    }
    impl Sim {
        pub fn new(storage: BodyStorage) -> Self {
            let statics = layout().into_iter().enumerate().map(|(i, (c, h))| {
                if i == 0 {
                    // The consumers' ground is a halfspace through the floor box's top.
                    fixed(1, Shape::HalfSpace, [0.0; 3])
                } else {
                    fixed(1 + i as u64, Shape::Box(v(h)), c)
                }
            });
            let settings = CharacterSettings {
                offset: OFFSET,
                snap_to_ground: Some(SNAP),
                ..CharacterSettings::default()
            };
            let (starts, roles): (Vec<_>, Vec<_>) = characters().into_iter().unzip();
            let bodies = capsules(CHARACTER_BASE, Shape::capsule(HALF, RADIUS), starts);
            let n = roles.len();
            Self {
                characters: Characters::new(world(storage, statics), bodies, settings),
                roles,
                intents: vec![Intent { vy: 0.0 }; n],
                grounded: vec![false; n],
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            let (roles, grounded, intents) = (&self.roles, &self.grounded, &mut self.intents);
            self.characters
                .move_all(|i| v(desired(roles[i], i, tick, grounded[i], &mut intents[i])));
            for (i, r) in self.characters.results.iter().enumerate() {
                self.grounded[i] = r.grounded;
                trace
                    .0
                    .push(Obs::Position(self.characters.position(r.body)));
                trace.0.push(Obs::Grounded(r.grounded));
            }
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::*;
    use crate::Obs;
    use crate::kcc::rapier::{Characters, cuboid, halfspace, mirror_controller, v, walkable};
    use rapier3d::prelude::*;

    pub struct Sim {
        characters: Characters,
        roles: Vec<Role>,
        intents: Vec<Intent>,
        grounded: Vec<bool>,
    }

    impl Sim {
        pub fn new(rebuild: bool) -> Self {
            // MMORPG's ground is a halfspace: Rapier falls through a large thin floor box.
            let mut statics = vec![halfspace()];
            statics.extend(layout().into_iter().skip(1).map(cuboid));
            let (starts, roles): (Vec<_>, Vec<_>) = characters().into_iter().unzip();
            let n = roles.len();
            Self {
                characters: Characters::new(
                    statics,
                    starts,
                    Capsule::new_y(HALF, RADIUS),
                    mirror_controller(OFFSET, Some(SNAP), None),
                    rebuild,
                ),
                roles,
                intents: vec![Intent { vy: 0.0 }; n],
                grounded: vec![false; n],
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            let c = &mut self.characters;
            c.start_tick();
            for i in 0..self.roles.len() {
                let d = desired(
                    self.roles[i],
                    i,
                    tick,
                    self.grounded[i],
                    &mut self.intents[i],
                );
                let movement = c.move_shape(i, v(d));
                let to = c.end(i, &movement);
                // MMORPG/ARPG pattern: grounded only on static support below.
                let support = movement.grounded && c.support(to).is_some_and(|(_, h)| walkable(&h));
                self.grounded[i] = support;
                c.commit(i, to);
                trace.0.push(Obs::Position([to.x, to.y, to.z]));
                trace.0.push(Obs::Grounded(support));
            }
        }
    }
}
