//! Grapple hook (#557, moenarch-games/shooter#5): a kinematic capsule tied to a fixed world
//! point by a rope it reels in, swings on and lets go of. The shooter's `GRAPPLE` scenario
//! (0.6 × 1.8 m capsule, 18 m/s² gravity, 60 Hz) runs beside a swing under a ledge, a release
//! in mid-air and a pull blocked by a pillar, each lane on its own geometry 40 m apart.
//!
//! Both sides share the shooter's consumer logic: gravity, the reel's 45 m/s² toward the anchor
//! up to 24 m/s, the rope shortening to the distance reached (at least 1 m), walking when
//! grounded and free, and a velocity taken from each move's translation. The engine holds the
//! rope as a `Tether` honoured inside `move_characters`; Rapier has no such feature, so its side
//! applies the same tether math to the desired motion around `move_shape`, as the shooter's
//! stand-in does around its world step.

use crate::kcc::{DT, Lanes, OFFSET, SNAP};
use crate::{Obs, Scenario, Trace};

const TICKS: u32 = 175;
const LANES: usize = 4;
const GRAVITY: f32 = 18.0;
const HALF: f32 = 0.6;
const RADIUS: f32 = 0.3;
const REST: f32 = HALF + RADIUS + OFFSET;
/// The shooter's eye, 1.65 m above the feet: where the rope holds on the character.
const EYE: [f32; 3] = [0.0, 1.65 - HALF - RADIUS, 0.0];
const PULL: f32 = 45.0;
const MAX_PULL_SPEED: f32 = 24.0;
const MIN_LENGTH: f32 = 1.0;
const WALK: f32 = 7.2;
/// Engine and Rapier character positions stay this close every tick (measured 3.3 mm; the
/// shooter's A/B tolerance for its stand-in is 0.25 m).
const TOLERANCE: f32 = 0.01;
/// While walking, one tick of walking more: Rapier 0.36.1's controller stalls for one tick on
/// the ledge's flat box top, 0.2 mm above rest (`probe -- --box 0,1.5,6,3,1.5,3 --capsule
/// 0.6,0.3 --at -0.00044617514,3.9026887,5.4579597 --desired 0,-0.0049113133,0.120000005`:
/// Rapier moves 4.6 mm, the engine 0.12 m), and catches up at the pillar.
const WALK_TOLERANCE: f32 = TOLERANCE + WALK * DT;
/// The most a rope's attachment may stand past the rope after a move: the linearized
/// constraint lets a swing's arc stretch it slightly (measured 1.4 mm), which the next move
/// draws back.
const STRETCH: f32 = 0.005;

const GRAPPLE: usize = 0;
const SWING: usize = 1;
const RELEASE: usize = 2;
const BLOCKED: usize = 3;

fn x0(lane: usize) -> f32 {
    40.0 * lane as f32
}

/// Static boxes (centre, half extents) of `lane`.
fn boxes(lane: usize) -> Vec<([f32; 3], [f32; 3])> {
    let x = x0(lane);
    match lane {
        // A 3 m ledge whose face is 9 m from the start, an 8 m pillar on it.
        GRAPPLE => vec![
            ([x, 1.5, 6.0], [3.0, 1.5, 3.0]),
            ([x, 4.0, 7.0], [0.5, 4.0, 0.5]),
        ],
        // An overhang 10 m up.
        SWING => vec![([x, 10.5, 0.0], [4.0, 0.5, 4.0])],
        // A 12 m wall.
        RELEASE => vec![([x, 6.0, 10.0], [3.0, 6.0, 0.5])],
        // A wall with a 4 m pillar in front of it.
        _ => vec![
            ([x, 3.0, 10.0], [3.0, 3.0, 0.5]),
            ([x, 2.0, 4.0], [0.5, 2.0, 0.5]),
        ],
    }
}

/// Where the hook holds: the pillar's face 5 m up, the overhang's underside, high on the
/// wall, the wall behind the pillar.
fn anchor(lane: usize) -> [f32; 3] {
    let x = x0(lane);
    match lane {
        GRAPPLE => [x, 5.0, 6.5],
        SWING => [x, 10.0, 0.0],
        RELEASE => [x, 12.0, 9.5],
        _ => [x, 2.0, 9.5],
    }
}

fn start(lane: usize) -> [f32; 3] {
    let x = x0(lane);
    match lane {
        GRAPPLE => [x, REST, -6.0],
        SWING => [x, 4.0, -5.0],
        RELEASE => [x, REST, 0.0],
        _ => [x, REST, -2.0],
    }
}

/// What `lane` does at `tick`.
#[derive(Clone, Copy, PartialEq)]
enum Phase {
    /// On the rope, reeling it in or not.
    Hooked { reel: bool },
    /// Free, walking along this horizontal velocity when grounded.
    Free([f32; 2]),
}

fn phase(lane: usize, tick: u32) -> Phase {
    let reel = Phase::Hooked { reel: true };
    let stand = Phase::Free([0.0; 2]);
    match (lane, tick) {
        // The shooter's legs: take the grapple, hook and reel in, let go onto the ledge, stand.
        (GRAPPLE, 0..15) => stand,
        (GRAPPLE, 15..105) => reel,
        (GRAPPLE, 105..145) => Phase::Free([0.0, WALK]),
        // Swing from rest without reeling and let go past the bottom of the arc.
        (SWING, 0..90) => Phase::Hooked { reel: false },
        // Reeled off the floor and let go in mid-air short of the wall.
        (RELEASE, 5..25) => reel,
        // Pulled into the pillar until the player gives up.
        (BLOCKED, 0..120) => reel,
        _ => stand,
    }
}

fn add(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn sub(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn scale(a: [f32; 3], s: f32) -> [f32; 3] {
    a.map(|x| x * s)
}
fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn length(a: [f32; 3]) -> f32 {
    dot(a, a).sqrt()
}

/// The attachment's distance from `lane`'s anchor with the character centred at `centre`.
fn span(lane: usize, centre: [f32; 3]) -> f32 {
    length(sub(anchor(lane), add(centre, EYE)))
}

/// The tether's change to `desired` (the engine's `Tether` rule, applied around Rapier).
#[cfg_attr(not(feature = "rapier"), allow(dead_code))]
fn tether(lane: usize, rope: f32, centre: [f32; 3], desired: [f32; 3]) -> [f32; 3] {
    let to_anchor = sub(anchor(lane), add(centre, EYE));
    let span = length(to_anchor);
    if span <= OFFSET * 1e-3 {
        return [0.0; 3];
    }
    let toward = scale(to_anchor, 1.0 / span);
    let wanted = (span - rope).min(MAX_PULL_SPEED * DT);
    let inward = dot(desired, toward);
    if inward < wanted {
        scale(toward, wanted - inward)
    } else {
        [0.0; 3]
    }
}

/// The rope a lane's move holds, if any.
#[derive(Clone, Copy)]
struct Rope {
    length: f32,
    /// First tick on the rope.
    attach: bool,
}

/// The shooter's per-tick player and hook logic for every lane, identical on both sides.
struct Consumer {
    velocity: [[f32; 3]; LANES],
    rope: [Option<f32>; LANES],
    grounded: [bool; LANES],
}

impl Consumer {
    fn new() -> Self {
        Self {
            velocity: [[0.0; 3]; LANES],
            rope: [None; LANES],
            grounded: [false; LANES],
        }
    }

    /// The desired motion of `lane` centred at `centre` this tick, and the rope it holds.
    fn plan(&mut self, lane: usize, tick: u32, centre: [f32; 3]) -> ([f32; 3], Option<Rope>) {
        let v = &mut self.velocity[lane];
        let rope = match phase(lane, tick) {
            Phase::Hooked { reel } => {
                let span = span(lane, centre);
                let attach = self.rope[lane].is_none();
                let mut rope = self.rope[lane].unwrap_or(span);
                if reel {
                    // The reel: toward the anchor until the pull's own speed limit, and the
                    // rope shortens to the distance reached.
                    let toward = scale(sub(anchor(lane), add(centre, EYE)), 1.0 / span);
                    if span > MIN_LENGTH && dot(*v, toward) < MAX_PULL_SPEED {
                        *v = add(*v, scale(toward, PULL * DT));
                    }
                    rope = rope.min(span.max(MIN_LENGTH));
                }
                self.rope[lane] = Some(rope);
                Some(Rope {
                    length: rope,
                    attach,
                })
            }
            Phase::Free(walk) => {
                if self.grounded[lane] {
                    v[0] = walk[0];
                    v[2] = walk[1];
                }
                self.rope[lane] = None;
                None
            }
        };
        v[1] -= GRAVITY * DT;
        (scale(*v, DT), rope)
    }

    /// The velocity the move realized.
    fn moved(&mut self, lane: usize, translation: [f32; 3], grounded: bool) {
        self.velocity[lane] = scale(translation, 1.0 / DT);
        self.grounded[lane] = grounded;
    }
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "grapple",
        capabilities: "C4 C27",
        ticks: TICKS,
        #[cfg(feature = "engine")]
        engine: |storage| Box::new(engine::Sim::new(storage)),
        #[cfg(feature = "rapier")]
        rapier: || Box::new(rapier::Sim::new(true)),
        #[cfg(feature = "rapier")]
        rapier_raw: Some(|| Box::new(rapier::Sim::new(false))),
        #[cfg(feature = "rapier")]
        bare_gated: true,
        check,
    }
}

/// Per tick and lane: position, grounded.
const PER_LANE: usize = 2;

/// The rope `lane` held on each tick, replayed from where each tick started.
fn ropes(trace: &Lanes<'_>, lane: usize) -> Vec<Option<f32>> {
    let mut rope = None;
    (0..TICKS)
        .map(|tick| {
            let centre = match tick {
                0 => start(lane),
                _ => trace.position(tick as usize - 1, lane),
            };
            rope = match phase(lane, tick) {
                Phase::Hooked { reel } => {
                    let span = span(lane, centre);
                    let rope = rope.unwrap_or(span);
                    Some(if reel {
                        rope.min(span.max(MIN_LENGTH))
                    } else {
                        rope
                    })
                }
                Phase::Free(_) => None,
            };
            rope
        })
        .collect()
}

/// How far a capsule centred at `p` reaches into box `(c, h)` (positive: inside).
fn penetration(p: [f32; 3], (c, h): ([f32; 3], [f32; 3])) -> f32 {
    let gap = |k: usize, extra: f32| ((p[k] - c[k]).abs() - h[k] - extra).max(0.0);
    let (dx, dy, dz) = (gap(0, 0.0), gap(1, HALF), gap(2, 0.0));
    RADIUS - (dx * dx + dy * dy + dz * dz).sqrt()
}

fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    let e = Lanes::new(engine, TICKS, LANES, PER_LANE, 0)?;
    let r = Lanes::new(rapier, TICKS, LANES, PER_LANE, 0)?;
    let ticks = TICKS as usize;
    for lane in 0..LANES {
        let (e_ropes, r_ropes) = (ropes(&e, lane), ropes(&r, lane));
        for tick in 0..ticks {
            let (a, b) = (e.position(tick, lane), r.position(tick, lane));
            let tolerance = match phase(lane, tick as u32) {
                Phase::Free(walk) if walk != [0.0; 2] => WALK_TOLERANCE,
                _ => TOLERANCE,
            };
            if (0..3).any(|k| (a[k] - b[k]).abs() > tolerance) {
                return Err(format!(
                    "lane {lane} tick {tick}: engine {a:?}, Rapier {b:?}"
                ));
            }
            for (side, ropes, p) in [("engine", &e_ropes, a), ("Rapier", &r_ropes, b)] {
                if p[1] < REST - 1e-4 {
                    return Err(format!("{side} lane {lane} sank to {p:?} at tick {tick}"));
                }
                if boxes(lane).into_iter().any(|b| penetration(p, b) > 1e-3) {
                    return Err(format!(
                        "{side} lane {lane} entered a box at tick {tick}: {p:?}"
                    ));
                }
                if let Some(rope) = ropes[tick]
                    && span(lane, p) > rope + STRETCH
                {
                    return Err(format!(
                        "{side} lane {lane} tick {tick} stands {} from its anchor on a {rope} rope",
                        span(lane, p)
                    ));
                }
            }
        }
    }
    let last = ticks - 1;
    for (side, t) in [("engine", &e), ("Rapier", &r)] {
        // Dragged across the floor, up the ledge's face and over its edge before letting go,
        // then standing on the ledge, stopped by the pillar.
        let over = t.position(104, GRAPPLE);
        if over[1] < 3.0 + REST - 0.01 || over[2] < 3.0 - RADIUS {
            return Err(format!(
                "{side} grapple did not come over the ledge: {over:?}"
            ));
        }
        let up = (15..105).find(|&k| t.position(k, GRAPPLE)[1] > REST + 0.5);
        if up.is_none_or(|k| t.position(k, GRAPPLE)[2] < 3.0 - RADIUS - 0.01) {
            return Err(format!("{side} grapple rose before the ledge's face"));
        }
        let end = t.position(last, GRAPPLE);
        if (end[1] - (3.0 + REST)).abs() > 0.01
            || !t.grounded(last, GRAPPLE, 1)
            || end[2] > 6.5 - RADIUS + 0.01
        {
            return Err(format!(
                "{side} grapple does not stand on the ledge: {end:?}"
            ));
        }
        // The swing crosses under the anchor to its far side off the ground, and is let go
        // in mid-air.
        let far = (0..90)
            .map(|k| t.position(k, SWING)[2])
            .fold(f32::MIN, f32::max);
        let let_go = t.position(89, SWING);
        if far < 3.0 || let_go[1] < REST + 0.5 {
            return Err(format!(
                "{side} swing reached z {far}, let go at {let_go:?}"
            ));
        }
        // Released in mid-air, the character flies on and lands.
        let release = t.position(24, RELEASE);
        let peak = (25..ticks)
            .map(|k| t.position(k, RELEASE)[1])
            .fold(f32::MIN, f32::max);
        if release[1] < REST + 0.75 || peak < release[1] + 0.5 || !t.grounded(last, RELEASE, 1) {
            return Err(format!("{side} release let go at {release:?}"));
        }
        // Blocked: the pull holds the character against the pillar's face.
        let blocked = t.position(119, BLOCKED);
        if (blocked[2] - (3.5 - RADIUS - OFFSET)).abs() > 0.01 {
            return Err(format!("{side} blocked pull ended at {blocked:?}"));
        }
    }
    Ok(())
}

#[cfg(feature = "engine")]
mod engine {
    use super::*;
    use crate::kcc::engine::{Characters, capsules, fixed, v, world};
    use physics_engine::approximate::{BodyStorage, CharacterSettings, Shape, Tether};

    pub struct Sim {
        characters: Characters,
        consumer: Consumer,
    }
    impl Sim {
        pub fn new(storage: BodyStorage) -> Self {
            let mut statics = vec![fixed(1, Shape::HalfSpace, [0.0; 3])];
            for lane in 0..LANES {
                for (k, (c, h)) in boxes(lane).into_iter().enumerate() {
                    statics.push(fixed(
                        10 * (lane as u64 + 1) + k as u64,
                        Shape::Box(v(h)),
                        c,
                    ));
                }
            }
            let settings = CharacterSettings {
                offset: OFFSET,
                snap_to_ground: Some(SNAP),
                ..CharacterSettings::default()
            };
            let bodies = capsules(1_000, Shape::capsule(HALF, RADIUS), (0..LANES).map(start));
            Self {
                characters: Characters::new(world(storage, statics), bodies, settings),
                consumer: Consumer::new(),
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            let mut desired = [[0.0; 3]; LANES];
            for (lane, d) in desired.iter_mut().enumerate() {
                let id = self.characters.ids[lane];
                let centre = self.characters.position(id);
                let (motion, rope) = self.consumer.plan(lane, tick, centre);
                *d = motion;
                let world = &mut self.characters.world;
                match rope {
                    Some(Rope {
                        length,
                        attach: true,
                    }) => world
                        .tether_character(
                            id,
                            Tether {
                                anchor: v(anchor(lane)),
                                attachment: v(EYE),
                                length,
                                max_pull: MAX_PULL_SPEED * DT,
                            },
                        )
                        .unwrap(),
                    Some(Rope { length, .. }) => world.set_tether_length(id, length).unwrap(),
                    None => {
                        world.release_tether(id);
                    }
                }
            }
            self.characters.move_all(|lane| v(desired[lane]));
            for lane in 0..LANES {
                let result = self.characters.results[lane];
                let t = result.translation;
                self.consumer
                    .moved(lane, [t.0, t.1, t.2], result.ground_contact);
                trace
                    .0
                    .push(Obs::Position(self.characters.position(result.body)));
                trace.0.push(Obs::Grounded(result.ground_contact));
            }
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::*;
    use crate::kcc::rapier::{Characters, cuboid, halfspace, mirror_controller, v};
    use rapier3d::prelude::*;

    pub struct Sim {
        characters: Characters,
        consumer: Consumer,
    }
    impl Sim {
        pub fn new(rebuild: bool) -> Self {
            let mut statics = vec![halfspace()];
            statics.extend((0..LANES).flat_map(|lane| boxes(lane).into_iter().map(cuboid)));
            Self {
                characters: Characters::new(
                    statics,
                    (0..LANES).map(start),
                    Capsule::new_y(HALF, RADIUS),
                    mirror_controller(OFFSET, Some(SNAP), None),
                    rebuild,
                ),
                consumer: Consumer::new(),
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            let mut desired = [Vector::ZERO; LANES];
            for (lane, d) in desired.iter_mut().enumerate() {
                let centre = self.characters.position(lane);
                let (mut motion, rope) = self.consumer.plan(lane, tick, centre);
                if let Some(rope) = rope {
                    motion = add(motion, tether(lane, rope.length, centre, motion));
                }
                *d = v(motion);
            }
            self.characters.move_all(|lane| desired[lane]);
            for lane in 0..LANES {
                let movement = &self.characters.movements[lane];
                let (t, grounded) = (movement.translation, movement.grounded);
                self.consumer.moved(lane, [t.x, t.y, t.z], grounded);
                trace.0.push(Obs::Position(self.characters.position(lane)));
                trace.0.push(Obs::Grounded(grounded));
            }
        }
    }
}
