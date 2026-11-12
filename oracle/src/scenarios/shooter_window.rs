//! A real shooter physics window (#558, moenarch-games/shooter#37): 2 s of a scripted deathmatch
//! in the shooter's greybox arena (a 40 m hall of boxes and two ramps), six fighters at 60 Hz
//! sprinting, sliding, jumping, launched off a jump pad and thrown by a rocket blast.
//!
//! `tests/fixtures/shooter-window.json` comes from `shooter-replay record --seed 33 --ticks 720`
//! and `promote --target physics --from 580 --to 699`. It holds every fighter's per-tick move
//! request (the displacement the shooter's step handed the character controller: gravity, the
//! ground settle and the slide around other fighters included) and the position and grounded
//! state physics-engine gave it. Both sides replay the requests, so no shooter movement rule is
//! reimplemented here:
//!
//! - the engine side is the shooter's adapter: centimetres (`length_unit` 100), a 0.25 cm
//!   offset, step-up 35/20 on every move, one `move_characters` batch per tick, capsules lifted
//!   by the offset. It must replay the recorded trajectory (the regression baseline);
//! - the Rapier side is the shooter's oracle backend: metres, a world rebuilt every tick, the
//!   consumers' upright queries, `move_shape` per fighter in slot order (none for a zero move).
//!
//! A fighter without a body (dead) is parked far outside the hall; one that respawns is placed
//! at its recorded position on that tick instead of moving.

use std::sync::OnceLock;

use crate::kcc::{Lanes, OFFSET, SNAP};
use crate::{Obs, Scenario, Trace};

const SLOTS: usize = 6;
/// Fixture rows; the first places the fighters, every later one is a tick.
const ROWS: usize = 120;
const TICKS: u32 = ROWS as u32 - 1;
/// Per tick and slot: position (metres), grounded.
const PER_LANE: usize = 2;
const CM: f32 = 0.01;
/// The shooter's contact offset in its centimetres (`OFFSET` in metres).
const OFFSET_CM: f32 = 0.25;
const STEP_HEIGHT_CM: f32 = 35.0;
const STEP_WIDTH_CM: f32 = 20.0;
/// Character box half extents (cm): an upright capsule of half segment 60, radius 30.
const HALF_SEGMENT_CM: f32 = 60.0;
const RADIUS_CM: f32 = 30.0;
/// The engine replays the shooter's recorded positions this closely (metres). Natively it
/// replays them exactly; no cross-target bit identity is claimed.
const REPLAY_TOLERANCE: f32 = 1.0e-5;
/// Engine and Rapier fighters stay this close (metres; measured 2.1 mm, the shooter's own A/B
/// allows 6 cm).
const PATH_TOLERANCE: f32 = 0.005;

#[derive(Clone, Copy)]
enum Piece {
    Block {
        centre: [f32; 3],
        half: [f32; 3],
    },
    /// A wedge in its box, its top rising toward `(x, z)`.
    Ramp {
        centre: [f32; 3],
        half: [f32; 3],
        rises: [i32; 2],
    },
}

#[derive(Clone, Copy)]
struct Fighter {
    /// The move this tick's step handed the controller (cm); `None` without a body.
    desired: Option<[f32; 3]>,
    /// Centre after the tick (cm).
    position: [f32; 3],
    grounded: bool,
}

struct Window {
    pieces: Vec<Piece>,
    rows: Vec<[Fighter; SLOTS]>,
}

fn window() -> &'static Window {
    static WINDOW: OnceLock<Window> = OnceLock::new();
    WINDOW.get_or_init(|| {
        let json: serde_json::Value =
            serde_json::from_str(include_str!("../../../tests/fixtures/shooter-window.json"))
                .unwrap();
        let vector = |v: &serde_json::Value| [0, 1, 2].map(|axis| v[axis].as_f64().unwrap() as f32);
        let pieces = json["arena"]["pieces"]
            .as_array()
            .unwrap()
            .iter()
            .map(|piece| {
                let (centre, half) = (vector(&piece["center"]), vector(&piece["half"]));
                match piece["kind"].as_str().unwrap() {
                    "block" => Piece::Block { centre, half },
                    _ => Piece::Ramp {
                        centre,
                        half,
                        rises: match piece["rises"].as_str().unwrap() {
                            "PosX" => [1, 0],
                            "NegX" => [-1, 0],
                            "PosZ" => [0, 1],
                            _ => [0, -1],
                        },
                    },
                }
            })
            .collect();
        let rows: Vec<[Fighter; SLOTS]> = json["ticks"]
            .as_array()
            .unwrap()
            .iter()
            .map(|tick| {
                let fighters = tick["fighters"].as_array().unwrap();
                std::array::from_fn(|slot| {
                    let f = &fighters[slot];
                    assert_eq!(f["slot"].as_u64(), Some(slot as u64));
                    Fighter {
                        desired: f["alive"]
                            .as_bool()
                            .unwrap()
                            .then(|| f["move"].as_array().map(|_| vector(&f["move"])))
                            .flatten(),
                        position: vector(&f["position"]),
                        grounded: f["grounded"].as_bool().unwrap(),
                    }
                })
            })
            .collect();
        assert_eq!(rows.len(), ROWS);
        assert_eq!(
            json["character_half_extents"],
            serde_json::json!([30, 90, 30])
        );
        assert_eq!(
            json["character"]["contact_offset"],
            serde_json::json!(OFFSET_CM)
        );
        Window { pieces, rows }
    })
}

/// What a slot does at `tick`: move by the recorded request, appear at its recorded position
/// (a respawn), or stay parked without a body.
#[derive(Clone, Copy, PartialEq)]
enum Step {
    Move([f32; 3]),
    Place([f32; 3]),
    Park,
}

fn step(tick: u32, slot: usize) -> Step {
    let rows = &window().rows;
    let (before, now) = (rows[tick as usize][slot], rows[tick as usize + 1][slot]);
    match (before.desired, now.desired) {
        (_, None) => Step::Park,
        (None, Some(_)) => Step::Place(now.position),
        (Some(_), Some(desired)) => Step::Move(desired),
    }
}

/// Where a slot without a body waits (cm): far outside the 40 m hall, in the air.
fn parked(slot: usize) -> [f32; 3] {
    [10_000.0 + 500.0 * slot as f32, 5_000.0, 0.0]
}

/// The slots' starting centres (cm).
fn starts() -> [[f32; 3]; SLOTS] {
    let first = &window().rows[0];
    std::array::from_fn(|slot| match first[slot].desired {
        Some(_) => first[slot].position,
        None => parked(slot),
    })
}

fn lifted(centre: [f32; 3]) -> [f32; 3] {
    [centre[0], centre[1] + OFFSET_CM, centre[2]]
}

fn metres(cm: [f32; 3]) -> [f32; 3] {
    cm.map(|axis| axis * CM)
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "shooter-window",
        capabilities: "C2 C4 C5 C7 C13",
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

fn distance(a: [f32; 3], b: [f32; 3]) -> f32 {
    (0..3).map(|i| (a[i] - b[i]).powi(2)).sum::<f32>().sqrt()
}

fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    let e = Lanes::new(engine, TICKS, SLOTS, PER_LANE, 0)?;
    let r = Lanes::new(rapier, TICKS, SLOTS, PER_LANE, 0)?;
    let rows = &window().rows;
    let (mut replay, mut path) = ((0.0f32, 0, 0), (0.0f32, 0, 0));
    let mut moved = 0;
    for tick in 0..TICKS as usize {
        for (slot, recorded) in rows[tick + 1].iter().enumerate() {
            if !matches!(step(tick as u32, slot), Step::Move(_)) {
                continue;
            }
            moved += 1;
            let at = e.position(tick, slot);
            let gap = distance(at, metres(recorded.position));
            if gap > replay.0 {
                replay = (gap, tick, slot);
            }
            if e.grounded(tick, slot, 1) != recorded.grounded {
                return Err(format!(
                    "tick {tick} slot {slot}: engine grounded {} where the shooter saw {}",
                    !recorded.grounded, recorded.grounded
                ));
            }
            let gap = distance(at, r.position(tick, slot));
            if gap > path.0 {
                path = (gap, tick, slot);
            }
            if e.grounded(tick, slot, 1) != r.grounded(tick, slot, 1) {
                return Err(format!(
                    "tick {tick} slot {slot}: engine grounded {} at {at:?}, Rapier {} at {:?}",
                    e.grounded(tick, slot, 1),
                    r.grounded(tick, slot, 1),
                    r.position(tick, slot)
                ));
            }
        }
    }
    if moved < (TICKS as usize) * 5 {
        return Err(format!("only {moved} fighter moves replayed"));
    }
    if replay.0 > REPLAY_TOLERANCE {
        return Err(format!(
            "engine leaves the shooter's recorded path by {:.6} m (tick {}, slot {})",
            replay.0, replay.1, replay.2
        ));
    }
    if path.0 > PATH_TOLERANCE {
        return Err(format!(
            "engine and Rapier diverge by {:.4} m (tick {}, slot {}): engine {:?} rapier {:?}",
            path.0,
            path.1,
            path.2,
            e.position(path.1, path.2),
            r.position(path.1, path.2)
        ));
    }
    Ok(())
}

#[cfg(feature = "engine")]
mod engine {
    use super::*;
    use physics_engine::BodyId;
    use physics_engine::approximate::{
        Autostep, Body, BodyStorage, CharacterMove, CharacterMoveResult, CharacterSettings, Config,
        Quaternion, Shape, Vector, World,
    };

    fn v(a: [f32; 3]) -> Vector {
        Vector(a[0], a[1], a[2])
    }

    /// The shooter's engine bodies: the floor (key 1), the pieces (keys from 2), fighters keyed
    /// from 1000 by slot.
    fn piece(key: u64, piece: Piece) -> Body {
        match piece {
            Piece::Block { centre, half } => Body::keyed(key, Shape::Box(v(half)), v(centre), 0.0),
            // The engine's wedge rises toward local -X, turned about Y toward the rise; along Z
            // its local X extent is the box's Z extent.
            Piece::Ramp {
                centre,
                half: [hx, hy, hz],
                rises,
            } => {
                let (half, turn) = match rises {
                    [-1, 0] => (Vector(hx, hy, hz), 0.0),
                    [1, 0] => (Vector(hx, hy, hz), core::f32::consts::PI),
                    [0, 1] => (Vector(hz, hy, hx), core::f32::consts::FRAC_PI_2),
                    _ => (Vector(hz, hy, hx), -core::f32::consts::FRAC_PI_2),
                };
                let mut wedge = Body::keyed(key, Shape::Wedge(half), v(centre), 0.0);
                let (sin, cos) = (turn / 2.0).sin_cos();
                wedge.orientation = Quaternion(0.0, sin, 0.0, cos);
                wedge
            }
        }
    }

    pub struct Sim {
        world: World,
        ids: [BodyId; SLOTS],
        autostep: Option<Autostep>,
        moves: Vec<CharacterMove>,
        results: Vec<CharacterMoveResult>,
        /// The slot of each move in the batch.
        slots: Vec<usize>,
        grounded: [bool; SLOTS],
    }

    impl Sim {
        pub fn new(storage: BodyStorage) -> Self {
            let mut world = World::new(Config {
                body_storage: storage,
                gravity: Vector::ZERO,
                length_unit: 100.0,
                ..Config::default()
            })
            .unwrap();
            world
                .add_body(Body::keyed(1, Shape::HalfSpace, Vector::ZERO, 0.0))
                .unwrap();
            for (i, &p) in window().pieces.iter().enumerate() {
                world.add_body(piece(2 + i as u64, p)).unwrap();
            }
            let settings = CharacterSettings {
                offset: OFFSET_CM,
                ..world.character_settings()
            };
            let starts = starts();
            let ids = std::array::from_fn(|slot| {
                let mut body = Body::keyed(
                    1_000 + slot as u64,
                    Shape::capsule(HALF_SEGMENT_CM, RADIUS_CM),
                    v(lifted(starts[slot])),
                    1.0,
                );
                body.external = true;
                let id = world.add_body(body).unwrap();
                world.add_character(id, settings).unwrap();
                id
            });
            Self {
                world,
                ids,
                autostep: Some(Autostep {
                    max_height: STEP_HEIGHT_CM,
                    min_width: STEP_WIDTH_CM,
                }),
                moves: Vec::with_capacity(SLOTS),
                results: Vec::with_capacity(SLOTS),
                slots: Vec::with_capacity(SLOTS),
                grounded: [false; SLOTS],
            }
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            self.moves.clear();
            self.slots.clear();
            for slot in 0..SLOTS {
                match step(tick, slot) {
                    Step::Move(desired) => {
                        self.moves.push(CharacterMove {
                            body: self.ids[slot],
                            desired: v(desired),
                            exclude: None,
                            autostep: self.autostep,
                        });
                        self.slots.push(slot);
                    }
                    Step::Place(at) => {
                        self.world
                            .set_kinematic_position(self.ids[slot], v(lifted(at)))
                            .unwrap();
                        self.grounded[slot] = window().rows[tick as usize + 1][slot].grounded;
                    }
                    Step::Park => {
                        let at = v(lifted(parked(slot)));
                        if self.world.body(self.ids[slot]).unwrap().position != at {
                            self.world
                                .set_kinematic_position(self.ids[slot], at)
                                .unwrap();
                        }
                        self.grounded[slot] = false;
                    }
                }
            }
            self.world
                .move_characters(&self.moves, &mut self.results)
                .unwrap();
            for (result, &slot) in self.results.iter().zip(&self.slots) {
                self.grounded[slot] = result.grounded;
            }
            for slot in 0..SLOTS {
                let p = self.world.body(self.ids[slot]).unwrap().position;
                trace
                    .0
                    .push(Obs::Position(metres([p.0, p.1 - OFFSET_CM, p.2])));
                trace.0.push(Obs::Grounded(self.grounded[slot]));
            }
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::*;
    use crate::kcc::rapier::{Characters, cuboid, halfspace, mirror_controller, v, walkable};
    use crate::scenarios::upright_queries::UprightShapes;
    use rapier3d::parry::query::{DefaultQueryDispatcher, QueryDispatcher};
    use rapier3d::prelude::*;

    /// The shooter's ramp hull (metres): the box's bottom rectangle and its top edge at the
    /// high end.
    fn wedge(centre: [f32; 3], half: [f32; 3], rises: [i32; 2]) -> ColliderBuilder {
        let [hx, hy, hz] = metres(half);
        let mut points = Vec::with_capacity(6);
        for sx in [-1.0, 1.0] {
            for sz in [-1.0, 1.0] {
                points.push(Vector::new(sx * hx, -hy, sz * hz));
                let high = (rises[0] != 0 && sx == rises[0] as f32)
                    || (rises[1] != 0 && sz == rises[1] as f32);
                if high {
                    points.push(Vector::new(sx * hx, hy, sz * hz));
                }
            }
        }
        ColliderBuilder::new(SharedShape::convex_hull(&points).expect("solid wedge"))
            .translation(v(metres(centre)))
    }

    pub struct Sim {
        characters: Characters,
        grounded: [bool; SLOTS],
    }

    impl Sim {
        pub fn new(rebuild: bool) -> Self {
            let mut statics = vec![halfspace()];
            for &p in &window().pieces {
                statics.push(match p {
                    Piece::Block { centre, half } => cuboid((metres(centre), metres(half))),
                    Piece::Ramp {
                        centre,
                        half,
                        rises,
                    } => wedge(centre, half, rises),
                });
            }
            let mut characters = Characters::new(
                statics,
                starts().map(|start| metres(lifted(start))),
                Capsule::new_y(HALF_SEGMENT_CM * CM, RADIUS_CM * CM),
                mirror_controller(
                    OFFSET,
                    Some(SNAP),
                    Some((STEP_HEIGHT_CM * CM, STEP_WIDTH_CM * CM)),
                ),
                rebuild,
            );
            characters.dispatcher = Box::new(UprightShapes.chain(DefaultQueryDispatcher));
            Self {
                characters,
                grounded: [false; SLOTS],
            }
        }

        /// Walkable static ground within the support reach below `slot` (the shooter's Rapier
        /// grounded test).
        fn supported(&self, slot: usize) -> bool {
            let c = &self.characters;
            c.support(c.positions[slot])
                .is_some_and(|(_, hit)| walkable(&hit))
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            self.characters.start_tick();
            for slot in 0..SLOTS {
                match step(tick, slot) {
                    Step::Move(desired) => {
                        if desired != [0.0; 3] {
                            let c = &self.characters;
                            let movement = c.move_shape(slot, v(metres(desired)));
                            let to = c.end(slot, &movement);
                            self.characters.commit(slot, to);
                        }
                        self.grounded[slot] = self.supported(slot);
                    }
                    Step::Place(at) => {
                        self.characters.commit(slot, v(metres(lifted(at))));
                        self.grounded[slot] = window().rows[tick as usize + 1][slot].grounded;
                    }
                    Step::Park => {
                        let at = v(metres(lifted(parked(slot))));
                        if self.characters.positions[slot] != at {
                            self.characters.commit(slot, at);
                        }
                        self.grounded[slot] = false;
                    }
                }
            }
            for slot in 0..SLOTS {
                let p = self.characters.position(slot);
                trace.0.push(Obs::Position([p[0], p[1] - OFFSET, p[2]]));
                trace.0.push(Obs::Grounded(self.grounded[slot]));
            }
        }
    }
}
