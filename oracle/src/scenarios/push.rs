//! C16 (MMORPG): bounded pushing. 24 lanes each hold a pusher walking towards a wall and a
//! resting character in its way; the pusher pushes it up to the wall (`max_push` = one step per
//! tick), then sidesteps
//! around it. Rapier mirrors MMORPG: a world rebuild per tick, `move_shape` per
//! character in id order (Rapier characters never push: the pusher sidesteps or blocks).

use crate::kcc::{DT, HALF, OFFSET, RADIUS, positions};
use crate::{Obs, Scenario, Trace};

const TICKS: u32 = 240;
const REST: f32 = HALF + RADIUS + OFFSET;
const LANES: usize = 24;
const SPEED: f32 = 1.5;
/// Face of the wall the lanes run into, and the wall's half extents and centre.
const WALL_X: f32 = 4.0;
const WALL_HALF: [f32; 3] = [0.25, 2.0, 30.0];
const WALL: [f32; 3] = [WALL_X + 0.25, 2.0, 0.0];
const GAP: f32 = 2.0 * RADIUS + OFFSET;

/// Start position and desired motion per tick of every character, in id order: pushers on
/// even ids, the characters they push on odd ids.
fn characters() -> Vec<([f32; 3], [f32; 3])> {
    let mut out = Vec::new();
    for k in 0..LANES {
        let z = -23.0 + 2.0 * k as f32;
        out.push(([-2.0, REST, z], [SPEED * DT, 0.0, 0.0]));
        out.push(([-2.0 + GAP, REST, z], [0.0; 3]));
    }
    out
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "push",
        capabilities: "C2 C7 C16",
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

fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    let chars = characters();
    let n = chars.len();
    positions(rapier, TICKS, n)?;
    let engine = positions(engine, TICKS, n)?;
    let at = |tick, i| engine.position(tick, i);
    let step = SPEED * DT;
    for tick in 0..TICKS as usize {
        for k in 0..LANES {
            let (a, b) = (at(tick, 2 * k), at(tick, 2 * k + 1));
            let z = chars[2 * k].0[2];
            // The pushed character stays in its lane; once it is pinned at the wall the
            // pusher sidesteps around it (head-on rule).
            if a[1] != REST || b[1] != REST || b[2] != z {
                return Err(format!(
                    "lane {k} left its line at tick {tick}: {a:?} {b:?}"
                ));
            }
            if ((b[0] - a[0]).powi(2) + (b[2] - a[2]).powi(2)).sqrt() < GAP - 1e-4 {
                return Err(format!("lane {k} overlaps at tick {tick}: {a:?} {b:?}"));
            }
            if b[0] + RADIUS + OFFSET > WALL_X + 1e-4 {
                return Err(format!(
                    "lane {k} pushed into the wall at tick {tick}: {b:?}"
                ));
            }
            let before = if tick == 0 {
                chars[2 * k + 1].0[0]
            } else {
                at(tick - 1, 2 * k + 1)[0]
            };
            if b[0] - before > step + 1e-4 || b[0] < before {
                return Err(format!("lane {k} pushed beyond the ceiling at tick {tick}"));
            }
        }
    }
    let last = TICKS as usize - 1;
    for k in 0..LANES {
        let (a, b) = (at(last, 2 * k), at(last, 2 * k + 1));
        if (b[0] + RADIUS + OFFSET - WALL_X).abs() > 1e-4 || a[2] == b[2] {
            return Err(format!("lane {k} did not push up to the wall: {a:?} {b:?}"));
        }
    }
    Ok(())
}

#[cfg(feature = "engine")]
mod engine {
    use super::*;
    use crate::kcc::engine::{Characters, capsules, fixed, v, world};
    use physics_engine::approximate::{BodyStorage, CharacterSettings, Shape};

    pub struct Sim {
        characters: Characters,
        directions: Vec<[f32; 3]>,
    }
    impl Sim {
        pub fn new(storage: BodyStorage) -> Self {
            let statics = [
                fixed(1, Shape::HalfSpace, [0.0; 3]),
                fixed(2, Shape::Box(v(WALL_HALF)), WALL),
            ];
            let settings = CharacterSettings {
                offset: OFFSET,
                plane: Some(REST),
                max_push: Some(SPEED * DT),
                ..CharacterSettings::default()
            };
            let (starts, directions): (Vec<_>, Vec<_>) = characters().into_iter().unzip();
            let bodies = capsules(1_000, Shape::capsule(HALF, RADIUS), starts);
            Self {
                characters: Characters::new(world(storage, statics), bodies, settings),
                directions,
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, _tick: u32, trace: &mut Trace) {
            let directions = &self.directions;
            self.characters.move_all(|i| v(directions[i]));
            for r in &self.characters.results {
                trace
                    .0
                    .push(Obs::Position(self.characters.position(r.body)));
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
        directions: Vec<[f32; 3]>,
    }

    impl Sim {
        pub fn new(rebuild: bool) -> Self {
            let (starts, directions): (Vec<_>, Vec<_>) = characters().into_iter().unzip();
            let mut characters = Characters::new(
                vec![halfspace(), cuboid((WALL, WALL_HALF))],
                starts,
                Capsule::new_y(HALF, RADIUS),
                mirror_controller(OFFSET, None, None),
                rebuild,
            );
            characters.exclude = Some(0);
            characters.planar = true;
            Self {
                characters,
                directions,
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, _tick: u32, trace: &mut Trace) {
            let directions = &self.directions;
            self.characters.move_all(|i| v(directions[i]));
            for i in 0..self.directions.len() {
                trace.0.push(Obs::Position(self.characters.position(i)));
            }
        }
    }
}
