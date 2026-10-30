//! C7/C16 (MMORPG): a crowd on an open plaza. 32 pairs of characters walk head-on into each
//! other and 16 more cross their paths; all moves of a tick form one batch. Rapier mirrors
//! MMORPG: a world rebuild per tick, `move_shape` per character in id order and
//! `detect_collisions` after each move (head-on Rapier characters block each other).

use crate::kcc::{DT, HALF, OFFSET, RADIUS, positions};
use crate::{Obs, Scenario, Trace};

const TICKS: u32 = 300;
const REST: f32 = HALF + RADIUS + OFFSET;
const PAIRS: usize = 32;
const CROSSERS: usize = 16;
const SPEED: f32 = 1.5;

/// Start position and walking direction (unit, horizontal) of every character, in id order.
fn characters() -> Vec<([f32; 3], [f32; 2])> {
    let mut out = Vec::new();
    for k in 0..PAIRS {
        let x = -31.0 + 2.0 * k as f32;
        out.push(([x, REST, -4.0], [0.0, 1.0]));
        out.push(([x, REST, 4.0], [0.0, -1.0]));
    }
    for k in 0..CROSSERS {
        let z = -7.5 + k as f32;
        out.push(([-40.0, REST, z], [1.0, 0.0]));
    }
    out
}

fn desired(direction: [f32; 2]) -> [f32; 3] {
    [direction[0] * SPEED * DT, 0.0, direction[1] * SPEED * DT]
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "crowd",
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
    let n = characters().len();
    positions(rapier, TICKS, n)?;
    let engine = positions(engine, TICKS, n)?;
    let at = |tick, i| engine.position(tick, i);
    for tick in (0..TICKS as usize).step_by(5) {
        for i in 0..n {
            let a = at(tick, i);
            if a[1] != REST {
                return Err(format!(
                    "character {i} left the ground plane at tick {tick}"
                ));
            }
            for j in i + 1..n {
                let b = at(tick, j);
                let d = ((a[0] - b[0]).powi(2) + (a[2] - b[2]).powi(2)).sqrt();
                if d < 2.0 * RADIUS - 1e-3 {
                    return Err(format!(
                        "characters {i} and {j} overlap by {} at tick {tick}",
                        2.0 * RADIUS - d
                    ));
                }
            }
        }
    }
    let last = TICKS as usize - 1;
    for k in 0..PAIRS {
        let (a, b) = (at(last, 2 * k), at(last, 2 * k + 1));
        if a[2] < 2.0 || b[2] > -2.0 {
            return Err(format!("head-on pair {k} did not pass: {a:?} {b:?}"));
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
        directions: Vec<[f32; 2]>,
    }
    impl Sim {
        pub fn new(storage: BodyStorage) -> Self {
            let settings = CharacterSettings {
                offset: OFFSET,
                plane: Some(REST),
                ..CharacterSettings::default()
            };
            let (starts, directions): (Vec<_>, Vec<_>) = characters().into_iter().unzip();
            let statics = [fixed(1, Shape::HalfSpace, [0.0; 3])];
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
            self.characters.move_all(|i| v(desired(directions[i])));
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
    use crate::kcc::rapier::{Characters, halfspace, mirror_controller, v};
    use rapier3d::prelude::*;

    pub struct Sim {
        characters: Characters,
        directions: Vec<[f32; 2]>,
    }

    impl Sim {
        pub fn new(rebuild: bool) -> Self {
            let (starts, directions): (Vec<_>, Vec<_>) = characters().into_iter().unzip();
            let mut characters = Characters::new(
                vec![halfspace()],
                starts,
                Capsule::new_y(HALF, RADIUS),
                mirror_controller(OFFSET, None, None),
                rebuild,
            );
            // MMORPG keeps the crowd on its plane, the floor left out of its moves.
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
            self.characters.move_all(|i| v(desired(directions[i])));
            for i in 0..self.directions.len() {
                trace.0.push(Obs::Position(self.characters.position(i)));
            }
        }
    }
}
