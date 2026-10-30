//! C6/C8 (ARPG): planar monsters steering through the dungeon of `static-queries`, some
//! placed into pillars (as when a door closes or a spawn overlaps). Rapier mirrors ARPG's
//! call pattern: a world rebuild per tick (#292), the planar `KinematicCharacterController`
//! (no snapping) per monster with the halfspace floor excluded, `set_translation` and
//! `detect_collisions` after each move.

use super::character_move::capsule_box_gap;
use super::static_queries::{BoxSpec, FLOOR, layout};
use crate::kcc::{DT, HALF, Lanes, OFFSET, RADIUS};
use crate::{Obs, Rng, Scenario, Trace};

const TICKS: u32 = 240;
/// Monster centre height: resting one offset above the floor.
const HEIGHT: f32 = HALF + RADIUS + OFFSET;
const MONSTERS: usize = 48;
/// Monsters placed into a pillar.
const OVERLAPPING: usize = 8;

/// Static boxes (floor first) and monster start positions.
fn scene() -> (Vec<BoxSpec>, Vec<[f32; 3]>) {
    let (walls, doors) = layout();
    let mut boxes = vec![FLOOR];
    boxes.extend(walls.iter().copied());
    boxes.extend(doors);
    // Pillars with free space around them, so leaving the overlap has a clear way out.
    let pillars: Vec<BoxSpec> = walls
        .iter()
        .copied()
        .filter(|(c, h)| {
            h[0] == 0.5
                && boxes[1..]
                    .iter()
                    .filter(|(o, _)| o != c)
                    .all(|b| capsule_box_gap([c[0], HEIGHT, c[2]], *b) > 1.5)
        })
        .collect();
    assert!(pillars.len() >= OVERLAPPING);
    let mut rng = Rng(0xa1b2_c3d4_e5f6_0718);
    let mut starts = Vec::new();
    for (c, _) in pillars.iter().take(OVERLAPPING) {
        // 0.25 into the pillar's side: penetrating by 0.55.
        starts.push([c[0] + 0.25, HEIGHT, c[2]]);
    }
    while starts.len() < MONSTERS {
        let p = [rng.range(-29.0, 29.0), HEIGHT, rng.range(-29.0, 29.0)];
        if boxes[1..].iter().all(|b| capsule_box_gap(p, *b) > 0.2) {
            starts.push(p);
        }
    }
    (boxes, starts)
}

/// Steering intent shared by both engines: a new heading every 30 ticks, 3 m/s.
fn desired(index: usize, tick: u32) -> [f32; 3] {
    let mut rng = Rng(0x51 + index as u64 * 7_919 + u64::from(tick / 30));
    let a = rng.range(0.0, core::f32::consts::TAU);
    [3.0 * a.cos() * DT, 0.0, 3.0 * a.sin() * DT]
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "arpg-planar",
        capabilities: "C2 C6 C8",
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
    let (boxes, _) = scene();
    Lanes::new(rapier, TICKS, MONSTERS, 2, 0)?;
    let engine = Lanes::new(engine, TICKS, MONSTERS, 2, 0)?;
    for tick in 0..TICKS as usize {
        for i in 0..MONSTERS {
            let (Obs::Position(p), Obs::Depenetration(d)) =
                (engine.obs(tick, i, 0), engine.obs(tick, i, 1))
            else {
                return Err("unexpected observation".into());
            };
            if p[1] != HEIGHT {
                return Err(format!("monster {i} left its plane at tick {tick}: {p:?}"));
            }
            let pushed = d != [0.0; 3];
            if pushed != (tick == 0 && i < OVERLAPPING) {
                return Err(format!("monster {i} depenetration {d:?} at tick {tick}"));
            }
            if d[1] != 0.0 {
                return Err(format!("monster {i} depenetrated off its plane: {d:?}"));
            }
            if tick % 20 == 0 || tick + 1 == TICKS as usize {
                for b in &boxes[1..] {
                    if capsule_box_gap(p, *b) < -1e-3 {
                        return Err(format!("monster {i} penetrates a box at tick {tick}"));
                    }
                }
            }
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
    }
    impl Sim {
        pub fn new(storage: BodyStorage) -> Self {
            let (boxes, starts) = scene();
            let statics = boxes.into_iter().enumerate().map(|(i, (c, h))| {
                if i == 0 {
                    // The consumers' ground is a halfspace through the floor box's top.
                    fixed(1, Shape::HalfSpace, [0.0; 3])
                } else {
                    fixed(1 + i as u64, Shape::Box(v(h)), c)
                }
            });
            let settings = CharacterSettings {
                offset: OFFSET,
                snap_to_ground: None,
                plane: Some(HEIGHT),
                ..CharacterSettings::default()
            };
            let bodies = capsules(10_000, Shape::capsule(HALF, RADIUS), starts);
            Self {
                characters: Characters::new(world(storage, statics), bodies, settings),
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            self.characters.move_all(|i| v(desired(i, tick)));
            for r in &self.characters.results {
                let d = r.depenetration;
                trace
                    .0
                    .push(Obs::Position(self.characters.position(r.body)));
                trace.0.push(Obs::Depenetration([d.0, d.1, d.2]));
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
    }

    impl Sim {
        /// Static boxes, then monsters in key order, then the halfspace floor.
        pub fn new(rebuild: bool) -> Self {
            let (boxes, starts) = scene();
            let statics: Vec<_> = boxes.into_iter().skip(1).map(cuboid).collect();
            let floor = statics.len();
            // ARPG inserts the floor last (its insertion changes how ties break).
            let mut characters = Characters::new(
                statics,
                starts,
                Capsule::new_y(HALF, RADIUS),
                mirror_controller(OFFSET, None, None),
                rebuild,
            )
            .with_trailing(vec![halfspace()]);
            // ARPG keeps monsters on the plane, the floor left out of their moves.
            characters.exclude = Some(floor);
            characters.planar = true;
            Self { characters }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            self.characters.move_all(|i| v(desired(i, tick)));
            for i in 0..MONSTERS {
                trace.0.push(Obs::Position(self.characters.position(i)));
                trace.0.push(Obs::Depenetration([0.0; 3]));
            }
        }
    }
}
