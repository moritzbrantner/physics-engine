//! C4/C5 (ARPG stairs, #353): characters walking under gravity into stairs with Rapier's
//! autostep (0.25 m, 0.15 m free width, dynamic bodies excluded). Steps up to the autostep
//! height are climbed onto the landing; higher ones stop the character at the first riser.
//! Both sides choose autostep for each move by ARPG's walking rule: the previous move ended in
//! ground contact.
//! Rapier mirrors the consumer call pattern: a world rebuild per tick, `move_shape` per
//! character in id order.

use crate::kcc::{DT, GRAVITY, HALF, OFFSET, RADIUS, SNAP, positions};
use crate::{Obs, Scenario, Trace};

const TICKS: u32 = 360;
const REST: f32 = HALF + RADIUS + OFFSET;
const SPEED: f32 = 2.0;
const LANES: usize = 24;
/// Every staircase's first riser; stairs rise toward -X.
const FOOT_X: f32 = 10.0;
const STEPS: usize = 4;
const TREAD: f32 = 0.4;
const LANDING: f32 = 14.0;
const MAX_STEP: f32 = 0.25;
const MIN_WIDTH: f32 = 0.15;
/// Positions of the two engines may differ by Rapier's normal nudge.
const RAPIER_TOLERANCE: f32 = 0.02;
/// How far apart the two engines may stop at a too-high riser (the capsule rides its edge).
const RISER_TOLERANCE: f32 = 0.05;

/// Rise of each lane's steps: 12 climbed by autostep (0.10–0.20 m), 12 too high (0.27–0.49
/// m). Lower steps are not autostep: a capsule slides over their edge as over a walkable slope.
/// Near the autostep height a capsule climbs a step's rounded edge only a little faster than
/// gravity pulls it back, on Rapier as on the engine, so whether it gets up is fragile there.
fn rise(lane: usize) -> f32 {
    if lane < 12 {
        0.10 + 0.0091 * lane as f32
    } else {
        0.27 + 0.02 * (lane - 12) as f32
    }
}

fn lane_z(lane: usize) -> f32 {
    -30.0 + 2.5 * lane as f32
}

/// Centres and half extents of lane `lane`'s steps, the last one its landing.
fn steps(lane: usize) -> Vec<([f32; 3], [f32; 3])> {
    (0..=STEPS)
        .map(|k| {
            let top = rise(lane) * (k + 1) as f32;
            let half = if k == STEPS {
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
    [FOOT_X + 1.0, REST, lane_z(lane)]
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "stairs",
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

fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    let (engine, rapier) = (
        positions(engine, TICKS, LANES)?,
        positions(rapier, TICKS, LANES)?,
    );
    let last = TICKS as usize - 1;
    for lane in 0..LANES {
        let z = lane_z(lane);
        for tick in 0..TICKS as usize {
            let p = engine.position(tick, lane);
            if p[2] != z || !p.iter().all(|a| a.is_finite()) {
                return Err(format!("lane {lane} left its line at tick {tick}: {p:?}"));
            }
            // A too-high step is never climbed: the character may ride up its rounded edge a
            // little, as on Rapier, but never reaches its top.
            if lane >= 12 && p[1] > REST + rise(lane) - RADIUS / 2.0 {
                return Err(format!(
                    "lane {lane} stepped up a too-high step to {} at tick {tick}",
                    p[1]
                ));
            }
        }
        let (e, r) = (engine.position(last, lane), rapier.position(last, lane));
        if lane < 12 {
            // Both climbed onto the landing. Rises up to 0.2 m sweep across the 45° step
            // threshold, where Rapier's climb is chaotic, so only end states are compared here;
            // `stairs-pace` compares the pace tick by tick on ARPG's stairs.
            let top = rise(lane) * (STEPS + 1) as f32 + REST;
            let landing = FOOT_X - TREAD * STEPS as f32;
            for (side, p) in [("engine", e), ("rapier", r)] {
                if (p[1] - top).abs() > RAPIER_TOLERANCE || p[0] > landing {
                    return Err(format!(
                        "lane {lane}: {side} did not reach the landing: {p:?}"
                    ));
                }
            }
            if (e[1] - top).abs() > 1e-3 {
                return Err(format!("lane {lane} does not rest on the landing: {e:?}"));
            }
        } else {
            // Both stop at the first riser. A rounded capsule rides a little up a too-high
            // step's edge and slips back, on Rapier as on the engine, so heights differ by
            // phase.
            let top = REST + rise(lane) - RADIUS / 2.0;
            if (e[0] - r[0]).abs() > RISER_TOLERANCE || (e[2] - r[2]).abs() > RAPIER_TOLERANCE {
                return Err(format!(
                    "lane {lane} stops apart at the riser: engine {e:?}, rapier {r:?}"
                ));
            }
            if r[1] > top {
                return Err(format!(
                    "lane {lane}: rapier climbed a too-high step: {r:?}"
                ));
            }
        }
    }
    Ok(())
}

#[cfg(feature = "engine")]
mod engine {
    use super::*;
    use crate::kcc::engine::{Characters, capsules, fixed, v, world};
    use physics_engine::approximate::{
        Autostep, BodyStorage, CharacterMove, CharacterSettings, Shape, Vector,
    };

    pub struct Sim {
        characters: Characters,
        vy: Vec<f32>,
        /// ARPG's walking rule: the previous move ended in ground contact.
        on_ground: Vec<bool>,
    }
    impl Sim {
        pub fn new(storage: BodyStorage) -> Self {
            let statics = std::iter::once(fixed(1, Shape::HalfSpace, [0.0; 3])).chain(
                (0..LANES).flat_map(|lane| {
                    steps(lane).into_iter().enumerate().map(move |(k, (c, h))| {
                        let key = 2 + (lane * (STEPS + 1) + k) as u64;
                        fixed(key, Shape::Box(v(h)), c)
                    })
                }),
            );
            let settings = CharacterSettings {
                offset: OFFSET,
                snap_to_ground: Some(SNAP),
                ..CharacterSettings::default()
            };
            let bodies = capsules(1_000, Shape::capsule(HALF, RADIUS), (0..LANES).map(start));
            Self {
                characters: Characters::new(world(storage, statics), bodies, settings),
                vy: vec![0.0; LANES],
                on_ground: vec![true; LANES],
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, _tick: u32, trace: &mut Trace) {
            let (vy, on_ground) = (&mut self.vy, &self.on_ground);
            self.characters.move_each(|lane, body| {
                vy[lane] -= GRAVITY * DT;
                CharacterMove {
                    body,
                    desired: Vector(-SPEED * DT, vy[lane] * DT, 0.0),
                    exclude: None,
                    autostep: on_ground[lane].then_some(Autostep {
                        max_height: MAX_STEP,
                        min_width: MIN_WIDTH,
                    }),
                }
            });
            for (lane, r) in self.characters.results.iter().enumerate() {
                if r.grounded {
                    self.vy[lane] = 0.0;
                }
                self.on_ground[lane] = r.ground_contact;
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
    use crate::kcc::rapier::{Characters, cuboid, halfspace, mirror_controller, walkable};
    use rapier3d::control::KinematicCharacterController;
    use rapier3d::prelude::*;

    pub struct Sim {
        characters: Characters,
        /// ARPG's controllers: walking players step, jumps and falls do not. A player walks
        /// when Rapier found it grounded last tick (a step's rounded edge included).
        walking: KinematicCharacterController,
        on_ground: Vec<bool>,
        vy: Vec<f32>,
    }

    impl Sim {
        pub fn new(rebuild: bool) -> Self {
            let mut statics = vec![halfspace()];
            statics.extend((0..LANES).flat_map(steps).map(cuboid));
            Self {
                characters: Characters::new(
                    statics,
                    (0..LANES).map(start),
                    Capsule::new_y(HALF, RADIUS),
                    mirror_controller(OFFSET, Some(SNAP), None),
                    rebuild,
                ),
                walking: mirror_controller(OFFSET, Some(SNAP), Some((MAX_STEP, MIN_WIDTH))),
                on_ground: vec![true; LANES],
                vy: vec![0.0; LANES],
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, _tick: u32, trace: &mut Trace) {
            let c = &mut self.characters;
            c.start_tick();
            for lane in 0..LANES {
                self.vy[lane] -= GRAVITY * DT;
                let controller = if self.on_ground[lane] {
                    &self.walking
                } else {
                    &c.controller
                };
                let desired = Vector::new(-SPEED * DT, self.vy[lane] * DT, 0.0);
                let movement = c.move_with(lane, controller, desired);
                let to = c.end(lane, &movement);
                // The consumers' grounded: Rapier's, and walkable static support below.
                let support = movement.grounded && c.support(to).is_some_and(|(_, h)| walkable(&h));
                if support {
                    self.vy[lane] = 0.0;
                }
                self.on_ground[lane] = movement.grounded;
                c.commit(lane, to);
                trace.0.push(Obs::Position([to.x, to.y, to.z]));
            }
        }
    }
}
