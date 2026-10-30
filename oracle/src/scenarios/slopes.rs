//! C4/C5 (MMORPG ramps, #59): characters walking under gravity into wedge ramps on a
//! halfspace ground. Ramps up to `max_slope` (45°) are climbed; steeper ones are walls, and the
//! character stays at their foot instead of climbing and sliding back. Rapier mirrors MMORPG:
//! wedges as convex hulls, a world rebuild per tick, `move_shape` per character in id order.

use crate::kcc::{DT, GRAVITY, HALF, OFFSET, RADIUS, SNAP, positions};
use crate::{Obs, Scenario, Trace};

const TICKS: u32 = 240;
const REST: f32 = HALF + RADIUS + OFFSET;
const SPEED: f32 = 2.0;
const LANES: usize = 24;
/// Every ramp's foot (its +X end); ramps rise toward -X.
const FOOT_X: f32 = 10.0;
const HEIGHT: f32 = 1.5;
/// Positions of the two engines' steep-lane characters may differ by Rapier's normal nudge.
const RAPIER_TOLERANCE: f32 = 0.02;

/// Ramp angle (radians) of each lane: 12 climbable (15°–42°), 12 too steep (50°–72°).
fn angle(lane: usize) -> f32 {
    let deg = if lane < 12 {
        15.0 + 2.5 * lane as f32
    } else {
        50.0 + 2.0 * (lane - 12) as f32
    };
    deg.to_radians()
}

/// Centre and half extents of lane `lane`'s wedge.
fn ramp(lane: usize) -> ([f32; 3], [f32; 3]) {
    let hx = HEIGHT / angle(lane).tan() / 2.0;
    let z = -30.0 + 2.5 * lane as f32;
    ([FOOT_X - hx, HEIGHT / 2.0, z], [hx, HEIGHT / 2.0, 0.8])
}

fn start(lane: usize) -> [f32; 3] {
    [FOOT_X + 1.0, REST, ramp(lane).0[2]]
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "slopes",
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
        let (c, h) = ramp(lane);
        let z = start(lane)[2];
        for tick in 0..TICKS as usize {
            let p = engine.position(tick, lane);
            if p[2] != z || !p.iter().all(|a| a.is_finite()) {
                return Err(format!("lane {lane} left its line at tick {tick}: {p:?}"));
            }
            // Never inside the floor's offset, even landing at a shallow angle (#349).
            if p[1] < REST - 1e-5 {
                return Err(format!("lane {lane} sank to {} at tick {tick}", p[1]));
            }
            if lane >= 12 && p[1] > REST + 1e-4 {
                return Err(format!(
                    "lane {lane} climbed the too-steep ramp to {} at tick {tick}",
                    p[1]
                ));
            }
        }
        let (e, r) = (engine.position(last, lane), rapier.position(last, lane));
        if lane < 12 {
            // Both walked up and over the top.
            let top = c[0] - h[0];
            if e[0] > top || r[0] > top {
                return Err(format!(
                    "lane {lane} did not cross the ramp: engine {e:?}, rapier {r:?}"
                ));
            }
        } else if (0..3).any(|a| (e[a] - r[a]).abs() > RAPIER_TOLERANCE) {
            return Err(format!(
                "lane {lane} rests apart at the steep foot: engine {e:?}, rapier {r:?}"
            ));
        }
    }
    Ok(())
}

#[cfg(feature = "engine")]
mod engine {
    use super::*;
    use crate::kcc::engine::{Characters, capsules, fixed, v, world};
    use physics_engine::approximate::{BodyStorage, CharacterSettings, Shape, Vector};

    pub struct Sim {
        characters: Characters,
        vy: Vec<f32>,
    }
    impl Sim {
        pub fn new(storage: BodyStorage) -> Self {
            let statics = std::iter::once(fixed(1, Shape::HalfSpace, [0.0; 3])).chain(
                (0..LANES).map(|lane| {
                    let (c, h) = ramp(lane);
                    fixed(2 + lane as u64, Shape::wedge(v(h)), c)
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
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, _tick: u32, trace: &mut Trace) {
            let vy = &mut self.vy;
            self.characters.move_all(|lane| {
                vy[lane] -= GRAVITY * DT;
                Vector(-SPEED * DT, vy[lane] * DT, 0.0)
            });
            for (lane, r) in self.characters.results.iter().enumerate() {
                if r.grounded {
                    self.vy[lane] = 0.0;
                }
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

    /// The wedge as MMORPG builds it: the box's bottom rectangle and its top edge at -X.
    fn wedge(h: [f32; 3]) -> SharedShape {
        let mut points = Vec::with_capacity(6);
        for sx in [-1.0, 1.0] {
            for sz in [-1.0, 1.0] {
                points.push(Vector::new(sx * h[0], -h[1], sz * h[2]));
                if sx < 0.0 {
                    points.push(Vector::new(sx * h[0], h[1], sz * h[2]));
                }
            }
        }
        SharedShape::convex_hull(&points).expect("solid wedge")
    }

    pub struct Sim {
        characters: Characters,
        vy: Vec<f32>,
    }

    impl Sim {
        pub fn new(rebuild: bool) -> Self {
            let mut statics = vec![halfspace()];
            statics.extend((0..LANES).map(|lane| {
                let (c, h) = ramp(lane);
                ColliderBuilder::new(wedge(h)).translation(v(c))
            }));
            Self {
                characters: Characters::new(
                    statics,
                    (0..LANES).map(start),
                    Capsule::new_y(HALF, RADIUS),
                    mirror_controller(OFFSET, Some(SNAP), None),
                    rebuild,
                ),
                vy: vec![0.0; LANES],
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, _tick: u32, trace: &mut Trace) {
            let vy = &mut self.vy;
            self.characters.move_all(|lane| {
                vy[lane] -= GRAVITY * DT;
                Vector::new(-SPEED * DT, vy[lane] * DT, 0.0)
            });
            for (lane, movement) in self.characters.movements.iter().enumerate() {
                if movement.grounded {
                    self.vy[lane] = 0.0;
                }
                trace.0.push(Obs::Position(self.characters.position(lane)));
            }
        }
    }
}
