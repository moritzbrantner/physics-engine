//! C1/C4/C5 (MMORPG moving platforms, #56/#352): 24 static platforms move on a schedule (up,
//! sideways, down, back) and carry the character standing on each. Both sides use MMORPG's
//! adapter pattern: a character found on a platform by a downward cast filtered to that
//! platform gets the platform's displacement added to its move; a sinking platform moves
//! before the characters and a rising one after them. Rapier rebuilds the world per tick as
//! MMORPG does and moves the fixed collider in place; the engine moves the platform with
//! `set_static_position`.

use crate::kcc::{DT, GRAVITY, HALF, Lanes, OFFSET, RADIUS, SNAP};
use crate::{Obs, Scenario, Trace};

const TICKS: u32 = 240;
/// Rapier's ground prediction beyond `offset`.
const PREDICT: f32 = OFFSET + 0.05;
const FOOT: f32 = HALF + RADIUS;
const LANES: usize = 24;
const PLATFORM_HALF: [f32; 3] = [1.0, 0.25, 1.0];
const STEP: f32 = 0.01;

fn platform_start(lane: usize) -> [f32; 3] {
    [
        -30.0 + 5.0 * (lane % 12) as f32,
        0.5,
        -5.0 + 10.0 * (lane / 12) as f32,
    ]
}

fn rider_start(lane: usize) -> [f32; 3] {
    let p = platform_start(lane);
    [p[0], p[1] + PLATFORM_HALF[1] + FOOT + OFFSET, p[2]]
}

/// Platform `lane`'s displacement this tick: up, +X, down and -X for 30 ticks each, shifted
/// by lane.
fn displacement(lane: usize, tick: u32) -> [f32; 3] {
    match ((tick as usize + lane * 7) / 30) % 4 {
        0 => [0.0, STEP, 0.0],
        1 => [STEP, 0.0, 0.0],
        2 => [0.0, -STEP, 0.0],
        _ => [-STEP, 0.0, 0.0],
    }
}

fn add(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "platforms",
        capabilities: "C1 C4 C5",
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

/// Per tick and lane: the platform's position, then the rider's.
fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    let engine = Lanes::new(engine, TICKS, LANES, 2, 0)?;
    let rapier = Lanes::new(rapier, TICKS, LANES, 2, 0)?;
    let at = |t: Lanes, tick, lane| match (t.obs(tick, lane, 0), t.obs(tick, lane, 1)) {
        (Obs::Position(p), Obs::Position(r)) => (p, r),
        _ => ([f32::NAN; 3], [f32::NAN; 3]),
    };
    for tick in 0..TICKS as usize {
        for lane in 0..LANES {
            let (p, r) = at(engine, tick, lane);
            let (_, rr) = at(rapier, tick, lane);
            // Carried exactly: centred over the platform, resting on its top.
            let top = p[1] + PLATFORM_HALF[1] + FOOT + OFFSET;
            if (r[0] - p[0]).abs() > 1e-3 || r[2] != p[2] || (r[1] - top).abs() > 2e-3 {
                return Err(format!(
                    "lane {lane} rider left its platform at tick {tick}: {r:?} on {p:?}"
                ));
            }
            if (0..3).any(|a| (r[a] - rr[a]).abs() > 0.01) {
                return Err(format!(
                    "lane {lane} at tick {tick}: engine {r:?}, rapier {rr:?}"
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
    use physics_engine::{
        BodyId,
        approximate::{
            BodyStorage, CharacterSettings, QueryFilter, QueryHit, QueryPose, Shape, ShapeCast,
            Vector,
        },
    };

    pub struct Sim {
        characters: Characters,
        platforms: Vec<BodyId>,
        vy: Vec<f32>,
        carry: Vec<Option<[f32; 3]>>,
        hits: Vec<QueryHit>,
    }
    impl Sim {
        pub fn new(storage: BodyStorage) -> Self {
            let mut world = world(storage, [fixed(1, Shape::HalfSpace, [0.0; 3])]);
            let platforms = (0..LANES)
                .map(|lane| {
                    let shape = Shape::Box(v(PLATFORM_HALF));
                    let body = fixed(10 + lane as u64, shape, platform_start(lane));
                    world.add_body(body).unwrap()
                })
                .collect();
            let settings = CharacterSettings {
                offset: OFFSET,
                snap_to_ground: Some(SNAP),
                ..CharacterSettings::default()
            };
            let bodies = capsules(
                1_000,
                Shape::capsule(HALF, RADIUS),
                (0..LANES).map(rider_start),
            );
            Self {
                characters: Characters::new(world, bodies, settings),
                platforms,
                vy: vec![0.0; LANES],
                carry: vec![None; LANES],
                hits: Vec::new(),
            }
        }

        /// Whether `rider` stands on `platform`: walkable top within the ground prediction.
        fn stands_on(&mut self, rider: BodyId, platform: BodyId) -> bool {
            let reach = PREDICT + OFFSET;
            let world = &self.characters.world;
            let p = world.body(rider).unwrap().position;
            let cast = ShapeCast::new(
                QueryPose::new(Shape::capsule(HALF, RADIUS), p),
                Vector(0.0, -reach, 0.0),
            );
            let filter = QueryFilter {
                exclude: Some(rider),
                ..QueryFilter::default()
            };
            world.cast_shape(cast, filter, &mut self.hits).unwrap();
            let min_up = core::f32::consts::FRAC_PI_4.cos();
            self.hits.iter().any(|h| {
                h.body == platform && h.normal.1 >= min_up && h.fraction * reach <= PREDICT
            })
        }

        fn shift_platform(&mut self, lane: usize, d: [f32; 3]) {
            let world = &mut self.characters.world;
            let p = self.platforms[lane];
            let to = world.body(p).unwrap().position + v(d);
            world.set_static_position(p, to).unwrap();
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            for lane in 0..LANES {
                let d = displacement(lane, tick);
                let rider = self.characters.ids[lane];
                self.carry[lane] = (self.vy[lane] <= 0.0
                    && self.stands_on(rider, self.platforms[lane]))
                .then_some(d);
            }
            for lane in 0..LANES {
                let d = displacement(lane, tick);
                if d[1] <= 0.0 {
                    self.shift_platform(lane, d);
                }
            }
            let (carry, vy) = (&self.carry, &mut self.vy);
            self.characters.move_all(|lane| {
                if carry[lane].is_none() {
                    vy[lane] -= GRAVITY * DT;
                }
                v(add(
                    carry[lane].unwrap_or([0.0; 3]),
                    [0.0, vy[lane] * DT, 0.0],
                ))
            });
            for lane in 0..LANES {
                if self.characters.results[lane].grounded {
                    self.vy[lane] = 0.0;
                }
                let d = displacement(lane, tick);
                if d[1] > 0.0 {
                    self.shift_platform(lane, d);
                }
            }
            for lane in 0..LANES {
                let c = &self.characters;
                trace
                    .0
                    .push(Obs::Position(c.position(self.platforms[lane])));
                trace.0.push(Obs::Position(c.position(c.ids[lane])));
            }
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::*;
    use crate::kcc::rapier::{Characters, cuboid, halfspace, mirror_controller, v};
    use rapier3d::parry::query::ShapeCastOptions;
    use rapier3d::prelude::*;

    pub struct Sim {
        /// Statics: the ground, then one platform per lane.
        characters: Characters,
        vy: Vec<f32>,
        carry: Vec<Option<[f32; 3]>>,
    }

    impl Sim {
        pub fn new(rebuild: bool) -> Self {
            let mut statics = vec![halfspace()];
            statics.extend((0..LANES).map(|l| cuboid((platform_start(l), PLATFORM_HALF))));
            Self {
                characters: Characters::new(
                    statics,
                    (0..LANES).map(rider_start),
                    Capsule::new_y(HALF, RADIUS),
                    mirror_controller(OFFSET, Some(SNAP), None),
                    rebuild,
                ),
                vy: vec![0.0; LANES],
                carry: vec![None; LANES],
            }
        }

        fn platform(&self, lane: usize) -> Vector {
            self.characters.statics[1 + lane].position.translation
        }

        fn shift_platform(&mut self, lane: usize, d: [f32; 3]) {
            let c = &mut self.characters;
            let to = c.statics[1 + lane].position.translation + v(d);
            c.statics[1 + lane].position.translation = to;
            c.world.colliders[c.static_handles[1 + lane]].set_translation(to);
            c.world.detect_collisions(&(), &());
        }

        fn stands_on(&self, lane: usize) -> bool {
            let c = &self.characters;
            let platform = c.static_handles[1 + lane];
            let predicate = |handle: ColliderHandle, _: &Collider| handle == platform;
            c.queries(QueryFilter::default().predicate(&predicate))
                .cast_shape(
                    &Pose::from_translation(c.positions[lane]),
                    -Vector::Y,
                    &c.shape,
                    ShapeCastOptions {
                        target_distance: 0.0,
                        stop_at_penetration: false,
                        max_time_of_impact: PREDICT + OFFSET,
                        compute_impact_geometry_on_penetration: true,
                    },
                )
                .is_some_and(|(_, hit)| {
                    hit.time_of_impact <= PREDICT
                        && hit.normal1.dot(Vector::Y) >= core::f32::consts::FRAC_PI_4.cos()
                })
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            self.characters.start_tick();
            for lane in 0..LANES {
                let d = displacement(lane, tick);
                self.carry[lane] = (self.vy[lane] <= 0.0 && self.stands_on(lane)).then_some(d);
            }
            for lane in 0..LANES {
                let d = displacement(lane, tick);
                if d[1] <= 0.0 {
                    self.shift_platform(lane, d);
                }
            }
            let c = &mut self.characters;
            for lane in 0..LANES {
                let carry = self.carry[lane].unwrap_or([0.0; 3]);
                if self.carry[lane].is_none() {
                    self.vy[lane] -= GRAVITY * DT;
                }
                let desired = v(add(carry, [0.0, self.vy[lane] * DT, 0.0]));
                let movement = c.move_shape(lane, desired);
                if movement.grounded {
                    self.vy[lane] = 0.0;
                }
                let to = c.end(lane, &movement);
                c.commit(lane, to);
            }
            for lane in 0..LANES {
                let d = displacement(lane, tick);
                if d[1] > 0.0 {
                    self.shift_platform(lane, d);
                }
            }
            for lane in 0..LANES {
                let p = self.platform(lane);
                trace.0.push(Obs::Position([p.x, p.y, p.z]));
                trace.0.push(Obs::Position(self.characters.position(lane)));
            }
        }
    }
}
