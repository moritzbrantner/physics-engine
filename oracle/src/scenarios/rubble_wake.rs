//! C26 (MMORPG persistent keyed debris lane, #524): a throw of debris settles into a pile on the
//! floor plane; the game puts each settled piece to sleep (Rapier's `RigidBody::sleep()`, the
//! engine's `World::set_sleeping`, the same rule as `debris`); a second throw lands on the pile
//! and the contacts wake it, after which it settles and sleeps again. The engine runs MMORPG's
//! exact-continuation configuration (`warm_start` off, `sleep_allowed` off, game-owned sleep);
//! Rapier's bodies are `can_sleep(false)` so only the game's rule sleeps them, in one retained
//! world, MMORPG's persistent lane. Every collider uses friction 0.8 and restitution 0.15.

use crate::{Obs, Rng, Scenario, Trace};

const TICKS: u32 = 390;
const DT: f32 = 1.0 / 30.0;
const GRAVITY: f32 = 9.0;
const FRICTION: f32 = 0.8;
const RESTITUTION: f32 = 0.15;
const DENSITY: f32 = 600.0;
const LINEAR_DAMPING: f32 = 0.2;
const ANGULAR_DAMPING: f32 = 0.8;
/// Pieces of the first throw and of the second.
const PILE: usize = 40;
const THROW: usize = 14;
/// The second throw's first tick.
const SECOND: u32 = 180;
/// The game's settle rule (as `debris`): from this age, slow enough to sleep.
const SETTLE_AFTER: u32 = 8;
const REST_SPEED: f32 = 0.18;
const REST_SPIN: f32 = 0.9;
/// Acceptance: at least this share of the final rest points lie within `REST_TOLERANCE` of
/// Rapier's or a quarter of Rapier's throw; every one within `REST_CAP`; no piece sinks into
/// the floor deeper than Rapier's deepest piece.
const REST_SHARE: f32 = 0.9;
const REST_TOLERANCE: f32 = 0.5;
const REST_CAP: f32 = 1.5;
/// Allowance on the Rapier comparison of how deep a piece sinks: the two engines' deepest contacts
/// differ a little between native and WASM.
const SLOP: f32 = 0.005;
/// Least share of the settled pile the throw must wake, on each side.
const WOKEN_SHARE: f32 = 0.15;

#[derive(Clone, Copy)]
struct Piece {
    position: [f32; 3],
    rotation: [f32; 4],
    velocity: [f32; 3],
    spin: [f32; 3],
    half: [f32; 3],
    tick: u32,
}

fn starts() -> Vec<Piece> {
    let mut rng = Rng(0x524_8EAD_1234_5678);
    let mut out = Vec::new();
    for k in 0..PILE + THROW {
        let second = k >= PILE;
        let half = match k % 3 {
            0 => [0.3, 0.07, 0.3],
            1 => [0.09, 0.35, 0.07],
            _ => [0.2, 0.08, 0.2],
        };
        let yaw = rng.range(-3.0, 3.0) * 0.5;
        let (s, c) = (yaw.sin(), yaw.cos());
        let (position, velocity, spin) = if second {
            (
                [
                    rng.range(-1.6, 1.6),
                    rng.range(0.9, 1.5),
                    rng.range(-1.6, 1.6),
                ],
                [
                    rng.range(-1.0, 1.0),
                    rng.range(-2.0, -1.0),
                    rng.range(-1.0, 1.0),
                ],
                [
                    rng.range(-2.0, 2.0),
                    rng.range(-2.0, 2.0),
                    rng.range(-2.0, 2.0),
                ],
            )
        } else {
            (
                [
                    rng.range(-1.6, 1.6),
                    rng.range(0.25, 0.75),
                    rng.range(-1.6, 1.6),
                ],
                [
                    rng.range(-1.0, 1.0),
                    rng.range(-1.0, 1.0),
                    rng.range(-1.0, 1.0),
                ],
                [
                    rng.range(-1.5, 1.5),
                    rng.range(-1.5, 1.5),
                    rng.range(-1.5, 1.5),
                ],
            )
        };
        out.push(Piece {
            position,
            rotation: [0.0, s, 0.0, c],
            velocity,
            spin,
            half,
            tick: if second {
                SECOND + (k - PILE) as u32
            } else {
                0
            },
        });
    }
    out
}

/// Whether a piece of `age` ticks is slow enough; the game sleeps the whole pile once every
/// live piece is (sleeping pieces beside awake ones, as a per-piece rule does, loses Rapier floor contacts in
/// this scene: pieces fall through).
fn settles(age: u32, velocity: [f32; 3], spin: [f32; 3]) -> bool {
    let length = |v: [f32; 3]| (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    age >= SETTLE_AFTER && length(velocity) < REST_SPEED && length(spin) < REST_SPIN
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "rubble-wake",
        capabilities: "C26",
        ticks: TICKS,
        #[cfg(feature = "engine")]
        engine: |storage| Box::new(engine::Sim::new(storage)),
        #[cfg(feature = "rapier")]
        rapier: || Box::new(rapier::Sim::new()),
        #[cfg(feature = "rapier")]
        rapier_raw: None,
        check,
    }
}

fn rotate(q: [f32; 4], v: [f32; 3]) -> [f32; 3] {
    let [x, y, z, w] = q;
    let u = [x, y, z];
    let cross = |a: [f32; 3], b: [f32; 3]| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    let t = cross(u, v).map(|c| 2.0 * c);
    let tt = cross(u, t);
    [
        v[0] + w * t[0] + tt[0],
        v[1] + w * t[1] + tt[1],
        v[2] + w * t[2] + tt[2],
    ]
}

/// A piece's deepest corner below the floor plane (positive when sunk).
fn sinking(p: [f32; 3], q: [f32; 4], half: [f32; 3]) -> f32 {
    let mut deepest: f32 = 0.0;
    for k in 0..8 {
        let sign = |bit: usize| if k & bit == 0 { -1.0 } else { 1.0 };
        let c = rotate(q, [sign(1) * half[0], sign(2) * half[1], sign(4) * half[2]]);
        deepest = deepest.max(-(p[1] + c[1]));
    }
    deepest
}

/// Each tick records the live pieces in throw order (position, rotation, asleep).
struct Reader<'a> {
    obs: &'a [Obs],
    starts: Vec<Piece>,
}
impl Reader<'_> {
    fn count(&self, tick: u32) -> usize {
        self.starts.iter().filter(|p| p.tick <= tick).count()
    }
    /// The trace position of tick `t`, before its first piece.
    fn offset(&self, t: u32) -> usize {
        (0..t).map(|t| 3 * self.count(t)).sum()
    }
    fn at(&self, t: u32, k: usize) -> ([f32; 3], [f32; 4], bool) {
        let o = self.offset(t) + 3 * k;
        match (self.obs[o], self.obs[o + 1], self.obs[o + 2]) {
            (Obs::Position(p), Obs::Rotation(q), Obs::Grounded(s)) => (p, q, s),
            other => panic!("unexpected observations at tick {t}: {other:?}"),
        }
    }
    fn lines(&self, t: u32) -> Vec<([f32; 3], [f32; 4], bool)> {
        (0..self.count(t)).map(|k| self.at(t, k)).collect()
    }
}

fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    let starts = starts();
    let expect: usize = (0..TICKS)
        .map(|t| 3 * starts.iter().filter(|p| p.tick <= t).count())
        .sum();
    if engine.0.len() != expect || rapier.0.len() != expect {
        return Err(format!(
            "trace lengths {} and {} (expected {expect})",
            engine.0.len(),
            rapier.0.len()
        ));
    }
    let e = Reader {
        obs: &engine.0,
        starts: starts.clone(),
    };
    let r = Reader {
        obs: &rapier.0,
        starts: starts.clone(),
    };
    // Never deeper into the floor than Rapier's deepest piece, plus the slop.
    let deepest = |reader: &Reader| {
        let mut worst = 0.0_f32;
        for t in 0..TICKS {
            for (k, s) in starts.iter().enumerate().take(reader.count(t)) {
                let (p, q, _) = reader.at(t, k);
                worst = worst.max(sinking(p, q, s.half));
            }
        }
        worst
    };
    let (sink, rapier_sink) = (deepest(&e), deepest(&r));
    if sink > rapier_sink + SLOP {
        return Err(format!(
            "sinks {sink:.4} m into the floor, deeper than Rapier's {rapier_sink:.4} m"
        ));
    }
    let sleep_tick = SECOND - 1;
    // The pile settled and the game slept it, before the second throw, on both sides.
    for (name, reader, share) in [("engine", &e, 0.9), ("rapier", &r, 0.9)] {
        let asleep = reader.lines(sleep_tick).iter().filter(|s| s.2).count();
        if (asleep as f32) < share * PILE as f32 {
            return Err(format!(
                "{name}: only {asleep} of {PILE} pile pieces asleep"
            ));
        }
    }
    // The throw woke part of the pile: a piece asleep before it, awake later on.
    let woken = |reader: &Reader| {
        let before = reader.lines(sleep_tick);
        (0..PILE)
            .filter(|&k| before[k].2 && (SECOND..TICKS).any(|t| !reader.at(t, k).2))
            .count()
    };
    let needed = (WOKEN_SHARE * PILE as f32).ceil() as usize;
    let (eng_woken, rap_woken) = (woken(&e), woken(&r));
    if eng_woken < needed || rap_woken < needed {
        return Err(format!(
            "the second throw woke {eng_woken} (engine) and {rap_woken} (rapier) of the pile, \
             need {needed}"
        ));
    }
    // The engine sleeps everything by the end, near Rapier's rest
    // points.
    let last = TICKS - 1;
    let mut near = 0;
    let n = starts.len();
    for (k, start) in starts.iter().enumerate() {
        let (p, _, asleep) = e.at(last, k);
        let (rp, _, _) = r.at(last, k);
        if !asleep {
            return Err(format!("piece {k} still awake at the end: {p:?}"));
        }
        let dist = |a: [f32; 3], b: [f32; 3]| {
            ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
        };
        let throw = dist(rp, start.position);
        let off = dist(p, rp);
        if off > REST_CAP {
            return Err(format!(
                "piece {k} rests {off:.3} m from Rapier's rest point (throw {throw:.3} m): \
                 {p:?} vs {rp:?}"
            ));
        }
        near += usize::from(off <= REST_TOLERANCE.max(0.25 * throw));
    }
    if (near as f32) < REST_SHARE * n as f32 {
        return Err(format!(
            "{near} of {n} pieces rest near Rapier's rest points"
        ));
    }
    Ok(())
}

#[cfg(feature = "engine")]
mod engine {
    use super::*;
    use physics_engine::{
        BodyId,
        approximate::{Body, Config, Quaternion, Shape, Vector, World},
    };

    use crate::kcc::engine::v;

    pub struct Sim {
        world: World,
        ids: Vec<Option<BodyId>>,
        ages: Vec<u32>,
        starts: Vec<Piece>,
    }

    impl Sim {
        pub fn new(storage: physics_engine::approximate::BodyStorage) -> Self {
            let mut world = World::new(Config {
                body_storage: storage,
                gravity: Vector(0.0, -GRAVITY, 0.0),
                // MMORPG's debris lane settings, without warm starting: the exact-continuation lane.
                substeps: 2,
                velocity_iterations: 4,
                warm_start: false,
                ..Config::default()
            })
            .unwrap();
            let mut floor = Body::keyed(1, Shape::HalfSpace, Vector::ZERO, 0.0);
            floor.friction = FRICTION;
            floor.restitution = RESTITUTION;
            world.add_body(floor).unwrap();
            let starts = starts();
            Self {
                world,
                ids: vec![None; starts.len()],
                ages: vec![0; starts.len()],
                starts,
            }
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            for (k, s) in self.starts.iter().enumerate() {
                if s.tick != tick {
                    continue;
                }
                let mass = DENSITY * 8.0 * s.half[0] * s.half[1] * s.half[2];
                let mut body =
                    Body::keyed(1000 + k as u64, Shape::Box(v(s.half)), v(s.position), mass);
                let [x, y, z, w] = s.rotation;
                body.orientation = Quaternion(x, y, z, w);
                body.velocity = v(s.velocity);
                body.angular_velocity = v(s.spin);
                body.friction = FRICTION;
                body.restitution = RESTITUTION;
                body.linear_damping = LINEAR_DAMPING;
                body.angular_damping = ANGULAR_DAMPING;
                body.sleep_allowed = false;
                self.ids[k] = Some(self.world.add_body(body).unwrap());
            }
            self.world.step(f64::from(DT)).unwrap();
            let live: Vec<usize> = (0..self.starts.len())
                .filter(|&k| self.ids[k].is_some())
                .collect();
            for &k in &live {
                self.ages[k] += 1;
            }
            let settled = live.iter().all(|&k| {
                let b = self.world.body(self.ids[k].unwrap()).unwrap();
                let (u, w) = (b.velocity, b.angular_velocity);
                b.is_sleeping() || settles(self.ages[k], [u.0, u.1, u.2], [w.0, w.1, w.2])
            });
            for &k in &live {
                let id = self.ids[k].unwrap();
                if settled {
                    // The whole pile has settled: the game puts it to sleep together.
                    self.world.set_sleeping(id, true).unwrap();
                }
                let body = self.world.body(id).unwrap();
                let (p, q) = (body.position, body.orientation);
                let asleep = body.is_sleeping();
                trace.0.push(Obs::Position([p.0, p.1, p.2]));
                trace.0.push(Obs::Rotation([q.0, q.1, q.2, q.3]));
                trace.0.push(Obs::Grounded(asleep));
            }
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::*;
    use rapier3d::prelude::*;

    use crate::kcc::rapier::v;

    pub struct Sim {
        world: PhysicsWorld,
        bodies: Vec<Option<RigidBodyHandle>>,
        ages: Vec<u32>,
        starts: Vec<Piece>,
    }

    impl Sim {
        pub fn new() -> Self {
            let mut world = PhysicsWorld::new();
            world.gravity = Vector::new(0.0, -GRAVITY, 0.0);
            world.integration_parameters.dt = DT;
            world.colliders.insert(
                ColliderBuilder::new(SharedShape::halfspace(Vector::Y))
                    .friction(FRICTION)
                    .restitution(RESTITUTION)
                    .build(),
            );
            let starts = starts();
            Self {
                world,
                bodies: vec![None; starts.len()],
                ages: vec![0; starts.len()],
                starts,
            }
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            for (k, s) in self.starts.iter().enumerate() {
                if s.tick != tick {
                    continue;
                }
                let [x, y, z, w] = s.rotation;
                let pose = Pose::from_parts(v(s.position), Rotation::from_xyzw(x, y, z, w));
                let body = self.world.bodies.insert(
                    RigidBodyBuilder::dynamic()
                        .pose(pose)
                        .linvel(v(s.velocity))
                        .angvel(v(s.spin))
                        .linear_damping(LINEAR_DAMPING)
                        .angular_damping(ANGULAR_DAMPING)
                        // Only the game's rule sleeps a piece.
                        .can_sleep(false)
                        .build(),
                );
                self.world.colliders.insert_with_parent(
                    ColliderBuilder::cuboid(s.half[0], s.half[1], s.half[2])
                        .density(DENSITY)
                        .friction(FRICTION)
                        .restitution(RESTITUTION)
                        .build(),
                    body,
                    &mut self.world.bodies,
                );
                self.bodies[k] = Some(body);
            }
            self.world.step();
            let live: Vec<usize> = (0..self.starts.len())
                .filter(|&k| self.bodies[k].is_some())
                .collect();
            for &k in &live {
                self.ages[k] += 1;
            }
            let settled = live.iter().all(|&k| {
                let b = &self.world.bodies[self.bodies[k].unwrap()];
                b.is_sleeping()
                    || settles(self.ages[k], b.linvel().to_array(), b.angvel().to_array())
            });
            for &k in &live {
                let body = self.world.bodies.get_mut(self.bodies[k].unwrap()).unwrap();
                if settled {
                    // The whole pile has settled: the game puts it to sleep together.
                    body.sleep();
                }
                let (p, q) = (body.translation(), *body.rotation());
                let asleep = body.is_sleeping();
                trace.0.push(Obs::Position([p.x, p.y, p.z]));
                trace.0.push(Obs::Rotation([q.x, q.y, q.z, q.w]));
                trace.0.push(Obs::Grounded(asleep));
            }
        }
    }
}
