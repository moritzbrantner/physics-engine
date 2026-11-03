//! C19 (Q33, mmorpg#300): contact enter/exit for physical traps and triggers. Twelve lanes on a
//! floor plane, two of each kind, each a fixed pressure plate (or none) and one dynamic body:
//!
//! - `Drop`: a crate falls onto the plate, rests and sleeps; at tick 150 it is shoved off and
//!   drops 0.1 m onto the floor.
//! - `Remove`: a crate rests on the plate; at tick 120 the plate is removed (Rapier:
//!   `remove_collider`, a `Stopped` flagged `REMOVED`) and the crate falls to the floor.
//! - `Spawn`: a crate is inserted above the plate at tick 60 and lands on it.
//! - `Sensor`: the plate is a sensor; the crate falls through it onto the floor. Rapier's
//!   sensor intersection events are logical-zone semantics, game-owned (D7): they are dropped
//!   from Rapier's trace and the engine reports no event for sensors.
//! - `Layer`: the plate's collision layer excludes the crate, which falls through it.
//! - `Boulder`: a boulder rolls along the floor into a kinematic capsule (a player), rides a
//!   little way up its rounded side, drops back onto the floor and rests against it.
//!
//! Crates are 0.5 m, density 600, rotations locked; 60 Hz, Rapier's defaults otherwise. Rapier
//! keeps one world with `ActiveEvents::COLLISION_EVENTS` on the dynamic colliders and reads
//! events through a `ChannelEventCollector`, as a game does; the engine reads
//! `Report::contact_events`. Both sides record each event with its tick and the pair's labels.

use crate::{Obs, Scenario, Trace};

const TICKS: u32 = 300;
const DT: f32 = 1.0 / 60.0;
const GRAVITY: f32 = 9.81;
const LANES: u32 = 12;
const HALF: f32 = 0.25;
const DENSITY: f32 = 600.0;
const MASS: f32 = DENSITY * 8.0 * HALF * HALF * HALF;
const PLATE_HALF: [f32; 3] = [0.6, 0.05, 0.6];
const BOULDER: f32 = 0.3;
const BOULDER_SPEED: f32 = 5.0;
const PLAYER_HALF: f32 = 0.5;
const PLAYER_RADIUS: f32 = 0.3;
const SPAWN_TICK: u32 = 60;
const REMOVE_TICK: u32 = 120;
const SHOVE_TICK: u32 = 150;
const SHOVE: f32 = 4.0;
/// Acceptance: every pair reports the same sequence of transitions as on Rapier, each within
/// this many ticks of Rapier's. Rapier detects contact at the start of a step within its 2 mm
/// prediction; the engine after the step's solve within twice its 5 mm slop.
const TICK_TOLERANCE: u32 = 3;

const FLOOR: u32 = 1;
fn plate_label(l: u32) -> u32 {
    100 + l
}
fn body_label(l: u32) -> u32 {
    200 + l
}
fn player_label(l: u32) -> u32 {
    300 + l
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum Lane {
    Drop,
    Remove,
    Spawn,
    Sensor,
    Layer,
    Boulder,
}
fn lane(l: u32) -> Lane {
    [
        Lane::Drop,
        Lane::Remove,
        Lane::Spawn,
        Lane::Sensor,
        Lane::Layer,
        Lane::Boulder,
    ][(l % 6) as usize]
}
fn z(l: u32) -> f32 {
    3.0 * l as f32
}
fn plate_centre(l: u32) -> [f32; 3] {
    [0.0, PLATE_HALF[1], z(l)]
}
/// Where the lane's crate starts: above the plate, a little higher in the second set of lanes.
fn crate_start(l: u32) -> [f32; 3] {
    let drop = match lane(l) {
        Lane::Remove => 0.001,
        _ => 0.3 + 0.4 * (l / 6) as f32,
    };
    [0.0, 2.0 * PLATE_HALF[1] + HALF + drop, z(l)]
}
fn boulder_start(l: u32) -> [f32; 3] {
    [-3.0, BOULDER, z(l)]
}
fn player_centre(l: u32) -> [f32; 3] {
    [0.0, PLAYER_HALF + PLAYER_RADIUS, z(l)]
}
fn has_plate(l: u32) -> bool {
    lane(l) != Lane::Boulder
}
/// Whether the lane's dynamic body exists from the start.
fn spawned_at_start(l: u32) -> bool {
    lane(l) != Lane::Spawn
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "contact-events",
        capabilities: "C19",
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

type Event = (u32, bool, [u32; 2]);
/// A pair and its transitions as (tick, started).
type PairLog = ([u32; 2], Vec<(u32, bool)>);

fn events(trace: &Trace) -> Result<Vec<Event>, String> {
    trace
        .0
        .iter()
        .map(|obs| match *obs {
            Obs::ContactEvent {
                tick,
                started,
                pair,
            } => Ok((tick, started, pair)),
            _ => Err(format!("unexpected observation {obs:?}")),
        })
        .collect()
}

/// Each pair's transitions: (pair, [(tick, started)]), pairs in label order.
fn by_pair(events: &[Event]) -> Vec<PairLog> {
    let mut out: Vec<PairLog> = Vec::new();
    let mut sorted = events.to_vec();
    sorted.sort_by_key(|&(tick, _, [a, b])| ([a.min(b), a.max(b)], tick));
    for (tick, started, [a, b]) in sorted {
        let pair = [a.min(b), a.max(b)];
        match out.last_mut() {
            Some((p, list)) if *p == pair => list.push((tick, started)),
            _ => out.push((pair, vec![(tick, started)])),
        }
    }
    out
}

fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    let engine = events(engine)?;
    let rapier = events(rapier)?;
    // The engine's own contract: per tick in pair order (labels are keys, so body order), the
    // pair's first body first.
    for w in engine.windows(2) {
        let ((t0, s0, p0), (t1, s1, p1)) = (w[0], w[1]);
        if t1 < t0 || (t0 == t1 && (p1, !s1) <= (p0, !s0)) {
            return Err(format!(
                "engine events out of order: {:?} then {:?}",
                w[0], w[1]
            ));
        }
    }
    if let Some(e) = engine.iter().find(|(_, _, [a, b])| a >= b) {
        return Err(format!("engine pair not in body order: {e:?}"));
    }
    for (pair, list) in by_pair(&engine) {
        if list
            .iter()
            .enumerate()
            .any(|(i, &(_, s))| s != (i % 2 == 0))
        {
            return Err(format!(
                "pair {pair:?} does not alternate Started/Stopped: {list:?}"
            ));
        }
        for l in 0..LANES {
            if matches!(lane(l), Lane::Sensor | Lane::Layer) && pair.contains(&plate_label(l)) {
                return Err(format!("lane {l} ({:?}) reported {pair:?}", lane(l)));
            }
        }
    }
    // Every lane's expected transitions happened.
    for l in 0..LANES {
        let (plate, body) = (plate_label(l), body_label(l));
        let expected: &[([u32; 2], &[bool])] = match lane(l) {
            Lane::Drop | Lane::Remove => {
                &[([FLOOR, body], &[true]), ([plate, body], &[true, false])]
            }
            Lane::Spawn => &[([plate, body], &[true])],
            Lane::Sensor | Lane::Layer => &[([FLOOR, body], &[true])],
            // The capsule's rounded side lifts the boulder off the floor for a few ticks.
            Lane::Boulder => &[
                ([FLOOR, body], &[true, false, true]),
                ([body, player_label(l)], &[true]),
            ],
        };
        for (pair, kinds) in expected {
            let got: Vec<bool> = by_pair(&engine)
                .into_iter()
                .find(|(p, _)| p == pair)
                .map(|(_, list)| list.into_iter().map(|(_, s)| s).collect())
                .unwrap_or_default();
            if got != *kinds {
                return Err(format!(
                    "lane {l} ({:?}) pair {pair:?}: engine reported {got:?}, expected {kinds:?}",
                    lane(l)
                ));
            }
        }
    }
    // Rapier parity: the same pairs, the same transitions, each within the tolerance.
    let (engine, rapier) = (by_pair(&engine), by_pair(&rapier));
    let pairs = |v: &[PairLog]| v.iter().map(|(p, _)| *p).collect::<Vec<_>>();
    if pairs(&engine) != pairs(&rapier) {
        return Err(format!(
            "pairs differ: engine {:?}, Rapier {:?}",
            pairs(&engine),
            pairs(&rapier)
        ));
    }
    for ((pair, e), (_, r)) in engine.iter().zip(&rapier) {
        let kinds = |v: &[(u32, bool)]| v.iter().map(|&(_, s)| s).collect::<Vec<_>>();
        if kinds(e) != kinds(r) {
            return Err(format!("pair {pair:?}: engine {e:?}, Rapier {r:?}"));
        }
        for (&(te, _), &(tr, _)) in e.iter().zip(r) {
            if te.abs_diff(tr) > TICK_TOLERANCE {
                return Err(format!(
                    "pair {pair:?}: engine at tick {te}, Rapier at tick {tr} ({e:?} vs {r:?})"
                ));
            }
        }
    }
    Ok(())
}

#[cfg(feature = "engine")]
mod engine {
    use super::*;
    use physics_engine::{
        BodyId, CollisionLayers3d,
        approximate::{Body, BodyStorage, Config, ContactEvent, Shape, Vector, World},
    };
    use std::collections::HashMap;

    use crate::kcc::engine::v;

    pub struct Sim {
        world: World,
        /// Labels by id; removed bodies keep theirs, as their last events name them.
        labels: HashMap<u64, u32>,
        lanes: Vec<(Option<BodyId>, Option<BodyId>)>,
    }

    fn crate_body(l: u32) -> Body {
        let mut body = Body::keyed(
            u64::from(body_label(l)),
            Shape::Box(Vector(HALF, HALF, HALF)),
            v(crate_start(l)),
            MASS,
        );
        body.rotation_locked = true;
        if lane(l) == Lane::Layer {
            body.layers = CollisionLayers3d::new(0b01, 0b01);
        }
        body
    }

    impl Sim {
        pub fn new(storage: BodyStorage) -> Self {
            let world = World::new(Config {
                body_storage: storage,
                gravity: Vector(0.0, -GRAVITY, 0.0),
                substeps: 2,
                velocity_iterations: 4,
                length_unit: 1.0,
                ..Config::default()
            })
            .unwrap();
            let mut sim = Self {
                world,
                labels: HashMap::new(),
                lanes: Vec::new(),
            };
            sim.add(Body::keyed(
                u64::from(FLOOR),
                Shape::HalfSpace,
                Vector::ZERO,
                0.0,
            ));
            for l in 0..LANES {
                let plate = has_plate(l).then(|| {
                    let mut plate = Body::keyed(
                        u64::from(plate_label(l)),
                        Shape::Box(v(PLATE_HALF)),
                        v(plate_centre(l)),
                        0.0,
                    );
                    plate.sensor = lane(l) == Lane::Sensor;
                    if lane(l) == Lane::Layer {
                        plate.layers = CollisionLayers3d::new(0b10, 0b10);
                    }
                    sim.add(plate)
                });
                let body = match lane(l) {
                    Lane::Boulder => {
                        let mut player = Body::keyed(
                            u64::from(player_label(l)),
                            Shape::capsule(PLAYER_HALF, PLAYER_RADIUS),
                            v(player_centre(l)),
                            0.0,
                        );
                        player.external = true;
                        sim.add(player);
                        let mass = DENSITY * 4.0 / 3.0 * std::f32::consts::PI * BOULDER.powi(3);
                        let mut boulder = Body::keyed(
                            u64::from(body_label(l)),
                            Shape::Sphere(BOULDER),
                            v(boulder_start(l)),
                            mass,
                        );
                        boulder.velocity = Vector(BOULDER_SPEED, 0.0, 0.0);
                        Some(sim.add(boulder))
                    }
                    _ if spawned_at_start(l) => Some(sim.add(crate_body(l))),
                    _ => None,
                };
                sim.lanes.push((plate, body));
            }
            sim
        }

        fn add(&mut self, body: Body) -> BodyId {
            let label = body.key.unwrap() as u32;
            let id = self.world.add_body(body).unwrap();
            self.labels.insert(id.to_bits(), label);
            id
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            for l in 0..LANES {
                match (lane(l), tick) {
                    (Lane::Spawn, SPAWN_TICK) => {
                        self.lanes[l as usize].1 = Some(self.add(crate_body(l)));
                    }
                    (Lane::Remove, REMOVE_TICK) => {
                        let plate = self.lanes[l as usize].0.take().unwrap();
                        self.world.remove_body(plate).unwrap();
                    }
                    (Lane::Drop, SHOVE_TICK) => {
                        let id = self.lanes[l as usize].1.unwrap();
                        self.world
                            .set_velocity(id, Vector(SHOVE, 0.0, 0.0))
                            .unwrap();
                    }
                    _ => {}
                }
            }
            let report = self.world.step(f64::from(DT)).unwrap();
            for event in report.contact_events {
                let (a, b) = event.bodies();
                trace.0.push(Obs::ContactEvent {
                    tick,
                    started: matches!(event, ContactEvent::Started(..)),
                    pair: [self.labels[&a.to_bits()], self.labels[&b.to_bits()]],
                });
            }
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::*;
    use rapier3d::prelude::*;
    use std::collections::HashMap;
    use std::sync::mpsc::{Receiver, channel};

    use crate::kcc::rapier::v;

    pub struct Sim {
        world: PhysicsWorld,
        events: ChannelEventCollector,
        collisions: Receiver<CollisionEvent>,
        /// Labels by collider; removed colliders keep theirs for their `REMOVED` events.
        labels: HashMap<ColliderHandle, u32>,
        plates: Vec<Option<ColliderHandle>>,
        bodies: Vec<Option<RigidBodyHandle>>,
    }

    impl Sim {
        pub fn new() -> Self {
            let mut world = PhysicsWorld::new();
            world.gravity = Vector::new(0.0, -GRAVITY, 0.0);
            world.integration_parameters.dt = DT;
            let (send, collisions) = channel();
            let (forces, _) = channel();
            let (tears, _) = channel();
            let mut sim = Self {
                world,
                events: ChannelEventCollector::new(send, forces, tears),
                collisions,
                labels: HashMap::new(),
                plates: Vec::new(),
                bodies: Vec::new(),
            };
            let floor = sim
                .world
                .colliders
                .insert(ColliderBuilder::new(SharedShape::halfspace(Vector::Y)).build());
            sim.labels.insert(floor, FLOOR);
            for l in 0..LANES {
                let plate = has_plate(l).then(|| {
                    let mut plate =
                        ColliderBuilder::cuboid(PLATE_HALF[0], PLATE_HALF[1], PLATE_HALF[2])
                            .translation(v(plate_centre(l)))
                            .sensor(lane(l) == Lane::Sensor);
                    if lane(l) == Lane::Layer {
                        plate = plate.collision_groups(InteractionGroups::new(
                            Group::GROUP_2,
                            Group::GROUP_2,
                            InteractionTestMode::And,
                        ));
                    }
                    let handle = sim.world.colliders.insert(plate.build());
                    sim.labels.insert(handle, plate_label(l));
                    handle
                });
                sim.plates.push(plate);
                let body = match lane(l) {
                    Lane::Boulder => {
                        let player = sim.world.bodies.insert(
                            RigidBodyBuilder::kinematic_position_based()
                                .translation(v(player_centre(l)))
                                .build(),
                        );
                        let collider = sim.world.colliders.insert_with_parent(
                            ColliderBuilder::capsule_y(PLAYER_HALF, PLAYER_RADIUS).build(),
                            player,
                            &mut sim.world.bodies,
                        );
                        sim.labels.insert(collider, player_label(l));
                        let boulder = sim.world.bodies.insert(
                            RigidBodyBuilder::dynamic()
                                .translation(v(boulder_start(l)))
                                .linvel(Vector::new(BOULDER_SPEED, 0.0, 0.0))
                                .build(),
                        );
                        let collider = sim.world.colliders.insert_with_parent(
                            ColliderBuilder::ball(BOULDER)
                                .density(DENSITY)
                                .active_events(ActiveEvents::COLLISION_EVENTS)
                                .build(),
                            boulder,
                            &mut sim.world.bodies,
                        );
                        sim.labels.insert(collider, body_label(l));
                        Some(boulder)
                    }
                    _ if spawned_at_start(l) => Some(sim.add_crate(l)),
                    _ => None,
                };
                sim.bodies.push(body);
            }
            sim
        }

        fn add_crate(&mut self, l: u32) -> RigidBodyHandle {
            let body = self.world.bodies.insert(
                RigidBodyBuilder::dynamic()
                    .translation(v(crate_start(l)))
                    .lock_rotations()
                    .build(),
            );
            let mut collider = ColliderBuilder::cuboid(HALF, HALF, HALF)
                .density(DENSITY)
                .active_events(ActiveEvents::COLLISION_EVENTS);
            if lane(l) == Lane::Layer {
                collider = collider.collision_groups(InteractionGroups::new(
                    Group::GROUP_1,
                    Group::GROUP_1,
                    InteractionTestMode::And,
                ));
            }
            let collider = self.world.colliders.insert_with_parent(
                collider.build(),
                body,
                &mut self.world.bodies,
            );
            self.labels.insert(collider, body_label(l));
            body
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            for l in 0..LANES {
                match (lane(l), tick) {
                    (Lane::Spawn, SPAWN_TICK) => {
                        self.bodies[l as usize] = Some(self.add_crate(l));
                    }
                    (Lane::Remove, REMOVE_TICK) => {
                        let plate = self.plates[l as usize].take().unwrap();
                        self.world.remove_collider(plate);
                    }
                    (Lane::Drop, SHOVE_TICK) => {
                        let body = &mut self.world.bodies[self.bodies[l as usize].unwrap()];
                        body.set_linvel(Vector::new(SHOVE, 0.0, 0.0), true);
                    }
                    _ => {}
                }
            }
            self.world.step_with_events(&(), &self.events);
            let mut events = Vec::new();
            while let Ok(event) = self.collisions.try_recv() {
                if event.sensor() {
                    continue;
                }
                let (a, b) = (
                    self.labels[&event.collider1()],
                    self.labels[&event.collider2()],
                );
                events.push((event.started(), [a.min(b), a.max(b)]));
            }
            events.sort_by_key(|&(started, pair)| (pair, !started));
            for (started, pair) in events {
                trace.0.push(Obs::ContactEvent {
                    tick,
                    started,
                    pair,
                });
            }
        }
    }
}
