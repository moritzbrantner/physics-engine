//! C21 (Q35/Q36, #451): generic collision layers applied the same way by solver contacts, CCD,
//! ray and shape casts and overlaps, through runtime filter changes, as Rapier's
//! `InteractionGroups` (`InteractionTestMode::And`) and `QueryFilter`. Layers are generic bits
//! (`WORLD`, `CHARACTER`, `PROP`, `MISSILE`, `TRIGGER`); which bodies share one is the game's
//! choice. Five lanes on a floor plane, each with one transition, and a row of query targets:
//!
//! - `Trapdoor`: a crate rests (and sleeps) on a fixed plate; at `OPEN_TICK` the plate's mask
//!   drops `PROP` (Rapier: `set_collision_groups`, engine: `World::set_layers`). Their cached
//!   contact stops and the crate falls to the floor.
//! - `Hatch`: a crate falls toward a fixed hatch whose mask excludes `PROP`; at `CLOSE_TICK`,
//!   before the crate reaches it, the hatch's mask widens and the crate lands on it.
//! - `Stack`: two crates stacked on the floor; at `OPEN_TICK` the top one's mask drops `PROP`
//!   (a dynamic/dynamic pair): it falls through the lower crate onto the floor.
//! - `Throw`: a CCD weapon is thrown from inside its thrower (a kinematic capsule) at 40 m/s,
//!   faster than a capsule's width per tick. It leaves on a fresh-throw mask without
//!   `CHARACTER`, which `RELEASE_TICK` widens once it is clear of the thrower: it never touches
//!   the thrower and hits the target character. A thrown weapon that must not hit its thrower
//!   is expressed with generic layers and one runtime change, no per-player bit or pair rule.
//! - `Pierce`: the same throw keeping the fresh-throw mask: CCD passes it through the target
//!   and it hits the wall behind.
//!
//! Each tick, rays, sphere casts and box overlaps from a caster (a kinematic capsule) across a
//! sensor, a guard character, a door and a wall run five filters: sight (`CHARACTER` against
//! `WORLD`, caster excluded), attack (`MISSILE` against `WORLD | CHARACTER`, caster excluded and
//! not), a trigger probe including sensors, and the default filter. At `ETHEREAL_TICK` the
//! guard's mask drops `MISSILE`; at `OPEN_TICK` the door's drops `CHARACTER`.
//!
//! Engine and Rapier semantics map one to one: a pair (or a query and a body) interacts when
//! each one's memberships intersect the other's mask, the engine's `CollisionLayers3d` and
//! Rapier's `InteractionGroups` with `And`. Differences, all explicit in this scenario: Rapier's
//! default groups are membership `GROUP_1` (engine: every layer); Rapier's `QueryFilter` admits
//! sensors and skips the group test unless asked (engine: `include_sensors: false` and the
//! filter's layers always tested, `ALL` by default); `QueryFilter::exclude` is Rapier's
//! `exclude_rigid_body`. Rapier's separate solver groups have no engine counterpart.

use crate::{Obs, Scenario, Trace};

const TICKS: u32 = 240;
const DT: f32 = 1.0 / 60.0;
const GRAVITY: f32 = 9.81;
const HALF: f32 = 0.25;
const DENSITY: f32 = 600.0;
const MASS: f32 = DENSITY * 8.0 * HALF * HALF * HALF;
const PLATE_HALF: [f32; 3] = [0.6, 0.05, 0.6];
const CAPSULE_HALF: f32 = 0.5;
const CAPSULE_RADIUS: f32 = 0.3;
const WEAPON: f32 = 0.1;
const THROW_SPEED: f32 = 40.0;
const TARGET_X: f32 = 4.0;
const WALL_X: f32 = 6.0;
const WALL_HALF: [f32; 3] = [0.05, 1.5, 1.0];
const CLOSE_TICK: u32 = 15;
const RELEASE_TICK: u32 = 2;
const ETHEREAL_TICK: u32 = 60;
const OPEN_TICK: u32 = 150;
/// Every pair reports Rapier's transitions, each within this many ticks of Rapier's (as C19).
const TICK_TOLERANCE: u32 = 3;
/// A weapon's later transitions, as it slides off what it hit and drops onto the floor: the
/// post-impact slide is friction-shaped, not filter-shaped (measured 3–4 ticks apart).
const SLIDE_TOLERANCE: u32 = 6;
const RAY_TOLERANCE: f32 = 1e-4;
const CAST_TOLERANCE: f32 = 2e-3;
const REST_TOLERANCE: f32 = 0.02;

const WORLD: u32 = 1;
const CHARACTER: u32 = 2;
const PROP: u32 = 4;
const MISSILE: u32 = 8;
const TRIGGER: u32 = 16;
const ALL: u32 = u32::MAX;
type Layers = (u32, u32);
const FIXED: Layers = (WORLD, ALL);
const CHARACTERS: Layers = (CHARACTER, WORLD | CHARACTER | PROP | MISSILE);
const PROPS: Layers = (PROP, WORLD | CHARACTER | PROP | MISSILE);
const THROWN: Layers = (MISSILE, WORLD | PROP);
const FLYING: Layers = (MISSILE, WORLD | PROP | CHARACTER);
const SENSOR_LAYERS: Layers = (TRIGGER, ALL);
const CLOSED_HATCH: Layers = (WORLD, ALL & !PROP);
const OPEN_TRAPDOOR: Layers = (WORLD, CHARACTER | MISSILE);
const FALLING_PROP: Layers = (PROP, WORLD);
const ETHEREAL: Layers = (CHARACTER, WORLD | PROP);
const OPEN_DOOR: Layers = (WORLD, PROP | MISSILE);

#[derive(Clone, Copy, PartialEq, Debug)]
enum Lane {
    Trapdoor,
    Hatch,
    Stack,
    Throw,
    Pierce,
}
const LANES: [Lane; 5] = [
    Lane::Trapdoor,
    Lane::Hatch,
    Lane::Stack,
    Lane::Throw,
    Lane::Pierce,
];
fn z(l: usize) -> f32 {
    4.0 + 4.0 * l as f32
}

// Labels: the query row below 64 (overlap bit sets), lane bodies by lane.
const FLOOR: u32 = 9;
const CASTER: u32 = 1;
const SENSOR: u32 = 2;
const GUARD: u32 = 3;
const DOOR: u32 = 4;
const ROW_WALL: u32 = 5;
fn plate_label(l: usize) -> u32 {
    100 + l as u32
}
fn crate_label(l: usize) -> u32 {
    200 + l as u32
}
fn top_label(l: usize) -> u32 {
    300 + l as u32
}
fn thrower_label(l: usize) -> u32 {
    400 + l as u32
}
fn target_label(l: usize) -> u32 {
    500 + l as u32
}
fn weapon_label(l: usize) -> u32 {
    600 + l as u32
}
fn wall_label(l: usize) -> u32 {
    700 + l as u32
}

/// A fixed box: label, centre, half extents, layers, sensor.
type Fixed = (u32, [f32; 3], [f32; 3], Layers, bool);
/// A dynamic crate: label, centre, layers.
type Crate = (u32, [f32; 3], Layers);

/// Fixed boxes of every lane and of the query row.
fn fixed_boxes() -> Vec<Fixed> {
    let mut out = Vec::new();
    for (l, lane) in LANES.iter().enumerate() {
        match lane {
            Lane::Trapdoor => {
                out.push((plate_label(l), [0.0, 0.5, z(l)], PLATE_HALF, FIXED, false))
            }
            Lane::Hatch => out.push((
                plate_label(l),
                [0.0, 1.0, z(l)],
                PLATE_HALF,
                CLOSED_HATCH,
                false,
            )),
            Lane::Throw | Lane::Pierce => out.push((
                wall_label(l),
                [WALL_X, WALL_HALF[1], z(l)],
                WALL_HALF,
                FIXED,
                false,
            )),
            Lane::Stack => {}
        }
    }
    out.push((
        SENSOR,
        [1.5, 1.0, ROW_Z],
        [0.2, 0.5, 0.5],
        SENSOR_LAYERS,
        true,
    ));
    out.push((DOOR, [5.0, 1.0, ROW_Z], [0.1, 1.0, 1.0], FIXED, false));
    out.push((ROW_WALL, [8.0, 1.5, ROW_Z], [0.1, 1.5, 1.5], FIXED, false));
    out
}
fn crates() -> Vec<Crate> {
    let mut out = Vec::new();
    for (l, lane) in LANES.iter().enumerate() {
        match lane {
            Lane::Trapdoor => out.push((crate_label(l), [0.0, 0.55 + HALF + 0.001, z(l)], PROPS)),
            Lane::Hatch => out.push((crate_label(l), [0.0, 2.5, z(l)], PROPS)),
            Lane::Stack => {
                out.push((crate_label(l), [0.0, HALF + 0.001, z(l)], PROPS));
                out.push((top_label(l), [0.0, 3.0 * HALF + 0.003, z(l)], PROPS));
            }
            Lane::Throw | Lane::Pierce => {}
        }
    }
    out
}
/// Kinematic capsules: label, centre.
fn characters() -> Vec<(u32, [f32; 3])> {
    let y = CAPSULE_HALF + CAPSULE_RADIUS;
    let mut out = vec![(CASTER, [0.0, y, ROW_Z]), (GUARD, [3.0, y, ROW_Z])];
    for (l, lane) in LANES.iter().enumerate() {
        if matches!(lane, Lane::Throw | Lane::Pierce) {
            out.push((thrower_label(l), [0.0, y, z(l)]));
            out.push((target_label(l), [TARGET_X, y, z(l)]));
        }
    }
    out
}
fn weapon_start(l: usize) -> [f32; 3] {
    [0.0, 1.0, z(l)]
}

/// A runtime layer change at the start of a tick: (label, new layers).
fn changes(tick: u32) -> Vec<(u32, Layers)> {
    let mut out = Vec::new();
    for (l, lane) in LANES.iter().enumerate() {
        match (lane, tick) {
            (Lane::Trapdoor, OPEN_TICK) => out.push((plate_label(l), OPEN_TRAPDOOR)),
            (Lane::Hatch, CLOSE_TICK) => out.push((plate_label(l), FIXED)),
            (Lane::Stack, OPEN_TICK) => out.push((top_label(l), FALLING_PROP)),
            (Lane::Throw, RELEASE_TICK) => out.push((weapon_label(l), FLYING)),
            _ => {}
        }
    }
    match tick {
        ETHEREAL_TICK => out.push((GUARD, ETHEREAL)),
        OPEN_TICK => out.push((DOOR, OPEN_DOOR)),
        _ => {}
    }
    out
}

const ROW_Z: f32 = -4.0;
const QUERY_ORIGIN: [f32; 3] = [0.0, 1.0, ROW_Z];
const QUERY_REACH: [f32; 3] = [10.0, 0.0, 0.0];
const CAST_RADIUS: f32 = 0.2;
const OVERLAP_CENTRE: [f32; 3] = [5.0, 1.05, ROW_Z];
const OVERLAP_HALF: [f32; 3] = [5.0, 1.0, 1.0];
/// A query filter: layers, include sensors, exclude the caster.
type Filter = (Layers, bool, bool);
const FILTERS: [Filter; 5] = [
    ((CHARACTER, WORLD), false, true),
    ((MISSILE, WORLD | CHARACTER), false, true),
    ((MISSILE, WORLD | CHARACTER), false, false),
    ((ALL, ALL), true, true),
    ((ALL, ALL), false, false),
];

pub fn scenario() -> Scenario {
    Scenario {
        name: "collision-filters",
        capabilities: "C21",
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
type PairLog = ([u32; 2], Vec<(u32, bool)>);

#[derive(Default)]
struct Parsed {
    events: Vec<Event>,
    /// Per tick, per filter: ray, cast and overlap.
    queries: Vec<Obs>,
    rest: Vec<[f32; 3]>,
}
fn parse(trace: &Trace) -> Parsed {
    let mut out = Parsed::default();
    for obs in &trace.0 {
        match *obs {
            Obs::ContactEvent {
                tick,
                started,
                pair,
            } => out.events.push((tick, started, pair)),
            Obs::Position(p) => out.rest.push(p),
            other => out.queries.push(other),
        }
    }
    out
}
fn by_pair(events: &[Event]) -> Vec<PairLog> {
    let mut sorted = events.to_vec();
    sorted.sort_by_key(|&(tick, started, pair)| (pair, tick, !started));
    let mut out: Vec<PairLog> = Vec::new();
    for (tick, started, pair) in sorted {
        match out.last_mut() {
            Some((p, list)) if *p == pair => list.push((tick, started)),
            _ => out.push((pair, vec![(tick, started)])),
        }
    }
    out
}
fn kinds(log: &[PairLog], pair: [u32; 2]) -> Vec<bool> {
    log.iter()
        .find(|(p, _)| *p == pair)
        .map(|(_, list)| list.iter().map(|&(_, s)| s).collect())
        .unwrap_or_default()
}

/// The query row's expected first hits per filter before, between and after the changes.
fn expected_hits(tick: u32) -> [u32; 5] {
    let attack = if tick < ETHEREAL_TICK { GUARD } else { DOOR };
    let sight = if tick < OPEN_TICK { DOOR } else { ROW_WALL };
    [sight, attack, CASTER, SENSOR, CASTER]
}

fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    let (e, r) = (parse(engine), parse(rapier));
    let (el, rl) = (by_pair(&e.events), by_pair(&r.events));
    // The engine's own contract: events alternate per pair, pairs in body (label) order.
    for (pair, list) in &el {
        if pair[0] >= pair[1] {
            return Err(format!("engine pair not in body order: {pair:?}"));
        }
        if list
            .iter()
            .enumerate()
            .any(|(i, &(_, s))| s != (i % 2 == 0))
        {
            return Err(format!("pair {pair:?} does not alternate: {list:?}"));
        }
    }
    for (l, lane) in LANES.iter().enumerate() {
        let (plate, body, top) = (plate_label(l), crate_label(l), top_label(l));
        let (thrower, target, weapon, wall) = (
            thrower_label(l),
            target_label(l),
            weapon_label(l),
            wall_label(l),
        );
        let expected: Vec<([u32; 2], &[bool])> = match lane {
            Lane::Trapdoor => vec![([plate, body], &[true, false]), ([FLOOR, body], &[true])],
            Lane::Hatch => vec![([plate, body], &[true]), ([FLOOR, body], &[])],
            Lane::Stack => vec![
                ([FLOOR, body], &[true]),
                ([body, top], &[true, false]),
                ([FLOOR, top], &[true]),
            ],
            Lane::Throw => vec![
                ([thrower, weapon], &[]),
                ([target, weapon], &[true, false]),
                ([wall, weapon], &[]),
                ([FLOOR, weapon], &[true]),
            ],
            Lane::Pierce => vec![
                ([thrower, weapon], &[]),
                ([target, weapon], &[]),
                ([wall, weapon], &[true, false]),
                ([FLOOR, weapon], &[true]),
            ],
        };
        for (pair, want) in expected {
            let pair = [pair[0].min(pair[1]), pair[0].max(pair[1])];
            let got = kinds(&el, pair);
            if got != want {
                return Err(format!(
                    "lane {l} ({lane:?}) pair {pair:?}: engine {got:?}, expected {want:?}"
                ));
            }
        }
    }
    // Rapier parity: the same pairs and transitions, each within the tolerance.
    let pairs = |v: &[PairLog]| v.iter().map(|(p, _)| *p).collect::<Vec<_>>();
    if pairs(&el) != pairs(&rl) {
        return Err(format!(
            "pairs differ: engine {:?}, Rapier {:?}",
            pairs(&el),
            pairs(&rl)
        ));
    }
    let weapons: Vec<u32> = (0..LANES.len()).map(weapon_label).collect();
    for ((pair, a), (_, b)) in el.iter().zip(&rl) {
        let slides = |k: usize| k > 0 && pair.iter().any(|p| weapons.contains(p));
        let same = a.len() == b.len()
            && a.iter()
                .zip(b)
                .enumerate()
                .all(|(k, (&(ta, sa), &(tb, sb)))| {
                    let tolerance = if slides(k) {
                        SLIDE_TOLERANCE
                    } else {
                        TICK_TOLERANCE
                    };
                    sa == sb && ta.abs_diff(tb) <= tolerance
                });
        if !same {
            return Err(format!("pair {pair:?}: engine {a:?}, Rapier {b:?}"));
        }
    }
    // Queries: the same target and fraction on both, and the expected target.
    let per_tick = FILTERS.len() * 3;
    if e.queries.len() != per_tick * TICKS as usize || r.queries.len() != e.queries.len() {
        return Err(format!(
            "query observations: engine {}, Rapier {}",
            e.queries.len(),
            r.queries.len()
        ));
    }
    for (i, (a, b)) in e.queries.iter().zip(&r.queries).enumerate() {
        let tick = (i / per_tick) as u32;
        let (filter, kind) = ((i % per_tick) / 3, i % 3);
        let fail =
            || format!("tick {tick} filter {filter} query {kind}: engine {a:?}, Rapier {b:?}");
        match (a, b) {
            (Obs::Hit(Some((la, fa))), Obs::Hit(Some((lb, fb)))) => {
                let tolerance = if kind == 0 {
                    RAY_TOLERANCE
                } else {
                    CAST_TOLERANCE
                };
                if la != lb || (fa - fb).abs() > tolerance {
                    return Err(fail());
                }
                if *la != expected_hits(tick)[filter] {
                    return Err(format!(
                        "{}: expected {}",
                        fail(),
                        expected_hits(tick)[filter]
                    ));
                }
            }
            (Obs::Overlaps(sa), Obs::Overlaps(sb)) if sa == sb => {}
            _ => return Err(fail()),
        }
    }
    // Where the crates came to rest.
    if e.rest.len() != r.rest.len() {
        return Err("rest positions differ in count".into());
    }
    for (a, b) in e.rest.iter().zip(&r.rest) {
        if a.iter().zip(b).any(|(x, y)| (x - y).abs() > REST_TOLERANCE) {
            return Err(format!("crate rests at {a:?}, on Rapier at {b:?}"));
        }
    }
    Ok(())
}

#[cfg(feature = "engine")]
mod engine {
    use super::*;
    use physics_engine::{
        BodyId, CollisionLayers3d,
        approximate::{
            Body, BodyStorage, Config, ContactEvent, QueryFilter, QueryHit, QueryPose, RayCast,
            Shape, ShapeCast, Vector, World,
        },
    };
    use std::collections::HashMap;

    use crate::kcc::engine::v;

    fn layers((m, f): Layers) -> CollisionLayers3d {
        CollisionLayers3d::new(m, f)
    }

    pub struct Sim {
        world: World,
        labels: HashMap<u64, u32>,
        ids: HashMap<u32, BodyId>,
        crates: Vec<BodyId>,
        hits: Vec<QueryHit>,
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
                ids: HashMap::new(),
                crates: Vec::new(),
                hits: Vec::new(),
            };
            let mut floor = Body::keyed(u64::from(FLOOR), Shape::HalfSpace, Vector::ZERO, 0.0);
            floor.layers = layers(FIXED);
            sim.add(floor);
            for (label, c, h, l, sensor) in fixed_boxes() {
                let mut b = Body::keyed(u64::from(label), Shape::Box(v(h)), v(c), 0.0);
                b.layers = layers(l);
                b.sensor = sensor;
                sim.add(b);
            }
            for (label, c) in characters() {
                let mut b = Body::keyed(
                    u64::from(label),
                    Shape::capsule(CAPSULE_HALF, CAPSULE_RADIUS),
                    v(c),
                    0.0,
                );
                b.external = true;
                b.layers = layers(CHARACTERS);
                sim.add(b);
            }
            for (label, c, l) in crates() {
                let mut b = Body::keyed(
                    u64::from(label),
                    Shape::Box(Vector(HALF, HALF, HALF)),
                    v(c),
                    MASS,
                );
                b.rotation_locked = true;
                b.layers = layers(l);
                let id = sim.add(b);
                sim.crates.push(id);
            }
            for (l, lane) in LANES.iter().enumerate() {
                if matches!(lane, Lane::Throw | Lane::Pierce) {
                    let mass = DENSITY * 4.0 / 3.0 * std::f32::consts::PI * WEAPON.powi(3);
                    let mut b = Body::keyed(
                        u64::from(weapon_label(l)),
                        Shape::Sphere(WEAPON),
                        v(weapon_start(l)),
                        mass,
                    );
                    b.ccd = true;
                    b.velocity = Vector(THROW_SPEED, 0.0, 0.0);
                    b.layers = layers(THROWN);
                    sim.add(b);
                }
            }
            sim
        }

        fn add(&mut self, body: Body) -> BodyId {
            let label = body.key.unwrap() as u32;
            let id = self.world.add_body(body).unwrap();
            self.labels.insert(id.to_bits(), label);
            self.ids.insert(label, id);
            id
        }

        fn filter(&self, ((m, f), sensors, exclude): Filter) -> QueryFilter {
            QueryFilter {
                layers: layers((m, f)),
                include_sensors: sensors,
                exclude: exclude.then(|| self.ids[&CASTER]),
            }
        }

        fn label(&self, id: BodyId) -> u32 {
            self.labels[&id.to_bits()]
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            for (label, l) in changes(tick) {
                self.world.set_layers(self.ids[&label], layers(l)).unwrap();
            }
            let report = self.world.step(f64::from(DT)).unwrap();
            for event in report.contact_events {
                let (a, b) = event.bodies();
                trace.0.push(Obs::ContactEvent {
                    tick,
                    started: matches!(event, ContactEvent::Started(..)),
                    pair: [self.label(a), self.label(b)],
                });
            }
            for f in FILTERS {
                let filter = self.filter(f);
                let ray = self
                    .world
                    .cast_ray_first(RayCast::new(v(QUERY_ORIGIN), v(QUERY_REACH)), filter)
                    .unwrap();
                trace
                    .0
                    .push(Obs::Hit(ray.map(|h| (self.label(h.body), h.fraction))));
                let pose = QueryPose::new(Shape::Sphere(CAST_RADIUS), v(QUERY_ORIGIN));
                self.world
                    .cast_shape(ShapeCast::new(pose, v(QUERY_REACH)), filter, &mut self.hits)
                    .unwrap();
                let first = self.hits.first().map(|h| (self.label(h.body), h.fraction));
                trace.0.push(Obs::Hit(first));
                let pose = QueryPose::new(Shape::Box(v(OVERLAP_HALF)), v(OVERLAP_CENTRE));
                self.world
                    .overlap_shape(pose, filter, &mut self.hits)
                    .unwrap();
                let set = self
                    .hits
                    .iter()
                    .fold(0u64, |s, h| s | 1 << self.label(h.body));
                trace.0.push(Obs::Overlaps(set));
            }
            if tick + 1 == TICKS {
                for &id in &self.crates {
                    let p = self.world.body(id).unwrap().position;
                    trace.0.push(Obs::Position([p.0, p.1, p.2]));
                }
            }
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::*;
    use rapier3d::parry::query::{DefaultQueryDispatcher, ShapeCastOptions};
    use rapier3d::prelude::*;
    use std::collections::HashMap;
    use std::sync::mpsc::{Receiver, channel};

    use crate::kcc::rapier::v;

    fn groups((m, f): Layers) -> InteractionGroups {
        InteractionGroups::new(
            Group::from_bits_retain(m),
            Group::from_bits_retain(f),
            InteractionTestMode::And,
        )
    }

    pub struct Sim {
        world: PhysicsWorld,
        events: ChannelEventCollector,
        collisions: Receiver<CollisionEvent>,
        labels: HashMap<ColliderHandle, u32>,
        colliders: HashMap<u32, ColliderHandle>,
        caster: RigidBodyHandle,
        crates: Vec<RigidBodyHandle>,
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
                colliders: HashMap::new(),
                caster: RigidBodyHandle::invalid(),
                crates: Vec::new(),
            };
            let floor = sim.world.colliders.insert(
                ColliderBuilder::new(SharedShape::halfspace(Vector::Y))
                    .collision_groups(groups(FIXED))
                    .build(),
            );
            sim.label(floor, FLOOR);
            for (label, c, h, l, sensor) in fixed_boxes() {
                let collider = ColliderBuilder::cuboid(h[0], h[1], h[2])
                    .translation(v(c))
                    .collision_groups(groups(l))
                    .sensor(sensor)
                    .build();
                let handle = sim.world.colliders.insert(collider);
                sim.label(handle, label);
            }
            for (label, c) in characters() {
                let body = sim.world.bodies.insert(
                    RigidBodyBuilder::kinematic_position_based()
                        .translation(v(c))
                        .build(),
                );
                let handle = sim.world.colliders.insert_with_parent(
                    ColliderBuilder::capsule_y(CAPSULE_HALF, CAPSULE_RADIUS)
                        .collision_groups(groups(CHARACTERS))
                        .build(),
                    body,
                    &mut sim.world.bodies,
                );
                sim.label(handle, label);
                if label == CASTER {
                    sim.caster = body;
                }
            }
            for (label, c, l) in crates() {
                let body = sim.world.bodies.insert(
                    RigidBodyBuilder::dynamic()
                        .translation(v(c))
                        .lock_rotations()
                        .build(),
                );
                let handle = sim.world.colliders.insert_with_parent(
                    ColliderBuilder::cuboid(HALF, HALF, HALF)
                        .density(DENSITY)
                        .collision_groups(groups(l))
                        .active_events(ActiveEvents::COLLISION_EVENTS)
                        .build(),
                    body,
                    &mut sim.world.bodies,
                );
                sim.label(handle, label);
                sim.crates.push(body);
            }
            for (l, lane) in LANES.iter().enumerate() {
                if matches!(lane, Lane::Throw | Lane::Pierce) {
                    let body = sim.world.bodies.insert(
                        RigidBodyBuilder::dynamic()
                            .translation(v(weapon_start(l)))
                            .linvel(Vector::new(THROW_SPEED, 0.0, 0.0))
                            .ccd_enabled(true)
                            .build(),
                    );
                    let handle = sim.world.colliders.insert_with_parent(
                        ColliderBuilder::ball(WEAPON)
                            .density(DENSITY)
                            .collision_groups(groups(THROWN))
                            .active_events(ActiveEvents::COLLISION_EVENTS)
                            .build(),
                        body,
                        &mut sim.world.bodies,
                    );
                    sim.label(handle, weapon_label(l));
                }
            }
            sim
        }

        fn label(&mut self, handle: ColliderHandle, label: u32) {
            self.labels.insert(handle, label);
            self.colliders.insert(label, handle);
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            for (label, l) in changes(tick) {
                let handle = self.colliders[&label];
                self.world.colliders[handle].set_collision_groups(groups(l));
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
            let w = &self.world;
            let dispatcher = DefaultQueryDispatcher;
            for (l, sensors, exclude) in FILTERS {
                let mut filter = QueryFilter::new().groups(groups(l));
                if !sensors {
                    filter = filter.exclude_sensors();
                }
                if exclude {
                    filter = filter.exclude_rigid_body(self.caster);
                }
                let pipeline =
                    w.broad_phase
                        .as_query_pipeline(&dispatcher, &w.bodies, &w.colliders, filter);
                let ray = Ray::new(v(QUERY_ORIGIN), v(QUERY_REACH));
                let hit = pipeline
                    .cast_ray(&ray, 1.0, true)
                    .map(|(h, toi)| (self.labels[&h], toi));
                trace.0.push(Obs::Hit(hit));
                let ball = Ball::new(CAST_RADIUS);
                let hit = pipeline
                    .cast_shape(
                        &Pose::from_translation(v(QUERY_ORIGIN)),
                        v(QUERY_REACH),
                        &ball,
                        ShapeCastOptions {
                            max_time_of_impact: 1.0,
                            ..ShapeCastOptions::default()
                        },
                    )
                    .map(|(h, hit)| (self.labels[&h], hit.time_of_impact));
                trace.0.push(Obs::Hit(hit));
                let cuboid = Cuboid::new(v(OVERLAP_HALF));
                let set = pipeline
                    .intersect_shape(Pose::from_translation(v(OVERLAP_CENTRE)), &cuboid)
                    .fold(0u64, |s, (h, _)| s | 1 << self.labels[&h]);
                trace.0.push(Obs::Overlaps(set));
            }
            if tick + 1 == TICKS {
                for &body in &self.crates {
                    let p = self.world.bodies[body].translation();
                    trace.0.push(Obs::Position([p.x, p.y, p.z]));
                }
            }
        }
    }
}
