//! C1/C11/C12: static boxes added and removed at runtime, segment ray casts (line of sight,
//! ground height) and capsule penetration tests, mirroring ARPG's Rapier call pattern: zero
//! gravity, no dynamic bodies, `detect_collisions` after every collider change, queries through
//! the query pipeline with an "only fixed" filter.

use crate::{Obs, Rng, Scenario, Trace};

const TICKS: u32 = 240;
const LOS_RAYS: usize = 48;
const GROUND_RAYS: usize = 16;
const OVERLAPS: usize = 16;
const DOORS: usize = 8;
const DOOR_PERIOD: u32 = 20;
/// Overlap below this depth counts as touching (ARPG's 1e-3 world units, in metres here).
const PENETRATION_TOLERANCE: f32 = 1e-3;
const CAPSULE_HALF: f32 = 0.5;
const CAPSULE_RADIUS: f32 = 0.4;
/// Distance tolerance between the two engines' hit points.
const DISTANCE_TOLERANCE: f32 = 1e-3;
/// Normal tolerance (cosine of 1 degree).
const NORMAL_COS: f32 = 0.999_847_7;

/// A static box: centre and half extents.
type BoxSpec = ([f32; 3], [f32; 3]);

const FLOOR: BoxSpec = ([0.0, -0.5, 0.0], [40.0, 0.5, 40.0]);

/// Walls on a 6x6 grid of 10 m rooms with doorway gaps, plus pillars; then the door boxes.
fn layout() -> (Vec<BoxSpec>, Vec<BoxSpec>) {
    let mut rng = Rng(0x9e37_79b9_7f4a_7c15);
    let mut walls = Vec::new();
    let mut gaps = Vec::new();
    for line in 0..=6 {
        let a = -30.0 + 10.0 * line as f32;
        for room in 0..6 {
            let mid = -25.0 + 10.0 * room as f32;
            for vertical in [true, false] {
                let inner = line != 0 && line != 6;
                if inner && rng.next_u64().is_multiple_of(3) {
                    // Doorway: two short walls leave a 2 m gap in the middle.
                    for side in [-1.0, 1.0] {
                        let c = mid + side * 3.0;
                        walls.push(wall(vertical, a, c, 2.0));
                    }
                    gaps.push(wall(vertical, a, mid, 1.0));
                } else {
                    walls.push(wall(vertical, a, mid, 5.0));
                }
            }
        }
    }
    for _ in 0..40 {
        let x = rng.range(-29.0, 29.0);
        let z = rng.range(-29.0, 29.0);
        walls.push(([x, 1.5, z], [0.5, 1.5, 0.5]));
    }
    gaps.truncate(DOORS);
    (walls, gaps)
}

fn wall(vertical: bool, line: f32, along: f32, half_length: f32) -> BoxSpec {
    if vertical {
        ([line, 1.5, along], [0.25, 1.5, half_length])
    } else {
        ([along, 1.5, line], [half_length, 1.5, 0.25])
    }
}

/// The queries of one tick, identical for both engines.
struct Queries {
    los: Vec<([f32; 3], [f32; 3])>,
    ground: Vec<([f32; 3], [f32; 3])>,
    capsules: Vec<[f32; 3]>,
}

fn queries(tick: u32) -> Queries {
    let mut rng = Rng(0x2545_f491_4f6c_dd1d ^ u64::from(tick + 1).wrapping_mul(0x9e37_79b9));
    let mut point = |y: f32| [rng.range(-29.0, 29.0), y, rng.range(-29.0, 29.0)];
    let los = (0..LOS_RAYS)
        .map(|_| {
            let from = point(1.0);
            let to = point(1.0);
            (
                from,
                [(to[0] - from[0]) * 0.4, 0.0, (to[2] - from[2]) * 0.4],
            )
        })
        .collect();
    let ground = (0..GROUND_RAYS)
        .map(|_| (point(3.25), [0.0, -10.0, 0.0]))
        .collect();
    let capsules = (0..OVERLAPS)
        .map(|_| point(CAPSULE_HALF + CAPSULE_RADIUS + 0.01))
        .collect();
    Queries {
        los,
        ground,
        capsules,
    }
}

/// Door `index` toggles at this tick.
fn door_toggle(tick: u32) -> Option<usize> {
    tick.is_multiple_of(DOOR_PERIOD)
        .then_some((tick / DOOR_PERIOD) as usize % DOORS)
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "static-queries",
        capabilities: "C1 C11 C12",
        ticks: TICKS,
        #[cfg(feature = "engine")]
        engine: || Box::new(engine::Sim::new()),
        #[cfg(feature = "rapier")]
        rapier: || Box::new(rapier::Sim::new()),
        check,
    }
}

fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    if engine.0.len() != rapier.0.len() {
        return Err(format!(
            "trace lengths differ: {} vs {}",
            engine.0.len(),
            rapier.0.len()
        ));
    }
    for (i, (e, r)) in engine.0.iter().zip(&rapier.0).enumerate() {
        let ok = match (e, r) {
            (Obs::Penetrates(a), Obs::Penetrates(b)) => a == b,
            (Obs::Ray(None), Obs::Ray(None)) => true,
            (Obs::Ray(Some((ta, na))), Obs::Ray(Some((tb, nb)))) => {
                let len = ray_length(i);
                (ta - tb).abs() * len <= DISTANCE_TOLERANCE
                    && (*ta == 0.0 || dot(*na, *nb) >= NORMAL_COS)
            }
            _ => false,
        };
        if !ok {
            return Err(format!(
                "observation {i} differs: engine {e:?}, rapier {r:?}"
            ));
        }
    }
    Ok(())
}

/// Segment length behind observation `index` (rays are recorded LoS first, then ground).
fn ray_length(index: usize) -> f32 {
    let per_tick = LOS_RAYS + GROUND_RAYS + OVERLAPS;
    let tick = (index / per_tick) as u32;
    let i = index % per_tick;
    let q = queries(tick);
    let d = if i < LOS_RAYS {
        q.los[i].1
    } else {
        q.ground[i - LOS_RAYS].1
    };
    dot(d, d).sqrt()
}

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

#[cfg(feature = "engine")]
mod engine {
    use super::*;
    use physics_engine::{
        BodyId,
        approximate::{
            Body, Config, QueryFilter, QueryHit, QueryPose, RayCast, Shape, Vector, World,
        },
    };

    const FLOOR_KEY: u64 = 1;
    const DOOR_BASE: u64 = 10_000;

    fn v(a: [f32; 3]) -> Vector {
        Vector(a[0], a[1], a[2])
    }
    fn fixed(key: u64, (c, h): BoxSpec) -> Body {
        Body::keyed(key, Shape::Box(v(h)), v(c), 0.0)
    }

    pub struct Sim {
        world: World,
        floor: BodyId,
        doors: Vec<BoxSpec>,
        /// The live door bodies (`None` while open), as the game keeps them.
        door_ids: Vec<Option<BodyId>>,
        hits: Vec<QueryHit>,
    }
    impl Sim {
        pub fn new() -> Self {
            let mut world = World::new(Config {
                gravity: Vector::ZERO,
                ..Config::default()
            })
            .unwrap();
            let floor = world.add_body(fixed(FLOOR_KEY, FLOOR)).unwrap();
            let (walls, doors) = layout();
            for (i, w) in walls.into_iter().enumerate() {
                world.add_body(fixed(100 + i as u64, w)).unwrap();
            }
            let door_ids = doors
                .iter()
                .enumerate()
                .map(|(i, d)| Some(world.add_body(fixed(DOOR_BASE + i as u64, *d)).unwrap()))
                .collect();
            Self {
                world,
                floor,
                doors,
                door_ids,
                hits: Vec::new(),
            }
        }
        fn ray(&mut self, origin: [f32; 3], d: [f32; 3], filter: QueryFilter) -> Obs {
            let hit = self
                .world
                .cast_ray_first(RayCast::new(v(origin), v(d)), filter)
                .unwrap();
            Obs::Ray(hit.map(|h| (h.fraction, [h.normal.0, h.normal.1, h.normal.2])))
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            if let Some(door) = door_toggle(tick) {
                self.door_ids[door] = match self.door_ids[door].take() {
                    Some(id) => {
                        self.world.remove_body(id);
                        None
                    }
                    None => {
                        let body = fixed(DOOR_BASE + door as u64, self.doors[door]);
                        Some(self.world.add_body(body).unwrap())
                    }
                };
            }
            let q = queries(tick);
            let no_floor = QueryFilter {
                exclude: Some(self.floor),
                ..QueryFilter::default()
            };
            for (o, d) in q.los {
                let obs = self.ray(o, d, no_floor);
                trace.0.push(obs);
            }
            for (o, d) in q.ground {
                let obs = self.ray(o, d, QueryFilter::default());
                trace.0.push(obs);
            }
            for c in q.capsules {
                let pose = QueryPose::new(Shape::capsule(CAPSULE_HALF, CAPSULE_RADIUS), v(c));
                self.world
                    .overlap_shape(pose, QueryFilter::default(), &mut self.hits)
                    .unwrap();
                let deep = self
                    .hits
                    .iter()
                    .any(|h| h.separation < -PENETRATION_TOLERANCE);
                trace.0.push(Obs::Penetrates(deep));
            }
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::*;
    use rapier3d::parry::query::{DefaultQueryDispatcher, QueryDispatcher};
    use rapier3d::prelude::*;

    fn v(a: [f32; 3]) -> Vector {
        Vector::new(a[0], a[1], a[2])
    }
    fn cuboid((c, h): BoxSpec) -> Collider {
        ColliderBuilder::cuboid(h[0], h[1], h[2])
            .translation(v(c))
            .build()
    }

    pub struct Sim {
        world: PhysicsWorld,
        floor: ColliderHandle,
        doors: Vec<(BoxSpec, Option<ColliderHandle>)>,
    }
    impl Sim {
        pub fn new() -> Self {
            let mut world = PhysicsWorld::new();
            world.gravity = Vector::ZERO;
            world.integration_parameters.dt = 1.0 / 60.0;
            let floor = world.colliders.insert(cuboid(FLOOR));
            let (walls, doors) = layout();
            for w in walls {
                world.colliders.insert(cuboid(w));
            }
            let doors = doors
                .into_iter()
                .map(|d| (d, Some(world.colliders.insert(cuboid(d)))))
                .collect();
            world.detect_collisions(&(), &());
            Self {
                world,
                floor,
                doors,
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            if let Some(door) = door_toggle(tick) {
                let (shape, handle) = &mut self.doors[door];
                match handle.take() {
                    Some(h) => {
                        self.world.remove_collider(h);
                    }
                    None => *handle = Some(self.world.colliders.insert(cuboid(*shape))),
                }
                self.world.detect_collisions(&(), &());
            }
            let q = queries(tick);
            let w = &self.world;
            let dispatcher = DefaultQueryDispatcher;
            let pipeline = |filter| {
                w.broad_phase
                    .as_query_pipeline(&dispatcher, &w.bodies, &w.colliders, filter)
            };
            let ray = |pipeline: &QueryPipeline, (o, d): ([f32; 3], [f32; 3])| {
                Obs::Ray(
                    pipeline
                        .cast_ray_and_get_normal(&Ray::new(v(o), v(d)), 1.0, true)
                        .map(|(_, hit)| {
                            (
                                hit.time_of_impact,
                                [hit.normal.x, hit.normal.y, hit.normal.z],
                            )
                        }),
                )
            };
            let no_floor = pipeline(QueryFilter::only_fixed().exclude_collider(self.floor));
            for r in q.los {
                trace.0.push(ray(&no_floor, r));
            }
            let fixed = pipeline(QueryFilter::only_fixed());
            for r in q.ground {
                trace.0.push(ray(&fixed, r));
            }
            let capsule = Capsule::new_y(CAPSULE_HALF, CAPSULE_RADIUS);
            for c in q.capsules {
                let pose = Pose::from_translation(v(c));
                let deep = fixed.intersect_shape(pose, &capsule).any(|(_, collider)| {
                    dispatcher
                        .contact(
                            &pose.inv_mul(collider.position()),
                            &capsule,
                            collider.shape(),
                            0.0,
                        )
                        .ok()
                        .flatten()
                        .is_some_and(|contact| contact.dist < -PENETRATION_TOLERANCE)
                });
                trace.0.push(Obs::Penetrates(deep));
            }
        }
    }
}
