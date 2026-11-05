//! MMORPG #49 / physics-engine #180: the first shipped Greyhaven rock's authored convex proxy.
//! This is the asset-tooling proxy in cm, not the rock's surrounding box. Both backends use the
//! same fixed hull and execute identical capsule/line-of-sight queries in metres.
use crate::{Obs, Scenario, Trace};

const TICKS: u32 = 64;
const ROCK: [[i32; 3]; 32] = [
    [-160, 5, 45],
    [-157, 1, 47],
    [-160, 72, 16],
    [-125, 109, 58],
    [-149, 79, 46],
    [-150, 1, -37],
    [-77, 214, -22],
    [-78, 216, -8],
    [-79, 193, 28],
    [-43, 220, 11],
    [-151, 5, -37],
    [-134, 84, -88],
    [-84, 1, -107],
    [-28, 80, -133],
    [-97, 197, -33],
    [-68, 161, -98],
    [-20, 199, -85],
    [21, 95, 122],
    [64, 2, 150],
    [66, 5, 147],
    [16, 126, 106],
    [18, 215, -10],
    [72, 183, -6],
    [54, 1, -150],
    [158, 90, -69],
    [152, 1, -89],
    [155, 0, -110],
    [160, 1, -107],
    [7, 200, -77],
    [45, 195, -66],
    [105, 174, -58],
    [154, 122, -53],
];

fn vertices() -> Vec<[f32; 3]> {
    ROCK.iter()
        .map(|p| {
            [
                p[0] as f32 * 0.01,
                (p[1] - 110) as f32 * 0.01,
                p[2] as f32 * 0.01,
            ]
        })
        .collect()
}

fn queries(tick: u32) -> Vec<([f32; 3], [f32; 3], bool)> {
    let jitter = ((tick % 7) as f32 - 3.0) * 0.025;
    let mut out = Vec::with_capacity(21);
    for x in [-1.5, -0.9, -0.4, 0.1, 0.6, 1.1] {
        out.push(([x + jitter, 3.4, 0.0], [0.0, -4.0, 0.0], false));
        out.push(([x + jitter, 3.4, 0.3], [0.0, -3.5, 0.0], true));
    }
    for y in [0.35, 1.1, 2.0] {
        out.push(([-2.3, y + jitter, -0.4], [4.6, 0.0, 0.0], false));
        out.push(([-2.3, y + jitter, -0.4], [4.6, 0.0, 0.0], true));
        out.push(([2.2, y + jitter, 2.2], [-2.6, 0.0, -2.6], true));
    }
    out
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "greyhaven-rock-hulls",
        capabilities: "C4 C11 C12",
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

fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    if engine.0.len() != rapier.0.len() {
        return Err(format!(
            "observation count differs: {} vs {}",
            engine.0.len(),
            rapier.0.len()
        ));
    }
    for (i, (a, b)) in engine.0.iter().zip(&rapier.0).enumerate() {
        let equal = match (a, b) {
            (Obs::Hit(None), Obs::Hit(None)) | (Obs::Ray(None), Obs::Ray(None)) => true,
            (Obs::Hit(Some((_, at))), Obs::Hit(Some((_, bt)))) => (at - bt).abs() <= 0.012,
            (Obs::Ray(Some((at, an))), Obs::Ray(Some((bt, bn)))) => {
                (at - bt).abs() <= 0.006 && an.iter().zip(bn).map(|(x, y)| x * y).sum::<f32>() > 0.9
            }
            _ => false,
        };
        if !equal {
            return Err(format!("rock query {i}: engine {a:?}, Rapier {b:?}"));
        }
    }
    Ok(())
}

#[cfg(feature = "engine")]
mod engine {
    use super::*;
    use physics_engine::approximate::{
        BodyStorage, Config, ConvexHullDesc, QueryFilter, QueryPose, RayCast, Shape, ShapeCast,
        Vector as V, World,
    };

    pub struct Sim {
        world: World,
        diagnose: bool,
        work: [u64; 12],
    }
    impl Sim {
        pub fn new(storage: BodyStorage) -> Self {
            let mut world = World::new(Config {
                body_storage: storage,
                gravity: V::ZERO,
                ..Config::default()
            })
            .unwrap();
            world
                .add_convex_hull(
                    ConvexHullDesc {
                        position: V(0.0, 1.1, 0.0),
                        vertices: vertices()
                            .into_iter()
                            .map(|p| V(p[0], p[1], p[2]))
                            .collect(),
                        material: Default::default(),
                    },
                    Some(1),
                )
                .unwrap();
            Self {
                world,
                diagnose: std::env::var_os("PHYSICS_ROCK_WORK").is_some(),
                work: [0; 12],
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            let mut hits = Vec::new();
            for (p, d, capsule) in queries(tick) {
                let origin = V(p[0], p[1], p[2]);
                let displacement = V(d[0], d[1], d[2]);
                if capsule {
                    let stats = self
                        .world
                        .cast_shape(
                            ShapeCast::new(
                                QueryPose::new(Shape::capsule(0.3, 0.2), origin),
                                displacement,
                            ),
                            QueryFilter::default(),
                            &mut hits,
                        )
                        .unwrap();
                    if self.diagnose {
                        self.work[0] += 1;
                        self.work[1] += stats.hull_face_hits;
                        self.work[2] += stats.hull_distance_queries;
                        self.work[3] += stats.hull_gjk_iterations;
                        self.work[4] += stats.hull_epa_iterations;
                        self.work[5] += stats.hull_face_planes_tested;
                        self.work[6] += stats.hull_gjk_intersection_iterations;
                        self.work[7] += stats.hull_gjk_limit_queries;
                        self.work[8] += stats.hull_segment_queries;
                        self.work[9] += stats.hull_segment_plane_tests;
                        self.work[10] += stats.hull_segment_triangles_considered;
                        self.work[11] += stats.hull_segment_triangles_pruned;
                    }
                    trace
                        .0
                        .push(Obs::Hit(hits.first().map(|h| (1, h.fraction))));
                } else {
                    let hit = self
                        .world
                        .cast_ray_first(RayCast::new(origin, displacement), QueryFilter::default())
                        .unwrap();
                    trace.0.push(Obs::Ray(
                        hit.map(|h| (h.fraction, [h.normal.0, h.normal.1, h.normal.2])),
                    ));
                }
            }
            if self.diagnose && tick + 1 == TICKS {
                eprintln!(
                    "greyhaven-rock-work casts={} face_hits={} segment_queries={} segment_planes={} triangles={} pruned={} gjk_calls={} gjk_iterations={} intersection_iterations={} gjk_limits={} epa_iterations={} face_planes={}",
                    self.work[0],
                    self.work[1],
                    self.work[8],
                    self.work[9],
                    self.work[10],
                    self.work[11],
                    self.work[2],
                    self.work[3],
                    self.work[6],
                    self.work[7],
                    self.work[4],
                    self.work[5]
                );
            }
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::*;
    use rapier3d::{
        parry::query::{DefaultQueryDispatcher, ShapeCastOptions},
        prelude::*,
    };

    pub struct Sim {
        world: PhysicsWorld,
    }
    impl Sim {
        pub fn new() -> Self {
            let mut world = PhysicsWorld::new();
            world.gravity = Vector::ZERO;
            let points: Vec<_> = vertices()
                .into_iter()
                .map(|p| Vector::new(p[0], p[1], p[2]))
                .collect();
            let hull = SharedShape::convex_hull(&points).expect("real Greyhaven rock convex proxy");
            world.colliders.insert(
                ColliderBuilder::new(hull)
                    .translation(Vector::new(0.0, 1.1, 0.0))
                    .build(),
            );
            world.detect_collisions(&(), &());
            Self { world }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            let dispatcher = DefaultQueryDispatcher;
            let pipeline = self.world.broad_phase.as_query_pipeline(
                &dispatcher,
                &self.world.bodies,
                &self.world.colliders,
                QueryFilter::only_fixed(),
            );
            let shape = Capsule::new_y(0.3, 0.2);
            for (p, d, capsule) in queries(tick) {
                let origin = Vector::new(p[0], p[1], p[2]);
                let displacement = Vector::new(d[0], d[1], d[2]);
                if capsule {
                    let hit = pipeline.cast_shape(
                        &Pose::from_translation(origin),
                        displacement,
                        &shape,
                        ShapeCastOptions {
                            max_time_of_impact: 1.0,
                            ..ShapeCastOptions::default()
                        },
                    );
                    trace
                        .0
                        .push(Obs::Hit(hit.map(|(_, h)| (1, h.time_of_impact))));
                } else {
                    let hit = pipeline.cast_ray_and_get_normal(
                        &Ray::new(origin, displacement),
                        1.0,
                        true,
                    );
                    trace.0.push(Obs::Ray(hit.map(|(_, h)| {
                        (h.time_of_impact, [h.normal.x, h.normal.y, h.normal.z])
                    })));
                }
            }
        }
    }
}
