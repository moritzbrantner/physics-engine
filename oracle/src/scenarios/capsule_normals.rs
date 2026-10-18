//! #289/#290: exact capsule contacts. Parallel upright capsules at every overlap depth
//! (separated, touching, deep, coincident axes) and capsule casts against walls from 5 m to
//! 120 m long. The engine is checked against closed forms; Rapier runs parry's default
//! dispatcher (ARPG/MMORPG replace it with their own exact closed forms) for timing only.

use crate::{Obs, Rng, Scenario, Trace};

const TICKS: u32 = 60;
const PAIRS: usize = 32;
const CASTS: usize = 32;
const HALF: f32 = 0.5;
const RADIUS: f32 = 0.3;
const SEPARATION_TOLERANCE: f32 = 1e-4;
const NORMAL_TOLERANCE: f32 = 1e-4;

/// Pair `k` of tick `tick`: the target capsule's centre and the query capsule's centre.
fn pair(tick: u32, k: usize) -> ([f32; 3], [f32; 3]) {
    let mut rng = Rng(0xc0ff_ee00 + u64::from(tick) * 977 + k as u64);
    let target = [k as f32 * 4.0 - 64.0, 1.0, 0.0];
    // Axis distance from coincident to clearly separated; the angle around the target varies.
    let d = if k == 0 { 0.0 } else { rng.range(0.0, 0.7) };
    let a = rng.range(0.0, core::f32::consts::TAU);
    let dy = rng.range(-0.3, 0.3);
    (
        target,
        [
            target[0] + d * a.cos(),
            target[1] + dy,
            target[2] + d * a.sin(),
        ],
    )
}

/// Cast `k`: a wall (centre, half extents), the capsule start and its displacement.
fn cast(tick: u32, k: usize) -> (([f32; 3], [f32; 3]), [f32; 3], [f32; 3]) {
    let mut rng = Rng(0xbead_0000 + u64::from(tick) * 131 + k as u64);
    let length = rng.range(2.5, 60.0);
    let wall = ([k as f32 * 150.0, 1.5, 50.0], [length, 1.5, 0.25]);
    let gap = rng.range(0.1, 3.0);
    let start = [
        wall.0[0] + rng.range(-0.8, 0.8) * length,
        1.0,
        50.0 - 0.25 - RADIUS - gap,
    ];
    let displacement = [rng.range(-2.0, 2.0), 0.0, gap + rng.range(0.5, 3.0)];
    (wall, start, displacement)
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "capsule-normals",
        capabilities: "#289 #290",
        ticks: TICKS,
        #[cfg(feature = "engine")]
        engine: || Box::new(engine::Sim::new()),
        #[cfg(feature = "rapier")]
        rapier: || Box::new(rapier::Sim::new()),
        #[cfg(feature = "rapier")]
        rapier_raw: None,
        check,
    }
}

fn check(engine: &Trace, _rapier: &Trace) -> Result<(), String> {
    let per_tick = PAIRS + CASTS;
    for tick in 0..TICKS {
        for k in 0..PAIRS {
            let (target, query) = pair(tick, k);
            let horizontal = [query[0] - target[0], 0.0, query[2] - target[2]];
            let d = (horizontal[0].powi(2) + horizontal[2].powi(2)).sqrt();
            let expected = d - 2.0 * RADIUS;
            match engine.0[tick as usize * per_tick + k] {
                Obs::Contact(None) if expected > SEPARATION_TOLERANCE => {}
                Obs::Contact(Some((separation, n))) => {
                    if (separation - expected).abs() > SEPARATION_TOLERANCE {
                        return Err(format!(
                            "pair {k} tick {tick}: separation {separation}, expected {expected}"
                        ));
                    }
                    let unit = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
                    if n[1].abs() > NORMAL_TOLERANCE || (unit - 1.0).abs() > NORMAL_TOLERANCE {
                        return Err(format!("pair {k} tick {tick}: normal {n:?} not horizontal"));
                    }
                    if d > 1e-3 {
                        let cos = (n[0] * horizontal[0] + n[2] * horizontal[2]) / d;
                        if cos < 1.0 - NORMAL_TOLERANCE {
                            return Err(format!("pair {k} tick {tick}: normal {n:?} not radial"));
                        }
                    }
                }
                ref other => {
                    return Err(format!(
                        "pair {k} tick {tick}: {other:?}, expected {expected}"
                    ));
                }
            }
        }
        for k in 0..CASTS {
            let ((wall, half), start, displacement) = cast(tick, k);
            let face = wall[2] - half[2];
            let expected = (face - RADIUS - start[2]) / displacement[2];
            match engine.0[tick as usize * per_tick + PAIRS + k] {
                Obs::Ray(Some((t, n))) => {
                    let length = (displacement[0].powi(2) + displacement[2].powi(2)).sqrt();
                    if (t - expected).abs() * length > SEPARATION_TOLERANCE {
                        return Err(format!(
                            "cast {k} tick {tick}: toi {t}, expected {expected}"
                        ));
                    }
                    if (n[2] + 1.0).abs() > NORMAL_TOLERANCE || n[0].abs() > NORMAL_TOLERANCE {
                        return Err(format!(
                            "cast {k} tick {tick}: normal {n:?} off the wall face (wall {half:?})"
                        ));
                    }
                }
                ref other => return Err(format!("cast {k} tick {tick}: {other:?}")),
            }
        }
    }
    Ok(())
}

#[cfg(feature = "engine")]
mod engine {
    use super::*;
    use physics_engine::approximate::{
        Body, Config, QueryFilter, QueryHit, QueryPose, Shape, ShapeCast, Vector, World,
    };

    fn v(a: [f32; 3]) -> Vector {
        Vector(a[0], a[1], a[2])
    }

    pub struct Sim {
        hits: Vec<QueryHit>,
    }
    impl Sim {
        pub fn new() -> Self {
            Self { hits: Vec::new() }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            // The game moves these bodies every tick: one world holds this tick's targets.
            let mut world = World::new(Config {
                gravity: Vector::ZERO,
                ..Config::default()
            })
            .unwrap();
            let capsule = Shape::capsule(HALF, RADIUS);
            for k in 0..PAIRS {
                world
                    .add_body(Body::new(capsule, v(pair(tick, k).0), 0.0))
                    .unwrap();
            }
            for k in 0..CASTS {
                let (wall, _, _) = cast(tick, k);
                world
                    .add_body(Body::new(Shape::Box(v(wall.1)), v(wall.0), 0.0))
                    .unwrap();
            }
            for k in 0..PAIRS {
                let pose = QueryPose::new(capsule, v(pair(tick, k).1));
                world
                    .overlap_shape(pose, QueryFilter::default(), &mut self.hits)
                    .unwrap();
                let deepest = self
                    .hits
                    .iter()
                    .min_by(|a, b| a.separation.total_cmp(&b.separation))
                    .map(|h| (h.separation, [h.normal.0, h.normal.1, h.normal.2]));
                trace.0.push(Obs::Contact(deepest));
            }
            for k in 0..CASTS {
                let (_, start, displacement) = cast(tick, k);
                let pose = QueryPose::new(capsule, v(start));
                world
                    .cast_shape(
                        ShapeCast::new(pose, v(displacement)),
                        QueryFilter::default(),
                        &mut self.hits,
                    )
                    .unwrap();
                trace.0.push(Obs::Ray(
                    self.hits
                        .first()
                        .map(|h| (h.fraction, [h.normal.0, h.normal.1, h.normal.2])),
                ));
            }
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::*;
    use rapier3d::parry::query::{DefaultQueryDispatcher, QueryDispatcher, ShapeCastOptions};
    use rapier3d::prelude::*;

    fn v(a: [f32; 3]) -> Vector {
        Vector::new(a[0], a[1], a[2])
    }

    pub struct Sim;
    impl Sim {
        pub fn new() -> Self {
            Self
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            let mut world = PhysicsWorld::new();
            let capsule = Capsule::new_y(HALF, RADIUS);
            for k in 0..PAIRS {
                world.colliders.insert(
                    ColliderBuilder::new(SharedShape::new(capsule))
                        .translation(v(pair(tick, k).0))
                        .build(),
                );
            }
            for k in 0..CASTS {
                let ((c, h), _, _) = cast(tick, k);
                world.colliders.insert(
                    ColliderBuilder::cuboid(h[0], h[1], h[2])
                        .translation(v(c))
                        .build(),
                );
            }
            world.detect_collisions(&(), &());
            let dispatcher = DefaultQueryDispatcher;
            let w = &world;
            let queries = w.broad_phase.as_query_pipeline(
                &dispatcher,
                &w.bodies,
                &w.colliders,
                QueryFilter::default(),
            );
            for k in 0..PAIRS {
                let pose = Pose::from_translation(v(pair(tick, k).1));
                let deepest = queries
                    .intersect_shape(pose, &capsule)
                    .filter_map(|(_, collider)| {
                        dispatcher
                            .contact(
                                &pose.inv_mul(collider.position()),
                                &capsule,
                                collider.shape(),
                                0.0,
                            )
                            .ok()
                            .flatten()
                    })
                    .min_by(|a, b| a.dist.total_cmp(&b.dist))
                    .map(|c| {
                        let n = pose.rotation * c.normal1;
                        (c.dist, [n.x, n.y, n.z])
                    });
                trace.0.push(Obs::Contact(deepest));
            }
            for k in 0..CASTS {
                let (_, start, displacement) = cast(tick, k);
                let hit = queries.cast_shape(
                    &Pose::from_translation(v(start)),
                    v(displacement),
                    &capsule,
                    ShapeCastOptions {
                        max_time_of_impact: 1.0,
                        ..ShapeCastOptions::default()
                    },
                );
                trace.0.push(Obs::Ray(hit.map(|(_, h)| {
                    let n = h.normal2;
                    (h.time_of_impact, [n.x, n.y, n.z])
                })));
            }
        }
    }
}
