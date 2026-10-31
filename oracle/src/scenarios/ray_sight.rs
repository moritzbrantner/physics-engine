//! C11 alone (#415): MMORPG's ray pattern in the Greyhaven-sized town of `character-move`
//! (~1,150 fixed boxes), with no other query or step in the tick, so the ray cost cannot hide
//! behind cheaper work. Every tick casts NPC sight lines (head height, 5–40 m, any direction),
//! ground-height probes and short projectile segments, each a first-hit `cast_ray_first`
//! against static geometry (Rapier: `cast_ray_and_get_normal` with `only_fixed`).

use super::character_move::{BoxSpec, layout};
use crate::{Obs, Rng, Scenario, Trace};

const TICKS: u32 = 120;
const SIGHT: usize = 160;
const GROUND: usize = 64;
const PROJECTILES: usize = 32;
/// Distance tolerance between the two engines' hit points.
const DISTANCE_TOLERANCE: f32 = 1e-3;
/// Normal tolerance (cosine of 1 degree).
const NORMAL_COS: f32 = 0.999_847_7;

/// The segments (origin, translation) of one tick, identical for both engines.
fn rays(tick: u32) -> Vec<([f32; 3], [f32; 3])> {
    let mut rng = Rng(0x51e7_4a1b_c0de_f00d ^ u64::from(tick + 1).wrapping_mul(0x9e37_79b9));
    let mut out = Vec::with_capacity(SIGHT + GROUND + PROJECTILES);
    for _ in 0..SIGHT {
        let (x, z) = (rng.range(-118.0, 118.0), rng.range(-118.0, 118.0));
        let angle = rng.range(0.0, std::f32::consts::TAU);
        let length = rng.range(5.0, 40.0);
        let rise = rng.range(-0.5, 0.5);
        out.push((
            [x, 1.5, z],
            [angle.cos() * length, rise, angle.sin() * length],
        ));
    }
    for _ in 0..GROUND {
        let (x, z) = (rng.range(-118.0, 118.0), rng.range(-118.0, 118.0));
        out.push(([x, 12.0, z], [0.0, -14.0, 0.0]));
    }
    for _ in 0..PROJECTILES {
        let (x, z) = (rng.range(-118.0, 118.0), rng.range(-118.0, 118.0));
        let angle = rng.range(0.0, std::f32::consts::TAU);
        let step = rng.range(0.5, 3.0);
        out.push((
            [x, rng.range(0.5, 3.0), z],
            [angle.cos() * step, rng.range(-0.3, 0.1), angle.sin() * step],
        ));
    }
    out
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "ray-sight",
        capabilities: "C11",
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
    let per_tick = SIGHT + GROUND + PROJECTILES;
    if engine.0.len() != rapier.0.len() || engine.0.len() != per_tick * TICKS as usize {
        return Err(format!(
            "trace lengths differ: {} vs {}",
            engine.0.len(),
            rapier.0.len()
        ));
    }
    let mut hits = 0;
    for (i, (e, r)) in engine.0.iter().zip(&rapier.0).enumerate() {
        let ok = match (e, r) {
            (Obs::Ray(None), Obs::Ray(None)) => true,
            (Obs::Ray(Some((ta, na))), Obs::Ray(Some((tb, nb)))) => {
                hits += 1;
                let d = rays((i / per_tick) as u32)[i % per_tick].1;
                (ta - tb).abs() * dot(d, d).sqrt() <= DISTANCE_TOLERANCE
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
    // Both hits and misses must be exercised.
    if hits == 0 || hits == engine.0.len() {
        return Err(format!("{hits} of {} rays hit", engine.0.len()));
    }
    Ok(())
}

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

#[cfg(feature = "engine")]
mod engine {
    use super::*;
    use physics_engine::{
        CollisionLayers3d,
        approximate::{Body, Config, QueryFilter, RayCast, Shape, Vector, World},
    };

    use crate::kcc::engine::v;

    /// Static geometry is on layer 1; sight sees only it (Rapier's `only_fixed`).
    const STATIC_ONLY: QueryFilter = QueryFilter {
        layers: CollisionLayers3d::new(1, 1),
        exclude: None,
        include_sensors: false,
    };

    pub struct Sim {
        world: World,
    }
    impl Sim {
        pub fn new(storage: physics_engine::approximate::BodyStorage) -> Self {
            let mut world = World::new(Config {
                body_storage: storage,
                gravity: Vector::ZERO,
                ..Config::default()
            })
            .unwrap();
            for (i, (c, h)) in layout().into_iter().enumerate() {
                let mut body = Body::keyed(1 + i as u64, Shape::Box(v(h)), v(c), 0.0);
                body.layers = CollisionLayers3d::new(1, u32::MAX);
                world.add_body(body).unwrap();
            }
            Self { world }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            for (o, d) in rays(tick) {
                let hit = self
                    .world
                    .cast_ray_first(RayCast::new(v(o), v(d)), STATIC_ONLY)
                    .unwrap();
                trace.0.push(Obs::Ray(
                    hit.map(|h| (h.fraction, [h.normal.0, h.normal.1, h.normal.2])),
                ));
            }
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::*;
    use rapier3d::parry::query::DefaultQueryDispatcher;
    use rapier3d::prelude::*;

    use crate::kcc::rapier::v;
    fn cuboid((c, h): BoxSpec) -> Collider {
        ColliderBuilder::cuboid(h[0], h[1], h[2])
            .translation(v(c))
            .build()
    }

    pub struct Sim {
        world: PhysicsWorld,
    }
    impl Sim {
        pub fn new() -> Self {
            let mut world = PhysicsWorld::new();
            world.gravity = Vector::ZERO;
            world.integration_parameters.dt = 1.0 / 60.0;
            for b in layout() {
                world.colliders.insert(cuboid(b));
            }
            world.detect_collisions(&(), &());
            Self { world }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            let w = &self.world;
            let dispatcher = DefaultQueryDispatcher;
            let fixed = w.broad_phase.as_query_pipeline(
                &dispatcher,
                &w.bodies,
                &w.colliders,
                QueryFilter::only_fixed(),
            );
            for (o, d) in rays(tick) {
                trace.0.push(Obs::Ray(
                    fixed
                        .cast_ray_and_get_normal(&Ray::new(v(o), v(d)), 1.0, true)
                        .map(|(_, hit)| {
                            (
                                hit.time_of_impact,
                                [hit.normal.x, hit.normal.y, hit.normal.z],
                            )
                        }),
                ));
            }
        }
    }
}
