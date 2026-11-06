//! Actual Redbrand meshes from MMORPG PR #640 (9ccc3a57), centimetres → metres.
//! Four real 1.8 m player routes exercise the mouth/floor seam, roof, lower passage and ramp;
//! rays and fixed-orientation capsule casts reuse the same prepared immutable chunks.
use crate::{Obs, Scenario, Trace};
const TICKS: u32 = 300;
const STARTS: [[f32; 3]; 4] = [
    [58.0, 11.9025, -128.0],
    [62.0, 14.1025, -143.0],
    [62.0, 11.9025, -140.0],
    [48.5, 11.9025, -143.0],
];
const RAYS: [([f32; 3], [f32; 3]); 4] = [
    ([62.0, 12.0, -138.0], [0.0, -2.0, 0.0]),
    ([62.0, 12.0, -138.0], [0.0, 5.0, 0.0]),
    ([62.0, 14.0, -143.0], [0.0, -2.0, 0.0]),
    ([58.0, 12.0, -127.0], [0.0, 0.0, -10.0]),
];
const CASTS: [([f32; 3], [f32; 3]); 4] = [
    ([62.0, 13.0, -138.0], [0.0, -3.0, 0.0]),
    ([62.0, 14.2, -143.0], [0.0, 2.0, 0.0]),
    ([58.0, 12.0, -127.0], [0.0, 0.0, -12.0]),
    ([50.0, 12.0, -138.0], [-10.0, 0.0, 0.0]),
];
fn desired(lane: usize, tick: u32) -> [f32; 3] {
    match lane {
        0 => [0.0, -0.05, if tick < 180 { -0.05 } else { 0.0 }],
        1 => [0.0, if tick < 8 { 0.2 } else { -0.05 }, 0.0],
        2 => [0.0, -0.05, if tick < 180 { -0.015 } else { 0.0 }],
        _ => [0.05, -0.05, 0.0],
    }
}
pub(super) struct Chunk {
    pub(super) key: u64,
    pub(super) position: [f32; 3],
    pub(super) vertices: Vec<[f32; 3]>,
    pub(super) triangles: Vec<[u32; 3]>,
}
pub(super) fn chunks() -> Vec<Chunk> {
    let json: serde_json::Value =
        serde_json::from_str(include_str!("../../../tests/fixtures/redbrand-cave.json")).unwrap();
    let vector = |v: &serde_json::Value| {
        [
            v[0].as_f64().unwrap() as f32 * 0.01,
            v[1].as_f64().unwrap() as f32 * 0.01,
            v[2].as_f64().unwrap() as f32 * 0.01,
        ]
    };
    json.as_array()
        .unwrap()
        .iter()
        .map(|c| Chunk {
            key: c["id"].as_u64().unwrap(),
            position: vector(&c["position"]),
            vertices: c["vertices"]
                .as_array()
                .unwrap()
                .iter()
                .map(vector)
                .collect(),
            triangles: c["triangles"]
                .as_array()
                .unwrap()
                .iter()
                .map(|t| {
                    [
                        t[0].as_u64().unwrap() as u32,
                        t[1].as_u64().unwrap() as u32,
                        t[2].as_u64().unwrap() as u32,
                    ]
                })
                .collect(),
        })
        .collect()
}
pub fn scenario() -> Scenario {
    Scenario {
        name: "redbrand-cave",
        capabilities: "C23",
        ticks: TICKS,
        #[cfg(feature = "engine")]
        engine: |s| Box::new(engine::Sim::new(s)),
        #[cfg(feature = "rapier")]
        rapier: || Box::new(rapier::Sim::new(true)),
        #[cfg(feature = "rapier")]
        rapier_raw: Some(|| Box::new(rapier::Sim::new(false))),
        check,
    }
}
fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    if engine.0.len() != rapier.0.len() {
        return Err("trace length mismatch".into());
    }
    for (i, (a, b)) in engine.0.iter().zip(&rapier.0).enumerate() {
        let ok = match (a, b) {
            // KCC edge nudges can accumulate differently over hundreds of moves. As with
            // the terrain/slopes scenarios, surface clearance and completed real routes below
            // are authoritative; matching one arbitrary trajectory tolerance is not.
            (Obs::Position(_), Obs::Position(_)) => true,
            (Obs::Grounded(a), Obs::Grounded(b)) => a == b,
            (Obs::Ray(None), Obs::Ray(None)) | (Obs::Hit(None), Obs::Hit(None)) => true,
            (Obs::Ray(Some((ta, na))), Obs::Ray(Some((tb, nb)))) => {
                (ta - tb).abs() < 0.001
                    && na.iter().zip(nb).map(|(a, b)| a * b).sum::<f32>() > 0.999
            }
            (Obs::Hit(Some((ka, ta))), Obs::Hit(Some((kb, tb)))) => {
                ka == kb && (ta - tb).abs() < 0.001
            }
            _ => false,
        };
        if !ok {
            return Err(format!("Redbrand observation{i}: engine{a:?}, Rapier{b:?}"));
        }
    }
    // The expected real routes prevent two equally broken implementations from passing.
    for trace in [engine, rapier] {
        let per_tick = 16;
        for tick in 0..TICKS as usize {
            for (lane, start) in STARTS.iter().enumerate() {
                let Obs::Position(p) = trace.0[tick * per_tick + lane * 2] else {
                    return Err("missing player position".into());
                };
                // Actual 0.6 m half-segment + 0.3 m radius; surfaces are at 11/13.2/15.5 m.
                if p[1] < 11.9 - 1e-3 || p[1] + 0.9 > 15.5 + 1e-3 {
                    return Err(format!(
                        "player{lane} crosses floor/ceiling at tick{tick}: {p:?}"
                    ));
                }
                if (lane == 0 || lane == 2)
                    && ((p[1] - 11.9025).abs() > 0.01 || (p[0] - start[0]).abs() > 0.01)
                {
                    return Err(format!("floor route{lane} left its passage: {p:?}"));
                }
                if lane == 1 && p[1] < 14.1 - 1e-3 {
                    return Err(format!("upper player sank through shelf: {p:?}"));
                }
                if lane == 3 && p[0] > 49.5 && p[0] < 55.7 {
                    let slope = 2.2_f32 / 7.0;
                    let separation = (p[1] - 0.6 - 11.0 - slope * (p[0] - 49.0))
                        / (1.0 + slope * slope).sqrt()
                        - 0.3;
                    if separation < -1e-3 {
                        return Err(format!("player sank through authored ramp: {p:?}"));
                    }
                }
            }
        }
        let Obs::Position(entered) = trace.0[(TICKS as usize - 1) * per_tick] else {
            return Err("missing route".into());
        };
        if entered[2] > -136.8 || (entered[1] - 11.9025).abs() > 0.015 {
            return Err(format!("mouth/floor seam blocked: {entered:?}"));
        }
        let Obs::Position(lower) = trace.0[(TICKS as usize - 1) * per_tick + 4] else {
            return Err("missing lower route".into());
        };
        if lower[2] > -142.5 {
            return Err(format!("phantom blocking under shelf: {lower:?}"));
        }
        let Obs::Position(upper) = trace.0[(TICKS as usize - 1) * per_tick + 2] else {
            return Err("missing upper route".into());
        };
        if (upper[1] - 14.1025).abs() > 0.01 {
            return Err(format!("upper player did not return to shelf: {upper:?}"));
        }
        let Obs::Position(shelf) = trace.0[(TICKS as usize - 1) * per_tick + 6] else {
            return Err("missing ramp route".into());
        };
        if shelf[0] < 57.8 || (shelf[1] - 14.1025).abs() > 0.02 {
            return Err(format!("ramp did not reach upper shelf: {shelf:?}"));
        }
    }
    Ok(())
}
#[cfg(feature = "engine")]
mod engine {
    use super::*;
    use crate::kcc::engine::{Characters, capsules, v};
    use physics_engine::approximate::{
        BodyStorage, Config, QueryFilter, QueryHit, QueryPose, RayCast, Shape, ShapeCast,
        TriangleMeshDesc, Vector, World,
    };
    pub struct Sim {
        c: Characters,
        hits: Vec<QueryHit>,
    }
    impl Sim {
        pub fn new(storage: BodyStorage) -> Self {
            let mut world = World::new(Config {
                gravity: Vector::ZERO,
                body_storage: storage,
                ..Config::default()
            })
            .unwrap();
            for c in chunks() {
                world
                    .add_triangle_mesh(
                        TriangleMeshDesc {
                            position: v(c.position),
                            vertices: c.vertices.into_iter().map(v).collect(),
                            triangles: c.triangles,
                            material: Default::default(),
                        },
                        Some(c.key),
                    )
                    .unwrap();
            }
            let settings = world.character_settings();
            Self {
                c: Characters::new(
                    world,
                    capsules(1, Shape::capsule(0.6, 0.3), STARTS).map(|mut b| {
                        b.layers = physics_engine::CollisionLayers3d::new(2, u32::MAX);
                        b
                    }),
                    settings,
                ),
                hits: Vec::new(),
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            self.c.move_all(|lane| v(desired(lane, tick)));
            for r in &self.c.results {
                trace.0.push(Obs::Position(self.c.position(r.body)));
                trace.0.push(Obs::Grounded(r.grounded));
            }
            let w = &self.c.world;
            let filter = QueryFilter {
                layers: physics_engine::CollisionLayers3d::new(1, 1),
                ..QueryFilter::default()
            };
            for (o, d) in RAYS {
                let hit = w.cast_ray_first(RayCast::new(v(o), v(d)), filter).unwrap();
                trace.0.push(Obs::Ray(
                    hit.map(|h| (h.fraction, [h.normal.0, h.normal.1, h.normal.2])),
                ));
            }
            for (o, d) in CASTS {
                w.cast_shape(
                    ShapeCast::new(QueryPose::new(Shape::capsule(0.6, 0.3), v(o)), v(d)),
                    filter,
                    &mut self.hits,
                )
                .unwrap();
                trace.0.push(Obs::Hit(
                    self.hits
                        .first()
                        .map(|h| (w.body(h.body).unwrap().key.unwrap() as u32, h.fraction)),
                ));
            }
        }
    }
}
#[cfg(feature = "rapier")]
mod rapier {
    use super::*;
    use crate::kcc::rapier::{Characters, mirror_controller, v};
    use rapier3d::{parry::query::ShapeCastOptions, prelude::*};
    pub struct Sim {
        c: Characters,
    }
    impl Sim {
        pub fn new(rebuild: bool) -> Self {
            let statics = chunks()
                .into_iter()
                .map(|c| {
                    ColliderBuilder::new(
                        SharedShape::trimesh(c.vertices.into_iter().map(v).collect(), c.triangles)
                            .unwrap(),
                    )
                    .translation(v(c.position))
                    .user_data(c.key as u128)
                })
                .collect();
            Self {
                c: Characters::new(
                    statics,
                    STARTS,
                    Capsule::new_y(0.6, 0.3),
                    mirror_controller(0.0025, Some(0.05), None),
                    rebuild,
                ),
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            self.c.start_tick();
            for lane in 0..STARTS.len() {
                let movement = self.c.move_shape(lane, v(desired(lane, tick)));
                let to = self.c.end(lane, &movement);
                let grounded = movement.grounded && self.c.support(to).is_some();
                self.c.commit(lane, to);
                trace.0.push(Obs::Position([to.x, to.y, to.z]));
                trace.0.push(Obs::Grounded(grounded));
            }
            let fixed = self.c.queries(QueryFilter::only_fixed());
            for (o, d) in RAYS {
                let hit = fixed.cast_ray_and_get_normal(&Ray::new(v(o), v(d)), 1.0, true);
                trace.0.push(Obs::Ray(hit.map(|(_, h)| {
                    (h.time_of_impact, [h.normal.x, h.normal.y, h.normal.z])
                })));
            }
            let capsule = Capsule::new_y(0.6, 0.3);
            for (o, d) in CASTS {
                let hit = fixed.cast_shape(
                    &Pose::from_translation(v(o)),
                    v(d),
                    &capsule,
                    ShapeCastOptions {
                        max_time_of_impact: 1.0,
                        ..Default::default()
                    },
                );
                trace.0.push(Obs::Hit(hit.map(|(handle, h)| {
                    (
                        self.c.world.colliders[handle].user_data as u32,
                        h.time_of_impact,
                    )
                })));
            }
        }
    }
}

#[cfg(all(test, feature = "engine", feature = "rapier"))]
mod tests {
    use super::*;
    #[test]
    fn single_move_at_the_real_ramp_foot_diagnoses_initial_embedding() {
        let geometry = crate::probe::Geometry {
            meshes: chunks()
                .into_iter()
                .map(|c| crate::probe::TriangleMesh {
                    position: c.position,
                    vertices: c.vertices,
                    triangles: c.triangles,
                })
                .collect(),
            ..Default::default()
        };
        for x in [49.0, 48.5] {
            let at = [x, 11.9025, -143.0];
            let desired = [0.05, -0.05, 0.0];
            let c = crate::probe::Controller {
                half_segment: 0.6,
                ..Default::default()
            };
            let r = crate::probe::rapier(&geometry, c, at, desired);
            let e = crate::probe::engine(&geometry, c, at, desired);
            eprintln!("Redbrand singlemove at{x}: Rapier{r:?}, Engine{e:?}");
            if x == 48.5 {
                assert!((0..3).all(|k| (e.translation[k] - r.translation[k]).abs() < 0.001));
            }
        }
        let at = [62.0, 11.9025, -140.20987];
        let desired = [0.0, -0.05, -0.015];
        let c = crate::probe::Controller {
            half_segment: 0.6,
            ..Default::default()
        };
        let r = crate::probe::rapier(&geometry, c, at, desired);
        let e = crate::probe::engine(&geometry, c, at, desired);
        eprintln!("Redbrand lowerpassage singlemove: Rapier{r:?}, Engine{e:?}");
        assert!((0..3).all(|k| (e.translation[k] - r.translation[k]).abs() < 0.001));
    }
}
