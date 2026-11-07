//! The #180 consumer slice: true convex rock queries, not bounding-box proxies.
use physics_engine::approximate::{
    Body, CharacterMove, Config, ConvexHullDesc, QueryFilter, QueryPose, RayCast, RayFeature,
    Shape, ShapeCast, Vector as V, World,
};

#[test]
fn separated_rock_segment_distance_matches_independent_box_reference() {
    use geometry_kernels::{
        hull::{Hull3, QuickHullWork, quickhull3},
        segment_hull::{PreparedSegmentHull, SegmentHullWork},
    };

    let corners: Vec<[f64; 3]> = [-1.0, 1.0]
        .into_iter()
        .flat_map(|x| {
            [-1.0, 1.0]
                .into_iter()
                .flat_map(move |y| [-1.0, 1.0].into_iter().map(move |z| [x, y, z]))
        })
        .collect();
    let mut hull = Hull3::default();
    quickhull3(&corners, &mut QuickHullWork::default(), &mut hull);
    let hull = PreparedSegmentHull::new(hull).unwrap();

    // The minimum point-to-AABB squared distance along a segment is convex.
    // Ternary search here is an independent oracle for the triangle/edge kernel.
    let box_distance = |p: [f64; 3], q: [f64; 3]| -> f64 {
        let squared = |t: f64| -> f64 {
            (0..3)
                .map(|i| {
                    let c = p[i] + (q[i] - p[i]) * t;
                    let d = c - c.clamp(-1.0, 1.0);
                    d * d
                })
                .sum()
        };
        let (mut lo, mut hi) = (0.0, 1.0);
        for _ in 0..100 {
            let a = lo + (hi - lo) / 3.0;
            let b = hi - (hi - lo) / 3.0;
            if squared(a) <= squared(b) {
                hi = b;
            } else {
                lo = a;
            }
        }
        squared((lo + hi) * 0.5).sqrt()
    };

    let mut seed = 0xa4f8_529f_92c4_1811_u64;
    let mut next = || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed as u32 as f64 / u32::MAX as f64) * 7.0 - 3.5
    };
    let mut separated = 0;
    let mut intersecting = 0;
    for _ in 0..400 {
        let p = [next(), next(), next()];
        let q = [next(), next(), next()];
        let expected = box_distance(p, q);
        let actual = hull.closest(p, q);
        if expected > 1e-6 {
            separated += 1;
            let actual = actual.expect("unambiguously separated segment");
            assert!(
                (actual.distance - expected).abs() < 1e-7,
                "segment {p:?} to {q:?}: hull={actual:?}, box={expected}"
            );
        } else {
            intersecting += 1;
            assert!(
                actual.is_none(),
                "crossing segment {p:?} to {q:?}: {actual:?}"
            );
        }
    }
    assert!(separated > 150 && intersecting > 20);

    let mut work = SegmentHullWork::default();
    let _ = hull
        .closest_with_work([2.0, 2.0, 0.0], [2.0, 2.0, 0.8], &mut work)
        .unwrap();
    assert!(
        work.triangles_pruned > 0,
        "expected geometry pruning: {work:?}"
    );
    assert!(work.edges_pruned > 0, "expected edge pruning: {work:?}");

    let diagonal = hull
        .closest([2.2, 0.0, 0.0], [0.0, 2.2, 0.0])
        .expect("diagonal segment misses the cube corner");
    assert!((diagonal.distance - 0.2 / 2.0_f64.sqrt()).abs() < 1e-12);
    assert_eq!(diagonal.point_hull, [1.0, 1.0, 0.0]);
}

fn wedge_rock() -> (World, physics_engine::BodyId) {
    let mut world = World::new(Config {
        gravity: V::ZERO,
        ..Config::default()
    })
    .unwrap();
    // A closed irregular rock with a sloping top at y = -x and an AABB reaching y = 1.
    let vertices = [-1.0, 1.0]
        .into_iter()
        .flat_map(|z| [V(-1.0, -1.0, z), V(1.0, -1.0, z), V(-1.0, 1.0, z)])
        .collect();
    let id = world
        .add_convex_hull(
            ConvexHullDesc {
                vertices,
                position: V::ZERO,
                material: Default::default(),
            },
            Some(10),
        )
        .unwrap();
    (world, id)
}

fn cube_rock() -> World {
    let mut world = World::new(Config {
        gravity: V::ZERO,
        ..Config::default()
    })
    .unwrap();
    let vertices = [-1.0, 1.0]
        .into_iter()
        .flat_map(|x| {
            [-1.0, 1.0]
                .into_iter()
                .flat_map(move |y| [-1.0, 1.0].into_iter().map(move |z| V(x, y, z)))
        })
        .collect();
    world
        .add_convex_hull(
            ConvexHullDesc {
                vertices,
                position: V::ZERO,
                material: Default::default(),
            },
            Some(500),
        )
        .unwrap();
    world
}

#[test]
fn capsule_sweeps_resolve_cube_face_edge_and_corner_at_analytic_contact() {
    let world = cube_rock();
    let mut hits = Vec::new();
    let diagonal_toi = 1.0 - 0.2 / 2.0_f64.sqrt();
    for (label, origin, displacement, expected, normal, half_segment) in [
        (
            "face",
            V(2.0, 0.0, 0.0),
            V(-1.0, 0.0, 0.0),
            0.8,
            V(1.0, 0.0, 0.0),
            0.3,
        ),
        (
            "edge",
            V(2.0, 0.0, 2.0),
            V(-1.0, 0.0, -1.0),
            diagonal_toi,
            V(0.707_106_77, 0.0, 0.707_106_77),
            0.3,
        ),
        (
            "corner",
            V(2.0, 2.0, 2.0),
            V(-1.0, -1.0, -1.0),
            1.0 - 0.2 / 3.0_f64.sqrt(),
            V(0.577_350_26, 0.577_350_26, 0.577_350_26),
            0.0,
        ),
    ] {
        world
            .cast_shape(
                ShapeCast::new(
                    QueryPose::new(Shape::capsule(half_segment, 0.2), origin),
                    displacement,
                ),
                QueryFilter::default(),
                &mut hits,
            )
            .unwrap();
        assert_eq!(hits.len(), 1, "{label}: {hits:?}");
        let hit = &hits[0];
        assert!(
            (f64::from(hit.fraction) - expected).abs() < 1e-3,
            "{label}: expected {expected}, got {hit:?}"
        );
        assert!(
            hit.normal.dot(normal) > 0.97,
            "{label}: incorrect outward normal {hit:?}"
        );
    }
}

#[test]
fn capsule_overlap_at_cube_edge_has_analytic_separation_and_normal() {
    let world = cube_rock();
    let mut hits = Vec::new();
    world
        .overlap_shape(
            QueryPose::new(Shape::capsule(0.3, 0.2), V(1.1, 0.0, 1.1)),
            QueryFilter::default(),
            &mut hits,
        )
        .unwrap();
    assert_eq!(hits.len(), 1, "{hits:?}");
    assert!((f64::from(hits[0].separation) - (0.1 * 2.0_f64.sqrt() - 0.2)).abs() < 1e-5);
    assert!(hits[0].normal.dot(V(0.707_106_77, 0.0, 0.707_106_77)) > 0.99);
}

#[test]
fn capsule_deep_inside_cube_uses_penetration_fallback() {
    let world = cube_rock();
    let mut hits = Vec::new();
    world
        .overlap_shape(
            QueryPose::new(Shape::capsule(0.3, 0.2), V::ZERO),
            QueryFilter::default(),
            &mut hits,
        )
        .unwrap();
    assert_eq!(hits.len(), 1, "{hits:?}");
    assert!(hits[0].separation < -1.0, "{hits:?}");
}

#[test]
fn rock_rejects_degenerate_geometry_and_cannot_be_forged_as_an_ordinary_body() {
    let (mut world, id) = wedge_rock();
    assert!(world.add_body(world.body(id).unwrap().clone()).is_err());
    assert!(
        world
            .add_convex_hull(
                ConvexHullDesc {
                    vertices: vec![V::ZERO; 4],
                    position: V::ZERO,
                    material: Default::default(),
                },
                Some(11)
            )
            .is_err()
    );
}

#[test]
fn rays_hit_the_sloping_face_and_miss_empty_corners_of_the_rock_aabb() {
    let (world, id) = wedge_rock();
    let hit = world
        .cast_ray_first(
            RayCast::new(V(0.0, 2.0, 0.0), V(0.0, -4.0, 0.0)),
            QueryFilter::default(),
        )
        .unwrap()
        .unwrap();
    assert_eq!(hit.body, id);
    assert!((hit.point.1).abs() < 1e-4, "{hit:?}");
    assert!(hit.normal.0 > 0.5 && hit.normal.1 > 0.5, "{hit:?}");
    assert!(matches!(hit.feature, RayFeature::ConvexHullFace(_)));
    // A box proxy would report a collision at x=.8, y=.8. The rock is below y=-.8.
    assert!(
        world
            .cast_ray_first(
                RayCast::new(V(0.8, 0.8, -2.0), V(0.0, 0.0, 4.0)),
                QueryFilter::default()
            )
            .unwrap()
            .is_none()
    );
}

#[test]
fn capsule_sweep_hits_real_slope_and_not_its_aabb() {
    let (world, id) = wedge_rock();
    let mut hits = Vec::new();
    let pose = QueryPose::new(Shape::capsule(0.4, 0.2), V(0.0, 2.0, 0.0));
    world
        .cast_shape(
            ShapeCast::new(pose, V(0.0, -3.0, 0.0)),
            QueryFilter::default(),
            &mut hits,
        )
        .unwrap();
    assert_eq!(hits.len(), 1, "{hits:?}");
    assert_eq!(hits[0].body, id);
    assert!(
        hits[0].fraction > 0.25 && hits[0].fraction < 0.65,
        "{hits:?}"
    );
    assert!(hits[0].normal.0 > 0.5 && hits[0].normal.1 > 0.5, "{hits:?}");
    world
        .overlap_shape(
            QueryPose::new(Shape::Sphere(0.1), V(0.8, 0.8, 0.0)),
            QueryFilter::default(),
            &mut hits,
        )
        .unwrap();
    assert!(
        hits.is_empty(),
        "an empty AABB corner is not rock: {hits:?}"
    );
}

#[test]
fn overlapping_capsule_and_sphere_report_rock_surface_not_aabb_contact() {
    let (world, id) = wedge_rock();
    let mut hits = Vec::new();
    world
        .overlap_shape(
            QueryPose::new(Shape::Sphere(0.3), V(0.0, 0.05, 0.0)),
            QueryFilter::default(),
            &mut hits,
        )
        .unwrap();
    assert_eq!(hits.len(), 1, "{hits:?}");
    assert_eq!(hits[0].body, id);
    assert!(hits[0].separation < 0.0, "{hits:?}");
    assert!(hits[0].normal.0 > 0.4 && hits[0].normal.1 > 0.4);
    world
        .overlap_shape(
            QueryPose::new(Shape::capsule(0.3, 0.2), V(0.0, 0.1, 0.0)),
            QueryFilter::default(),
            &mut hits,
        )
        .unwrap();
    assert_eq!(hits.len(), 1, "{hits:?}");
    assert!(hits[0].separation < 0.0, "{hits:?}");
}

#[test]
fn removing_and_moving_static_rocks_updates_queries_without_aabb_ghosts() {
    let (mut world, id) = wedge_rock();
    world.set_static_position(id, V(10.0, 0.0, 0.0)).unwrap();
    let ray = RayCast::new(V(0.0, 2.0, 0.0), V(0.0, -4.0, 0.0));
    assert!(
        world
            .cast_ray_first(ray, QueryFilter::default())
            .unwrap()
            .is_none()
    );
    assert!(
        world
            .cast_ray_first(
                RayCast::new(V(10.0, 2.0, 0.0), V(0.0, -4.0, 0.0)),
                QueryFilter::default(),
            )
            .unwrap()
            .is_some()
    );
    assert!(world.remove_body(id).is_some());
    assert!(
        world
            .cast_ray_first(
                RayCast::new(V(10.0, 2.0, 0.0), V(0.0, -4.0, 0.0)),
                QueryFilter::default(),
            )
            .unwrap()
            .is_none()
    );
}

#[test]
fn character_approaches_rock_without_crossing_slope() {
    let (mut world, id) = wedge_rock();
    let mut body = Body::keyed(11, Shape::capsule(0.3, 0.2), V(-0.6, 2.0, 0.0), 1.0);
    body.external = true;
    let character = world.add_body(body).unwrap();
    world
        .add_character(character, world.character_settings())
        .unwrap();
    let mut results = Vec::new();
    for _ in 0..10 {
        world
            .move_characters(
                &[CharacterMove {
                    body: character,
                    desired: V(0.0, -0.35, 0.0),
                    exclude: None,
                    autostep: None,
                }],
                &mut results,
            )
            .unwrap();
    }
    assert_eq!(results[0].support, Some(id), "{results:?}");
    let centre = world.body(character).unwrap().position;
    assert!(centre.1 > 0.8, "penetrated sloped rock: {centre:?}");
}

#[cfg(feature = "checkpoints")]
#[test]
fn checkpoint_restores_the_exact_rock_queries() {
    use physics_engine::approximate::{Checkpoint, CheckpointContext, CheckpointLimits};
    let (world, _) = wedge_rock();
    let context = CheckpointContext {
        build: [1; 32],
        content: [2; 32],
    };
    let pose = QueryPose::new(Shape::Box(V(0.29, 0.29, 0.03)), V::ZERO);
    let mut occupancy = Vec::new();
    let before = world.checkpoint(context).unwrap().to_bytes();
    world
        .overlap_shape(pose, QueryFilter::default(), &mut occupancy)
        .unwrap();
    assert_eq!(before, world.checkpoint(context).unwrap().to_bytes());
    let bytes = world.checkpoint(context).unwrap().to_bytes();
    let restored = Checkpoint::from_bytes(&bytes, context, CheckpointLimits::default())
        .unwrap()
        .restore(&[])
        .unwrap();
    let mut recovered = Vec::new();
    let stats = restored
        .overlap_shape(pose, QueryFilter::default(), &mut recovered)
        .unwrap();
    assert_eq!(occupancy, recovered);
    assert_eq!(stats.geometry_preparations, 1);
    let cast = RayCast::new(V(0.0, 2.0, 0.0), V(0.0, -4.0, 0.0));
    assert_eq!(
        world.cast_ray_first(cast, QueryFilter::default()).unwrap(),
        restored
            .cast_ray_first(cast, QueryFilter::default())
            .unwrap()
    );
}

/// Advisory local diagnosis only, never a portable CI performance ratchet.
#[test]
#[ignore]
fn local_ray_vs_capsule_cost() {
    let (world, _) = wedge_rock();
    let runs = 10_000;
    let mut stats = None;
    let start = std::time::Instant::now();
    for i in 0..runs {
        let x = (i % 11) as f32 * 0.2 - 1.0;
        std::hint::black_box(
            world
                .cast_ray_first(
                    RayCast::new(V(x, 2.0, 0.0), V(0.0, -3.0, 0.0)),
                    QueryFilter::default(),
                )
                .unwrap(),
        );
    }
    let ray = start.elapsed();
    let start = std::time::Instant::now();
    let mut hits = Vec::new();
    for i in 0..runs {
        let x = (i % 11) as f32 * 0.2 - 1.0;
        stats = Some(
            world
                .cast_shape(
                    ShapeCast::new(
                        QueryPose::new(Shape::capsule(0.3, 0.2), V(x, 2.0, 0.0)),
                        V(0.0, -3.0, 0.0),
                    ),
                    QueryFilter::default(),
                    &mut hits,
                )
                .unwrap(),
        );
        std::hint::black_box(&hits);
    }
    let sweep = start.elapsed();
    eprintln!("rock hull 10000 each: ray={ray:?} capsule={sweep:?} last_stats={stats:?}");
}

#[test]
fn unsupported_rigid_hull_pairs_fail_atomically_instead_of_tunneling() {
    // Spheres take hull contacts and CCD (#519, tests/sphere_static_contacts.rs).
    for shape in [Shape::capsule(0.3, 0.2), Shape::Box(V(0.2, 0.2, 0.2))] {
        let mut world = cube_rock();
        let mut body = Body::keyed(42, shape, V(-3.0, 0.0, 0.0), 1.0);
        body.velocity = V(600.0, 0.0, 0.0);
        body.ccd = true;
        let id = world.add_body(body).unwrap();
        let before = world.body(id).unwrap().clone();
        assert!(matches!(
            world.step(1.0 / 60.0),
            Err(physics_engine::approximate::Error::CollisionSearchFailed {
                reason: physics_engine::approximate::SweepFailure::InvalidGeometryInput,
                ..
            })
        ));
        assert_eq!(world.body(id).unwrap(), &before);
    }
}

#[path = "support/greyhaven_rock.rs"]
mod authored;

#[test]
fn actual_rock_projectile_segments_and_contact_features_are_repeatable() {
    use physics_engine::approximate::BodyStorage;
    let mut prior = None;
    for storage in [BodyStorage::Ordered, BodyStorage::Indexed] {
        let mut world = World::new(Config {
            body_storage: storage,
            gravity: V::ZERO,
            ..Config::default()
        })
        .unwrap();
        for key in [200, 100] {
            world
                .add_convex_hull(
                    ConvexHullDesc {
                        vertices: authored::vertices()
                            .into_iter()
                            .map(|p| V(p[0], p[1], p[2]))
                            .collect(),
                        position: V(0.0, 1.1, 0.0),
                        material: Default::default(),
                    },
                    Some(key),
                )
                .unwrap();
        }
        let cast = RayCast::new(V(-4.0, 1.1, 0.0), V(10.0, 0.0, 0.0));
        let hit = world
            .cast_ray_first(cast, QueryFilter::default())
            .unwrap()
            .unwrap();
        assert_eq!(world.body(hit.body).unwrap().key, Some(100));
        assert!(matches!(hit.feature, RayFeature::ConvexHullFace(_)));
        assert!(hit.fraction > 0.0 && hit.fraction < 0.5);
        assert!(hit.normal.dot(cast.displacement) < 0.0);
        // The game's bouncing projectile reflects the remaining complete segment about this normal.
        let reflected = cast.displacement - hit.normal * (2.0 * hit.normal.dot(cast.displacement));
        let reflected = RayCast::new(
            hit.point + hit.normal * 0.001,
            reflected * (1.0 - hit.fraction),
        );
        assert!(
            world
                .cast_ray_first(reflected, QueryFilter::default())
                .unwrap()
                .is_none()
        );
        assert!(
            world
                .cast_ray_first(
                    RayCast::new(V(-1.5, 3.4, 1.4), V(0.0, -4.0, 0.0)),
                    QueryFilter::default()
                )
                .unwrap()
                .is_none(),
            "empty authored-rock AABB corner blocked projectile"
        );
        let mut hits = Vec::new();
        world
            .cast_shape(
                ShapeCast::new(
                    QueryPose::new(Shape::capsule(0.3, 0.2), V(0.0, 3.4, 0.3)),
                    V(0.0, -3.5, 0.0),
                ),
                QueryFilter::default(),
                &mut hits,
            )
            .unwrap();
        assert_eq!(world.body(hits[0].body).unwrap().key, Some(100));
        assert!(matches!(
            hits[0].feature,
            Some(RayFeature::ConvexHullFace(_))
        ));
        let result = (hit, hits);
        if let Some(expected) = &prior {
            assert_eq!(&result, expected);
        } else {
            prior = Some(result);
        }
    }
}

#[cfg(feature = "checkpoints")]
#[test]
fn checkpoint_preserves_hulls_and_meshes_with_distinct_tags_and_removed_slots() {
    use physics_engine::approximate::{
        Checkpoint, CheckpointContext, CheckpointLimits, TriangleMeshDesc,
    };
    let mut world = cube_rock();
    let removed = world.body_id(500).unwrap();
    world.remove_body(removed).unwrap();
    world
        .add_convex_hull(
            ConvexHullDesc {
                vertices: authored::vertices()
                    .into_iter()
                    .map(|p| V(p[0], p[1], p[2]))
                    .collect(),
                position: V(0.0, 1.1, 0.0),
                material: Default::default(),
            },
            Some(501),
        )
        .unwrap();
    world
        .add_triangle_mesh(
            TriangleMeshDesc {
                position: V(10.0, 0.0, 0.0),
                vertices: vec![V(-1.0, 0.0, -1.0), V(1.0, 0.0, -1.0), V(0.0, 0.0, 1.0)],
                triangles: vec![[0, 1, 2]],
                material: Default::default(),
            },
            Some(502),
        )
        .unwrap();
    let context = CheckpointContext {
        build: [3; 32],
        content: [7; 32],
    };
    let saved = world.checkpoint(context).unwrap().to_bytes();
    assert_eq!(u32::from_le_bytes(saved[8..12].try_into().unwrap()), 11);
    let restored = Checkpoint::from_bytes(&saved, context, CheckpointLimits::default())
        .unwrap()
        .restore(&[])
        .unwrap();
    assert_eq!(saved, restored.checkpoint(context).unwrap().to_bytes());
    for x in [0.0, 10.0] {
        let cast = RayCast::new(V(x, 3.4, 0.0), V(0.0, -4.0, 0.0));
        let before = world
            .cast_ray_first(cast, QueryFilter::default())
            .unwrap()
            .unwrap();
        let after = restored
            .cast_ray_first(cast, QueryFilter::default())
            .unwrap()
            .unwrap();
        assert_eq!(before, after);
        assert!(matches!(
            before.feature,
            RayFeature::ConvexHullFace(_) | RayFeature::TriangleMeshTriangle(_)
        ));
    }
}

#[test]
fn non_ccd_boxes_stop_at_the_hull_across_the_entire_step_interval() {
    for storage in [
        physics_engine::approximate::BodyStorage::Ordered,
        physics_engine::approximate::BodyStorage::Indexed,
    ] {
        for speed in [6.0, 60.0, 600.0] {
            // A separate world admits the hull before or after the box: pair-normal
            // orientation must not depend on which shape owns the lower body ID.
            for hull_first in [true, false] {
                let mut world = World::new(Config {
                    gravity: V::ZERO,
                    body_storage: storage,
                    ..Config::default()
                })
                .unwrap();
                let add_hull = |world: &mut World| {
                    world
                        .add_convex_hull(
                            ConvexHullDesc {
                                vertices: [-1.0, 1.0]
                                    .into_iter()
                                    .flat_map(|x| {
                                        [-1.0, 1.0].into_iter().flat_map(move |y| {
                                            [-1.0, 1.0].into_iter().map(move |z| V(x, y, z))
                                        })
                                    })
                                    .collect(),
                                position: V::ZERO,
                                material: Default::default(),
                            },
                            Some(10),
                        )
                        .unwrap()
                };
                if hull_first {
                    add_hull(&mut world);
                }
                let mut body =
                    Body::keyed(42, Shape::Box(V(0.2, 0.2, 0.2)), V(-3.0, 0.0, 0.0), 1.0);
                body.velocity = V(speed, 0.0, 0.0);
                body.rotation_locked = true;
                body.friction = 0.0;
                let id = world.add_body(body).unwrap();
                if !hull_first {
                    add_hull(&mut world);
                }
                for _ in 0..60 {
                    world.step(1.0 / 60.0).unwrap();
                    let b = world.body(id).unwrap();
                    assert!(b.position.0 <= -1.2 + 0.005, "speed {speed}: {b:?}");
                }
            }
        }
    }
}

#[cfg(feature = "checkpoints")]
#[test]
fn box_rock_contact_continues_exactly_after_checkpoint_restore() {
    let mut world = cube_rock();
    let mut body = Body::new(Shape::Box(V(0.2, 0.2, 0.2)), V(-1.5, 0.0, 0.0), 1.0);
    body.velocity = V(5.0, 0.0, 0.0);
    body.rotation_locked = true;
    let id = world.add_body(body).unwrap();
    for _ in 0..8 {
        world.step(1.0 / 60.0).unwrap();
    }
    let mut restored = world
        .checkpoint(physics_engine::approximate::CheckpointContext {
            build: [1; 32],
            content: [2; 32],
        })
        .unwrap()
        .restore(&[])
        .unwrap();
    for _ in 0..30 {
        world.step(1.0 / 60.0).unwrap();
        restored.step(1.0 / 60.0).unwrap();
        assert_eq!(world.body(id), restored.body(id));
    }
}

#[test]
fn box_occupancy_uses_the_hull_surface_and_keeps_sweeps_unsupported() {
    let (world, id) = wedge_rock();
    let mut hits = Vec::new();
    let filter = QueryFilter::default();
    let half = V(0.125, 0.125, 0.125);
    // The upper-right AABB corner is empty; the sloped face is x + y = 0.
    for (position, count) in [
        (V(0.6, 0.6, 0.0), 0),
        (V(0.125, 0.125, 0.0), 1),
        (V::ZERO, 1),
    ] {
        world
            .overlap_shape(
                QueryPose::new(Shape::Box(half), position),
                filter,
                &mut hits,
            )
            .unwrap();
        assert_eq!(hits.len(), count, "{position:?}");
        if let Some(hit) = hits.first() {
            assert_eq!(hit.body, id);
            assert_eq!(hit.fraction, 0.0);
            assert!(hit.separation <= 0.0);
            assert!((hit.normal.length() - 1.0).abs() < 1e-5);
            assert!(
                (hit.query_point - hit.target_point - hit.normal * hit.separation).length() < 1e-5
            );
        }
        let previous = hits.clone();
        world
            .overlap_shape(
                QueryPose::new(Shape::Box(half), position),
                filter,
                &mut hits,
            )
            .unwrap();
        assert_eq!(hits, previous);
    }
    assert!(
        world
            .cast_shape(
                ShapeCast::new(QueryPose::new(Shape::Box(half), V::ZERO), V(0.1, 0.0, 0.0)),
                filter,
                &mut hits
            )
            .is_err()
    );
    assert!(hits.is_empty());
}

#[test]
fn box_overlap_frame_reuses_translation_and_refreshes_shape_and_orientation() {
    use physics_engine::approximate::Quaternion;
    let (world, _) = wedge_rock();
    let mut hits = Vec::new();
    let filter = QueryFilter::default();
    let mut pose = QueryPose::new(Shape::Box(V(0.29, 0.29, 0.03)), V::ZERO);
    let stats = world.overlap_shape(pose, filter, &mut hits).unwrap();
    assert_eq!(stats.geometry_preparations, 1);
    let stats = world.overlap_shape(pose, filter, &mut hits).unwrap();
    assert_eq!(stats.geometry_preparations, 0);
    pose.position = V(0.6, 0.6, 0.0);
    let stats = world.overlap_shape(pose, filter, &mut hits).unwrap();
    assert_eq!(stats.geometry_preparations, 0);
    assert!(hits.is_empty());
    for (shape, angle) in [
        (Shape::Box(V(0.2, 0.5, 0.1)), 0.0_f32),
        (Shape::Box(V(0.2, 0.5, 0.1)), 0.4),
    ] {
        pose.shape = shape;
        pose.orientation = Quaternion(0.0, (angle / 2.0).sin(), 0.0, (angle / 2.0).cos());
        let stats = world.overlap_shape(pose, filter, &mut hits).unwrap();
        assert_eq!(stats.geometry_preparations, 1);
        let (fresh, _) = wedge_rock();
        let mut expected = Vec::new();
        fresh.overlap_shape(pose, filter, &mut expected).unwrap();
        assert_eq!(hits, expected);
    }
}
