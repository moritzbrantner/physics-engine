//! The #180 consumer slice: true convex rock queries, not bounding-box proxies.
use physics_engine::approximate::{
    Body, CharacterMove, Config, ConvexHullDesc, QueryFilter, QueryPose, RayCast, RayFeature,
    Shape, ShapeCast, Vector as V, World,
};

#[test]
fn separated_rock_segment_distance_matches_independent_box_reference() {
    use geometry_kernels::{
        hull::{Hull3, QuickHullWork, quickhull3},
        segment_hull::{
            SegmentHullWork, separated_segment_hull_closest,
            separated_segment_hull_closest_with_work,
        },
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
        let actual = separated_segment_hull_closest(p, q, &hull);
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
    let _ = separated_segment_hull_closest_with_work(
        [2.0, 2.0, 0.0],
        [2.0, 2.0, 0.8],
        &hull,
        &mut work,
    )
    .unwrap();
    assert!(
        work.triangles_pruned > 0,
        "expected geometry pruning: {work:?}"
    );

    let diagonal = separated_segment_hull_closest([2.2, 0.0, 0.0], [0.0, 2.2, 0.0], &hull)
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
    let bytes = world.checkpoint(context).unwrap().to_bytes();
    let restored = Checkpoint::from_bytes(&bytes, context, CheckpointLimits::default())
        .unwrap()
        .restore(&[])
        .unwrap();
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
