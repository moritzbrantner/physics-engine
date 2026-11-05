//! The #180 consumer slice: true convex rock queries, not bounding-box proxies.
use physics_engine::approximate::{
    Body, CharacterMove, Config, ConvexHullDesc, QueryFilter, QueryPose, RayCast, RayFeature,
    Shape, ShapeCast, Vector as V, World,
};

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
