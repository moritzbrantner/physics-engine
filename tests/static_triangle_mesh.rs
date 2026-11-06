use physics_engine::approximate::{
    Body, BodyStorage, CharacterMove, Config, QueryFilter, QueryPose, RayCast, RayFeature, Shape,
    ShapeCast, Vector as V, World,
};
#[path = "support/redbrand_mesh.rs"]
mod redbrand;

fn cave(storage: BodyStorage) -> World {
    let mut world = World::new(Config {
        gravity: V::ZERO,
        body_storage: storage,
        ..Config::default()
    })
    .unwrap();
    for (key, desc) in redbrand::chunks() {
        world.add_triangle_mesh(desc, Some(key)).unwrap();
    }
    world
}
#[test]
fn authored_cave_rays_hit_floor_roof_shelf_and_leave_the_mouth_open() {
    for storage in [BodyStorage::Ordered, BodyStorage::Indexed] {
        let w = cave(storage);
        let filter = QueryFilter::default();
        let floor = w
            .cast_ray_first(
                RayCast::new(V(62.0, 12.0, -138.0), V(0.0, -2.0, 0.0)),
                filter,
            )
            .unwrap()
            .unwrap();
        assert_eq!(floor.body, w.body_id(640).unwrap());
        assert!((floor.point.1 - 11.0).abs() < 1e-4);
        assert!(floor.normal.1 > 0.99);
        assert!(matches!(floor.feature, RayFeature::TriangleMeshTriangle(_)));
        let ceiling = w
            .cast_ray_first(
                RayCast::new(V(62.0, 12.0, -138.0), V(0.0, 5.0, 0.0)),
                filter,
            )
            .unwrap()
            .unwrap();
        assert_eq!(ceiling.body, w.body_id(547).unwrap());
        assert!((ceiling.point.1 - 15.5).abs() < 1e-4);
        assert!(ceiling.normal.1 < -0.99);
        let shelf = w
            .cast_ray_first(
                RayCast::new(V(62.0, 14.0, -143.0), V(0.0, -2.0, 0.0)),
                filter,
            )
            .unwrap()
            .unwrap();
        assert_eq!(shelf.body, w.body_id(645).unwrap());
        assert!((shelf.point.1 - 13.2).abs() < 1e-4);
        assert!(
            w.cast_ray_first(
                RayCast::new(V(58.0, 12.0, -127.0), V(0.0, 0.0, -10.0)),
                filter
            )
            .unwrap()
            .is_none()
        );
    }
}
#[test]
fn actual_cave_capsule_uses_both_heights_and_ceiling_without_phantom_solids() {
    let w = cave(BodyStorage::Ordered);
    let pose = QueryPose::new(Shape::capsule(0.6, 0.3), V(62.0, 11.9, -140.0));
    let mut hits = Vec::new();
    w.cast_shape(
        ShapeCast::new(pose, V(0.0, 0.0, -3.3)),
        QueryFilter::default(),
        &mut hits,
    )
    .unwrap();
    assert!(
        hits.iter().all(|h| h.normal.1 > 0.99),
        "passage under shelf: {hits:?}"
    );
    let pose = QueryPose::new(Shape::capsule(0.6, 0.3), V(62.0, 14.1, -143.0));
    w.cast_shape(
        ShapeCast::new(pose, V(0.0, 2.0, 0.0)),
        QueryFilter::default(),
        &mut hits,
    )
    .unwrap();
    let roof = hits
        .iter()
        .find(|h| h.body == w.body_id(547).unwrap())
        .unwrap();
    assert!((roof.fraction - 0.25).abs() < 0.002, "{roof:?}");
    assert!(roof.normal.1 < -0.99);
    assert!(matches!(
        roof.feature,
        Some(RayFeature::TriangleMeshTriangle(_))
    ));
}
#[test]
fn real_player_capsule_walks_the_authored_entrance_floor_seam() {
    let mut results = Vec::new();
    for storage in [BodyStorage::Ordered, BodyStorage::Indexed] {
        let mut w = cave(storage);
        let mut player = Body::keyed(1, Shape::capsule(0.6, 0.3), V(58.0, 11.9025, -128.0), 1.0);
        player.external = true;
        let id = w.add_body(player).unwrap();
        w.add_character(id, w.character_settings()).unwrap();
        let mut out = Vec::new();
        for _ in 0..60 {
            w.move_characters(
                &[CharacterMove {
                    body: id,
                    desired: V(0.0, -0.05, -0.1),
                    exclude: None,
                    autostep: None,
                }],
                &mut out,
            )
            .unwrap();
            assert!(out[0].grounded, "{out:?}");
            let p = w.body(id).unwrap().position;
            assert!((p.1 - 11.9025).abs() < 0.01, "{p:?}");
        }
        let p = w.body(id).unwrap().position;
        assert!(p.2 < -133.8, "floor seam blocked traversal: {p:?}");
        results.push((p, out));
    }
    assert_eq!(results[0], results[1]);
}

#[test]
fn invalid_meshes_fail_atomically_and_chunks_move_remove_and_reuse_without_repreparing() {
    let mut w = cave(BodyStorage::Ordered);
    let floor = w.body_id(640).unwrap();
    let stats = w.triangle_mesh_stats(floor).unwrap();
    assert_eq!(stats.triangles, 12);
    assert!(stats.nodes > 0 && stats.retained_bytes > 0);
    let (key, mut bad) = redbrand::chunks().remove(0);
    bad.triangles[0][0] = u32::MAX;
    let count = w.bodies().count();
    assert!(w.add_triangle_mesh(bad, Some(key + 10000)).is_err());
    assert_eq!(w.bodies().count(), count);
    let original = w.body(floor).unwrap().position;
    w.set_static_position(floor, original + V(0.0, 0.5, 0.0))
        .unwrap();
    assert_eq!(w.triangle_mesh_stats(floor).unwrap(), stats);
    let ray = RayCast::new(V(62.0, 12.0, -138.0), V(0.0, -2.0, 0.0));
    let raised = w
        .cast_ray_first(ray, QueryFilter::default())
        .unwrap()
        .unwrap();
    assert!((raised.point.1 - 11.5).abs() < 1e-4);
    let clone = w.clone();
    w.remove_body(floor).unwrap();
    assert!(
        w.cast_ray_first(ray, QueryFilter::default())
            .unwrap()
            .is_none()
    );
    assert!(
        clone
            .cast_ray_first(ray, QueryFilter::default())
            .unwrap()
            .is_some()
    );
    assert!(w.triangle_mesh_stats(floor).is_err());
}

#[cfg(feature = "checkpoints")]
#[test]
fn checkpoint_keeps_mesh_ids_removed_slots_and_character_continuation() {
    use physics_engine::approximate::{Checkpoint, CheckpointContext, CheckpointLimits};
    let mut w = cave(BodyStorage::Ordered);
    w.remove_body(w.body_id(545).unwrap()).unwrap();
    let mut player = Body::keyed(1, Shape::capsule(0.6, 0.3), V(58.0, 11.9025, -128.0), 1.0);
    player.external = true;
    let player = w.add_body(player).unwrap();
    w.add_character(player, w.character_settings()).unwrap();
    let movement = CharacterMove {
        body: player,
        desired: V(0.0, -0.05, -0.1),
        exclude: None,
        autostep: None,
    };
    w.move_characters(&[movement], &mut Vec::new()).unwrap();
    let context = CheckpointContext {
        build: [21; 32],
        content: [182; 32],
    };
    let bytes = w.checkpoint(context).unwrap().to_bytes();
    let mut restored = Checkpoint::from_bytes(&bytes, context, CheckpointLimits::default())
        .unwrap()
        .restore(&[])
        .unwrap();
    let ray = RayCast::new(V(62.0, 12.0, -138.0), V(0.0, 5.0, 0.0));
    assert_eq!(
        w.cast_ray_first(ray, QueryFilter::default()).unwrap(),
        restored
            .cast_ray_first(ray, QueryFilter::default())
            .unwrap()
    );
    assert_eq!(
        w.bodies().cloned().collect::<Vec<_>>(),
        restored.bodies().cloned().collect::<Vec<_>>()
    );
    for _ in 0..50 {
        let (mut original, mut replay) = (Vec::new(), Vec::new());
        w.move_characters(&[movement], &mut original).unwrap();
        restored.move_characters(&[movement], &mut replay).unwrap();
        assert_eq!(original, replay);
        assert_eq!(w.body(player).unwrap(), restored.body(player).unwrap());
    }
    for world in [&mut w, &mut restored] {
        let (_, desc) = redbrand::chunks().remove(0);
        world.add_triangle_mesh(desc, Some(999)).unwrap();
    }
    assert_eq!(
        w.body(w.body_id(999).unwrap()).unwrap(),
        restored.body(restored.body_id(999).unwrap()).unwrap()
    );
}

#[test]
fn supported_fast_sphere_hits_authored_wall_and_uses_the_remaining_interval() {
    for storage in [BodyStorage::Ordered, BodyStorage::Indexed] {
        for key in [1, 10_000] {
            let mut w = cave(storage);
            let mut projectile = Body::keyed(key, Shape::Sphere(0.1), V(58.0, 12.0, -135.0), 1.0);
            projectile.velocity = V(0.0, 0.0, -900.0);
            projectile.restitution = 1.0;
            projectile.friction = 0.0;
            projectile.ccd = true;
            projectile.sleep_allowed = false;
            let id = w.add_body(projectile).unwrap();
            let report = w.step(1.0 / 60.0).unwrap();
            let body = w.body(id).unwrap();
            assert!(body.velocity.2 > 0.0, "wall did not bounce: {body:?}");
            assert!(
                body.position.2 > -143.0,
                "remaining interval was lost: {body:?}"
            );
            assert!(report.swept_contacts > 0);
        }
    }
}

#[test]
fn unsupported_rigid_mesh_pair_is_an_explicit_atomic_error() {
    let mut w = cave(BodyStorage::Ordered);
    let mut body = Body::keyed(1, Shape::capsule(0.6, 0.3), V(58.0, 12.0, -135.0), 1.0);
    body.velocity = V(0.0, 0.0, -900.0);
    body.ccd = true;
    let id = w.add_body(body).unwrap();
    let before = w.body(id).unwrap().clone();
    assert!(matches!(
        w.step(1.0 / 60.0),
        Err(physics_engine::approximate::Error::CollisionSearchFailed {
            reason: physics_engine::approximate::SweepFailure::InvalidGeometryInput,
            ..
        })
    ));
    assert_eq!(w.body(id).unwrap(), &before);
}

#[test]
fn removing_a_chunk_wakes_its_supported_sphere_and_keeps_other_sleepers_parked() {
    let mut w = World::new(Config::default()).unwrap();
    for (key, desc) in redbrand::chunks() {
        w.add_triangle_mesh(desc, Some(key)).unwrap();
    }
    let lower = w
        .add_body(Body::keyed(
            1,
            Shape::Sphere(0.1),
            V(58.0, 11.1, -138.0),
            1.0,
        ))
        .unwrap();
    let upper = w
        .add_body(Body::keyed(
            2,
            Shape::Sphere(0.1),
            V(62.0, 13.3, -143.0),
            1.0,
        ))
        .unwrap();
    for _ in 0..180 {
        w.step(1.0 / 60.0).unwrap();
    }
    assert!(
        w.body(lower).unwrap().is_sleeping(),
        "lower sphere did not settle: {:?}",
        w.body(lower)
    );
    assert!(
        w.body(upper).unwrap().is_sleeping(),
        "upper sphere did not settle: {:?}",
        w.body(upper)
    );
    let upper_position = w.body(upper).unwrap().position;
    w.remove_body(w.body_id(545).unwrap()).unwrap();
    assert!(w.body(lower).unwrap().is_sleeping());
    assert!(w.body(upper).unwrap().is_sleeping());
    w.remove_body(w.body_id(640).unwrap()).unwrap();
    assert!(!w.body(lower).unwrap().is_sleeping());
    assert!(w.body(upper).unwrap().is_sleeping());
    for _ in 0..20 {
        w.step(1.0 / 60.0).unwrap();
    }
    assert!(
        w.body(lower).unwrap().position.1 < 10.9,
        "removed floor still supports sphere"
    );
    assert_eq!(w.body(upper).unwrap().position, upper_position);
    assert!(w.body(upper).unwrap().is_sleeping());
}

#[test]
fn real_cave_mouth_roundtrip_crosses_from_flat_terrace_to_mesh_and_back() {
    use physics_engine::approximate::HeightfieldDesc;
    let mut w = cave(BodyStorage::Ordered);
    // PR #640 cave.rs flattens the mouth to basin::TERRACE_Y (1100 cm). Keep only
    // the outside strip: the cave's authored entrance mesh begins at z = -127.5 m.
    let terrace = w
        .add_heightfield(
            HeightfieldDesc {
                origin: V(56.0, 11.0, -127.5),
                cell_size: (4.0, 2.0),
                columns: 2,
                rows: 2,
                heights: vec![0.0; 4],
                removed_cells: None,
                material: Default::default(),
            },
            Some(50),
        )
        .unwrap();
    let mut body = Body::keyed(1, Shape::capsule(0.6, 0.3), V(58.0, 11.9025, -126.5), 1.0);
    body.external = true;
    let id = w.add_body(body).unwrap();
    w.add_character(id, w.character_settings()).unwrap();
    let mut out = Vec::new();
    let mut saw_terrace = false;
    let mut saw_mesh = false;
    for dz in [-0.1, 0.1] {
        for _ in 0..70 {
            w.move_characters(
                &[CharacterMove {
                    body: id,
                    desired: V(0.0, -0.05, dz),
                    exclude: None,
                    autostep: None,
                }],
                &mut out,
            )
            .unwrap();
            assert!(out[0].grounded);
            assert!((w.body(id).unwrap().position.1 - 11.9025).abs() < 0.01);
            saw_terrace |= out[0].support == Some(terrace);
            saw_mesh |= out[0].support == w.body_id(643) || out[0].support == w.body_id(640);
        }
    }
    assert!(saw_terrace && saw_mesh);
    assert!((w.body(id).unwrap().position.2 + 126.5).abs() < 0.01);
}

#[test]
fn unfiltered_box_occupancy_queries_authored_cave_surfaces() {
    for storage in [BodyStorage::Ordered, BodyStorage::Indexed] {
        let world = cave(storage);
        let mut hits = Vec::new();
        for (position, expected) in [
            (V(62.0, 11.0, -138.0), true),
            (V(62.0, 12.0, -138.0), false),
        ] {
            world
                .overlap_shape(
                    QueryPose::new(Shape::Box(V(0.29, 0.29, 0.03)), position),
                    QueryFilter::default(),
                    &mut hits,
                )
                .unwrap();
            assert_eq!(!hits.is_empty(), expected, "{storage:?} {position:?}");
            assert!(
                hits.windows(2)
                    .all(|pair| pair[0].body.to_bits() < pair[1].body.to_bits())
            );
        }
    }
}
