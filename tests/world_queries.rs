use physics_engine::{
    Aabb, BodyId, ContactNormal, QueryError, Ray, RigidBody, SUBTICKS_PER_TICK, Vec3i, World,
};

fn fixed(id: u64, center: Vec3i, half_extents: Vec3i) -> RigidBody {
    RigidBody::fixed(BodyId(id), center, half_extents)
}

#[test]
fn sparse_snapshot_queries_preserve_order_across_unrelated_world_scales() {
    // Query-result correctness is a prerequisite for replacing the current O(n)
    // scans with indexed candidates. Work counts still need instrumentation.
    for unrelated in [0_u64, 32, 128, 256] {
        let mut world = World::default();
        world
            .add_body(fixed(1, Vec3i::new(10, 0, 0), Vec3i::new(1, 1, 1)))
            .unwrap();
        world
            .add_body(fixed(2, Vec3i::new(20, 0, 0), Vec3i::new(1, 1, 1)))
            .unwrap();

        for offset in 0..unrelated {
            let x = 10_000 + i32::try_from(offset).unwrap() * 16;
            world
                .add_body(fixed(
                    100 + offset,
                    Vec3i::new(x, 0, 0),
                    Vec3i::new(1, 1, 1),
                ))
                .unwrap();
        }

        let ray = Ray::new(Vec3i::ZERO, Vec3i::new(10, 0, 0));
        let hits = world.ray_cast(ray, 4).unwrap();
        assert_eq!(
            hits.iter().map(|hit| hit.body).collect::<Vec<_>>(),
            vec![BodyId(1), BodyId(2)],
            "unrelated population = {unrelated}"
        );
        assert_eq!(world.ray_cast_first(ray, 4).unwrap(), hits.first().copied());
        assert_eq!(
            world
                .cast_aabb(Aabb::new(Vec3i::ZERO, Vec3i::ZERO), ray.direction, 4)
                .unwrap(),
            hits
        );
        assert_eq!(
            world
                .overlap_query(Aabb::new(Vec3i::new(10, 0, 0), Vec3i::ZERO))
                .unwrap(),
            vec![BodyId(1)]
        );

        world.set_position(BodyId(1), Vec3i::new(14, 0, 0)).unwrap();
        assert_eq!(
            world
                .overlap_query(Aabb::new(Vec3i::new(10, 0, 0), Vec3i::ZERO))
                .unwrap(),
            Vec::<BodyId>::new()
        );
        assert_eq!(world.ray_cast(ray, 4).unwrap().len(), 2);
        assert_eq!(
            world.ray_cast_first(ray, 4).unwrap().unwrap().body,
            BodyId(1)
        );
    }
}

#[test]
fn overlap_query_returns_stable_body_id_order() {
    let mut world = World::default();
    world
        .add_body(fixed(3, Vec3i::new(2, 0, 0), Vec3i::new(2, 2, 2)))
        .unwrap();
    world
        .add_body(fixed(1, Vec3i::new(-2, 0, 0), Vec3i::new(2, 2, 2)))
        .unwrap();
    world
        .add_body(fixed(2, Vec3i::new(20, 0, 0), Vec3i::new(2, 2, 2)))
        .unwrap();

    let hits = world
        .overlap_query(Aabb::new(Vec3i::ZERO, Vec3i::new(1, 1, 1)))
        .unwrap();

    assert_eq!(hits, vec![BodyId(1), BodyId(3)]);
}

#[test]
fn ray_cast_returns_nearest_hit_first() {
    let mut world = World::default();
    world
        .add_body(fixed(2, Vec3i::new(18, 0, 0), Vec3i::new(1, 2, 2)))
        .unwrap();
    world
        .add_body(fixed(1, Vec3i::new(8, 0, 0), Vec3i::new(1, 2, 2)))
        .unwrap();

    let hits = world
        .ray_cast(Ray::new(Vec3i::ZERO, Vec3i::new(10, 0, 0)), 2)
        .unwrap();

    assert_eq!(hits.len(), 2);
    assert_eq!(hits[0].body, BodyId(1));
    assert_eq!(hits[0].time.subticks(), 7 * SUBTICKS_PER_TICK / 10 + 1);
    assert_eq!(hits[0].normal, Some(ContactNormal { x: -1, y: 0, z: 0 }));
    assert_eq!(hits[1].body, BodyId(2));
}

#[test]
fn ray_cast_first_matches_full_order_for_time_and_body_id_ties() {
    let mut world = World::default();
    world
        .add_body(fixed(9, Vec3i::new(8, 0, 0), Vec3i::new(1, 2, 2)))
        .unwrap();
    world
        .add_body(fixed(3, Vec3i::new(8, 0, 0), Vec3i::new(1, 2, 2)))
        .unwrap();
    world
        .add_body(fixed(1, Vec3i::new(18, 0, 0), Vec3i::new(1, 2, 2)))
        .unwrap();

    let ray = Ray::new(Vec3i::ZERO, Vec3i::new(10, 0, 0));
    let first = world.ray_cast_first(ray, 2).unwrap();
    let full = world.ray_cast(ray, 2).unwrap();

    assert_eq!(first, full.first().copied());
    assert_eq!(first.unwrap().body, BodyId(3));
}

#[test]
fn ray_cast_first_reports_initial_overlap_without_inventing_a_normal() {
    let mut world = World::default();
    world
        .add_body(fixed(7, Vec3i::ZERO, Vec3i::new(5, 5, 5)))
        .unwrap();

    let hit = world
        .ray_cast_first(Ray::new(Vec3i::ZERO, Vec3i::new(100, 0, 0)), 1)
        .unwrap()
        .unwrap();

    assert_eq!(hit.body, BodyId(7));
    assert_eq!(hit.time.subticks(), 0);
    assert_eq!(hit.normal, None);
}

#[test]
fn aabb_cast_uses_snapshot_geometry_not_body_velocity() {
    let mut world = World::default();
    world
        .add_body(RigidBody::dynamic(
            BodyId(9),
            Vec3i::new(10, 0, 0),
            Vec3i::new(1_000, 0, 0),
            Vec3i::new(1, 1, 1),
        ))
        .unwrap();

    let hits = world
        .cast_aabb(
            Aabb::new(Vec3i::ZERO, Vec3i::new(1, 1, 1)),
            Vec3i::new(20, 0, 0),
            1,
        )
        .unwrap();

    assert_eq!(hits.len(), 1);
    assert_eq!(hits[0].body, BodyId(9));
    assert_eq!(hits[0].time.subticks(), 2 * SUBTICKS_PER_TICK / 5 + 1);
}

#[test]
fn query_rejects_invalid_geometry_and_time() {
    let world = World::default();

    assert_eq!(
        world.overlap_query(Aabb::new(Vec3i::ZERO, Vec3i::new(-1, 1, 1))),
        Err(QueryError::InvalidHalfExtents)
    );
    assert_eq!(
        world.ray_cast(Ray::new(Vec3i::ZERO, Vec3i::new(1, 0, 0)), 0),
        Err(QueryError::NonPositiveTicks(0))
    );
}

#[test]
fn borrowed_ray_query_matches_exhaustive_order_without_world_reconstruction() {
    let bodies = [
        fixed(9, Vec3i::new(8, 0, 0), Vec3i::new(1, 2, 2)),
        fixed(3, Vec3i::new(8, 0, 0), Vec3i::new(1, 2, 2)),
        fixed(1, Vec3i::new(18, 0, 0), Vec3i::new(1, 2, 2)),
    ];
    let mut world = World::default();
    for body in bodies.iter().cloned() {
        world.add_body(body).unwrap();
    }
    for (origin, direction, ticks) in [
        (Vec3i::ZERO, Vec3i::new(10, 0, 0), 2),
        (Vec3i::new(8, 0, 0), Vec3i::ZERO, 1),
        (Vec3i::ZERO, Vec3i::new(1, 0, 0), 1),
        (Vec3i::new(0, 10, 0), Vec3i::new(10, 0, 0), 2),
    ] {
        let ray = Ray::new(origin, direction);
        let expected = world.ray_cast(ray, ticks).unwrap().first().copied();
        assert_eq!(
            physics_engine::ray_cast_first(bodies.iter(), ray, ticks).unwrap(),
            expected
        );
        assert_eq!(
            physics_engine::ray_cast_first(bodies.iter().rev(), ray, ticks).unwrap(),
            expected
        );
    }
    assert_eq!(
        physics_engine::ray_cast_first(bodies.iter(), Ray::new(Vec3i::ZERO, Vec3i::ZERO), 0),
        Err(QueryError::NonPositiveTicks(0))
    );
}
