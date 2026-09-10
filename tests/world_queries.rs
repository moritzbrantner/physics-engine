use physics_engine::{
    Aabb, BodyId, ContactNormal, QueryError, Ray, RigidBody, SUBTICKS_PER_TICK, Vec3i, World,
};

fn fixed(id: u64, center: Vec3i, half_extents: Vec3i) -> RigidBody {
    RigidBody::fixed(BodyId(id), center, half_extents)
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
