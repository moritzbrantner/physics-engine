use super::{BodyStateChange3d, RotatingWorld3d, RotatingWorldConfig3d, RotatingWorldError3d};
use crate::{
    AngularState3d, AngularVelocity3d, BodyId, BodyKind, Orientation3d, RigidBody, RigidBox3d,
    Vec3i,
};

fn dynamic(id: u64, x: i32) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::dynamic(
            BodyId(id),
            Vec3i::new(x, 0, 0),
            Vec3i::ZERO,
            Vec3i::new(1, 1, 1),
        ),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .unwrap()
}

fn world() -> RotatingWorld3d {
    RotatingWorld3d::new(RotatingWorldConfig3d {
        gravity: Vec3i::ZERO,
        sample_count: 8,
        refinement_steps: 2,
        solver_passes: 4,
        max_events: 8,
    })
}

#[test]
fn lifecycle_deltas_keep_warm_contacts_equal_to_a_fresh_world() {
    let mut world = world();
    for (id, x) in [(1, 0), (2, 2), (3, 20), (4, 22), (5, 40)] {
        let mut body = dynamic(id, x);
        if id == 2 || id == 4 {
            body.body.kind = BodyKind::Fixed;
        }
        if id == 3 {
            body = body.with_external_motion();
        }
        world.add_box(body).unwrap();
    }
    // Warm the graph and the fixed-subject cache. A fresh world below is the cold-cache oracle.
    let shape = world.box_by_id(BodyId(1)).unwrap().oriented_box();
    assert_eq!(
        world.overlap_query(shape).unwrap(),
        vec![BodyId(1), BodyId(2)]
    );
    for id in 1..=5 {
        world.body_contacts(BodyId(id)).unwrap();
    }
    let graph_builds = world.current_contact_cache_stats().0;
    for (id, change) in [
        (1, BodyStateChange3d::SleepProxy(BodyKind::Fixed)),
        (1, BodyStateChange3d::SleepProxy(BodyKind::Dynamic)),
        (1, BodyStateChange3d::Position(Vec3i::new(38, 0, 0))),
        (3, BodyStateChange3d::Position(Vec3i::new(21, 0, 0))),
        (1, BodyStateChange3d::SleepProxy(BodyKind::Fixed)),
        (5, BodyStateChange3d::SleepProxy(BodyKind::Fixed)),
        (5, BodyStateChange3d::SleepProxy(BodyKind::Dynamic)),
    ] {
        world.apply_body_change(BodyId(id), change).unwrap();
        let mut fresh = RotatingWorld3d::new(world.config());
        for body in world.boxes() {
            fresh.add_box(body.clone()).unwrap();
        }
        for body in world.boxes() {
            let id = body.body().id();
            assert_eq!(
                world.body_contacts(id).unwrap(),
                fresh.body_contacts(id).unwrap(),
                "body {id:?}"
            );
            if body.body().kind() == BodyKind::Dynamic {
                assert_eq!(
                    world.overlap_query(body.oriented_box()).unwrap(),
                    fresh.overlap_query(body.oriented_box()).unwrap()
                );
            }
        }
        assert_eq!(world.current_contact_cache_stats().0, graph_builds);
    }
}

#[test]
fn stopping_motion_preserves_geometry_caches_and_other_body_state() {
    let mut world = world();
    let mut moving = dynamic(1, 0);
    moving.body.velocity = Vec3i::new(10, 20, 30);
    moving.angular.angular_velocity = AngularVelocity3d::new(3, 2, 1);
    world.add_box(moving.clone()).unwrap();
    world.add_box(dynamic(2, 2)).unwrap();
    let contacts = world.body_contacts(BodyId(1)).unwrap();
    let overlaps = world.overlap_query(moving.oriented_box()).unwrap();
    let before = world.current_contact_cache_stats();
    world
        .apply_body_change(BodyId(1), BodyStateChange3d::StopMotion)
        .unwrap();
    moving.body.velocity = Vec3i::ZERO;
    moving.angular.angular_velocity = AngularVelocity3d::default();
    assert_eq!(world.box_by_id(BodyId(1)), Some(&moving));
    assert_eq!(world.body_contacts(BodyId(1)).unwrap(), contacts);
    assert_eq!(
        world.overlap_query(moving.oriented_box()).unwrap(),
        overlaps
    );
    assert_eq!(world.current_contact_cache_stats(), before);
    assert_eq!(world.current_contact_incremental_stats(), (0, 0, 0));
}

#[test]
fn missing_lifecycle_target_does_not_mutate_the_world() {
    let mut world = world();
    let body = dynamic(1, 0);
    world.add_box(body.clone()).unwrap();
    let before = world.current_contact_cache_stats();
    assert_eq!(
        world.apply_body_change(BodyId(99), BodyStateChange3d::SleepProxy(BodyKind::Fixed)),
        Err(RotatingWorldError3d::MissingBody(BodyId(99)))
    );
    assert_eq!(world.box_by_id(BodyId(1)), Some(&body));
    assert_eq!(world.current_contact_cache_stats(), before);
}
