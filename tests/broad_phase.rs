use physics_engine::{BodyId, RigidBody, Vec3i, World, WorldConfig};

#[test]
fn swept_broad_phase_preserves_fast_collision_detection() {
    let mut world = World::new(WorldConfig {
        gravity: Vec3i::ZERO,
        ..WorldConfig::default()
    });
    world
        .add_body(RigidBody::dynamic(
            BodyId(1),
            Vec3i::new(-100, 0, 0),
            Vec3i::new(250, 0, 0),
            Vec3i::new(1, 1, 1),
        ))
        .unwrap();
    world
        .add_body(RigidBody::fixed(
            BodyId(2),
            Vec3i::ZERO,
            Vec3i::new(1, 10, 10),
        ))
        .unwrap();

    for index in 0..64_u64 {
        world
            .add_body(RigidBody::fixed(
                BodyId(100 + index),
                Vec3i::new(10_000 + i32::try_from(index).unwrap() * 100, 10_000, 0),
                Vec3i::new(1, 1, 1),
            ))
            .unwrap();
    }

    let report = world.step(1).unwrap();

    assert_eq!(report.events.len(), 1);
    assert_eq!(report.events[0].left, BodyId(1));
    assert_eq!(report.events[0].right, BodyId(2));
    assert_eq!(world.body(BodyId(1)).unwrap().position().x, -2);
    assert_eq!(report.stats.toi_tests, 1);
    assert!(report.stats.pair_checks < 8);
}

#[test]
fn broad_phase_result_is_independent_of_insertion_order() {
    let run = |reverse: bool| {
        let mut world = World::new(WorldConfig {
            gravity: Vec3i::ZERO,
            ..WorldConfig::default()
        });
        let bodies = [
            RigidBody::dynamic(
                BodyId(1),
                Vec3i::new(-20, 0, 0),
                Vec3i::new(30, 0, 0),
                Vec3i::new(1, 1, 1),
            ),
            RigidBody::fixed(BodyId(2), Vec3i::ZERO, Vec3i::new(1, 5, 5)),
            RigidBody::dynamic(
                BodyId(3),
                Vec3i::new(20, 0, 0),
                Vec3i::new(-30, 0, 0),
                Vec3i::new(1, 1, 1),
            ),
        ];

        if reverse {
            for body in bodies.into_iter().rev() {
                world.add_body(body).unwrap();
            }
        } else {
            for body in bodies {
                world.add_body(body).unwrap();
            }
        }

        let report = world.step(1).unwrap();
        (world.bodies().cloned().collect::<Vec<_>>(), report.events)
    };

    assert_eq!(run(false), run(true));
}
