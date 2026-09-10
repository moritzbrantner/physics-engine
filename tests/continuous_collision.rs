use physics_engine::{
    BodyId, ContactNormal, Material, RigidBody, SUBTICKS_PER_TICK, Vec3i, World, WorldConfig,
    swept_aabb,
};

fn body(id: u64, position: Vec3i, velocity: Vec3i) -> RigidBody {
    RigidBody::dynamic(BodyId(id), position, velocity, Vec3i::new(1, 1, 1))
}

#[test]
fn fast_body_does_not_tunnel_through_thin_wall() {
    let mut world = World::new(WorldConfig {
        gravity: Vec3i::ZERO,
        ..WorldConfig::default()
    });
    world
        .add_body(body(1, Vec3i::new(-10, 0, 0), Vec3i::new(40, 0, 0)))
        .unwrap();
    world
        .add_body(RigidBody::fixed(
            BodyId(2),
            Vec3i::ZERO,
            Vec3i::new(1, 8, 8),
        ))
        .unwrap();

    let report = world.step(1).unwrap();

    assert_eq!(world.body(BodyId(1)).unwrap().position().x, -2);
    assert_eq!(world.body(BodyId(1)).unwrap().velocity().x, 0);
    assert_eq!(report.events.len(), 1);
    assert_eq!(report.events[0].normal, ContactNormal { x: -1, y: 0, z: 0 });
}

#[test]
fn equal_mass_elastic_collision_exchanges_axis_velocity() {
    let elastic = Material::new(1_000);
    let mut world = World::new(WorldConfig {
        gravity: Vec3i::ZERO,
        ..WorldConfig::default()
    });
    world
        .add_body(
            body(1, Vec3i::new(-5, 0, 0), Vec3i::new(10, 0, 0)).with_material(elastic),
        )
        .unwrap();
    world
        .add_body(
            body(2, Vec3i::new(5, 0, 0), Vec3i::new(-10, 0, 0)).with_material(elastic),
        )
        .unwrap();

    world.step(1).unwrap();

    assert_eq!(world.body(BodyId(1)).unwrap().velocity().x, -10);
    assert_eq!(world.body(BodyId(2)).unwrap().velocity().x, 10);
    assert_eq!(world.body(BodyId(1)).unwrap().position().x, -7);
    assert_eq!(world.body(BodyId(2)).unwrap().position().x, 7);
}

#[test]
fn gravity_uses_continuous_floor_impact() {
    let mut world = World::new(WorldConfig {
        gravity: Vec3i::new(0, -10, 0),
        ..WorldConfig::default()
    });
    world
        .add_body(body(1, Vec3i::new(0, 10, 0), Vec3i::ZERO))
        .unwrap();
    world
        .add_body(RigidBody::fixed(
            BodyId(2),
            Vec3i::ZERO,
            Vec3i::new(20, 1, 20),
        ))
        .unwrap();

    world.step(1).unwrap();

    assert_eq!(world.body(BodyId(1)).unwrap().position().y, 2);
    assert_eq!(world.body(BodyId(1)).unwrap().velocity().y, 0);
}

#[test]
fn sweep_reports_quantized_time_of_impact() {
    let moving = body(1, Vec3i::new(-10, 0, 0), Vec3i::new(40, 0, 0));
    let wall = RigidBody::fixed(BodyId(2), Vec3i::ZERO, Vec3i::new(1, 8, 8));

    let hit = swept_aabb(&moving, &wall, 1).unwrap();

    assert_eq!(hit.time.subticks(), SUBTICKS_PER_TICK / 5 + 1);
    assert_eq!(hit.normal, ContactNormal { x: -1, y: 0, z: 0 });
}

#[test]
fn insertion_order_does_not_change_the_answer() {
    let build = |reverse: bool| {
        let mut world = World::new(WorldConfig {
            gravity: Vec3i::ZERO,
            ..WorldConfig::default()
        });
        let bodies = [
            body(1, Vec3i::new(-10, 0, 0), Vec3i::new(40, 0, 0)),
            RigidBody::fixed(BodyId(2), Vec3i::ZERO, Vec3i::new(1, 8, 8)),
            body(3, Vec3i::new(15, 4, 0), Vec3i::new(-2, 0, 0)),
        ];
        if reverse {
            for entry in bodies.into_iter().rev() {
                world.add_body(entry).unwrap();
            }
        } else {
            for entry in bodies {
                world.add_body(entry).unwrap();
            }
        }
        world.step(1).unwrap();
        world.bodies().cloned().collect::<Vec<_>>()
    };

    assert_eq!(build(false), build(true));
}
