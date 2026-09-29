use physics_engine::{BodyId, PhysicsError, RigidBody, Vec3i, World, WorldConfig};

fn main() -> Result<(), PhysicsError> {
    let mut world = World::new(WorldConfig {
        gravity: Vec3i::ZERO,
        ..WorldConfig::default()
    });
    let id = BodyId(1);

    world.add_body(RigidBody::dynamic(
        id,
        Vec3i::ZERO,
        Vec3i::new(1, 0, 0),
        Vec3i::new(1, 1, 1),
    ))?;
    world.step(1)?;

    assert_eq!(world.body(id).expect("body remains").position(), Vec3i::new(1, 0, 0));
    Ok(())
}
