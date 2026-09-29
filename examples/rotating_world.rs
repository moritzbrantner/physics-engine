use std::error::Error;

use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d,
    RotatingWorld3d, RotatingWorldConfig3d, Vec3i,
};

fn main() -> Result<(), Box<dyn Error>> {
    let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
        gravity: Vec3i::ZERO,
        ..RotatingWorldConfig3d::default()
    });
    let id = BodyId(1);
    let body = RigidBox3d::new(
        RigidBody::dynamic(
            id,
            Vec3i::ZERO,
            Vec3i::new(60, 0, 0),
            Vec3i::new(1, 1, 1),
        ),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )?;

    world.add_box(body)?;
    world.step(1, 60)?;

    assert_eq!(
        world
            .box_by_id(id)
            .expect("body remains")
            .body()
            .position(),
        Vec3i::new(1, 0, 0)
    );
    Ok(())
}
