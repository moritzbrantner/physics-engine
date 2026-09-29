use std::error::Error;

use physics_engine::{
    BodyId,
    approximate::{Body, Config, Shape, Vector, World},
};

fn main() -> Result<(), Box<dyn Error>> {
    let mut world = World::new(Config {
        gravity: Vector::ZERO,
        substeps: 1,
        velocity_iterations: 1,
        ..Config::default()
    })?;
    let id = BodyId(1);
    let mut body = Body::new(id, Shape::Box(Vector(1.0, 1.0, 1.0)), Vector::ZERO, 1.0);
    body.velocity = Vector(60.0, 0.0, 0.0);

    world.add_body(body)?;
    world.step(1.0 / 60.0)?;

    let position = world.body(id).expect("body remains").position;
    assert!((position.0 - 1.0).abs() < 1e-12);
    Ok(())
}
