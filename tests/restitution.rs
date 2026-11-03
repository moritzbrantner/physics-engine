//! #458: a new contact bounces once the step is solved, speculative ones included, as Rapier's
//! end-of-step restitution pass: a fast sphere leaves a wall at `e` times its speed from the wall
//! face instead of stopping dead (Rapier 0.36.1: exactly 0.8 × at every speed below).

use physics_engine::approximate::{Body, Config, Shape, Vector, World};

fn throw(speed: f32, substeps: u8, restitution: f32) -> (f32, f32) {
    let mut w = World::new(Config {
        gravity: Vector::ZERO,
        substeps,
        velocity_iterations: 4,
        ..Config::default()
    })
    .unwrap();
    let mut wall = Body::new(
        Shape::Box(Vector(0.5, 2.0, 2.0)),
        Vector(3.0, 0.0, 0.0),
        0.0,
    );
    wall.restitution = restitution;
    w.add_body(wall).unwrap();
    let mut ball = Body::new(Shape::Sphere(0.1), Vector::ZERO, 1.0);
    ball.velocity = Vector(speed, 0.0, 0.0);
    ball.restitution = restitution;
    let ball = w.add_body(ball).unwrap();
    let mut furthest: f32 = 0.0;
    for _ in 0..180 {
        w.step(1.0 / 60.0).unwrap();
        furthest = furthest.max(w.body(ball).unwrap().position.0);
    }
    (w.body(ball).unwrap().velocity.0, furthest)
}

#[test]
fn fast_bodies_bounce_off_the_surface() {
    for speed in [2.0, 5.0, 10.0, 20.0, 40.0] {
        for substeps in [1, 2] {
            let (v, furthest) = throw(speed, substeps, 0.8);
            assert!(
                (v + 0.8 * speed).abs() < 1e-3 * speed,
                "{speed} m/s, {substeps} substeps: {v}"
            );
            // Face at 2.5, radius 0.1: it reaches the surface, never past it.
            assert!((furthest - 2.4).abs() < 0.005, "{speed} m/s: {furthest}");
        }
    }
}

#[test]
fn slow_and_inelastic_contacts_do_not_bounce() {
    // At or below the 1 m/s threshold, and at zero restitution, the body stops at the wall.
    assert_eq!(throw(0.9, 2, 0.8).0, 0.0);
    assert_eq!(throw(20.0, 2, 0.0).0, 0.0);
}
