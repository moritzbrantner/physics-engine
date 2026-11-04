//! A CCD body that starts a step inside the contact margin of a surface it is closing on must
//! still consume the whole step: impact at its time of impact, then the remaining time along
//! the response velocity. A speculative row would stop it at the surface and bounce it only
//! after the step, losing the rest of that step's travel (found by the #465 experiment).
use physics_engine::approximate::{Body, BodyStorage, Config, Shape, Vector as V, World};

const DT: f64 = 1.0 / 60.0;
const SPEED: f32 = 30.0;
const RADIUS: f32 = 0.196;
/// Wall faces at `±FACE`, walls `2 * HALF` thick.
const FACE: f32 = 0.4;
const HALF: f32 = 0.05;

fn world(storage: BodyStorage, walls: &[f32]) -> World {
    let mut world = World::new(Config {
        body_storage: storage,
        gravity: V::ZERO,
        substeps: 1,
        velocity_iterations: 8,
        ..Config::default()
    })
    .unwrap();
    for &side in walls {
        let mut wall = Body::new(
            Shape::Box(V(HALF, 2.0, 8.0)),
            V(side * (FACE + HALF), 0.0, 3.0),
            0.0,
        );
        wall.restitution = 1.0;
        wall.friction = 0.0;
        world.add_body(wall).unwrap();
    }
    world
}

fn ball(x: f32, z: f32, ccd: bool) -> Body {
    let mut b = Body::new(Shape::Sphere(RADIUS), V(x, 0.0, z), 1.0);
    b.velocity = V(SPEED, 0.0, 0.0);
    b.restitution = 1.0;
    b.friction = 0.0;
    b.ccd = ccd;
    b.sleep_allowed = false;
    b
}

fn one_step(gap: f32, ccd: bool) -> (V, V) {
    let mut world = world(BodyStorage::Ordered, &[1.0]);
    let id = world.add_body(ball(FACE - RADIUS - gap, 0.0, ccd)).unwrap();
    world.step(DT).unwrap();
    let b = world.body(id).unwrap();
    (b.position, b.velocity)
}

#[test]
fn ccd_body_inside_the_margin_keeps_its_remaining_travel() {
    // 0.1 and 0.02 start outside the contact margin; the rest start inside it.
    for gap in [0.1_f32, 0.02, 0.003, 0.0005, 0.0] {
        let (p, v) = one_step(gap, true);
        let expected = FACE - RADIUS - SPEED * (DT as f32 - gap / SPEED);
        assert!((v.0 + SPEED).abs() < 0.01, "gap {gap}: velocity {v:?}");
        assert!(
            (p.0 - expected).abs() < 1e-3,
            "gap {gap}: x {} expected {expected}",
            p.0
        );
    }
}

#[test]
fn non_ccd_body_inside_the_margin_matches_single_step_rapier() {
    // Rapier 0.36.1, one 60 Hz step from the same pose, CCD off: the speculative contact stops
    // the sphere at the wall (x = 0.254) and reverses its velocity for the next step.
    let (p, v) = one_step(0.003, false);
    assert!((p.0 - 0.204).abs() < 1e-3, "x {}", p.0);
    assert!((v.0 + SPEED).abs() < 0.01, "velocity {v:?}");
}

/// Elastic corridor trajectory: unfold the constant-speed path and fold it back between the
/// centre's limits `±(FACE - RADIUS)`.
fn corridor(start: f32, t: f64) -> (f32, f32) {
    let limit = f64::from(FACE - RADIUS);
    let unfolded = (f64::from(start) + limit + f64::from(SPEED) * t).rem_euclid(4.0 * limit);
    if unfolded <= 2.0 * limit {
        ((unfolded - limit) as f32, SPEED)
    } else {
        ((3.0 * limit - unfolded) as f32, -SPEED)
    }
}

#[test]
fn ccd_corridor_keeps_every_bounce_on_both_storages() {
    // Bouncing between two walls, every lane keeps starting ticks within millimetres of a wall.
    for storage in [BodyStorage::Ordered, BodyStorage::Indexed] {
        let mut world = world(storage, &[-1.0, 1.0]);
        let gaps = [0.1_f32, 0.02, 0.003, 0.0005];
        let ids: Vec<_> = gaps
            .iter()
            .enumerate()
            .map(|(k, &gap)| {
                world
                    .add_body(ball(FACE - RADIUS - gap, 2.0 * k as f32, true))
                    .unwrap()
            })
            .collect();
        for tick in 1..=60 {
            world.step(DT).unwrap();
            for (&id, &gap) in ids.iter().zip(&gaps) {
                let b = world.body(id).unwrap();
                let (x, vx) = corridor(FACE - RADIUS - gap, DT * f64::from(tick));
                assert!(
                    (b.position.0 - x).abs() < 0.01 && (b.velocity.0 - vx).abs() < 0.1,
                    "{storage:?} gap {gap} tick {tick}: {:?}/{:?}, analytic {x}/{vx}",
                    b.position,
                    b.velocity
                );
            }
        }
    }
}
