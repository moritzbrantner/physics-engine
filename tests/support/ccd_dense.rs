//! Shared exact first-tick fixture for dense timing controls and work contracts.

use physics_engine::approximate::{Body, BodyStorage, Config, Shape, Vector as V, World};

pub const H: f64 = 1.0 / 60.0;
const RADIUS: f32 = 0.1;
pub const SEPARATION: f32 = 0.3;
pub const IRRELEVANT_STATICS: usize = 128;

#[derive(Clone, Copy)]
pub enum Case {
    #[allow(dead_code)] // The work contract uses convergence; the timing control uses co-motion.
    CoMoving,
    Converging,
}

pub fn world(case: Case, count: usize, storage: BodyStorage) -> World {
    let mut w = World::new(Config {
        gravity: V::ZERO,
        substeps: 1,
        velocity_iterations: 8,
        body_storage: storage,
        ..Config::default()
    })
    .unwrap();

    // Match the sparse CCD baseline's unrelated-static count. Nothing in this
    // distant row can intersect a moving body's swept bounds.
    for i in 0..IRRELEVANT_STATICS {
        w.add_body(Body::keyed(
            2 + i as u64,
            Shape::Box(V(0.2, 0.2, 0.2)),
            V(10_000.0 + i as f32, 0.0, 0.0),
            0.0,
        ))
        .unwrap();
    }

    // Co-motion: every sphere starts separated, and all relative velocities
    // are exactly zero. A sufficiently long common displacement makes every
    // pair's *swept AABBs* overlap despite zero possible physical impacts.
    // Convergence: initially separated spheres aim through the same center in
    // one tick, forcing real, dense dynamic/dynamic CCD interactions.
    let ring_radius = 0.75 + count as f32 * 0.04;
    for i in 0..count {
        let (position, velocity) = match case {
            Case::CoMoving => (
                V(i as f32 * SEPARATION, 0.0, 0.0),
                V((count as f32 * SEPARATION + 1.0) * 60.0, 0.0, 0.0),
            ),
            Case::Converging => {
                let angle = std::f32::consts::TAU * i as f32 / count as f32;
                let (sin, cos) = angle.sin_cos();
                let speed = (ring_radius + 0.5) * 60.0;
                (
                    V(ring_radius * cos, ring_radius * sin, 0.0),
                    V(-speed * cos, -speed * sin, 0.0),
                )
            }
        };
        let mut body = Body::keyed(10_000 + i as u64, Shape::Sphere(RADIUS), position, 1.0);
        body.velocity = velocity;
        body.restitution = 1.0;
        body.friction = 0.0;
        body.ccd = true;
        body.sleep_allowed = false;
        w.add_body(body).unwrap();
    }
    w
}
