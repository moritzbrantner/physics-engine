//! Adverse controls at the edge of the documented f32 precision envelope
//! (docs/numerics.md): positions up to |x| = 16384 units, linear speed 4000 units/s,
//! default gravity and substeps. They run in both builds with the same physical
//! thresholds; the f64 build is the reference.
use physics_engine::{
    BodyId,
    approximate::{Body, Config, Real, Shape, Vector as V, World},
};

/// Envelope edge and the origin control.
const OFFSETS: [Real; 3] = [-16384.0, 0.0, 16384.0];
const CRATE_HALF: Real = 18.0;

fn world(gravity: V) -> World {
    World::new(Config {
        gravity,
        ..Config::default()
    })
    .unwrap()
}

#[test]
fn resting_crate_settles_without_penetration_at_the_envelope_edge_and_origin() {
    let slop = Config::default().contact_slop;
    for offset in OFFSETS {
        let mut w = world(Config::default().gravity);
        let floor = Body::new(
            BodyId(1),
            Shape::Box(V(520.0, 16.0, 520.0)),
            V(offset, -16.0, offset),
            0.0,
        );
        w.add_body(floor).unwrap();
        let crate_ = Body::new(
            BodyId(2),
            Shape::Box(V(CRATE_HALF, CRATE_HALF, CRATE_HALF)),
            V(offset, CRATE_HALF + 2.0, offset),
            2.0,
        );
        w.add_body(crate_).unwrap();
        let mut deepest: Real = 0.0;
        for _ in 0..240 {
            w.step(1.0 / 60.0).unwrap();
            let b = w.body(BodyId(2)).unwrap();
            deepest = deepest.max(CRATE_HALF - b.position.1);
        }
        let b = w.body(BodyId(2)).unwrap();
        assert!(b.is_sleeping(), "offset {offset}: crate must sleep");
        assert!(
            deepest <= slop,
            "offset {offset}: penetration {deepest} exceeds contact_slop {slop}"
        );
        assert!(
            (b.position.0 - offset).abs() <= slop && (b.position.2 - offset).abs() <= slop,
            "offset {offset}: crate drifted to {:?}",
            b.position
        );
        assert!(b.velocity.length() <= Config::default().sleep_speed);
    }
}

#[test]
fn ccd_sphere_at_envelope_speed_does_not_tunnel_a_thin_wall() {
    for offset in OFFSETS {
        // Approach from inside the envelope so every position stays within |x| <= 16384.
        let direction: Real = if offset > 0.0 { 1.0 } else { -1.0 };
        let mut w = world(V::ZERO);
        let wall = Body::new(
            BodyId(1),
            Shape::Box(V(0.25, 5.0, 5.0)),
            V(offset, 0.0, 0.0),
            0.0,
        );
        w.add_body(wall).unwrap();
        let mut sphere = Body::new(
            BodyId(2),
            Shape::Sphere(0.25),
            V(offset - direction * 6.0, 0.0, 0.0),
            1.0,
        );
        sphere.velocity = V(direction * 4000.0, 0.0, 0.0);
        sphere.ccd = true;
        w.add_body(sphere).unwrap();
        let mut swept = 0;
        for _ in 0..8 {
            swept += w.step(1.0 / 60.0).unwrap().swept_contacts;
            let b = w.body(BodyId(2)).unwrap();
            // Signed distance of the sphere centre in front of the wall's near face.
            let front = direction * (offset - b.position.0) - 0.25;
            assert!(
                front >= 0.25 - Config::default().contact_slop,
                "offset {offset}: sphere tunnelled or embedded (front gap {front})"
            );
            assert!(
                b.velocity.0 * direction <= 0.0,
                "offset {offset}: still approaching"
            );
        }
        assert!(swept > 0, "offset {offset}: impact must be swept");
    }
}
