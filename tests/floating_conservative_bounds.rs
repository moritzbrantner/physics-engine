//! Minimized broad-phase rejection: narrow phase admits this rotated corner contact.
use physics_engine::{
    BodyId,
    approximate::{Body, Config, Quaternion, REAL_BITS, Real, Shape, Vector as V, World},
};

/// The f64 build keeps the original sub-ulp fixture (radius 1e-6 at |x| near 11). With f32
/// state an ulp there is ~1e-6, so the sphere uses a resolvable radius and the contact inset
/// and separated gap are a few ulps; the rounded corner can differ from the kernel's by ulps.
fn fixture() -> (Real, usize, Real) {
    if REAL_BITS == 64 {
        (1e-6, 1, 1e-14)
    } else {
        (1e-3, 8, 4e-5)
    }
}

#[cfg_attr(test, test)]
pub fn rotated_corner_contact_survives_sub_ulp_contact_slop() {
    let mut world = World::new(Config {
        gravity: V::ZERO,
        substeps: 1,
        contact_slop: 1e-20,
        ..Config::default()
    })
    .unwrap();
    let mut fixed = Body::new(
        BodyId(1),
        Shape::Box(V(1.0, 2.0, 3.0)),
        V(-11.0, 4.0, 3.0),
        0.0,
    );
    fixed.orientation = Quaternion(0.2, 0.3, 0.1, 0.9).normalized();
    let (radius, inset, gap) = fixture();
    let corner = fixed.position + fixed.orientation.rotate(V(-1.0, -2.0, 3.0));
    let mut center = corner - V::Y * radius;
    for _ in 0..inset {
        center.1 = center.1.next_up();
    }
    world.add_body(fixed).unwrap();
    world
        .add_body(Body::new(BodyId(2), Shape::Sphere(radius), center, 1.0))
        .unwrap();
    let report = world.step(1.0 / 60.0).unwrap();
    assert!(
        report.narrow_tests > 0,
        "rounded bounds rejected the corner contact"
    );
    assert!(report.contact_points > 0);
    assert!(report.geometry.manifold_candidates > 0);
    assert_eq!(world.elapsed_seconds(), 1.0 / 60.0);
    // The numerical guard also admits a nearby separated candidate. Its narrow-phase
    // result must remain a miss: bound padding is not physical contact slop.
    world.remove_body(BodyId(2)).unwrap();
    let separated = center - V::Y * gap;
    world
        .add_body(Body::new(BodyId(2), Shape::Sphere(radius), separated, 1.0))
        .unwrap();
    let report = world.step(1.0 / 60.0).unwrap();
    assert!(report.narrow_tests > 0);
    assert_eq!(report.contact_points, 0);
    assert_eq!(world.body(BodyId(2)).unwrap().position, separated);
    assert_eq!(world.body(BodyId(2)).unwrap().velocity, V::ZERO);
}
