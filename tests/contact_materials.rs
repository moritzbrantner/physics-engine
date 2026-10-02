use physics_engine::approximate::Real;

/// Rounding-only bound: the f64 reference, or 64 ulps of `scale` when larger (f32 state).
fn rounding(reference: Real, scale: Real) -> Real {
    reference.max(64.0 * Real::EPSILON * scale)
}

#[path = "support/contact_materials.rs"]
mod contract;

#[test]
fn floating_contact_material_contract() {
    contract::run();
    contract::run();
    assert_eq!(
        contract::reciprocal_current_contact_momentum(),
        contract::reciprocal_current_contact_momentum()
    );
}

#[test]
fn rotated_reciprocal_contacts_preserve_momentum() {
    use physics_engine::{
        BodyId,
        approximate::{Body, Config, Quaternion, Shape, Vector as V, World},
    };
    // Cube inertia is isotropic, so this independent angular-momentum sum remains
    // valid after rotating the whole fixture; no engine inertia helper is used.
    let momentum = |w: &World| {
        w.bodies().fold((V::ZERO, V::ZERO), |(p, l), b| {
            let linear = b.velocity * b.mass;
            (
                p + linear,
                l + b.position.cross(linear) + b.angular_velocity * (b.mass * 216.0),
            )
        })
    };
    for q in [
        Quaternion::IDENTITY,
        Quaternion(0.1, 0.2, 0.3, 0.9).normalized(),
        Quaternion(-0.4, 0.7, 0.2, 0.3).normalized(),
    ] {
        for origin in [V::ZERO, V(123.0, -456.0, 789.0)] {
            for ids in [[1, 2], [2, 1]] {
                let fixture = || {
                    let mut w = World::new(Config {
                        gravity: V::ZERO,
                        substeps: 1,
                        convergence: None,
                        ..Config::default()
                    })
                    .unwrap();
                    for (i, id) in ids.into_iter().enumerate() {
                        let mut b = Body::new(
                            BodyId(id),
                            Shape::Box(V(18.0, 18.0, 18.0)),
                            origin + q.rotate(V(0.0, i as Real * 35.99, 0.0)),
                            2.0,
                        );
                        b.orientation = q;
                        b.friction = 0.6;
                        b.sleep_allowed = false;
                        if i == 1 {
                            b.velocity = q.rotate(V(10.0, -1.0, 0.0));
                        }
                        w.add_body(b).unwrap();
                    }
                    w
                };
                let mut w = fixture();
                let mut repeat = fixture();
                let before = momentum(&w);
                let energy = w.bodies().map(Body::kinetic_energy).sum::<Real>();
                let r = w.step(1.0 / 240.0).unwrap();
                repeat.step(1.0 / 240.0).unwrap();
                assert!(r.contact_points > 0 && r.contact_points <= 4);
                assert!(r.impulse_iterations <= 8);
                assert_eq!(r.swept_contacts, 0);
                let after = momentum(&w);
                assert!(
                    (after.0 - before.0).length() / (1.0 + before.0.length())
                        <= rounding(1e-10, 1.0)
                );
                assert!(
                    (after.1 - before.1).length() / (36.0 + before.1.length())
                        <= rounding(1e-10, 1.0)
                );
                assert!(
                    w.bodies().map(Body::kinetic_energy).sum::<Real>()
                        <= energy * (1.0 + rounding(1e-10, 1.0))
                );
                assert_eq!(
                    w.bodies().collect::<Vec<_>>(),
                    repeat.bodies().collect::<Vec<_>>()
                );
            }
        }
    }
}

#[test]
fn generic_support_witnesses_keep_the_full_reciprocal_midpoint() {
    use physics_engine::{
        BodyId,
        approximate::{Body, Config, Shape, Vector as V, World},
    };
    for ids in [[1, 2], [2, 1]] {
        let mut world = World::new(Config {
            gravity: V::ZERO,
            substeps: 1,
            convergence: None,
            ..Config::default()
        })
        .unwrap();
        let mut cube = Body::new(BodyId(ids[0]), Shape::Box(V(1.0, 1.0, 1.0)), V::ZERO, 2.0);
        let mut wedge = Body::new(
            BodyId(ids[1]),
            Shape::wedge(V(1.0, 1.0, 1.0)),
            V(0.0, 1.99, 0.0),
            2.0,
        );
        wedge.rotation_locked = true;
        wedge.velocity = V(0.0, -0.25, 0.0);
        for body in [&mut cube, &mut wedge] {
            body.friction = 0.0;
            body.sleep_allowed = false;
        }
        world.add_body(cube).unwrap();
        world.add_body(wedge).unwrap();
        let report = world.step(1.0 / 240.0).unwrap();
        assert!(report.contact_points > 0);
        let cube = world.body(BodyId(ids[0])).unwrap();
        assert!(cube.velocity.1 < -1e-3, "normal response required");
        // Independent support vertices are (1,1,1) and (-1,.99,-1).
        // Their common midpoint lies on the cube's Y axis, giving zero torque.
        assert!(cube.angular_velocity.length() <= rounding(1e-12, 1.0));
    }
}
