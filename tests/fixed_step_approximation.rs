use physics_engine::{
    BodyId,
    approximate::{Body, Config, Quaternion, Shape, Vector as V, World},
};
fn world(g: V) -> World {
    World::new(Config {
        gravity: g,
        ..Config::default()
    })
    .unwrap()
}
fn cuboid(id: u64, p: V, h: V, m: f64) -> Body {
    Body::new(BodyId(id), Shape::Box(h), p, m)
}
#[test]
fn forces_and_impulses_accumulate_and_updated_velocity_survives_the_next_tick() {
    let mut w = world(V::ZERO);
    w.add_body(cuboid(1, V::ZERO, V(1.0, 1.0, 1.0), 2.0))
        .unwrap();
    w.apply_impulse(BodyId(1), V(2.0, 0.0, 0.0), V::ZERO)
        .unwrap();
    w.apply_impulse(BodyId(1), V(0.0, 4.0, 0.0), V::ZERO)
        .unwrap();
    w.add_force(BodyId(1), V(6.0, 0.0, 0.0)).unwrap();
    w.add_force(BodyId(1), V(4.0, 0.0, 0.0)).unwrap();
    w.step(0.1).unwrap();
    let b = w.body(BodyId(1)).unwrap();
    assert!((b.velocity.0 - 1.5).abs() < 1e-12);
    assert_eq!(b.velocity.1, 2.0);
    let p = b.position;
    w.step(0.1).unwrap();
    let b = w.body(BodyId(1)).unwrap();
    assert_eq!(b.velocity, V(1.5, 2.0, 0.0));
    assert!((b.position.0 - p.0 - 0.15).abs() < 1e-12);
}
#[test]
fn invalid_values_are_rejected_without_changing_state() {
    let mut w = world(V::ZERO);
    w.add_body(cuboid(1, V::ZERO, V(1.0, 1.0, 1.0), 1.0))
        .unwrap();
    let b = w.body(BodyId(1)).unwrap().clone();
    assert!(w.step(f64::NAN).is_err());
    assert!(
        w.apply_impulse(BodyId(1), V(f64::INFINITY, 0.0, 0.0), V::ZERO)
            .is_err()
    );
    assert_eq!(w.body(BodyId(1)), Some(&b));
}
#[test]
fn off_center_impulse_changes_rotation_without_quantizing_state() {
    let mut w = world(V::ZERO);
    w.add_body(cuboid(1, V::ZERO, V(1.0, 1.0, 1.0), 2.0))
        .unwrap();
    w.apply_impulse(BodyId(1), V(0.5, 0.0, 0.0), V(0.0, 1.0, 0.0))
        .unwrap();
    w.step(1.0 / 60.0).unwrap();
    let b = w.body(BodyId(1)).unwrap();
    assert_eq!(b.velocity.0, 0.25);
    assert!(b.angular_velocity.2 < 0.0);
    assert_ne!(b.orientation, Quaternion::IDENTITY);
    assert!(b.position.0 > 0.0 && b.position.0 < 0.01);
}
#[test]
fn fast_arrow_and_sphere_do_not_cross_a_thin_wall() {
    for shape in [Shape::Box(V(0.1, 0.1, 1.0)), Shape::Sphere(0.1)] {
        let mut w = world(V::ZERO);
        w.add_body(cuboid(1, V::ZERO, V(10.0, 10.0, 0.02), 0.0))
            .unwrap();
        let mut p = Body::new(BodyId(2), shape, V(0.0, 0.0, 10.0), 1.0);
        p.velocity = V(0.0, 0.0, -10000.0);
        p.ccd = true;
        p.retire_on_impact = true;
        w.add_body(p).unwrap();
        let r = w.step(1.0 / 60.0).unwrap();
        assert_eq!(r.retired, vec![BodyId(2)]);
        assert!(r.swept_contacts > 0);
    }
}
#[test]
fn swept_sphere_corner_near_miss_is_not_an_expanded_aabb_hit() {
    let mut w = world(V::ZERO);
    w.add_body(cuboid(1, V::ZERO, V(1.0, 1.0, 1.0), 0.0))
        .unwrap();
    let mut p = Body::new(BodyId(2), Shape::Sphere(0.5), V(1.4, 1.4, 5.0), 1.0);
    p.velocity = V(0.0, 0.0, -1000.0);
    p.ccd = true;
    p.retire_on_impact = true;
    w.add_body(p).unwrap();
    w.step(1.0 / 60.0).unwrap();
    assert!(w.body(BodyId(2)).unwrap().position.2 < -1.0);
}
#[test]
fn symmetric_impact_conserves_linear_momentum() {
    let mut w = world(V::ZERO);
    for (id, x, v) in [(1, -1.0, 10.0), (2, 1.0, -10.0)] {
        let mut b = cuboid(id, V(x, 0.0, 0.0), V(1.0, 1.0, 1.0), 1.0);
        b.velocity = V(v, 0.0, 0.0);
        b.restitution = 1.0;
        w.add_body(b).unwrap();
    }
    w.step(1.0 / 60.0).unwrap();
    let a = w.body(BodyId(1)).unwrap();
    let b = w.body(BodyId(2)).unwrap();
    assert!((a.velocity + b.velocity).length() < 1e-9);
    assert!(a.velocity.0 < 0.0 && b.velocity.0 > 0.0);
}
#[test]
fn touching_thirty_two_box_tower_settles_and_misses_do_not_wake_it() {
    let mut w = world(V(0.0, -3600.0, 0.0));
    w.add_body(cuboid(10, V(0.0, -16.0, 0.0), V(520.0, 16.0, 520.0), 0.0))
        .unwrap();
    for level in 0..4 {
        for depth in 0..2 {
            for col in 0..4 {
                let id = 100 + level * 8 + depth * 4 + col;
                let b = cuboid(
                    id,
                    V(
                        -54.0 + col as f64 * 36.0,
                        18.0 + level as f64 * 36.0,
                        82.0 + depth as f64 * 36.0,
                    ),
                    V(18.0, 18.0, 18.0),
                    2.0,
                );
                w.add_body(b).unwrap();
            }
        }
    }
    for _ in 0..240 {
        w.step(1.0 / 60.0).unwrap();
    }
    let asleep = w
        .bodies()
        .filter(|b| b.id.0 >= 100 && b.is_sleeping())
        .count();
    assert_eq!(asleep, 32);
    let before = w
        .bodies()
        .filter(|b| b.id.0 >= 100)
        .cloned()
        .collect::<Vec<_>>();
    let mut p = Body::new(BodyId(1000), Shape::Sphere(3.0), V(140.0, 50.0, 300.0), 1.0);
    p.velocity = V(0.0, 0.0, -5760.0);
    p.ccd = true;
    p.retire_on_impact = true;
    w.add_body(p).unwrap();
    for _ in 0..30 {
        w.step(1.0 / 60.0).unwrap();
    }
    for b in before {
        let actual = w.body(b.id).unwrap();
        assert!(actual.is_sleeping());
        assert_eq!(actual.position, b.position);
        assert_eq!(actual.orientation, b.orientation);
    }
}

#[test]
fn nearer_wall_shields_target_regardless_of_body_id_order() {
    for (wall_id, target_id) in [(1, 2), (2, 1)] {
        for shape in [Shape::Sphere(0.1), Shape::Box(V(0.1, 0.1, 0.5))] {
            let mut w = world(V::ZERO);
            w.add_body(cuboid(wall_id, V(0.0, 0.0, 5.0), V(10.0, 10.0, 0.1), 0.0))
                .unwrap();
            w.add_body(cuboid(target_id, V::ZERO, V(1.0, 1.0, 1.0), 2.0))
                .unwrap();
            let mut projectile = Body::new(BodyId(9), shape, V(0.0, 0.0, 10.0), 1.0);
            projectile.velocity = V(0.0, 0.0, -10000.0);
            projectile.ccd = true;
            projectile.retire_on_impact = true;
            w.add_body(projectile).unwrap();
            w.step(1.0 / 60.0).unwrap();
            let target = w.body(BodyId(target_id)).unwrap();
            assert_eq!(
                target.velocity,
                V::ZERO,
                "a wall must shield targets, including lower-ID targets"
            );
            assert_eq!(target.position, V::ZERO);
        }
    }
}

#[test]
fn removal_of_support_wakes_a_sleeping_box() {
    let mut w = world(V(0.0, -10.0, 0.0));
    w.add_body(cuboid(1, V(0.0, -1.0, 0.0), V(10.0, 1.0, 10.0), 0.0))
        .unwrap();
    w.add_body(cuboid(2, V(0.0, 1.0, 0.0), V(1.0, 1.0, 1.0), 1.0))
        .unwrap();
    for _ in 0..240 {
        w.step(1.0 / 60.0).unwrap();
    }
    assert!(w.body(BodyId(2)).unwrap().is_sleeping());
    w.remove_body(BodyId(1));
    w.step(1.0 / 60.0).unwrap();
    assert!(w.body(BodyId(2)).unwrap().velocity.1 < 0.0);
}

#[test]
fn fast_capsule_does_not_tunnel_through_thin_box_or_wedge() {
    for target in [
        Shape::Box(V(10.0, 10.0, 0.02)),
        Shape::wedge(V(10.0, 10.0, 0.02)),
    ] {
        let mut w = world(V::ZERO);
        w.add_body(Body::new(BodyId(1), target, V::ZERO, 0.0))
            .unwrap();

        let mut projectile = Body::new(
            BodyId(2),
            Shape::capsule(0.5, 0.1),
            V(-5.0, -5.0, 10.0),
            1.0,
        );
        projectile.velocity = V(0.0, 0.0, -10_000.0);
        projectile.ccd = true;
        projectile.retire_on_impact = true;
        w.add_body(projectile).unwrap();

        let report = w.step(1.0 / 60.0).unwrap();
        assert_eq!(report.retired, vec![BodyId(2)]);
        assert!(report.swept_contacts > 0);
        assert!(report.geometry.primitive_queries > 0);
    }
}

#[test]
fn fast_cylinder_does_not_tunnel_through_thin_box_or_cylinder() {
    for target in [
        Shape::Box(V(10.0, 10.0, 0.02)),
        Shape::cylinder(10.0, 0.02),
    ] {
        let mut w = world(V::ZERO);
        w.add_body(Body::new(BodyId(1), target, V::ZERO, 0.0))
            .unwrap();

        let mut projectile = Body::new(
            BodyId(2),
            Shape::cylinder(0.5, 0.1),
            V(0.0, 0.0, 10.0),
            1.0,
        );
        projectile.velocity = V(0.0, 0.0, -10_000.0);
        projectile.ccd = true;
        projectile.retire_on_impact = true;
        w.add_body(projectile).unwrap();

        let report = w.step(1.0 / 60.0).unwrap();
        assert_eq!(report.retired, vec![BodyId(2)]);
        assert!(report.swept_contacts > 0);
        assert!(report.geometry.primitive_queries > 0);
    }
}

#[test]
fn wedge_requires_locked_rotation_only_for_solver_owned_dynamic_bodies() {
    let mut w = world(V::ZERO);
    let wedge = Body::new(BodyId(1), Shape::wedge(V(2.0, 1.0, 3.0)), V::ZERO, 1.0);
    assert!(w.add_body(wedge.clone()).is_err());

    let mut locked = wedge;
    locked.rotation_locked = true;
    w.add_body(locked).unwrap();

    w.add_body(Body::new(
        BodyId(2),
        Shape::wedge(V(2.0, 1.0, 3.0)),
        V(10.0, 0.0, 0.0),
        0.0,
    ))
    .unwrap();
}

#[test]
fn cylinder_off_center_impulse_uses_solid_cylinder_inertia() {
    let mut w = world(V::ZERO);
    w.add_body(Body::new(
        BodyId(1),
        Shape::cylinder(2.0, 1.0),
        V::ZERO,
        3.0,
    ))
    .unwrap();
    w.apply_impulse(BodyId(1), V(1.0, 0.0, 0.0), V(0.0, 1.0, 0.0))
        .unwrap();
    w.step(1.0 / 60.0).unwrap();
    let body = w.body(BodyId(1)).unwrap();

    let expected_inverse_radial = 12.0 / (3.0 * (3.0 + 16.0));
    assert!((body.angular_velocity.2 + expected_inverse_radial).abs() <= 1e-12);
    assert_ne!(body.orientation, Quaternion::IDENTITY);
}

#[test]
fn cylinder_dimensions_must_be_finite_and_positive() {
    let mut w = world(V::ZERO);
    for shape in [
        Shape::cylinder(0.0, 1.0),
        Shape::cylinder(1.0, 0.0),
        Shape::cylinder(f64::NAN, 1.0),
        Shape::cylinder(1.0, f64::INFINITY),
    ] {
        assert!(
            w.add_body(Body::new(BodyId(100), shape, V::ZERO, 1.0))
                .is_err()
        );
    }
}

#[test]
fn capsule_off_center_impulse_uses_capsule_inertia_without_quantization() {
    let mut w = world(V::ZERO);
    w.add_body(Body::new(BodyId(1), Shape::capsule(1.5, 0.5), V::ZERO, 2.0))
        .unwrap();
    w.apply_impulse(BodyId(1), V(0.5, 0.0, 0.0), V(0.0, 1.0, 0.0))
        .unwrap();
    w.step(1.0 / 60.0).unwrap();
    let body = w.body(BodyId(1)).unwrap();
    assert!(body.angular_velocity.2 < 0.0);
    assert_ne!(body.orientation, Quaternion::IDENTITY);
}
