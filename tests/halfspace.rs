//! Halfspace floors (C2): rays, capsule queries and characters standing and walking on them.
use physics_engine::approximate::{
    Body, CharacterMove, Config, QueryFilter, QueryPose, RayCast, Shape, ShapeCast, Vector as V,
    World,
};

fn floor(height: f32) -> (World, physics_engine::BodyId) {
    let mut w = World::new(Config {
        gravity: V::ZERO,
        ..Config::default()
    })
    .unwrap();
    let id = w
        .add_body(Body::keyed(1, Shape::HalfSpace, V(0.0, height, 0.0), 0.0))
        .unwrap();
    (w, id)
}

#[test]
fn rays_hit_the_plane_anywhere() {
    let (w, plane) = floor(-1.0);
    for x in [0.0, 1.0e3, -5.0e4] {
        let hit = w
            .cast_ray_first(
                RayCast::new(V(x, 10.0, 3.0), V(0.0, -20.0, 0.0)),
                QueryFilter::default(),
            )
            .unwrap()
            .expect("plane below");
        assert_eq!(hit.body, plane);
        assert!((hit.point.1 + 1.0).abs() < 1e-5);
        assert_eq!(hit.normal, V(0.0, 1.0, 0.0));
    }
    // Rays parallel to or pointing away from the plane miss.
    for d in [V(10.0, 0.0, 0.0), V(0.0, 5.0, 0.0)] {
        assert!(
            w.cast_ray_first(RayCast::new(V(0.0, 1.0, 0.0), d), QueryFilter::default())
                .unwrap()
                .is_none()
        );
    }
}

#[test]
fn tilted_plane_normal_follows_orientation() {
    let mut w = World::new(Config::default()).unwrap();
    let mut body = Body::new(Shape::HalfSpace, V::ZERO, 0.0);
    // Rotate +Y toward +X by 90 degrees about Z: a wall facing +X.
    let h = std::f32::consts::FRAC_1_SQRT_2;
    body.orientation = physics_engine::approximate::Quaternion(0.0, 0.0, -h, h);
    let wall = w.add_body(body).unwrap();
    let hit = w
        .cast_ray_first(
            RayCast::new(V(5.0, 0.0, 0.0), V(-10.0, 0.0, 0.0)),
            QueryFilter::default(),
        )
        .unwrap()
        .unwrap();
    assert_eq!(hit.body, wall);
    assert!((hit.normal.0 - 1.0).abs() < 1e-5 && hit.point.0.abs() < 1e-5);
}

#[test]
fn capsule_overlap_and_cast_against_plane() {
    let (w, plane) = floor(0.0);
    let shape = Shape::capsule(0.5, 0.3);
    let mut hits = Vec::new();
    w.overlap_shape(
        QueryPose::new(shape, V(0.0, 0.7, 0.0)),
        QueryFilter::default(),
        &mut hits,
    )
    .unwrap();
    assert_eq!(hits.len(), 1);
    assert_eq!(hits[0].body, plane);
    assert!(hits[0].separation < 0.0);
    hits.clear();
    w.overlap_shape(
        QueryPose::new(shape, V(0.0, 0.9, 0.0)),
        QueryFilter::default(),
        &mut hits,
    )
    .unwrap();
    assert!(hits.is_empty());
    w.cast_shape(
        ShapeCast::new(QueryPose::new(shape, V(0.0, 3.0, 0.0)), V(0.0, -4.0, 0.0)),
        QueryFilter::default(),
        &mut hits,
    )
    .unwrap();
    let hit = hits[0];
    assert_eq!(hit.body, plane);
    // Bottom at 2.2 above the plane: 2.2 of 4.0 travel.
    assert!((hit.fraction - 0.55).abs() < 1e-4, "{}", hit.fraction);
    assert_eq!(hit.normal, V(0.0, 1.0, 0.0));
}

#[test]
fn box_and_wedge_queries_meet_the_plane_at_their_deepest_point() {
    let (w, plane) = floor(0.0);
    let mut hits = Vec::new();
    // A door-probe box resting into the floor overlaps it; one above does not.
    let probe = |y| QueryPose::new(Shape::Box(V(1.0, 1.0, 0.1)), V(3.0, y, 0.0));
    w.overlap_shape(probe(0.9), QueryFilter::default(), &mut hits)
        .unwrap();
    assert_eq!(hits.len(), 1);
    assert_eq!(hits[0].body, plane);
    assert!((hits[0].separation + 0.1).abs() < 1e-5);
    w.overlap_shape(probe(1.1), QueryFilter::default(), &mut hits)
        .unwrap();
    assert!(hits.is_empty());
    // A tilted box lands on its lowest corner.
    let mut pose = QueryPose::new(Shape::Box(V(0.5, 0.5, 0.5)), V(0.0, 3.0, 0.0));
    // 45° about z: (0, 0, sin 22.5°, cos 22.5°).
    pose.orientation = physics_engine::approximate::Quaternion(0.0, 0.0, 0.382_683_43, 0.923_879_5);
    w.cast_shape(
        ShapeCast::new(pose, V(0.0, -4.0, 0.0)),
        QueryFilter::default(),
        &mut hits,
    )
    .unwrap();
    // Rotated 45° about z: the lowest edge is 0.5·√2 below the centre.
    let expected = (3.0 - 0.5 * std::f32::consts::SQRT_2) / 4.0;
    assert!(
        (hits[0].fraction - expected).abs() < 1e-4,
        "{}",
        hits[0].fraction
    );
    // A wedge sliding sideways above the floor never meets it.
    w.cast_shape(
        ShapeCast::new(
            QueryPose::new(Shape::wedge(V(1.0, 0.5, 1.0)), V(0.0, 0.6, 0.0)),
            V(10.0, 0.0, 0.0),
        ),
        QueryFilter::default(),
        &mut hits,
    )
    .unwrap();
    assert!(hits.is_empty());
}

#[test]
fn characters_stand_and_walk_on_the_plane() {
    let (mut w, plane) = floor(0.0);
    let (half, radius) = (0.5, 0.3);
    let mut body = Body::keyed(10, Shape::capsule(half, radius), V(0.0, 3.0, 0.0), 1.0);
    body.external = true;
    let c = w.add_body(body).unwrap();
    let settings = w.character_settings();
    w.add_character(c, settings).unwrap();
    let mut out = Vec::new();
    let mut vy = 0.0;
    let mut rest = None;
    for tick in 0..300 {
        vy -= 9.81 / 60.0;
        let walk = if tick > 100 { 0.05 } else { 0.0 };
        w.move_characters(
            &[CharacterMove {
                body: c,
                desired: V(walk, vy / 60.0, 0.0),
                exclude: None,
                autostep: None,
            }],
            &mut out,
        )
        .unwrap();
        if out[0].grounded {
            vy = 0.0;
            assert_eq!(out[0].support, Some(plane));
        }
        let p = w.body(c).unwrap().position;
        if (60..100).contains(&tick) {
            assert!(out[0].grounded, "tick {tick}");
            match rest {
                None => rest = Some(p),
                Some(q) => assert_eq!(p, q, "tick {tick}: moved while resting"),
            }
        }
        assert!(p.1 - half - radius > -2e-3, "tick {tick}: sank");
    }
    let p = w.body(c).unwrap().position;
    assert!(p.0 > 9.0, "walked: {p:?}");
}

#[test]
fn halfspace_rejects_misuse_and_removes_cleanly() {
    let (mut w, plane) = floor(0.0);
    // Halfspaces are static only.
    assert!(
        w.add_body(Body::new(Shape::HalfSpace, V::ZERO, 1.0))
            .is_err()
    );
    // Stepping with a dynamic body is fine; halfspaces take no part in rigid contacts.
    w.add_body(Body::new(Shape::Sphere(0.5), V(0.0, 10.0, 0.0), 1.0))
        .unwrap();
    w.step(1.0 / 60.0).unwrap();
    w.remove_body(plane).unwrap();
    assert!(
        w.cast_ray_first(
            RayCast::new(V(0.0, 10.0, 0.0), V(0.0, -20.0, 0.0)),
            QueryFilter::default()
        )
        .unwrap()
        .is_none_or(|h| h.body != plane)
    );
}

fn falling_world() -> World {
    World::new(Config {
        gravity: V(0.0, -9.0, 0.0),
        ..Config::default()
    })
    .unwrap()
}

fn crate_body(position: V) -> Body {
    let mut body = Body::new(Shape::Box(V(0.35, 0.35, 0.35)), position, 176.4);
    body.rotation_locked = true;
    body.friction = 0.8;
    body
}

#[test]
fn dynamic_box_rests_on_the_plane_in_either_pair_order() {
    for plane_first in [true, false] {
        let mut w = falling_world();
        let plane = Body::new(Shape::HalfSpace, V(0.0, 0.0, 0.0), 0.0);
        if plane_first {
            w.add_body(plane.clone()).unwrap();
        }
        let id = w.add_body(crate_body(V(1.0, 0.5, -2.0))).unwrap();
        if !plane_first {
            w.add_body(plane.clone()).unwrap();
        }
        for _ in 0..90 {
            w.step(1.0 / 30.0).unwrap();
        }
        let b = w.body(id).unwrap();
        assert!((b.position.1 - 0.35).abs() < 0.01, "{:?}", b.position);
        assert!(b.velocity.length() < 0.05, "{:?}", b.velocity);
        assert!((b.position.0 - 1.0).abs() < 1e-4 && (b.position.2 + 2.0).abs() < 1e-4);
    }
}

#[test]
fn fast_falling_box_and_rotating_box_stop_at_the_plane() {
    let mut w = falling_world();
    w.add_body(Body::new(Shape::HalfSpace, V(0.0, -1.0, 0.0), 0.0))
        .unwrap();
    let mut fast = crate_body(V(0.0, 20.0, 0.0));
    fast.velocity = V(0.0, -400.0, 0.0);
    let fast = w.add_body(fast).unwrap();
    let mut tumbling = Body::new(Shape::Box(V(0.2, 0.1, 0.3)), V(3.0, 2.0, 0.0), 10.0);
    tumbling.orientation = physics_engine::approximate::Quaternion(0.3, 0.1, 0.2, 0.9).normalized();
    tumbling.angular_velocity = V(2.0, 0.0, 3.0);
    let tumbling = w.add_body(tumbling).unwrap();
    for _ in 0..240 {
        w.step(1.0 / 30.0).unwrap();
        for id in [fast, tumbling] {
            // Never below the plane by more than a few millimetres at any tick.
            let b = w.body(id).unwrap();
            let lowest = b.position.1 - b.shape.radius();
            assert!(
                lowest > -1.0 - b.shape.radius(),
                "{id:?} tunnelled: {:?}",
                b.position
            );
        }
    }
    let b = w.body(fast).unwrap();
    assert!((b.position.1 + 0.65).abs() < 0.01, "{:?}", b.position);
    let t = w.body(tumbling).unwrap();
    assert!(
        t.position.1 > -1.0 && t.position.1 < -0.6,
        "{:?}",
        t.position
    );
    assert!(t.velocity.length() < 0.1, "{:?}", t.velocity);
}

#[test]
fn sliding_crate_stops_by_friction_and_damping() {
    let mut w = falling_world();
    let mut floor = Body::new(Shape::HalfSpace, V::ZERO, 0.0);
    floor.friction = 0.8;
    w.add_body(floor).unwrap();
    let mut body = crate_body(V(0.0, 0.35, 0.0));
    body.linear_damping = 0.2;
    body.velocity = V(5.91, 0.0, 0.0);
    let id = w.add_body(body).unwrap();
    for _ in 0..90 {
        w.step(1.0 / 30.0).unwrap();
    }
    let b = w.body(id).unwrap();
    // Coulomb friction 0.8 · 9 m/s² alone stops it after 2.43 m; damping shortens that.
    assert!(
        b.position.0 > 2.0 && b.position.0 < 2.45,
        "{:?}",
        b.position
    );
    assert!(b.velocity.length() < 0.01, "{:?}", b.velocity);
    assert!((b.position.1 - 0.35).abs() < 0.01 && b.position.2.abs() < 1e-4);
}

#[test]
fn damping_scales_velocity_per_substep() {
    let mut w = World::new(Config {
        gravity: V::ZERO,
        ..Config::default()
    })
    .unwrap();
    let mut body = Body::new(Shape::Sphere(0.5), V::ZERO, 1.0);
    body.velocity = V(10.0, 0.0, 0.0);
    body.angular_velocity = V(0.0, 4.0, 0.0);
    body.linear_damping = 1.0;
    body.angular_damping = 2.0;
    let id = w.add_body(body).unwrap();
    for _ in 0..10 {
        w.step(0.1).unwrap();
    }
    // Over one second the product of 1 / (1 + h·c) lies between 1 / (1 + c) and e^-c.
    let b = w.body(id).unwrap();
    assert!(
        b.velocity.0 > 10.0 * (-1.0f32).exp() && b.velocity.0 < 5.0,
        "{:?}",
        b.velocity
    );
    assert!(b.angular_velocity.1 > 4.0 * (-2.0f32).exp() && b.angular_velocity.1 < 4.0 / 3.0);
    let mut bad = Body::new(Shape::Sphere(0.5), V::ZERO, 1.0);
    bad.linear_damping = -1.0;
    assert!(w.add_body(bad).is_err());
}
