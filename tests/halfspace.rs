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
    // Box queries are not supported against a halfspace.
    assert!(
        w.overlap_shape(
            QueryPose::new(Shape::Box(V(1.0, 1.0, 1.0)), V(0.0, 0.0, 0.0)),
            QueryFilter::default(),
            &mut Vec::new(),
        )
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
