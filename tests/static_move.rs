//! Static bodies moved in place (#352): moving platforms without removing and re-adding them.
use physics_engine::approximate::{
    Body, CharacterMove, Config, QueryFilter, RayCast, Shape, Vector as V, World,
};

fn world() -> (World, physics_engine::BodyId) {
    let mut w = World::new(Config {
        gravity: V::ZERO,
        ..Config::default()
    })
    .unwrap();
    w.add_body(Body::keyed(1, Shape::HalfSpace, V::ZERO, 0.0))
        .unwrap();
    let platform = w
        .add_body(Body::keyed(
            2,
            Shape::Box(V(1.0, 0.25, 1.0)),
            V(0.0, 0.25, 0.0),
            0.0,
        ))
        .unwrap();
    (w, platform)
}

#[test]
fn a_moved_static_body_is_found_where_it_went() {
    let (mut w, platform) = world();
    w.set_static_position(platform, V(5.0, 0.25, 0.0)).unwrap();
    let down = |w: &World, x: f32| {
        w.cast_ray_first(
            RayCast::new(V(x, 5.0, 0.0), V(0.0, -10.0, 0.0)),
            QueryFilter::default(),
        )
        .unwrap()
        .map(|h| h.body)
    };
    assert_eq!(down(&w, 5.0), Some(platform));
    assert_ne!(down(&w, 0.0), Some(platform));
    assert_eq!(w.body(platform).unwrap().position, V(5.0, 0.25, 0.0));
}

/// The adapter pattern: the game adds the platform's displacement to its riders' moves.
#[test]
fn riders_carried_by_the_game_stay_on_a_moving_platform() {
    let (mut w, platform) = world();
    let mut body = Body::keyed(10, Shape::capsule(0.5, 0.3), V(0.0, 1.3025, 0.0), 1.0);
    body.external = true;
    let rider = w.add_body(body).unwrap();
    let settings = w.character_settings();
    w.add_character(rider, settings).unwrap();
    let mut out = Vec::new();
    for tick in 0..60 {
        // Up 1 cm a tick for half a second, then sideways.
        let by = if tick < 30 {
            V(0.0, 0.01, 0.0)
        } else {
            V(0.01, 0.0, 0.0)
        };
        let to = w.body(platform).unwrap().position + by;
        w.set_static_position(platform, to).unwrap();
        w.move_characters(
            &[CharacterMove {
                body: rider,
                desired: by,
                exclude: None,
                autostep: None,
            }],
            &mut out,
        )
        .unwrap();
        assert!(out[0].grounded, "tick {tick}");
        assert_eq!(out[0].support, Some(platform), "tick {tick}");
    }
    let (p, r) = (
        w.body(platform).unwrap().position,
        w.body(rider).unwrap().position,
    );
    assert!((r.0 - p.0).abs() < 1e-4, "{p:?} {r:?}");
    assert!(
        (r.1 - (p.1 + 0.25 + 0.8 + settings.offset)).abs() < 1e-3,
        "{p:?} {r:?}"
    );
}

/// A platform moved into a resting character pushes it out on its next move.
#[test]
fn a_character_overlapped_by_a_moved_platform_is_pushed_out() {
    let (mut w, platform) = world();
    let mut body = Body::keyed(10, Shape::capsule(0.5, 0.3), V(3.0, 0.8025, 0.0), 1.0);
    body.external = true;
    let c = w.add_body(body).unwrap();
    let settings = w.character_settings();
    w.add_character(c, settings).unwrap();
    let mut out = Vec::new();
    let rest = [CharacterMove {
        body: c,
        desired: V::ZERO,
        exclude: None,
        autostep: None,
    }];
    w.move_characters(&rest, &mut out).unwrap();
    w.move_characters(&rest, &mut out).unwrap();
    // The platform's edge now reaches 0.1 m into the capsule.
    w.set_static_position(platform, V(3.0 - 1.3 + 0.1, 0.25, 0.0))
        .unwrap();
    w.move_characters(&rest, &mut out).unwrap();
    assert!(out[0].depenetration.0 > 0.09, "{:?}", out[0]);
}

#[test]
fn only_static_movable_bodies_move_in_place() {
    let (mut w, platform) = world();
    let floor = w.body_id(1).unwrap();
    assert!(w.set_static_position(floor, V(0.0, 1.0, 0.0)).is_err());
    let dynamic = w
        .add_body(Body::keyed(3, Shape::Sphere(0.5), V(9.0, 3.0, 0.0), 1.0))
        .unwrap();
    assert!(w.set_static_position(dynamic, V::ZERO).is_err());
    assert!(
        w.set_static_position(platform, V(f32::NAN, 0.0, 0.0))
            .is_err()
    );
}

#[test]
fn bodies_sleeping_on_a_moved_platform_wake() {
    let mut w = World::new(Config::default()).unwrap();
    let platform = w
        .add_body(Body::keyed(
            2,
            Shape::Box(V(2.0, 0.25, 2.0)),
            V(0.0, 0.25, 0.0),
            0.0,
        ))
        .unwrap();
    let ball = w
        .add_body(Body::keyed(3, Shape::Sphere(0.25), V(0.0, 0.75, 0.0), 1.0))
        .unwrap();
    for _ in 0..600 {
        w.step(1.0 / 60.0).unwrap();
    }
    assert!(w.body(ball).unwrap().is_sleeping(), "settled asleep");
    w.set_static_position(platform, V(0.0, 0.1, 0.0)).unwrap();
    assert!(!w.body(ball).unwrap().is_sleeping(), "woken by the move");
    for _ in 0..120 {
        w.step(1.0 / 60.0).unwrap();
    }
    let y = w.body(ball).unwrap().position.1;
    assert!(y < 0.65, "fell after the platform: {y}");
}
