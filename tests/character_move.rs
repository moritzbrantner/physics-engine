//! Kinematic character movement: exact rest against floors, walls and ceilings (#303–#307),
//! static-only support (#309) and separately reported depenetration (#295).
use physics_engine::{
    BodyId,
    approximate::{Body, CharacterMove, CharacterSettings, Config, Shape, Vector as V, World},
};

const OFFSET: f32 = 0.0025;
const HALF: f32 = 0.5;
const RADIUS: f32 = 0.3;
const FOOT: f32 = HALF + RADIUS;

fn world() -> World {
    let mut w = World::new(Config {
        gravity: V::ZERO,
        ..Config::default()
    })
    .unwrap();
    // Large floor box: support on a big box top must be exact (#305).
    w.add_body(Body::new(
        BodyId(1),
        Shape::Box(V(200.0, 0.5, 200.0)),
        V(0.0, -0.5, 0.0),
        0.0,
    ))
    .unwrap();
    w.add_body(Body::new(
        BodyId(2),
        Shape::Box(V(0.25, 3.0, 20.0)),
        V(5.0, 3.0, 0.0),
        0.0,
    ))
    .unwrap();
    w.add_body(Body::new(
        BodyId(3),
        Shape::Box(V(3.0, 0.25, 3.0)),
        V(-10.0, 2.5, 0.0),
        0.0,
    ))
    .unwrap();
    w
}

fn character(w: &mut World, id: u64, position: V) -> BodyId {
    let mut body = Body::new(BodyId(id), Shape::capsule(HALF, RADIUS), position, 1.0);
    body.external = true;
    w.add_body(body).unwrap();
    w.add_character(BodyId(id), CharacterSettings::default())
        .unwrap();
    BodyId(id)
}

fn step(w: &mut World, id: BodyId, desired: V) -> physics_engine::approximate::CharacterMoveResult {
    let mut out = Vec::new();
    w.move_characters(&[CharacterMove { body: id, desired }], &mut out)
        .unwrap();
    out[0]
}

fn position(w: &World, id: BodyId) -> V {
    w.body(id).unwrap().position
}

#[test]
fn falls_to_the_floor_and_rests_exactly_under_gravity() {
    let mut w = world();
    let c = character(&mut w, 10, V(0.0, FOOT + 1.0, 0.0));
    let mut vy = 0.0;
    let mut rest = None;
    for tick in 0..200 {
        vy -= 9.81 / 60.0;
        let r = step(&mut w, c, V(0.0, vy / 60.0, 0.0));
        if r.grounded {
            vy = 0.0;
            assert_eq!(r.support, Some(BodyId(1)));
            let p = position(&w, c);
            match rest {
                None => rest = Some(p),
                Some(q) => assert_eq!(p, q, "tick {tick}: grounded character moved"),
            }
        }
    }
    let p = rest.expect("character lands");
    assert!(
        (p.1 - (FOOT + OFFSET)).abs() < 1e-4,
        "rests one offset above the floor: {p:?}"
    );
}

#[test]
fn zero_desired_motion_never_moves_a_resting_character() {
    let mut w = world();
    let c = character(&mut w, 10, V(1.0, FOOT + OFFSET, 1.0));
    let before = position(&w, c);
    for _ in 0..100 {
        let r = step(&mut w, c, V::ZERO);
        assert!(r.grounded);
        assert_eq!(r.translation, V::ZERO);
        assert_eq!(r.depenetration, V::ZERO);
    }
    assert_eq!(position(&w, c), before);
}

#[test]
fn pushing_into_a_wall_rests_at_the_offset_without_jitter() {
    let mut w = world();
    let c = character(&mut w, 10, V(3.0, FOOT + OFFSET, 0.0));
    for _ in 0..60 {
        step(&mut w, c, V(0.1, 0.0, 0.0));
    }
    let settled = position(&w, c);
    let wall = 5.0 - 0.25;
    assert!(
        (wall - (settled.0 + RADIUS) - OFFSET).abs() < 1e-4,
        "{settled:?}"
    );
    for _ in 0..60 {
        step(&mut w, c, V(0.1, 0.0, 0.0));
        assert_eq!(position(&w, c), settled);
    }
}

#[test]
fn sliding_along_a_wall_keeps_the_gap() {
    let mut w = world();
    let c = character(&mut w, 10, V(4.0, FOOT + OFFSET, 0.0));
    for _ in 0..60 {
        step(&mut w, c, V(0.1, 0.0, 0.1));
    }
    let p = position(&w, c);
    assert!(p.2 > 5.0, "slides along the wall: {p:?}");
    assert!((5.0 - 0.25 - (p.0 + RADIUS) - OFFSET).abs() < 1e-4);
}

#[test]
fn rising_into_a_ceiling_stops_at_the_offset() {
    let mut w = world();
    // Ceiling slab underside at 2.25.
    let c = character(&mut w, 10, V(-10.0, FOOT + OFFSET, 0.0));
    for _ in 0..30 {
        step(&mut w, c, V(0.0, 0.2, 0.0));
    }
    let p = position(&w, c);
    assert!((2.25 - (p.1 + FOOT) - OFFSET).abs() < 1e-4, "{p:?}");
}

#[test]
fn standing_on_another_character_is_not_grounded() {
    let mut w = world();
    let below = character(&mut w, 10, V(20.0, FOOT + OFFSET, 0.0));
    let above = character(&mut w, 11, V(20.0, 3.0 * FOOT + 2.0 * OFFSET + 0.01, 0.0));
    let r = step(&mut w, above, V(0.0, -0.05, 0.0));
    assert!(!r.grounded);
    assert!(position(&w, above).1 > position(&w, below).1 + 2.0 * FOOT);
}

#[test]
fn penetration_is_resolved_and_reported_apart_from_motion() {
    let mut w = world();
    let c = character(&mut w, 10, V(0.0, FOOT - 0.1, 0.0));
    let r = step(&mut w, c, V::ZERO);
    assert!(r.depenetration.1 > 0.09, "{r:?}");
    assert_eq!(r.translation, V::ZERO);
    assert!(r.grounded);
}
