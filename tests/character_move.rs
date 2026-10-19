//! Kinematic character movement: exact rest against floors, walls and ceilings (#303–#307),
//! static-only support (#309) and separately reported depenetration (#295).
use physics_engine::{
    BodyId,
    approximate::{
        Body, CharacterMove, CharacterMoveResult, CharacterSettings, Config, Shape, Vector as V,
        World,
    },
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
    w.add_body(Body::keyed(
        1,
        Shape::Box(V(200.0, 0.5, 200.0)),
        V(0.0, -0.5, 0.0),
        0.0,
    ))
    .unwrap();
    w.add_body(Body::keyed(
        2,
        Shape::Box(V(0.25, 3.0, 20.0)),
        V(5.0, 3.0, 0.0),
        0.0,
    ))
    .unwrap();
    w.add_body(Body::keyed(
        3,
        Shape::Box(V(3.0, 0.25, 3.0)),
        V(-10.0, 2.5, 0.0),
        0.0,
    ))
    .unwrap();
    w
}

fn character(w: &mut World, id: u64, position: V) -> BodyId {
    let mut body = Body::keyed(id, Shape::capsule(HALF, RADIUS), position, 1.0);
    body.external = true;
    let id = w.add_body(body).unwrap();
    w.add_character(id, CharacterSettings::default()).unwrap();
    id
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
    let mut grounded_ticks = 0;
    for tick in 0..200 {
        vy -= 9.81 / 60.0;
        let r = step(&mut w, c, V(0.0, vy / 60.0, 0.0));
        if r.grounded {
            vy = 0.0;
            grounded_ticks += 1;
            assert_eq!(r.support, w.body_id(1));
            let p = position(&w, c);
            // Grounded within the ground prediction, the character snaps down on its next
            // move (as in Rapier); from then on it rests exactly.
            if grounded_ticks > 2 {
                assert_eq!(Some(p), rest, "tick {tick}: grounded character moved");
            }
            rest = Some(p);
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

#[test]
fn planar_characters_keep_their_height_and_need_no_floor() {
    let mut w = World::new(Config {
        gravity: V::ZERO,
        ..Config::default()
    })
    .unwrap();
    w.add_body(Body::keyed(
        2,
        Shape::Box(V(0.25, 3.0, 20.0)),
        V(5.0, 3.0, 0.0),
        0.0,
    ))
    .unwrap();
    let mut body = Body::keyed(10, Shape::capsule(HALF, RADIUS), V(3.0, 1.0, 0.0), 1.0);
    body.external = true;
    let c = w.add_body(body).unwrap();
    let settings = CharacterSettings {
        plane: Some(1.0),
        ..CharacterSettings::default()
    };
    w.add_character(c, settings).unwrap();
    for _ in 0..60 {
        // Downward intent is ignored on the plane; the wall still blocks and slides.
        let r = step(&mut w, c, V(0.1, -0.5, 0.05));
        assert!(r.grounded);
        assert_eq!(r.support, None);
    }
    let p = position(&w, c);
    assert_eq!(p.1, 1.0);
    assert!(p.2 > 2.0);
    assert!((5.0 - 0.25 - (p.0 + RADIUS) - OFFSET).abs() < 1e-4, "{p:?}");
}

#[test]
fn a_lone_mover_leaves_an_overlapped_character_the_whole_way() {
    let mut w = world();
    let a = character(&mut w, 10, V(30.0, FOOT + OFFSET, 0.0));
    let b = character(&mut w, 11, V(30.3, FOOT + OFFSET, 0.0));
    let r = step(&mut w, a, V::ZERO);
    assert!((r.depenetration.0 + 0.3025).abs() < 1e-4, "{r:?}");
    assert_eq!(r.translation, V::ZERO);
    assert_eq!(position(&w, b).0, 30.3);
}

#[test]
fn head_on_characters_sidestep_and_pass() {
    let mut w = world();
    let a = character(&mut w, 10, V(40.0, FOOT + OFFSET, -3.0));
    let b = character(&mut w, 11, V(40.0, FOOT + OFFSET, 3.0));
    let mut out = Vec::new();
    for _ in 0..120 {
        w.move_characters(
            &[
                CharacterMove {
                    body: a,
                    desired: V(0.0, 0.0, 0.05),
                },
                CharacterMove {
                    body: b,
                    desired: V(0.0, 0.0, -0.05),
                },
            ],
            &mut out,
        )
        .unwrap();
        let (pa, pb) = (position(&w, a), position(&w, b));
        let d = V(pa.0 - pb.0, 0.0, pa.2 - pb.2).length();
        assert!(d >= 2.0 * RADIUS + OFFSET - 1e-4, "characters overlap: {d}");
    }
    assert!(
        position(&w, a).2 > 2.0 && position(&w, b).2 < -2.0,
        "they pass each other"
    );
}

#[test]
fn batch_results_do_not_depend_on_slice_order() {
    let run = |reverse: bool| {
        let mut w = world();
        let ids: Vec<_> = (0..6)
            .map(|k| character(&mut w, 20 + k, V(50.0 + 0.5 * k as f32, FOOT + OFFSET, 0.0)))
            .collect();
        let mut out = Vec::new();
        for tick in 0..30 {
            let mut moves: Vec<_> = ids
                .iter()
                .enumerate()
                .map(|(k, &body)| CharacterMove {
                    body,
                    desired: V(
                        if k % 2 == 0 { 0.05 } else { -0.05 },
                        0.0,
                        0.01 * tick as f32,
                    ),
                })
                .collect();
            if reverse {
                moves.reverse();
            }
            w.move_characters(&moves, &mut out).unwrap();
        }
        ids.iter().map(|&id| position(&w, id)).collect::<Vec<_>>()
    };
    assert_eq!(run(false), run(true));
}

#[test]
fn overlapping_characters_separate_half_each() {
    let mut w = world();
    let a = character(&mut w, 10, V(60.0, FOOT + OFFSET, 0.0));
    let b = character(&mut w, 11, V(60.4, FOOT + OFFSET, 0.0));
    let mut out = Vec::new();
    w.move_characters(
        &[
            CharacterMove {
                body: b,
                desired: V::ZERO,
            },
            CharacterMove {
                body: a,
                desired: V::ZERO,
            },
        ],
        &mut out,
    )
    .unwrap();
    let (pa, pb) = (position(&w, a), position(&w, b));
    assert!(
        (pb.0 - pa.0 - (2.0 * RADIUS + OFFSET)).abs() < 1e-4,
        "{pa:?} {pb:?}"
    );
    assert!(((pa.0 + pb.0) * 0.5 - 60.2).abs() < 1e-4, "half each");
    assert_eq!(out[0].body, b);
    assert!(out[0].depenetration.0 > 0.0 && out[1].depenetration.0 < 0.0);
    assert_eq!(out[0].translation, V::ZERO);
}

#[test]
fn overlap_is_resolved_after_a_door_closes_or_the_game_places_a_character() {
    let mut w = world();
    let c = character(&mut w, 10, V(70.0, FOOT + OFFSET, 0.0));
    let other = character(&mut w, 11, V(75.0, FOOT + OFFSET, 0.0));
    for _ in 0..3 {
        assert_eq!(step(&mut w, c, V::ZERO).depenetration, V::ZERO);
    }
    // A door closes onto the resting character.
    w.add_body(Body::keyed(
        50,
        Shape::Box(V(0.25, 2.0, 2.0)),
        V(70.2, 2.0, 0.0),
        0.0,
    ))
    .unwrap();
    let r = step(&mut w, c, V::ZERO);
    assert!(r.depenetration.0 < -0.1, "{r:?}");
    // The game places the other character into this one.
    w.set_kinematic_position(other, V(69.7, FOOT + OFFSET, 0.0))
        .unwrap();
    let mut out = Vec::new();
    w.move_characters(
        &[CharacterMove {
            body: c,
            desired: V::ZERO,
        }],
        &mut out,
    )
    .unwrap();
    let d = (position(&w, c).0 - position(&w, other).0).abs();
    assert!(d >= 2.0 * RADIUS + OFFSET - 1e-4, "{d}");
}

/// The same scene in metres and in centimetres moves identically (#291).
#[test]
fn character_motion_does_not_depend_on_the_length_unit() {
    let run = |unit: f32| {
        let mut w = World::new(Config {
            gravity: V::ZERO,
            length_unit: unit,
            ..Config::default()
        })
        .unwrap();
        let s = |v: V| v * unit;
        w.add_body(Body::keyed(
            1,
            Shape::Box(s(V(200.0, 0.5, 200.0))),
            s(V(0.0, -0.5, 0.0)),
            0.0,
        ))
        .unwrap();
        w.add_body(Body::keyed(
            2,
            Shape::Box(s(V(0.25, 3.0, 20.0))),
            s(V(5.0, 3.0, 0.0)),
            0.0,
        ))
        .unwrap();
        let mut body = Body::keyed(
            10,
            Shape::capsule(HALF * unit, RADIUS * unit),
            s(V(3.0, FOOT + OFFSET + 0.5, 0.0)),
            1.0,
        );
        body.external = true;
        let c = w.add_body(body).unwrap();
        w.add_character(c, w.character_settings()).unwrap();
        let mut out = Vec::new();
        let mut trace = Vec::new();
        let mut vy = 0.0;
        for _ in 0..120 {
            vy -= 9.81 / 60.0;
            let desired = s(V(0.05, vy / 60.0, 0.02));
            w.move_characters(&[CharacterMove { body: c, desired }], &mut out)
                .unwrap();
            if out[0].grounded {
                vy = 0.0;
            }
            trace.push((w.body(c).unwrap().position * (1.0 / unit), out[0].grounded));
        }
        trace
    };
    let (metres, centimetres) = (run(1.0), run(100.0));
    for (tick, ((a, ga), (b, gb))) in metres.iter().zip(&centimetres).enumerate() {
        assert_eq!(ga, gb, "grounded differs at tick {tick}");
        assert!((*a - *b).length() < 1e-4, "tick {tick}: {a:?} vs {b:?}");
    }
}

fn pusher(w: &mut World, id: u64, position: V, max_push: Option<f32>) -> BodyId {
    let mut body = Body::keyed(id, Shape::capsule(HALF, RADIUS), position, 1.0);
    body.external = true;
    let id = w.add_body(body).unwrap();
    let settings = CharacterSettings {
        max_push,
        ..CharacterSettings::default()
    };
    w.add_character(id, settings).unwrap();
    id
}

fn push_tick(w: &mut World, a: BodyId, b: BodyId, desired: V) -> Vec<CharacterMoveResult> {
    let mut out = Vec::new();
    w.move_characters(
        &[
            CharacterMove { body: a, desired },
            CharacterMove {
                body: b,
                desired: V::ZERO,
            },
        ],
        &mut out,
    )
    .unwrap();
    out
}

#[test]
fn a_mover_pushes_a_character_by_at_most_the_smaller_ceiling() {
    let mut w = world();
    let gap = 2.0 * RADIUS + OFFSET;
    let a = pusher(&mut w, 10, V(80.0, FOOT + OFFSET, 0.0), Some(0.04));
    let b = pusher(&mut w, 11, V(80.0 + gap, FOOT + OFFSET, 0.0), Some(0.02));
    let out = push_tick(&mut w, a, b, V(0.05, 0.0, 0.0));
    assert!((out[1].pushed.0 - 0.02).abs() < 1e-5, "{out:?}");
    assert_eq!(out[1].translation, V::ZERO);
    assert_eq!(out[0].pushed, V::ZERO);
    // The pusher follows by the pushed distance; the blocked rest sidesteps as head-on.
    assert!(
        (position(&w, a).0 - 80.02).abs() < 1e-4,
        "{:?}",
        position(&w, a)
    );
}

#[test]
fn a_mover_within_both_ceilings_pushes_straight_ahead() {
    let mut w = world();
    let gap = 2.0 * RADIUS + OFFSET;
    let a = pusher(&mut w, 10, V(80.0, FOOT + OFFSET, 0.0), Some(0.1));
    let b = pusher(&mut w, 11, V(80.0 + gap, FOOT + OFFSET, 0.0), Some(0.1));
    for tick in 1..=10 {
        let out = push_tick(&mut w, a, b, V(0.05, 0.0, 0.0));
        assert!((out[1].pushed.0 - 0.05).abs() < 1e-5, "{out:?}");
        let (pa, pb) = (position(&w, a), position(&w, b));
        assert!(
            (pb.0 - 80.0 - gap - 0.05 * tick as f32).abs() < 1e-4,
            "{pb:?}"
        );
        assert!((pb.0 - pa.0 - gap).abs() < 1e-4, "{pa:?} {pb:?}");
        assert!(pa.2 == 0.0 && pb.2 == 0.0, "no sidestep: {pa:?} {pb:?}");
    }
}

#[test]
fn a_pushed_character_stops_at_the_wall_gap() {
    let mut w = world();
    let gap = 2.0 * RADIUS + OFFSET;
    // The wall's face is at x = 4.75.
    let b = pusher(&mut w, 11, V(4.0, FOOT + OFFSET, 0.0), Some(0.1));
    let a = pusher(&mut w, 10, V(4.0 - gap, FOOT + OFFSET, 0.0), Some(0.1));
    for _ in 0..30 {
        push_tick(&mut w, a, b, V(0.05, 0.0, 0.0));
        let (pa, pb) = (position(&w, a), position(&w, b));
        assert!(pb.0 + RADIUS + OFFSET <= 4.75 + 1e-4, "{pb:?}");
        let d = V(pb.0 - pa.0, 0.0, pb.2 - pa.2).length();
        assert!(d >= gap - 1e-4, "{pa:?} {pb:?}");
    }
    let pb = position(&w, b);
    assert!((pb.0 + RADIUS + OFFSET - 4.75).abs() < 1e-4, "{pb:?}");
}

#[test]
fn characters_without_a_ceiling_are_never_pushed() {
    for (ca, cb) in [(Some(0.1), None), (None, Some(0.1))] {
        let mut w = world();
        let gap = 2.0 * RADIUS + OFFSET;
        let a = pusher(&mut w, 10, V(90.0, FOOT + OFFSET, 0.0), ca);
        let b = pusher(&mut w, 11, V(90.0 + gap, FOOT + OFFSET, 0.0), cb);
        for _ in 0..10 {
            let out = push_tick(&mut w, a, b, V(0.05, 0.0, 0.0));
            assert_eq!(out[1].pushed, V::ZERO);
        }
        assert_eq!(position(&w, b), V(90.0 + gap, FOOT + OFFSET, 0.0));
    }
}

/// A free move reports exactly the desired motion, also far from the origin (#336).
#[test]
fn a_free_move_reports_the_desired_translation_exactly() {
    let mut w = world();
    let c = character(&mut w, 10, V(21.0, FOOT + OFFSET, 21.0));
    let desired = V(0.07, 0.0, -0.03);
    let r = step(&mut w, c, desired);
    assert_eq!(r.translation, desired);
    assert_eq!(r.depenetration, V::ZERO);
}

/// Snapping depends on the geometry at the move's start, not on earlier moves, so a world
/// rebuilt from game state snaps on its first move like the original (#340).
#[test]
fn the_first_move_in_a_rebuilt_world_snaps_down_a_ledge() {
    let mut w = world();
    w.add_body(Body::keyed(
        60,
        Shape::Box(V(1.0, 0.015, 1.0)),
        V(100.0, 0.015, 0.0),
        0.0,
    ))
    .unwrap();
    let c = character(&mut w, 10, V(100.9, 0.03 + FOOT + OFFSET, 0.0));
    let r = step(&mut w, c, V(0.5, 0.0, 0.0));
    assert!(r.grounded, "{r:?}");
    assert!(
        (position(&w, c).1 - (FOOT + OFFSET)).abs() < 1e-5,
        "{:?}",
        position(&w, c)
    );
}

/// Ground within Rapier's prediction (offset + 5 cm) counts as support without moving the
/// character (#347).
#[test]
fn ground_within_the_prediction_is_support_without_snapping() {
    let mut w = world();
    // Feet 12 cm above the floor box top, falling 8 cm: they end 4 cm above it.
    let c = character(&mut w, 10, V(20.0, FOOT + 0.12, 0.0));
    let r = step(&mut w, c, V(0.0, -0.08, 0.0));
    assert!(r.grounded, "{r:?}");
    assert_eq!(r.support, w.body_id(1));
    assert!(
        (position(&w, c).1 - (FOOT + 0.04)).abs() < 1e-5,
        "not snapped"
    );
    // Beyond the prediction: not grounded.
    let d = character(&mut w, 11, V(24.0, FOOT + 0.14, 0.0));
    let r = step(&mut w, d, V(0.0, -0.08, 0.0));
    assert!(!r.grounded, "{r:?}");
    // Grounded at the start, the next move snaps down to the offset.
    let r = step(&mut w, c, V(0.0, -0.001, 0.0));
    assert!(r.grounded);
    assert!(
        (position(&w, c).1 - (FOOT + OFFSET)).abs() < 1e-5,
        "{:?}",
        position(&w, c)
    );
}

/// A wedge rising toward -X from x = `run` / 2 (its foot) to height `2 * half_height`, and a
/// character walking -X into it under gravity for 240 ticks. Returns its final position.
fn walk_into_wedge(half_height: f32, run: f32) -> V {
    let mut w = world();
    w.add_body(Body::keyed(
        9,
        Shape::wedge(V(run / 2.0, half_height, 2.0)),
        V(20.0, half_height, 0.0),
        0.0,
    ))
    .unwrap();
    let c = character(&mut w, 10, V(20.0 + run / 2.0 + 1.0, FOOT + OFFSET, 0.0));
    let mut vy = 0.0;
    for _ in 0..240 {
        vy -= 9.81 / 60.0;
        let r = step(&mut w, c, V(-2.0 / 60.0, vy / 60.0, 0.0));
        if r.grounded {
            vy = 0.0;
        }
    }
    position(&w, c)
}

/// Slopes steeper than `max_slope` are walls: walking into one keeps the character at its
/// foot instead of climbing it and sliding back (as Rapier's `max_slope_climb_angle`).
#[test]
fn walking_into_a_too_steep_slope_does_not_climb_it() {
    // 60°: 2.08 m over 1.2 m.
    let p = walk_into_wedge(1.04, 1.2);
    assert!(
        (p.1 - (FOOT + OFFSET)).abs() < 1e-3,
        "stays on the floor at the foot: {p:?}"
    );
    assert!(p.0 > 20.6, "{p:?}");
    // 20°: 2.2 m over 6 m. Climbed and walked on beyond the top.
    let p = walk_into_wedge(1.1, 6.0);
    assert!(p.0 < 17.0, "climbed the shallow slope: {p:?}");
}
