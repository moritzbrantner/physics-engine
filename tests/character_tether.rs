//! Character tethers (#557): a rope from a fixed world point to a kinematic character, honoured
//! inside `move_characters` (a grapple hook). The `grapple` Rapier oracle scenario covers the
//! shooter's fixtures; these pin the rule itself and the API.
use physics_engine::{
    BodyId,
    approximate::{
        Body, CharacterMove, CharacterMoveResult, CharacterSettings, Config, Error, Shape, Tether,
        Vector as V, World,
    },
};

const HALF: f32 = 0.6;
const RADIUS: f32 = 0.3;

fn world() -> World {
    World::new(Config {
        gravity: V::ZERO,
        ..Config::default()
    })
    .unwrap()
}

fn character(w: &mut World, key: u64, at: V) -> BodyId {
    let mut body = Body::keyed(key, Shape::capsule(HALF, RADIUS), at, 1.0);
    body.external = true;
    let id = w.add_body(body).unwrap();
    w.add_character(id, CharacterSettings::default()).unwrap();
    id
}

fn tether(anchor: V, length: f32) -> Tether {
    Tether {
        anchor,
        attachment: V::ZERO,
        length,
        max_pull: 0.4,
    }
}

fn step(w: &mut World, id: BodyId, desired: V) -> CharacterMoveResult {
    let mut out = Vec::new();
    w.move_characters(
        &[CharacterMove {
            body: id,
            desired,
            exclude: None,
            autostep: None,
        }],
        &mut out,
    )
    .unwrap();
    out[0]
}

fn close(a: V, b: V) -> bool {
    (a - b).abs().max_component() < 1e-5
}

#[test]
fn a_taut_rope_removes_outward_motion_and_keeps_the_swing() {
    let mut w = world();
    let id = character(&mut w, 1, V(0.0, 5.0, 0.0));
    w.tether_character(id, tether(V(0.0, 10.0, 0.0), 5.0))
        .unwrap();
    // Falling and moving across the rope: the fall goes, the swing stays.
    let r = step(&mut w, id, V(0.1, -0.2, 0.0));
    assert!(close(r.translation, V(0.1, 0.0, 0.0)), "{r:?}");
    assert!(close(r.tether, V(0.0, 0.2, 0.0)), "{r:?}");
    // Inward motion is never touched.
    let r = step(&mut w, id, V(0.0, 0.3, 0.0));
    assert!(close(r.translation, V(0.0, 0.3, 0.0)), "{r:?}");
    assert_eq!(r.tether, V::ZERO);
}

#[test]
fn a_slack_rope_admits_outward_motion_up_to_its_slack() {
    let mut w = world();
    let id = character(&mut w, 1, V(0.0, 7.0, 0.0));
    w.tether_character(id, tether(V(0.0, 10.0, 0.0), 5.0))
        .unwrap();
    let r = step(&mut w, id, V(0.0, -1.5, 0.0));
    assert!(close(r.translation, V(0.0, -1.5, 0.0)), "{r:?}");
    let r = step(&mut w, id, V(0.0, -1.5, 0.0));
    assert!(close(r.translation, V(0.0, -0.5, 0.0)), "{r:?}");
}

#[test]
fn a_stretched_rope_draws_the_character_back_by_at_most_max_pull() {
    let mut w = world();
    let id = character(&mut w, 1, V(0.0, 4.0, 0.0));
    w.tether_character(id, tether(V(0.0, 10.0, 0.0), 5.0))
        .unwrap();
    // A metre past the rope, at rest: drawn in 0.4 m, then 0.4 m, then the last 0.2 m.
    for want in [0.4, 0.4, 0.2, 0.0] {
        let r = step(&mut w, id, V::ZERO);
        assert!(close(r.translation, V(0.0, want, 0.0)), "{want}: {r:?}");
    }
    // Reeling in shortens the rope; a short rope pulls again.
    w.set_tether_length(id, 4.8).unwrap();
    let r = step(&mut w, id, V::ZERO);
    assert!(close(r.translation, V(0.0, 0.2, 0.0)), "{r:?}");
}

#[test]
fn the_attachment_point_is_offset_from_the_centre() {
    let mut w = world();
    let id = character(&mut w, 1, V(0.0, 4.0, 0.0));
    let rope = Tether {
        attachment: V(0.0, 0.75, 0.0),
        ..tether(V(0.0, 10.0, 0.0), 5.25)
    };
    w.tether_character(id, rope).unwrap();
    let r = step(&mut w, id, V(0.0, -0.1, 0.0));
    assert!(close(r.translation, V::ZERO), "{r:?}");
}

#[test]
fn collisions_still_resolve_the_tethered_move() {
    let mut w = world();
    // A wall between the character and the anchor: the pull stops at it.
    w.add_body(Body::keyed(
        10,
        Shape::Box(V(2.0, 2.0, 0.5)),
        V(0.0, 5.0, 3.0),
        0.0,
    ))
    .unwrap();
    let id = character(&mut w, 1, V(0.0, 5.0, 0.0));
    w.tether_character(id, tether(V(0.0, 5.0, 10.0), 1.0))
        .unwrap();
    for _ in 0..60 {
        step(&mut w, id, V::ZERO);
    }
    let z = w.body(id).unwrap().position.2;
    let rest = 2.5 - RADIUS - CharacterSettings::default().offset;
    assert!((z - rest).abs() < 1e-4, "stopped at {z}, wall at {rest}");
}

#[test]
fn tethers_are_set_reeled_and_released_on_characters_only() {
    let mut w = world();
    let id = character(&mut w, 1, V(0.0, 5.0, 0.0));
    let plain = w
        .add_body(Body::keyed(
            2,
            Shape::Box(V(1.0, 1.0, 1.0)),
            V(9.0, 0.0, 0.0),
            0.0,
        ))
        .unwrap();
    let rope = tether(V(0.0, 10.0, 0.0), 5.0);
    assert_eq!(
        w.tether_character(plain, rope),
        Err(Error::MissingBody(plain))
    );
    assert_eq!(w.set_tether_length(id, 3.0), Err(Error::InvalidInput));
    for bad in [
        Tether {
            length: 0.0,
            ..rope
        },
        Tether {
            max_pull: -1.0,
            ..rope
        },
        Tether {
            anchor: V(f32::NAN, 0.0, 0.0),
            ..rope
        },
        Tether {
            attachment: V(0.0, f32::INFINITY, 0.0),
            ..rope
        },
    ] {
        assert_eq!(w.tether_character(id, bad), Err(Error::InvalidInput));
    }
    assert_eq!(w.tether(id), None);
    w.tether_character(id, rope).unwrap();
    w.set_tether_length(id, 3.0).unwrap();
    assert_eq!(w.set_tether_length(id, f32::NAN), Err(Error::InvalidInput));
    assert_eq!(w.tether(id).map(|t| t.length), Some(3.0));
    assert_eq!(w.release_tether(id).map(|t| t.length), Some(3.0));
    assert_eq!(w.release_tether(id), None);
    // Without a tether, the move is untouched.
    let r = step(&mut w, id, V(0.0, -0.2, 0.0));
    assert!(close(r.translation, V(0.0, -0.2, 0.0)), "{r:?}");
    // Adding the character again or removing its body releases the tether.
    w.tether_character(id, rope).unwrap();
    w.add_character(id, CharacterSettings::default()).unwrap();
    assert_eq!(w.tether(id), None);
    w.tether_character(id, rope).unwrap();
    w.remove_body(id).unwrap();
    assert_eq!(w.tether(id), None);
    assert_eq!(w.release_tether(id), None);
}

#[cfg(feature = "checkpoints")]
#[test]
fn a_tethered_world_is_not_checkpointed() {
    use physics_engine::approximate::{CheckpointContext, CheckpointError};
    let mut w = world();
    let id = character(&mut w, 1, V(0.0, 5.0, 0.0));
    w.tether_character(id, tether(V(0.0, 10.0, 0.0), 5.0))
        .unwrap();
    let context = CheckpointContext {
        build: [1; 32],
        content: [2; 32],
    };
    assert_eq!(
        w.checkpoint(context).err(),
        Some(CheckpointError::UnsupportedTethers)
    );
    w.release_tether(id);
    assert!(w.checkpoint(context).is_ok());
}
