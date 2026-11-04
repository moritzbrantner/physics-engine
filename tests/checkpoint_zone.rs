//! Checkpoint continuation of an MMORPG-style zone (#452): floor halfspace, heightfield terrain,
//! a static platform, kinematic characters and touching dynamic crates. An uninterrupted run and
//! one that captures, drops the world, restores (supplying the game's terrain content again) and
//! replays the same commands match bit for bit, on both body storages.
#[path = "support/checkpoint_zone.rs"]
mod zone;

use physics_engine::approximate::{
    BodyStorage, Checkpoint, CheckpointContext, CheckpointError, CheckpointLimits, World,
};
use std::time::Instant;
use zone::{CAPTURES, Game, TICKS};

const CONTEXT: CheckpointContext = CheckpointContext {
    build: [41; 32],
    content: [43; 32],
};

fn bytes(w: &World) -> Vec<u8> {
    w.checkpoint(CONTEXT).unwrap().to_bytes()
}

fn restore(saved: &[u8]) -> World {
    Checkpoint::from_bytes(saved, CONTEXT, CheckpointLimits::default())
        .unwrap()
        .restore(&[zone::terrain()])
        .unwrap()
}

/// Every tick's observation and exact physical state (checkpoint bytes: bodies, contact
/// history, impulses, sleep, characters), recovering through a save at each of `captures`.
fn run(storage: BodyStorage, captures: &[u32]) -> (Vec<String>, Vec<Vec<u8>>, Vec<String>) {
    let (mut w, mut game) = zone::zone(storage);
    let mut out = Vec::new();
    let (mut seen, mut states, mut bodies) = (vec![], vec![], vec![]);
    for _ in 0..TICKS {
        if captures.contains(&game.tick) {
            // The game's save: its own state plus the physics bytes.
            let save: (Game, Vec<u8>) = (game, bytes(&w));
            drop(w);
            game = save.0;
            w = restore(&save.1);
            assert!(w.last_report.contact_events.is_empty());
        }
        seen.push(zone::tick(&mut w, &mut game, &mut out));
        states.push(bytes(&w));
        bodies.push(format!("{:?}", w.bodies().collect::<Vec<_>>()));
    }
    (seen, states, bodies)
}

#[test]
fn zone_recovers_bit_identically_through_capture_drop_restore() {
    for storage in [BodyStorage::Ordered, BodyStorage::Indexed] {
        let expected = run(storage, &[]);
        let actual = run(storage, &CAPTURES);
        for t in 0..TICKS as usize {
            assert_eq!(
                actual.0[t], expected.0[t],
                "{storage:?} tick {t}: observation"
            );
            assert_eq!(
                actual.1[t], expected.1[t],
                "{storage:?} tick {t}: physical state"
            );
            assert_eq!(
                actual.2[t], expected.2[t],
                "{storage:?} tick {t}: bodies and bounds"
            );
        }
        // The fixture exercises what it claims: failures, contact events, terrain and
        // platform support, pushes and the floor-excluded climb.
        let all = expected.0.join("\n");
        assert_eq!(all.matches("failed").count(), 2, "{all}");
        assert!(all.contains("Started") && all.contains("Stopped"));
        // Standing on the terrain on the hill and in the floor-excluded basin.
        for t in [150, 210] {
            assert!(
                expected.0[t]
                    .contains("grounded: true, ground_contact: true, support: Some(BodyId(1))"),
                "{}",
                expected.0[t]
            );
        }
        assert!(expected.0.iter().any(|o| o.contains("PUSHED")), "a push");
    }
}

/// Why the checkpoint holds character registrations: a game that registered its characters
/// again after restore (settings from its own data) loses the floor exclusion the idler's last
/// move made, and the walker's push between the idler's moves comes out differently.
#[test]
fn re_registering_characters_after_restore_diverges() {
    let (mut expected, mut game) = zone::zone(BodyStorage::Ordered);
    let mut out = Vec::new();
    while game.tick < 208 {
        zone::tick(&mut expected, &mut game, &mut out);
    }
    let mut reregistered = restore(&bytes(&expected));
    let settings = expected.character_settings();
    for key in [zone::WALKER, zone::IDLER] {
        let id = reregistered.body_id(key).unwrap();
        reregistered
            .add_character(
                id,
                physics_engine::approximate::CharacterSettings {
                    max_push: Some(0.05),
                    ..settings
                },
            )
            .unwrap();
    }
    let mut restored = restore(&bytes(&expected));
    let (mut a, mut b) = (game, game);
    let mut diverged = None;
    for _ in 208..TICKS {
        let e = zone::tick(&mut expected, &mut game, &mut out);
        assert_eq!(zone::tick(&mut restored, &mut a, &mut out), e);
        let r = zone::tick(&mut reregistered, &mut b, &mut out);
        if diverged.is_none() && r != e {
            diverged = Some(game.tick - 1);
        }
    }
    assert!(
        diverged.is_some(),
        "re-registration should lose the idler's exclusion"
    );
}

#[test]
fn restore_takes_the_games_terrain_and_rejects_any_other() {
    let (mut w, _) = zone::zone(BodyStorage::Ordered);
    w.step(zone::DT).unwrap();
    let saved = Checkpoint::from_bytes(&bytes(&w), CONTEXT, CheckpointLimits::default()).unwrap();
    let terrain_body = w.body_id(zone::TERRAIN).unwrap();
    assert_eq!(
        saved.heightfield_bodies().collect::<Vec<_>>(),
        vec![terrain_body]
    );
    let mismatch = |terrain: &[physics_engine::approximate::HeightfieldDesc]| {
        saved.clone().restore(terrain).err()
    };
    let index = |index| Some(CheckpointError::TerrainMismatch { index });
    assert_eq!(mismatch(&[]), index(0));
    assert_eq!(mismatch(&[zone::terrain(), zone::terrain()]), index(1));
    let mut raised = zone::terrain();
    raised.heights[40] += 1e-6;
    assert_eq!(mismatch(&[raised]), index(0));
    let mut filled = zone::terrain();
    filled.removed_cells = None;
    assert_eq!(mismatch(&[filled]), index(0));
    let mut moved = zone::terrain();
    moved.origin.0 += 2.0;
    assert_eq!(mismatch(&[moved]), index(0));
    let mut invalid = zone::terrain();
    invalid.heights.pop();
    assert_eq!(mismatch(&[invalid]), index(0));
    // The material is the body's, saved inline: a different one in the description is ignored.
    let mut material = zone::terrain();
    material.material.friction = 0.1;
    let restored = saved.clone().restore(&[material]).unwrap();
    assert_eq!(bytes(&restored), bytes(&w));
    // Characters come back registered: moving them needs no add_character.
    let mut restored = saved.restore(&[zone::terrain()]).unwrap();
    let mut game = Game {
        tick: 1,
        vy: [0.0; 2],
        ground_contact: [false; 2],
    };
    zone::tick(&mut restored, &mut game, &mut Vec::new());
}

#[test]
fn removed_terrain_keeps_later_terrain_ids() {
    let (mut w, _) = zone::zone(BodyStorage::Ordered);
    let mut second = zone::terrain();
    second.origin.2 += 40.0;
    let second_body = w.add_heightfield(second.clone(), Some(5)).unwrap();
    w.remove_body(w.body_id(zone::TERRAIN).unwrap()).unwrap();
    w.step(zone::DT).unwrap();
    let saved = Checkpoint::from_bytes(&bytes(&w), CONTEXT, CheckpointLimits::default()).unwrap();
    assert_eq!(
        saved.heightfield_bodies().collect::<Vec<_>>(),
        vec![second_body]
    );
    assert_eq!(
        saved.clone().restore(&[zone::terrain()]).err(),
        Some(CheckpointError::TerrainMismatch { index: 0 })
    );
    let mut restored = saved.restore(&[second.clone()]).unwrap();
    assert_eq!(
        format!("{:?}", restored.body(second_body).unwrap().shape),
        format!("{:?}", w.body(second_body).unwrap().shape),
        "terrain ids survive"
    );
    // A terrain added after restore takes the id the original world would give it.
    let mut third = second;
    third.origin.2 += 40.0;
    let a = w.add_heightfield(third.clone(), Some(6)).unwrap();
    let b = restored.add_heightfield(third, Some(6)).unwrap();
    assert_eq!(
        format!("{:?}", w.body(a).unwrap().shape),
        format!("{:?}", restored.body(b).unwrap().shape)
    );
    for _ in 0..30 {
        w.step(zone::DT).unwrap();
        restored.step(zone::DT).unwrap();
    }
    assert_eq!(bytes(&restored), bytes(&w));
}

#[test]
fn damaged_zone_checkpoints_are_rejected_without_a_world() {
    let (mut w, mut game) = zone::zone(BodyStorage::Ordered);
    let mut out = Vec::new();
    // During the floor-excluded climb: characters, exclusion, terrain and floor contacts.
    while game.tick < 160 {
        zone::tick(&mut w, &mut game, &mut out);
    }
    let saved = bytes(&w);
    for length in 0..saved.len() {
        assert!(
            Checkpoint::from_bytes(&saved[..length], CONTEXT, CheckpointLimits::default()).is_err(),
            "truncated to {length}"
        );
    }
    for index in 0..saved.len() {
        for bit in [0, 7] {
            let mut damaged = saved.clone();
            damaged[index] ^= 1 << bit;
            assert!(
                Checkpoint::from_bytes(&damaged, CONTEXT, CheckpointLimits::default()).is_err(),
                "bit {bit} of byte {index}"
            );
        }
    }
    let tight = CheckpointLimits {
        bodies: 3,
        ..CheckpointLimits::default()
    };
    assert_eq!(
        Checkpoint::from_bytes(&saved, CONTEXT, tight).unwrap_err(),
        CheckpointError::ResourceLimit
    );
}

fn rehash(bytes: &mut [u8]) {
    use sha2::{Digest, Sha256};
    let len = bytes.len() - 32;
    let digest = Sha256::digest(&bytes[..len]);
    bytes[len..].copy_from_slice(&digest);
}

/// Halfspace and character records are checked against the bodies. A floor halfspace and one
/// character with default settings (snap, no plane or push, no exclusion) end the payload:
/// halfspace count and id, character count, then id 8, offset, slope, snap tag + value,
/// slides, plane tag, push tag, embedded, exclusion tag and clear flag.
#[test]
fn halfspace_and_character_records_must_match_the_bodies() {
    use physics_engine::approximate::{Body, Config, Shape, Vector as V};
    let mut w = World::new(Config::default()).unwrap();
    let floor = w
        .add_body(Body::keyed(1, Shape::HalfSpace, V::ZERO, 0.0))
        .unwrap();
    let mut c = Body::keyed(2, Shape::capsule(0.5, 0.3), V(0.0, 0.81, 0.0), 1.0);
    c.external = true;
    let c = w.add_body(c).unwrap();
    w.add_character(c, w.character_settings()).unwrap();
    let saved = bytes(&w);
    let end = saved.len() - 32;
    let w_ = size_of::<physics_engine::approximate::Real>();
    let character = end - (8 + 2 * w_ + 1 + w_ + 6);
    let halfspace = character - 8 - 8;
    assert_eq!(
        saved[halfspace..halfspace + 8],
        floor.to_bits().to_le_bytes()
    );
    assert_eq!(saved[character..character + 8], c.to_bits().to_le_bytes());
    let sign = saved[character + 8 + w_ - 1] | 0x80;
    let cases: [(&str, usize, &[u8]); 7] = [
        ("clear flag 2", end - 1, &[2]),
        ("exclusion tag 2", end - 2, &[2]),
        ("embedded 2", end - 3, &[2]),
        ("zero slides", end - 6, &[0]),
        ("negative offset", character + 8 + w_ - 1, &[sign]),
        (
            "halfspace list names the character",
            halfspace,
            &c.to_bits().to_le_bytes(),
        ),
        (
            "character on the halfspace",
            character,
            &floor.to_bits().to_le_bytes(),
        ),
    ];
    for (name, at, value) in cases {
        let mut damaged = saved.clone();
        damaged[at..at + value.len()].copy_from_slice(value);
        rehash(&mut damaged);
        assert_eq!(
            Checkpoint::from_bytes(&damaged, CONTEXT, CheckpointLimits::default()).unwrap_err(),
            CheckpointError::InvalidData,
            "{name}"
        );
    }
    let restored = Checkpoint::from_bytes(&saved, CONTEXT, CheckpointLimits::default())
        .unwrap()
        .restore(&[])
        .unwrap();
    assert_eq!(bytes(&restored), saved);
}

/// Advisory: capture/encode/decode/restore cost and size of the zone checkpoint, apart from
/// the tick cost. `cargo test --release --features checkpoints --test checkpoint_zone -- --ignored --nocapture`
#[test]
#[ignore]
fn zone_checkpoint_cost() {
    let (mut w, mut game) = zone::zone(BodyStorage::Ordered);
    let mut out = Vec::new();
    while game.tick < 160 {
        zone::tick(&mut w, &mut game, &mut out);
    }
    const N: u32 = 2000;
    let median = |mut v: Vec<f64>| {
        v.sort_by(f64::total_cmp);
        v[v.len() / 2]
    };
    let time = |f: &mut dyn FnMut()| {
        let start = Instant::now();
        f();
        start.elapsed().as_secs_f64() * 1e6
    };
    let (mut capture, mut encode, mut decode, mut restore_us, mut ticks) =
        (vec![], vec![], vec![], vec![], vec![]);
    let mut encoded = 0;
    let mut stats = None;
    for _ in 0..N {
        let mut saved = None;
        capture.push(time(&mut || saved = Some(w.checkpoint(CONTEXT).unwrap())));
        let saved = saved.unwrap();
        stats = Some(saved.stats());
        let mut b = vec![];
        encode.push(time(&mut || b = saved.to_bytes()));
        encoded = b.len();
        let mut decoded = None;
        decode.push(time(&mut || {
            decoded =
                Some(Checkpoint::from_bytes(&b, CONTEXT, CheckpointLimits::default()).unwrap())
        }));
        let terrain = [zone::terrain()];
        let mut restored = None;
        restore_us.push(time(&mut || {
            restored = Some(decoded.take().unwrap().restore(&terrain).unwrap())
        }));
        let mut probe = w.clone();
        let mut g = game;
        ticks.push(time(&mut || {
            zone::tick(&mut probe, &mut g, &mut out);
        }));
    }
    println!(
        "zone checkpoint (median µs): capture {:.2}, encode {:.2}, decode {:.2}, restore {:.2} (incl. terrain rebuild); tick {:.2}; encoded {} bytes; {:?}",
        median(capture),
        median(encode),
        median(decode),
        median(restore_us),
        median(ticks),
        encoded,
        stats.unwrap()
    );
}
