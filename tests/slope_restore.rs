//! #394: a character on a heightfield slope moves the same way in a world rebuilt from its
//! position as in the world that carried it there (snapshot/restore determinism).
use physics_engine::BodyId;
use physics_engine::approximate::{
    Body, CharacterMove, CharacterSettings, Config, HeightfieldDesc, QueryFilter, QueryPose, Shape,
    Vector as V, World,
};

/// The mmorpg adapter's scale: centimetres, 4 m cells, a 0.25-unit offset.
const UNIT: f32 = 100.0;
const CELL: f32 = 400.0;
const N: usize = 9;
const GRAVITY: f32 = 9.81 * UNIT / 3600.0;

/// A lake-rim-like bowl: rises 9 m over about 13 m, integer heights as the game authors them.
fn height(i: usize, j: usize) -> f32 {
    let (x, z) = (i as f32 - 4.0, j as f32 - 4.0);
    let r = (x * x + z * z).sqrt();
    (r * 270.0 + 37.0 * (x * 1.3).sin() * (z * 0.7).cos()).round()
}

fn world(at: V) -> (World, BodyId) {
    let mut w = World::new(Config {
        gravity: V::ZERO,
        length_unit: UNIT,
        ..Config::default()
    })
    .unwrap();
    w.add_body(Body::keyed(1, Shape::HalfSpace, V::ZERO, 0.0))
        .unwrap();
    w.add_heightfield(
        HeightfieldDesc {
            origin: V(0.0, 0.0, 0.0),
            cell_size: (CELL, CELL),
            columns: N,
            rows: N,
            heights: (0..N * N).map(|k| height(k % N, k / N)).collect(),
            removed_cells: None,
            material: Default::default(),
        },
        None,
    )
    .unwrap();
    let mut body = Body::keyed(302, Shape::capsule(45.0, 35.0), at, 1.0);
    body.external = true;
    let c = w.add_body(body).unwrap();
    let settings = CharacterSettings {
        offset: 0.25,
        ..w.character_settings()
    };
    w.add_character(c, settings).unwrap();
    (w, c)
}

fn step(w: &mut World, c: BodyId, desired: V) -> (V, bool) {
    let mut out = Vec::new();
    w.move_characters(
        &[CharacterMove {
            body: c,
            desired,
            exclude: None,
            autostep: None,
        }],
        &mut out,
    )
    .unwrap();
    (w.body(c).unwrap().position, out[0].grounded)
}

/// The deepest separation between the character at `p` and anything else (negative: inside).
fn inside(w: &World, c: BodyId, p: V) -> f32 {
    let mut hits = Vec::new();
    w.overlap_shape(
        QueryPose::new(Shape::capsule(45.0, 35.0), p),
        QueryFilter {
            exclude: Some(c),
            ..QueryFilter::default()
        },
        &mut hits,
    )
    .unwrap();
    hits.iter()
        .map(|h| h.separation)
        .fold(f32::INFINITY, f32::min)
}

/// Lurkers wander the rim and stop: every tick, a world rebuilt from the character's
/// position makes exactly the move the running world makes.
#[test]
fn a_rebuilt_world_moves_a_slope_character_as_the_running_one() {
    let mut seed = 0x2545_f491_4f6c_dd1du64;
    let mut rand = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 40) as f32 / (1u64 << 24) as f32
    };
    let mut failures = Vec::new();
    for run in 0..12 {
        let start = V(
            CELL * (1.0 + 6.0 * rand()),
            2000.0,
            CELL * (1.0 + 6.0 * rand()),
        );
        let (mut w, c) = world(start);
        let mut velocity = V::ZERO;
        let mut grounded = false;
        for tick in 0..400 {
            if tick % 40 == 0 {
                let walking = rand() < 0.6;
                let angle = rand() * core::f32::consts::TAU;
                let speed = if walking { 2.0 + 3.0 * rand() } else { 0.0 };
                velocity.0 = speed * angle.cos();
                velocity.2 = speed * angle.sin();
            }
            if grounded && velocity.1 <= 0.0 {
                velocity.1 = 0.0;
            } else {
                velocity.1 -= GRAVITY;
            }
            let here = w.body(c).unwrap().position;
            let (mut restored, rc) = world(here);
            let (p, g) = step(&mut w, c, velocity);
            let (q, _) = step(&mut restored, rc, velocity);
            // The root cause: a slide up the slope ran into the next triangle and ended
            // inside it, which only the rebuilt world's first move depenetrated.
            let depth = inside(&w, c, p);
            if p != q || depth < 0.0 {
                failures.push(format!(
                    "run {run} tick {tick}: from {here:?} running {p:?} (separation \
                     {depth}), rebuilt {q:?}"
                ));
                break;
            }
            grounded = g;
            if grounded {
                velocity.1 = 0.0;
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// The uneven Hollow approach from MMORPG: the slide can touch the adjacent triangle
/// near a cell diagonal even though its starting pose was clear.
fn hollow_world(at: V) -> (World, BodyId) {
    let mut w = World::new(Config {
        gravity: V::ZERO,
        length_unit: UNIT,
        ..Config::default()
    })
    .unwrap();
    w.add_heightfield(
        HeightfieldDesc {
            origin: V(800.0, 0.0, -8000.0),
            cell_size: (CELL, CELL),
            columns: 5,
            rows: 5,
            heights: vec![
                1100., 1100., 1100., 1100., 1100., 1100., 1100., 1100., 1100., 1100., 1287., 1150.,
                1100., 1100., 1100., 1027., 1242., 1198., 1100., 1100., 324., 560., 560., 560.,
                560.,
            ],
            removed_cells: None,
            material: Default::default(),
        },
        None,
    )
    .unwrap();
    let mut body = Body::keyed(401, Shape::capsule(60., 30.), at, 1.);
    body.external = true;
    let c = w.add_body(body).unwrap();
    w.add_character(
        c,
        CharacterSettings {
            offset: 0.25,
            ..w.character_settings()
        },
    )
    .unwrap();
    (w, c)
}

#[test]
fn hollow_triangle_seams_preserve_reconstruction_after_walking_and_stopping() {
    let (mut w, c) = hollow_world(V(1204.7852, 1190.25, -7605.4375));
    let (p, _) = step(&mut w, c, V(-1.5664063, 0., 3.6953125));
    let mut hits = Vec::new();
    w.overlap_shape(
        QueryPose::new(Shape::capsule(60., 30.), p),
        QueryFilter {
            exclude: Some(c),
            ..QueryFilter::default()
        },
        &mut hits,
    )
    .unwrap();
    assert!(
        hits.iter().all(|hit| hit.separation >= 0.),
        "end {p:?}: {hits:?}"
    );
    let (mut restored, rc) = hollow_world(p);
    assert_eq!(step(&mut w, c, V::ZERO), step(&mut restored, rc, V::ZERO));
}
