//! An MMORPG-style zone for checkpoint continuation (#452), shared by `tests/checkpoint_zone.rs`
//! and the WASM acceptance example `examples/checkpoint_zone.rs`.
//!
//! A floor halfspace, hilly heightfield terrain over it, a static platform, two kinematic
//! characters (one walks into a crate resting against the platform, turns, pushes the other and
//! climbs the hills with the floor excluded, as MMORPG opens its floor over a lake basin), a
//! crate on the hills that is removed and dropped again under the same key, an impulse, and a
//! far body whose steps fail twice before the game removes it. The game's own state ([`Game`]:
//! the tick, vertical speeds and last ground contact) and its terrain content ([`terrain`]) are
//! what a save holds beside the physics checkpoint.
#![allow(dead_code)]

use physics_engine::BodyId;
use physics_engine::approximate::{
    Autostep, Body, BodyStorage, CharacterMove, CharacterMoveResult, Config, HeightfieldDesc, Real,
    Report, Shape, Vector as V, World,
};

pub const TICKS: u32 = 260;
pub const DT: f64 = 1.0 / 60.0;
pub const FLOOR: u64 = 1;
pub const TERRAIN: u64 = 2;
pub const PLATFORM: u64 = 3;
pub const WALKER: u64 = 10;
pub const IDLER: u64 = 11;
pub const CRATE: u64 = 20;
pub const HILL_CRATE: u64 = 21;
pub const FAR: u64 = 99;
/// Tick boundaries a recovering game captures at: settling, against the crate, the first and
/// second failed tick, on the hill, in the floor-excluded basin (between the idler's moves,
/// while the walker pushes it), standing.
pub const CAPTURES: [u32; 13] = [1, 20, 61, 100, 101, 151, 185, 199, 205, 209, 219, 230, 245];

/// f32 positions near the 1e12 limit are 65536 apart: the far body's approach is scaled to stay
/// resolvable (as in `floating_checkpoints.rs`).
const EDGE: Real = 16_777_216.0;

/// The game's content: the zone's terrain, which it supplies again on restore.
pub fn terrain() -> HeightfieldDesc {
    const N: usize = 9;
    let heights = (0..N * N)
        .map(|k| {
            // A hill rising from the floor at the terrain's edge, then a basin below it (the
            // floor is excluded there, as MMORPG opens it over a lake).
            let (x, z) = ((k % N) as Real * 2.0, (k / N) as Real * 2.0 - 8.0);
            0.8 * (x * 0.39).sin() * (1.0 - 0.01 * z * z)
        })
        .collect();
    let mut removed = vec![false; (N - 1) * (N - 1)];
    // A hole far from the path: removed cells are part of the terrain's identity.
    removed[(N - 2) * (N - 1) + N - 2] = true;
    HeightfieldDesc {
        origin: V(4.0, 0.0, -8.0),
        cell_size: (2.0, 2.0),
        columns: N,
        rows: N,
        heights,
        removed_cells: Some(removed),
        material: Default::default(),
    }
}

/// Game state the consumer saves beside the physics checkpoint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Game {
    pub tick: u32,
    pub vy: [Real; 2],
    pub ground_contact: [bool; 2],
}

pub fn zone(storage: BodyStorage) -> (World, Game) {
    let mut w = World::new(Config {
        body_storage: storage,
        ..Config::default()
    })
    .unwrap();
    w.add_body(Body::keyed(FLOOR, Shape::HalfSpace, V::ZERO, 0.0))
        .unwrap();
    w.add_heightfield(terrain(), Some(TERRAIN)).unwrap();
    w.add_body(Body::keyed(
        PLATFORM,
        Shape::Box(V(2.0, 0.25, 2.0)),
        V(-5.0, 0.25, 0.0),
        0.0,
    ))
    .unwrap();
    // Resting on the floor against the platform's side.
    let mut crate_ = Body::keyed(CRATE, Shape::Box(V(0.4, 0.4, 0.4)), V(-2.6, 0.4, 0.0), 2.0);
    crate_.friction = 0.6;
    w.add_body(crate_).unwrap();
    w.add_body(hill_crate(0.0)).unwrap();
    let mut settings = w.character_settings();
    settings.max_push = Some(0.05);
    for (key, x, z) in [(WALKER, 1.0, 0.0), (IDLER, 16.5, 0.2)] {
        let mut c = Body::keyed(key, Shape::capsule(0.5, 0.3), V(x, 0.9, z), 1.0);
        c.external = true;
        let id = w.add_body(c).unwrap();
        w.add_character(id, settings).unwrap();
    }
    let game = Game {
        tick: 0,
        vy: [0.0; 2],
        ground_contact: [false; 2],
    };
    (w, game)
}

fn hill_crate(z: Real) -> Body {
    let mut b = Body::keyed(
        HILL_CRATE,
        Shape::Box(V(0.3, 0.3, 0.3)),
        V(8.0, 2.0, 3.0 + z),
        1.0,
    );
    b.friction = 0.8;
    b
}

fn id(w: &World, key: u64) -> BodyId {
    w.body_id(key).unwrap()
}

/// One game tick: commands, the character batch, then the step. Returns what the game observes.
pub fn tick(w: &mut World, game: &mut Game, out: &mut Vec<CharacterMoveResult>) -> String {
    let t = game.tick;
    match t {
        40 => {
            let c = id(w, CRATE);
            let p = w.body(c).unwrap().position;
            w.apply_impulse(c, V(0.0, 0.0, 1.5), p + V(0.2, 0.3, 0.0))
                .unwrap();
        }
        70 => {
            w.remove_body(id(w, HILL_CRATE)).unwrap();
            w.add_body(hill_crate(0.5)).unwrap();
        }
        100 => {
            let mut far = Body::keyed(
                FAR,
                Shape::Sphere(0.1),
                V(1e12 - 0.01 * EDGE, 0.0, 0.0),
                1.0,
            );
            far.velocity = V(EDGE, 0.0, 0.0);
            let far = w.add_body(far).unwrap();
            w.add_force(far, V(0.1 * EDGE, 0.0, 0.0)).unwrap();
        }
        102 => {
            w.remove_body(id(w, FAR)).unwrap();
        }
        _ => {}
    }
    let gravity = 9.81 * DT as Real;
    let walk = match t {
        0..60 => -0.05,
        60..230 => 0.12,
        _ => 0.0,
    };
    let floor = id(w, FLOOR);
    let mut moves = vec![];
    for (k, key) in [WALKER, IDLER].into_iter().enumerate() {
        // The idler moves every third tick only: pushes between its moves keep its exclusion.
        if k == 1 && !t.is_multiple_of(3) {
            continue;
        }
        game.vy[k] -= gravity;
        let horizontal = if k == 0 { walk } else { 0.0 };
        moves.push(CharacterMove {
            body: id(w, key),
            desired: V(horizontal, game.vy[k] * DT as Real, 0.0),
            // The idler stays in the basin; the walker crosses it.
            exclude: (k == 1 || (170..240).contains(&t)).then_some(floor),
            autostep: game.ground_contact[k].then_some(Autostep {
                max_height: 0.3,
                min_width: 0.2,
            }),
        });
    }
    w.move_characters(&moves, out).unwrap();
    for (m, r) in moves.iter().zip(out.iter()) {
        let k = usize::from(m.body != id(w, WALKER));
        if r.grounded {
            game.vy[k] = 0.0;
        }
        game.ground_contact[k] = r.ground_contact;
    }
    let step = match w.step(DT) {
        Ok(report) => semantic(&report),
        Err(e) => format!("failed {e:?}"),
    };
    game.tick += 1;
    let pushed = if out.iter().any(|r| r.pushed != V::ZERO) {
        " PUSHED"
    } else {
        ""
    };
    format!("{t}: {out:?} {step}{pushed}")
}

/// What the game sees of a step: work counts, retirements, contact events in order.
pub fn semantic(r: &Report) -> String {
    format!(
        "{:?}",
        (
            r.substeps,
            r.contact_points,
            r.impulse_iterations,
            r.integrated_bodies,
            r.woken_bodies,
            r.swept_contacts,
            &r.retired,
            r.max_penetration.to_bits(),
            r.position.corrections,
            &r.contact_events,
        )
    )
}
