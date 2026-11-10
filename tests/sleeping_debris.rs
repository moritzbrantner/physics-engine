//! Game-owned sleep and exact continuation from canonical state (#524).
use physics_engine::approximate::{
    Body, BodyStorage, Config, ContactGeneration, Quaternion, Shape, Vector as V, World,
};

const DT: f64 = 1.0 / 30.0;
/// Dynamic keys start here; statics take keys below it.
const DYNAMIC: u64 = 1000;

fn config(storage: BodyStorage, generation: ContactGeneration) -> Config {
    Config {
        body_storage: storage,
        gravity: V(0.0, -9.0, 0.0),
        substeps: 2,
        velocity_iterations: 4,
        warm_start: false,
        contact_generation: generation,
        ..Config::default()
    }
}

/// Statics, in the order a rebuild adds them too (they have stable keys).
fn add_statics(w: &mut World) {
    w.add_body(Body::keyed(1, Shape::HalfSpace, V::ZERO, 0.0))
        .unwrap();
    let mut wall = Body::keyed(2, Shape::Box(V(0.5, 2.0, 4.0)), V(-3.5, 1.9, 0.0), 0.0);
    wall.friction = 0.8;
    wall.restitution = 0.15;
    w.add_body(wall).unwrap();
}

fn piece(k: u64) -> Body {
    let x = -1.5 + 0.7 * ((k % 5) as f32);
    let z = -1.0 + 0.7 * ((k / 5) as f32);
    let (shape, y) = match k % 4 {
        0 => (Shape::Box(V(0.3, 0.1, 0.25)), 0.2 + 0.25 * (k % 3) as f32),
        1 => (Shape::Sphere(0.2), 0.3 + 0.5 * (k % 3) as f32),
        2 => (
            Shape::Capsule {
                half_segment: 0.2,
                radius: 0.1,
            },
            0.7,
        ),
        _ => (Shape::Box(V(0.2, 0.2, 0.2)), 0.9),
    };
    let mut b = Body::keyed(DYNAMIC + k, shape, V(x, y, z), 2.0);
    b.friction = 0.8;
    b.restitution = 0.15;
    b.linear_damping = 0.2;
    b.angular_damping = 0.8;
    b.sleep_allowed = false;
    b.orientation = Quaternion(0.1, 0.2 * k as f32, -0.1, 1.0).normalized();
    b.velocity = V(0.6 - 0.15 * (k % 7) as f32, 0.2, 0.3);
    b.angular_velocity = V(1.0, 2.0 - (k % 5) as f32, 0.5);
    b
}

fn live_world(storage: BodyStorage, generation: ContactGeneration) -> World {
    let mut w = World::new(config(storage, generation)).unwrap();
    add_statics(&mut w);
    for k in 0..24 {
        w.add_body(piece(k)).unwrap();
    }
    w
}

/// The canonical record: each dynamic body's key, pose, velocities and sleep flag.
type Canon = Vec<(u64, V, Quaternion, V, V, bool)>;

fn canon(w: &World) -> Canon {
    let mut out = Vec::new();
    for b in w.bodies().filter(|b| b.mass > 0.0) {
        out.push((
            b.key.unwrap(),
            b.position,
            b.orientation,
            b.velocity,
            b.angular_velocity,
            b.is_sleeping(),
        ));
    }
    out
}

fn bits(c: &Canon) -> Vec<(u64, [u32; 16], bool)> {
    let u = |v: V| [v.0.to_bits(), v.1.to_bits(), v.2.to_bits()];
    c.iter()
        .map(|&(k, p, q, v, w, s)| {
            let mut a = [0u32; 16];
            a[..3].copy_from_slice(&u(p));
            a[3..7].copy_from_slice(&[q.0.to_bits(), q.1.to_bits(), q.2.to_bits(), q.3.to_bits()]);
            a[7..10].copy_from_slice(&u(v));
            a[10..13].copy_from_slice(&u(w));
            (k, a, s)
        })
        .collect()
}

/// A world rebuilt only from statics and the canonical record, the pieces in reverse key
/// order (a rebuild's insertion order is not part of the state).
fn rebuilt(c: &Canon, storage: BodyStorage, generation: ContactGeneration) -> World {
    let mut w = World::new(config(storage, generation)).unwrap();
    add_statics(&mut w);
    for &(k, p, q, v, spin, sleeping) in c.iter().rev() {
        let mut b = piece(k - DYNAMIC);
        (b.position, b.orientation, b.velocity, b.angular_velocity) = (p, q, v, spin);
        let id = w.add_body(b).unwrap();
        w.set_sleeping(id, sleeping).unwrap();
    }
    w
}

/// Exact continuation: a rebuilt world, at every tick, steps to exactly the live world's next
/// state, through settling, game-owned sleep, a wake from a second throw and bounces.
#[test]
fn rebuilt_world_continues_bit_identically() {
    for (storage, generation) in [BodyStorage::Ordered, BodyStorage::Indexed]
        .into_iter()
        .flat_map(|s| {
            [
                (s, ContactGeneration::Adaptive),
                (s, ContactGeneration::PerStep),
            ]
        })
    {
        let mut live = live_world(storage, generation);
        let mut woke = 0;
        for tick in 0..240 {
            // Game-owned sleep: settled, slow pieces are put to sleep.
            if tick > 30 {
                let quiet: Vec<_> = live
                    .bodies()
                    .filter(|b| {
                        b.mass > 0.0
                            && !b.is_sleeping()
                            && b.velocity.length() < 0.2
                            && b.angular_velocity.length() < 0.4
                    })
                    .map(|b| b.id)
                    .collect();
                for id in quiet {
                    live.set_sleeping(id, true).unwrap();
                    assert!(live.is_sleeping(id));
                }
            }
            // The second throw: new pieces arrive and wake what they hit.
            if tick == 150 {
                for k in 0..6 {
                    let mut b = piece(100 + k);
                    b.position = V(-1.5 + 0.6 * k as f32, 1.2, -0.2);
                    b.velocity = V(0.0, -5.0, 0.0);
                    live.add_body(b).unwrap();
                }
            }
            let c = canon(&live);
            let mut copy = rebuilt(&c, storage, generation);
            assert_eq!(
                bits(&canon(&copy)),
                bits(&c),
                "rebuild altered state at tick {tick}"
            );
            let report = live.step(DT).unwrap();
            woke += report.woken_bodies;
            let copy_report = copy.step(DT).unwrap();
            assert_eq!(
                bits(&canon(&copy)),
                bits(&canon(&live)),
                "diverged at tick {tick} ({storage:?}, {generation:?})"
            );
            assert_eq!(report.woken_bodies, copy_report.woken_bodies, "tick {tick}");
            // The rebuild also keeps stepping on: more than one step of continuation.
            let mut copy2 = copy.clone();
            let mut live2 = live.clone();
            for _ in 0..3 {
                live2.step(DT).unwrap();
                copy2.step(DT).unwrap();
            }
            assert_eq!(
                bits(&canon(&copy2)),
                bits(&canon(&live2)),
                "diverged 3 ticks after rebuild at tick {tick}"
            );
        }
        assert!(woke > 0, "the second throw never woke a sleeper");
        assert!(
            live.bodies().any(|b| b.is_sleeping()),
            "nothing is left asleep at the end"
        );
    }
}

/// The world keeps its stored bits: re-adding a read-back orientation changes nothing.
#[test]
fn add_body_keeps_a_unit_orientation_exactly() {
    let q = Quaternion(0.1, 0.2, -0.3, 0.9).normalized();
    let mut w = World::new(Config::default()).unwrap();
    let mut b = Body::keyed(1, Shape::Sphere(1.0), V::ZERO, 1.0);
    b.orientation = q;
    let id = w.add_body(b).unwrap();
    let stored = w.body(id).unwrap().orientation;
    assert_eq!(stored.0.to_bits(), q.0.to_bits());
    assert_eq!(stored.3.to_bits(), q.3.to_bits());
    // A clearly non-unit input is still normalized.
    let mut b = Body::keyed(2, Shape::Sphere(1.0), V(5.0, 0.0, 0.0), 1.0);
    b.orientation = Quaternion(0.0, 0.0, 0.0, 2.0);
    let id = w.add_body(b).unwrap();
    assert_eq!(w.body(id).unwrap().orientation, Quaternion::IDENTITY);
}

#[test]
fn set_sleeping_rests_a_body_and_rejects_what_cannot_sleep() {
    let mut w = World::new(config(BodyStorage::Ordered, ContactGeneration::Adaptive)).unwrap();
    let floor = w
        .add_body(Body::keyed(1, Shape::HalfSpace, V::ZERO, 0.0))
        .unwrap();
    let mut b = Body::keyed(2, Shape::Sphere(0.5), V(0.0, 0.5, 0.0), 1.0);
    b.velocity = V(1.0, 2.0, 3.0);
    let id = w.add_body(b).unwrap();
    assert!(!w.is_sleeping(id));
    w.add_force(id, V(0.0, 5.0, 0.0)).unwrap();
    w.set_sleeping(id, true).unwrap();
    assert!(w.is_sleeping(id));
    let body = w.body(id).unwrap();
    assert_eq!(body.velocity, V::ZERO);
    assert_eq!(body.angular_velocity, V::ZERO);
    // It stays put and the world is quiescent.
    assert!(w.is_quiescent());
    let report = w.step(DT).unwrap();
    assert_eq!(report.integrated_bodies, 0);
    assert_eq!(w.body(id).unwrap().position.1, 0.5);
    // Forces and impulses wake it, as do set_sleeping(false) and a velocity change.
    w.apply_impulse(id, V(0.0, 1.0, 0.0), V(0.0, 0.5, 0.0))
        .unwrap();
    assert!(!w.is_sleeping(id));
    w.set_sleeping(id, true).unwrap();
    w.set_sleeping(id, false).unwrap();
    assert!(!w.is_sleeping(id));
    assert!(matches!(
        w.set_sleeping(floor, true),
        Err(physics_engine::approximate::Error::InvalidInput)
    ));
    assert!(!w.is_sleeping(floor));
}

/// Settle, sleep the pile, fly a body in: contact wakes the piece it strikes and, through the
/// touching pile, the rest; with warm starting the same throw wakes through the contact cache.
#[test]
fn contact_from_an_awake_body_wakes_the_touching_pile() {
    for warm_start in [false, true] {
        let mut w = World::new(Config {
            warm_start,
            ..config(BodyStorage::Ordered, ContactGeneration::Adaptive)
        })
        .unwrap();
        w.add_body(Body::keyed(1, Shape::HalfSpace, V::ZERO, 0.0))
            .unwrap();
        let mut ids = Vec::new();
        for k in 0..3 {
            let mut b = Body::keyed(
                2 + k,
                Shape::Box(V(0.5, 0.5, 0.5)),
                V(1.02 * k as f32, 0.5, 0.0),
                1.0,
            );
            b.rotation_locked = true;
            b.sleep_allowed = false;
            ids.push(w.add_body(b).unwrap());
        }
        for _ in 0..30 {
            w.step(DT).unwrap();
        }
        for &id in &ids {
            w.set_sleeping(id, true).unwrap();
        }
        w.step(DT).unwrap();
        assert!(ids.iter().all(|&id| w.is_sleeping(id)));
        let mut b = Body::keyed(9, Shape::Sphere(0.3), V(-3.0, 0.5, 0.0), 2.0);
        b.velocity = V(15.0, 0.0, 0.0);
        b.sleep_allowed = false;
        w.add_body(b).unwrap();
        for _ in 0..30 {
            w.step(DT).unwrap();
        }
        assert!(
            ids.iter().all(|&id| !w.is_sleeping(id)),
            "pile did not wake (warm_start {warm_start})"
        );
    }
}

/// The debris path (#543) admits a fast pair of thin dynamic pieces speculatively over the whole
/// step instead of sweeping each substep: a thin plate thrown down at a thin plank resting on
/// the floor, from any gap up to a whole step's travel, lands on it and never passes through.
#[test]
fn per_step_contacts_stop_a_fast_thin_piece_on_a_thin_piece() {
    for generation in [ContactGeneration::Adaptive, ContactGeneration::PerStep] {
        for speed in [6.0, 12.0, 20.0] {
            // Gaps within one substep's travel, between one substep's and one step's, and
            // beyond the step's.
            let substep = speed * DT as f32 / 2.0;
            for gap in [0.5 * substep, 1.5 * substep, 2.5 * substep] {
                let mut w = World::new(config(BodyStorage::Ordered, generation)).unwrap();
                w.add_body(Body::keyed(1, Shape::HalfSpace, V::ZERO, 0.0))
                    .unwrap();
                let mut plank =
                    Body::keyed(2, Shape::Box(V(0.6, 0.05, 0.6)), V(0.0, 0.05, 0.0), 4.0);
                plank.sleep_allowed = false;
                let plank = w.add_body(plank).unwrap();
                let mut plate = Body::keyed(
                    3,
                    Shape::Box(V(0.3, 0.04, 0.3)),
                    V(0.0, 0.14 + gap, 0.0),
                    2.0,
                );
                plate.velocity = V(0.0, -speed, 0.0);
                plate.sleep_allowed = false;
                let plate = w.add_body(plate).unwrap();
                for tick in 0..60 {
                    w.step(DT).unwrap();
                    let (p, q) = (w.body(plate).unwrap(), w.body(plank).unwrap());
                    assert!(
                        p.position.1 > q.position.1 + 0.05,
                        "{generation:?}: the plate at {speed} m/s from a {gap} m gap passed the \
                         plank at tick {tick}: {:?} vs {:?}",
                        p.position,
                        q.position
                    );
                }
                let p = w.body(plate).unwrap();
                assert!(
                    (p.position.1 - 0.14).abs() < 0.02,
                    "{generation:?}: the plate rests at {:?}",
                    p.position
                );
            }
        }
    }
}
