//! `World::remove_bodies` against removing the same bodies one by one in body order (#523): the
//! surviving world, the wakes, the contact events and the checkpoints are identical.
use physics_engine::{
    BodyId,
    approximate::{Body, BodyStorage, Config, DistanceJoint, Error, Shape, Vector as V, World},
};

const DT: f64 = 1.0 / 60.0;
const BOX: V = V(0.5, 0.5, 0.5);

fn stack(w: &mut World, column: u64, height: u64) -> Vec<BodyId> {
    (0..height)
        .map(|j| {
            let p = V(column as f32 * 3.0, 0.5 + j as f32, 0.0);
            w.add_body(Body::keyed(100 * column + j + 10, Shape::Box(BOX), p, 1.0))
                .unwrap()
        })
        .collect()
}

/// Stacks asleep and awake on a floor, a fixed plate under one, a rope and keyless debris; returns
/// the world and the bodies to remove.
fn scene(storage: BodyStorage, sleep: bool, rope: bool) -> (World, Vec<BodyId>) {
    let mut w = World::new(Config {
        body_storage: storage,
        ..Config::default()
    })
    .unwrap();
    w.add_body(Body::keyed(
        0,
        Shape::Box(V(50.0, 0.5, 50.0)),
        V(0.0, -0.5, 0.0),
        0.0,
    ))
    .unwrap();
    let plate = w
        .add_body(Body::keyed(1, Shape::Box(BOX), V(12.0, 0.5, 3.0), 0.0))
        .unwrap();
    let stacks: Vec<_> = (0..5).map(|c| stack(&mut w, c, 4)).collect();
    let on_plate = w
        .add_body(Body::keyed(90, Shape::Box(BOX), V(12.0, 1.5, 3.0), 1.0))
        .unwrap();
    if rope {
        let held = w
            .add_body(Body::keyed(91, Shape::Box(BOX), V(0.0, 4.5, 6.0), 1.0))
            .unwrap();
        w.add_joint(DistanceJoint::rope(stacks[3][3], held, 2.0))
            .unwrap();
    }
    for _ in 0..if sleep { 300 } else { 2 } {
        w.step(DT).unwrap();
    }
    // Awake debris that touches stack 0 and appends to storage; the sleeping stacks stay down.
    let debris: Vec<_> = (0..3)
        .map(|k| {
            let p = V(-0.1 + 0.1 * k as f32, 4.5, 0.0);
            w.add_body(Body::new(Shape::Box(V(0.2, 0.5, 0.2)), p, 1.0))
                .unwrap()
        })
        .collect();
    w.step(DT).unwrap();
    let mut ids = vec![
        stacks[1][1], // middle of a sleeping stack: the rest of it wakes
        stacks[2][3], // its top: its neighbour's island wakes
        stacks[3][3], // roped: the held body wakes
        plate,        // a fixed support: what stands on it wakes
        debris[1],
        debris[2],
        on_plate,
    ];
    // A scrambled batch: the result must not depend on it.
    ids.reverse();
    ids.swap(1, 4);
    (w, ids)
}

fn body_order(w: &World, ids: &[BodyId]) -> Vec<BodyId> {
    let order: Vec<_> = w.bodies().map(|b| b.id).collect();
    let mut sorted = ids.to_vec();
    sorted.sort_by_key(|id| order.iter().position(|o| o == id).unwrap());
    sorted
}

#[cfg(feature = "checkpoints")]
fn checkpoint_bytes(w: &World) -> Vec<u8> {
    use physics_engine::approximate::CheckpointContext;
    let context = CheckpointContext {
        build: [1; 32],
        content: [2; 32],
    };
    w.checkpoint(context).unwrap().to_bytes()
}

fn assert_same(a: &World, b: &World) {
    let (x, y): (Vec<_>, Vec<_>) = (a.bodies().collect(), b.bodies().collect());
    assert_eq!(x, y);
    assert_eq!(a.is_quiescent(), b.is_quiescent());
}

#[test]
fn a_batch_matches_sequential_removal_in_body_order() {
    let storages = [BodyStorage::Ordered, BodyStorage::Indexed];
    for (storage, sleep, rope) in storages
        .into_iter()
        .flat_map(|s| [(s, true, false), (s, true, true), (s, false, false)])
    {
        let (mut a, ids) = scene(storage, sleep, rope);
        let sleepers = a.bodies().filter(|b| b.is_sleeping()).count();
        assert_eq!(sleepers > 0, sleep, "the scene must test waking");
        let mut b = a.clone();
        let ordered = body_order(&a, &ids);
        let seq: Vec<_> = ordered
            .iter()
            .map(|&id| a.remove_body(id).unwrap())
            .collect();
        let batch = b.remove_bodies(&ids).unwrap();
        assert_eq!(
            batch.iter().map(|x| x.id).collect::<Vec<_>>(),
            ids,
            "returned in the caller's order"
        );
        // The returned records hold the state at the call: a batch does not wake its own bodies,
        // so compare what the body is, not whether a co-removed body woke it first.
        let key = |x: &Body| (x.id, x.key, x.position, x.velocity, x.orientation);
        let sorted: Vec<_> = ordered
            .iter()
            .map(|id| batch.iter().find(|x| x.id == *id).unwrap())
            .collect();
        assert_eq!(
            seq.iter().map(key).collect::<Vec<_>>(),
            sorted.iter().map(|x| key(x)).collect::<Vec<_>>()
        );
        assert_same(&a, &b);
        assert_eq!(a.body_count(), b.body_count());
        assert!(
            !sleep || a.bodies().filter(|x| x.is_sleeping()).count() < sleepers,
            "the removals must wake some island"
        );
        // Slots, ids, events and every later state follow.
        let fresh = Body::new(Shape::Sphere(0.2), V(30.0, 5.0, 0.0), 1.0);
        assert_eq!(
            a.add_body(fresh.clone()).unwrap(),
            b.add_body(fresh).unwrap()
        );
        for _ in 0..60 {
            assert_eq!(
                a.step(DT).unwrap().contact_events,
                b.step(DT).unwrap().contact_events
            );
            assert_same(&a, &b);
        }
        #[cfg(feature = "checkpoints")]
        if !rope {
            assert_eq!(checkpoint_bytes(&a), checkpoint_bytes(&b));
        }
    }
}

#[test]
fn an_invalid_batch_changes_nothing() {
    let (mut w, ids) = scene(BodyStorage::Ordered, false, false);
    let snapshot = w.clone();
    let mut stale = ids[1..].to_vec();
    let gone = w.remove_body(ids[0]).unwrap().id;
    stale.push(gone);
    let reference = w.clone();
    assert_eq!(w.remove_bodies(&stale), Err(Error::MissingBody(gone)));
    assert_same(&w, &reference);
    let (mut w, ids) = (snapshot.clone(), ids);
    assert_eq!(w.remove_bodies(&[ids[0], ids[0]]), Err(Error::InvalidInput));
    assert_same(&w, &snapshot);
    assert_eq!(w.body_count(), snapshot.body_count());
}
