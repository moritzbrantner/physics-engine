//! `BodyStorage::Ordered` and `BodyStorage::Indexed` give identical results for the same calls:
//! bodies, ids, step results, queries and character moves, through out-of-order adds, removals
//! of resting and touching bodies, key reuse, impulses and steps.
use physics_engine::{
    BodyId,
    approximate::{
        Body, BodyStorage, CharacterMove, CharacterMoveResult, Config, QueryFilter, QueryHit,
        QueryPose, RayCast, RayHit, Shape, ShapeCast, Vector as V, World,
    },
};

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        self.0 >> 33
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
    fn unit(&mut self) -> f32 {
        self.next() as f32 / (1u64 << 31) as f32
    }
}

fn world(storage: BodyStorage) -> World {
    World::new(Config {
        gravity: V(0.0, -9.81, 0.0),
        body_storage: storage,
        ..Config::default()
    })
    .unwrap()
}

/// One step of the script, applied to a world; returns everything observable it produced.
#[derive(Debug, PartialEq)]
enum Seen {
    Added(Result<BodyId, physics_engine::approximate::Error>),
    Removed(Option<Body>),
    Queries(Vec<RayHit>, Option<RayHit>, Vec<QueryHit>, Vec<QueryHit>),
    Moves(Vec<CharacterMoveResult>),
    Stepped(u32, u64),
    Unit,
}

struct Script {
    rng: Rng,
    keys: Vec<u64>,
    characters: Vec<u64>,
}

impl Script {
    fn op(&mut self, tick: u32) -> Op {
        let r = self.rng.below(100);
        let x = (self.rng.unit() - 0.5) * 40.0;
        let z = (self.rng.unit() - 0.5) * 40.0;
        match r {
            // Static boxes with random (out-of-order) keys, some keyless.
            0..=24 => {
                let key = (r < 20).then(|| self.rng.below(10_000));
                Op::Add(
                    key,
                    Shape::Box(V(1.0, 0.5 + self.rng.unit(), 1.0)),
                    V(x, 0.5, z),
                    0.0,
                )
            }
            // Dynamic spheres dropped onto the floor and boxes.
            25..=39 => Op::Add(
                Some(10_000 + self.rng.below(10_000)),
                Shape::Sphere(0.3 + 0.3 * self.rng.unit()),
                V(x, 3.0 + 4.0 * self.rng.unit(), z),
                1.0,
            ),
            // Remove a random live body (resting, touching or static).
            40..=59 if !self.keys.is_empty() => {
                let i = self.rng.below(self.keys.len() as u64) as usize;
                Op::Remove(self.keys.swap_remove(i))
            }
            60..=69 => Op::Impulse(self.rng.below(20_000), V(x * 0.1, 2.0, z * 0.1)),
            70..=79 => Op::Queries(V(x, 10.0, z)),
            80..=84 if tick > 2 => Op::Characters,
            _ => Op::Step,
        }
    }
}

enum Op {
    Add(Option<u64>, Shape, V, f32),
    Remove(u64),
    Impulse(u64, V),
    Queries(V),
    Characters,
    Step,
}

fn apply(w: &mut World, op: &Op, characters: &[u64]) -> Seen {
    match op {
        Op::Add(key, shape, p, mass) => {
            let mut body = Body::new(*shape, *p, *mass);
            body.key = *key;
            Seen::Added(w.add_body(body))
        }
        Op::Remove(key) => Seen::Removed(w.body_id(*key).and_then(|id| w.remove_body(id))),
        Op::Impulse(key, j) => {
            if let Some(id) = w.body_id(*key) {
                let at = w.body(id).unwrap().position;
                let _ = w.apply_impulse(id, *j, at);
            }
            Seen::Unit
        }
        Op::Queries(from) => {
            let mut all = Vec::new();
            let ray = RayCast::new(*from, V(0.5, -20.0, 0.3));
            w.cast_ray(ray, QueryFilter::default(), &mut all).unwrap();
            let first = w.cast_ray_first(ray, QueryFilter::default()).unwrap();
            let mut overlap = Vec::new();
            let pose = QueryPose::new(Shape::capsule(0.5, 0.4), V(from.0, 1.0, from.2));
            w.overlap_shape(pose, QueryFilter::default(), &mut overlap)
                .unwrap();
            let mut cast = Vec::new();
            w.cast_shape(
                ShapeCast::new(pose, V(6.0, 0.0, 2.0)),
                QueryFilter::default(),
                &mut cast,
            )
            .unwrap();
            Seen::Queries(all, first, overlap, cast)
        }
        Op::Characters => {
            let moves = characters
                .iter()
                .filter_map(|&k| w.body_id(k))
                .enumerate()
                .map(|(n, body)| CharacterMove {
                    body,
                    desired: V(0.05 * (n as f32 - 2.0), -0.02, 0.04),
                    exclude: None,
                })
                .collect::<Vec<_>>();
            let mut out = Vec::new();
            w.move_characters(&moves, &mut out).unwrap();
            Seen::Moves(out)
        }
        Op::Step => {
            let r = w.step(1.0 / 60.0).unwrap();
            Seen::Stepped(r.substeps, r.contact_points)
        }
    }
}

fn snapshot(w: &World) -> Vec<Body> {
    w.bodies().cloned().collect()
}

#[test]
fn storage_modes_give_identical_results() {
    for seed in [1u64, 7, 42] {
        let mut ordered = world(BodyStorage::Ordered);
        let mut indexed = world(BodyStorage::Indexed);
        let mut script = Script {
            rng: Rng(seed),
            keys: Vec::new(),
            characters: Vec::new(),
        };
        // A floor box, then characters on it, keyed in the middle of the static key range.
        for w in [&mut ordered, &mut indexed] {
            w.add_body(Body::keyed(
                50_000,
                Shape::Box(V(30.0, 0.5, 30.0)),
                V(0.0, -0.5, 0.0),
                0.0,
            ))
            .unwrap();
        }
        for c in 0..5u64 {
            let key = 5_000 + c * 997;
            script.characters.push(key);
            for w in [&mut ordered, &mut indexed] {
                let mut body = Body::keyed(
                    key,
                    Shape::capsule(0.5, 0.3),
                    V(-6.0 + 3.0 * c as f32, 0.81, 18.0),
                    1.0,
                );
                body.external = true;
                let id = w.add_body(body).unwrap();
                let settings = w.character_settings();
                w.add_character(id, settings).unwrap();
            }
        }
        let characters = script.characters.clone();
        let (mut removed, mut touching_steps) = (0, 0);
        for tick in 0..600 {
            let op = script.op(tick);
            let a = apply(&mut ordered, &op, &characters);
            let b = apply(&mut indexed, &op, &characters);
            assert_eq!(a, b, "seed {seed} tick {tick}");
            removed += usize::from(matches!(a, Seen::Removed(Some(_))));
            // Several calls between steps, then a step.
            let (a, b) = if tick % 3 == 2 {
                (
                    apply(&mut ordered, &Op::Step, &characters),
                    apply(&mut indexed, &Op::Step, &characters),
                )
            } else {
                (a, b)
            };
            assert_eq!(a, b, "seed {seed} tick {tick} step");
            touching_steps += usize::from(matches!(a, Seen::Stepped(_, points) if points > 0));
            if let (Op::Add(Some(key), ..), Seen::Added(Ok(_))) = (&op, &a)
                && !characters.contains(key)
            {
                script.keys.push(*key);
            }
            assert_eq!(
                snapshot(&ordered),
                snapshot(&indexed),
                "seed {seed} tick {tick}"
            );
            assert_eq!(ordered.body_count(), indexed.body_count());
        }
        assert!(
            ordered.body_count() > 20,
            "the script kept a populated world"
        );
        assert!(
            removed > 30 && touching_steps > 30,
            "{removed} removals, {touching_steps} steps"
        );
    }
}

#[test]
fn indexed_storage_adds_out_of_order_without_moving_records_and_settles_on_step() {
    let mut w = world(BodyStorage::Indexed);
    for key in (0..64u64).rev() {
        w.add_body(Body::keyed(
            key,
            Shape::Sphere(0.5),
            V(key as f32 * 2.0, 0.0, 0.0),
            0.0,
        ))
        .unwrap();
    }
    let keys = w.bodies().map(|b| b.key.unwrap()).collect::<Vec<_>>();
    assert_eq!(
        keys,
        (0..64).collect::<Vec<_>>(),
        "bodies() is in body order"
    );
    w.step(1.0 / 60.0).unwrap();
    let keys = w.bodies().map(|b| b.key.unwrap()).collect::<Vec<_>>();
    assert_eq!(keys, (0..64).collect::<Vec<_>>());
}

#[cfg(feature = "checkpoints")]
#[test]
fn checkpoints_keep_the_storage_mode() {
    use physics_engine::approximate::{CheckpointContext, CheckpointLimits};
    let context = CheckpointContext {
        build: [1; 32],
        content: [2; 32],
    };
    let mut w = world(BodyStorage::Indexed);
    for key in [9u64, 3, 7] {
        w.add_body(Body::keyed(
            key,
            Shape::Sphere(0.5),
            V(key as f32, 2.0, 0.0),
            1.0,
        ))
        .unwrap();
    }
    w.remove_body(w.body_id(7).unwrap());
    let bytes = w.checkpoint(context).unwrap().to_bytes();
    let restored = physics_engine::approximate::Checkpoint::from_bytes(
        &bytes,
        context,
        CheckpointLimits::default(),
    )
    .unwrap()
    .restore();
    assert_eq!(restored.config().body_storage, BodyStorage::Indexed);
    assert_eq!(snapshot(&restored), snapshot(&w));
}
