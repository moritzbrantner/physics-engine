//! Reproducible local A/B probe for issue #443. Run with and without `contact-adjacency`:
//! `cargo run --release --locked --example contact_index_probe --features work-counters`.
//! These are informative local timings, not an absolute CI gate.
use physics_engine::approximate::{Body, BodyStorage, Config, Shape, Vector as V, World};
use physics_engine::work::{self, Counter};
use std::{hint::black_box, time::Instant};

const HZ: f64 = 60.0;
const MEASUREMENTS: usize = 3;
const STATIC_MOVE_KEY: u64 = 500_000;
const REMOVABLE_KEY: u64 = 600_000;
const DOOR_KEY: u64 = 700_000;
const UNSUPPORTED_KEY: u64 = 800_000;

#[derive(Clone, Copy, Debug)]
enum Layout {
    Independent,
    SharedPlatform,
}

fn fixture(pairs: u64, inert: u64, storage: BodyStorage, layout: Layout) -> World {
    let mut world = World::new(Config {
        body_storage: storage,
        ..Config::default()
    })
    .unwrap();
    if matches!(layout, Layout::SharedPlatform) {
        let span = (pairs as f32 * 1.5).max(4.0);
        world
            .add_body(Body::keyed(
                0,
                Shape::Box(V(span, 0.5, 2.0)),
                V(0.0, 0.5, 0.0),
                0.0,
            ))
            .unwrap();
    }
    for k in 0..pairs {
        let at = match layout {
            Layout::Independent => V((k % 32) as f32 * 4.0, 0.5, (k / 32) as f32 * 4.0),
            Layout::SharedPlatform => V((k as f32 - pairs as f32 / 2.0) * 2.8, 0.5, 0.0),
        };
        if matches!(layout, Layout::Independent) {
            world
                .add_body(Body::keyed(2 * k, Shape::Box(V(0.5, 0.5, 0.5)), at, 0.0))
                .unwrap();
        }
        let mut sphere = Body::keyed(
            if matches!(layout, Layout::Independent) {
                2 * k + 1
            } else {
                k + 1
            },
            Shape::Sphere(0.3),
            at + V(0.0, 0.8, 0.0),
            1.0,
        );
        sphere.sleep_allowed = false;
        world.add_body(sphere).unwrap();
    }
    for k in 0..inert {
        world
            .add_body(Body::keyed(
                100_000 + k,
                Shape::Box(V(0.5, 0.5, 0.5)),
                V(10_000.0 + (k % 50) as f32 * 4.0, 0.5, (k / 50) as f32 * 4.0),
                0.0,
            ))
            .unwrap();
    }
    for k in 0..48 {
        world
            .add_body(Body::keyed(
                REMOVABLE_KEY + k,
                Shape::Box(V(0.5, 0.5, 0.5)),
                V(-12_000.0 - k as f32 * 4.0, 0.5, 0.0),
                0.0,
            ))
            .unwrap();
    }
    for (key, x, dynamic) in [
        (STATIC_MOVE_KEY, -11_000.0, false),
        (DOOR_KEY, -13_000.0, false),
        (UNSUPPORTED_KEY, -14_000.0, true),
    ] {
        world
            .add_body(Body::keyed(
                key,
                Shape::Sphere(0.3),
                V(x, 0.5, 0.0),
                if dynamic { 1.0 } else { 0.0 },
            ))
            .unwrap();
    }
    for _ in 0..3 {
        world.step(1.0 / HZ).unwrap();
    }
    world
}

#[derive(Clone, Copy, Default)]
struct Measurement {
    ns_per_op: f64,
    visits: u64,
    updates: u64,
}

fn measure(base: &World, repetitions: u32, mut action: impl FnMut(&mut World, u32)) -> Measurement {
    let mut samples = Vec::with_capacity(MEASUREMENTS);
    let mut visits = 0;
    let mut updates = 0;
    for sample in 0..MEASUREMENTS {
        let mut world = base.clone();
        work::reset();
        let start = Instant::now();
        for i in 0..repetitions {
            action(&mut world, i);
        }
        let ns = start.elapsed().as_secs_f64() * 1e9 / f64::from(repetitions);
        black_box(&world);
        if sample == 0 {
            visits = work::get(Counter::CachedPairVisits);
            updates = work::get(Counter::AdjacencyUpdates);
        }
        samples.push(ns);
    }
    samples.sort_by(f64::total_cmp);
    Measurement {
        ns_per_op: samples[MEASUREMENTS / 2],
        visits,
        updates,
    }
}

fn measure_case(pairs: u64, inert: u64, storage: BodyStorage, layout: Layout) {
    work::reset();
    let base = fixture(pairs, inert, storage, layout);
    let initial_updates = work::get(Counter::AdjacencyUpdates);
    #[cfg(feature = "work-counters")]
    let (cache_pairs, index_bytes) = (
        base.cached_pair_count().to_string(),
        base.contact_index_payload_bytes().to_string(),
    );
    #[cfg(not(feature = "work-counters"))]
    let (cache_pairs, index_bytes) = ("unmeasured".to_owned(), "unmeasured".to_owned());

    let support = measure(&base, 8_192, |world, _| {
        let found = world.has_support(world.body_id(UNSUPPORTED_KEY).unwrap());
        black_box(found);
    });
    let movement = measure(&base, 96, |world, i| {
        world
            .set_static_position(
                world.body_id(STATIC_MOVE_KEY).unwrap(),
                V(-11_000.0, 0.5 + 0.1 * (i % 2) as f32, 0.0),
            )
            .unwrap();
    });
    let removal = measure(&base, 48, |world, i| {
        black_box(world.remove_body(world.body_id(REMOVABLE_KEY + u64::from(i)).unwrap()));
    });
    let ticks = measure(&base, 60, |world, i| {
        if i % 10 == 0 {
            black_box(world.has_support(world.body_id(UNSUPPORTED_KEY).unwrap()));
        }
        if i % 20 == 0 {
            world
                .set_static_position(
                    world.body_id(STATIC_MOVE_KEY).unwrap(),
                    V(-11_000.0, 0.5 + 0.1 * ((i / 20) % 2) as f32, 0.0),
                )
                .unwrap();
        }
        if i % 30 == 5 {
            world.remove_body(world.body_id(DOOR_KEY).unwrap()).unwrap();
            world
                .add_body(Body::keyed(
                    DOOR_KEY,
                    Shape::Sphere(0.3),
                    V(-13_000.0, 0.5, 0.0),
                    0.0,
                ))
                .unwrap();
        }
        black_box(world.step(1.0 / HZ).unwrap());
    });
    let hub_removal = matches!(layout, Layout::SharedPlatform).then(|| {
        measure(&base, 1, |world, _| {
            black_box(world.remove_body(world.body_id(0).unwrap()));
        })
    });

    println!(
        "{layout:?},{storage:?},{pairs},{inert},{cache_pairs},{},{index_bytes},{initial_updates},{:.1},{},{:.1},{},{:.1},{},{:.1},{},{},{:.1}",
        size_of::<World>(),
        support.ns_per_op,
        support.visits,
        movement.ns_per_op,
        movement.visits,
        removal.ns_per_op,
        removal.visits,
        ticks.ns_per_op,
        ticks.visits,
        ticks.updates,
        hub_removal.map_or(0.0, |m| m.ns_per_op)
    );
}

fn main() {
    eprintln!(
        "contact-index probe; rustc={}, target={}, features: work-counters={}, contact-adjacency={}",
        option_env!("RUSTC_VERSION").unwrap_or("see cargo -vV"),
        std::env::consts::ARCH,
        cfg!(feature = "work-counters"),
        cfg!(feature = "contact-adjacency")
    );
    println!(
        "layout,storage,pairs,inert,actual_pairs,world_struct_bytes,index_payload_min_bytes,initial_index_updates,support_ns,support_pair_visits,move_ns,move_pair_visits,remove_ns,remove_pair_visits,tick_ns,tick_pair_visits,tick_index_updates,hub_remove_ns"
    );
    for storage in [BodyStorage::Ordered, BodyStorage::Indexed] {
        for (pairs, inert, layout) in [
            (0, 1_150, Layout::Independent),
            (64, 1_150, Layout::Independent),
            (256, 1_150, Layout::Independent),
            (256, 1_150, Layout::SharedPlatform),
            (1_024, 1_150, Layout::Independent),
        ] {
            measure_case(pairs, inert, storage, layout);
        }
    }
}
