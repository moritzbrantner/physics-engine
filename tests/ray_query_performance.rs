use std::{hint::black_box, time::Instant};

use physics_engine::{BodyId, Ray, RigidBody, Vec3i, World};

fn populated_world(body_count: u64) -> World {
    let mut world = World::default();
    for id in 0..body_count {
        // Body iteration is BodyId ordered, while this odd multiplier deterministically permutes
        // spatial/time order for power-of-two scene sizes. The prior implementation therefore has
        // genuine ordering work to do instead of receiving an already sorted hit vector.
        let spatial_rank = id.wrapping_mul(7_919) % body_count;
        let x = 8_i32
            + i32::try_from(spatial_rank).expect("benchmark spatial rank fits i32") * 4;
        world
            .add_body(RigidBody::fixed(
                BodyId(id),
                Vec3i::new(x, 0, 0),
                Vec3i::new(1, 1, 1),
            ))
            .expect("unique valid benchmark body");
    }
    world
}

#[test]
fn first_ray_scan_matches_the_prior_full_sort_oracle() {
    let world = populated_world(1_024);
    let rays = [
        Ray::new(Vec3i::ZERO, Vec3i::new(4, 0, 0)),
        Ray::new(Vec3i::new(9, 0, 0), Vec3i::new(4, 0, 0)),
        Ray::new(Vec3i::new(-500, 0, 0), Vec3i::new(17, 0, 0)),
    ];

    for ray in rays {
        let optimized = world.ray_cast_first(ray, 4_096).expect("first-hit query");
        let prior = world
            .ray_cast(ray, 4_096)
            .expect("full ordered query")
            .first()
            .copied();
        assert_eq!(optimized, prior);
    }
}

#[test]
fn first_ray_scan_preserves_equal_time_body_id_ties() {
    let mut world = World::default();
    for id in [BodyId(9), BodyId(2), BodyId(5)] {
        world
            .add_body(RigidBody::fixed(
                id,
                Vec3i::new(20, 0, 0),
                Vec3i::new(2, 2, 2),
            ))
            .expect("unique tie body");
    }

    let ray = Ray::new(Vec3i::ZERO, Vec3i::new(10, 0, 0));
    let optimized = world.ray_cast_first(ray, 4).expect("first-hit query");
    let prior = world
        .ray_cast(ray, 4)
        .expect("full ordered query")
        .first()
        .copied();

    assert_eq!(optimized, prior);
    assert_eq!(optimized.expect("tie should hit").body, BodyId(2));
}

#[test]
#[ignore = "microbenchmark; run explicitly with cargo test --release ray_cast_first_scan_benchmark -- --ignored --nocapture"]
fn ray_cast_first_scan_benchmark() {
    let world = populated_world(4_096);
    let ray = Ray::new(Vec3i::ZERO, Vec3i::new(16, 0, 0));
    let ticks = 2_048;
    let iterations = 200;

    let expected = world
        .ray_cast(ray, ticks)
        .expect("legacy full query")
        .first()
        .copied();
    assert_eq!(
        world.ray_cast_first(ray, ticks).expect("optimized query"),
        expected
    );

    for _ in 0..16 {
        black_box(world.ray_cast_first(black_box(ray), ticks).expect("warmup"));
        black_box(
            world
                .ray_cast(black_box(ray), ticks)
                .expect("legacy warmup")
                .first()
                .copied(),
        );
    }

    let legacy_start = Instant::now();
    for _ in 0..iterations {
        black_box(
            world
                .ray_cast(black_box(ray), ticks)
                .expect("legacy benchmark")
                .first()
                .copied(),
        );
    }
    let legacy_elapsed = legacy_start.elapsed();

    let optimized_start = Instant::now();
    for _ in 0..iterations {
        black_box(
            world
                .ray_cast_first(black_box(ray), ticks)
                .expect("optimized benchmark"),
        );
    }
    let optimized_elapsed = optimized_start.elapsed();

    let speedup = legacy_elapsed.as_secs_f64() / optimized_elapsed.as_secs_f64();
    eprintln!(
        "ray_cast_first 4096 permuted hits × {iterations}: prior_full_sort={legacy_elapsed:?}, single_scan={optimized_elapsed:?}, speedup={speedup:.2}x"
    );
}
