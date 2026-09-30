// This fixture deliberately uses only the pre-optimization public API, so it can
// run unchanged against the pinned production baseline and the candidate.
use physics_engine::{BodyId, RigidBody, Vec3i, World, WorldConfig};
use std::{
    collections::hash_map::DefaultHasher,
    hash::{Hash, Hasher},
    hint::black_box,
    time::Instant,
};

fn world(count: u64, mode: &str) -> World {
    let supported = mode == "supported-gravity";
    let mut world = World::new(WorldConfig {
        gravity: if supported {
            Vec3i::new(0, -1, 0)
        } else {
            Vec3i::ZERO
        },
        ..WorldConfig::default()
    });
    for id in 0..count {
        let rank = id.wrapping_mul(7919) % count;
        let spacing = if mode == "crowded-spawn" { 55 } else { 100 };
        let position = Vec3i::new(
            (rank % 32) as i32 * spacing - 1600,
            90,
            (rank / 32) as i32 * spacing - 3200,
        );
        world
            .add_body(RigidBody::dynamic(
                BodyId(id),
                position,
                Vec3i::ZERO,
                Vec3i::new(30, 90, 30),
            ))
            .unwrap();
    }
    if supported {
        world
            .add_body(RigidBody::fixed(
                BodyId(100_000),
                Vec3i::new(0, -10, 0),
                Vec3i::new(100_000, 10, 100_000),
            ))
            .unwrap();
    }
    world
}

#[test]
#[ignore = "whole-call production baseline comparison; run explicitly in release mode"]
fn translational_consumer_workload_matrix() {
    for (mode, count, moving) in [
        ("quiet-zero-gravity", 128, 0),
        ("quiet-zero-gravity", 512, 0),
        ("quiet-zero-gravity", 2048, 0),
        ("sparse-travel", 512, 1),
        ("sparse-travel", 512, 8),
        ("supported-gravity", 512, 0),
        ("crowded-spawn", 64, 0),
    ] {
        let mut previous_hash = None;
        for trial in 0..3 {
            let mut world = world(count, mode);
            let mut replay = DefaultHasher::new();
            let mut times = Vec::new();
            let mut event_count = 0;
            let mut pair_checks = 0;
            let mut contact_resolutions = 0;
            let mut last = None;
            for tick in 0..120 {
                // Real controllers also repeat unchanged zero-velocity commands.
                for id in 0..moving.max(1) {
                    world
                        .set_velocity(
                            BodyId(id),
                            Vec3i::new(
                                if id < moving {
                                    if tick % 2 == 0 { 1 } else { -1 }
                                } else {
                                    0
                                },
                                0,
                                0,
                            ),
                        )
                        .unwrap();
                }
                let start = Instant::now();
                let report = black_box(world.step(1).unwrap());
                times.push(start.elapsed().as_secs_f64() * 1000.0);
                event_count += report.events.len();
                pair_checks += report.stats.pair_checks;
                contact_resolutions += report.stats.contact_resolutions;
                format!("{:?}", report.events).hash(&mut replay);
                format!("{:?}", world.bodies().collect::<Vec<_>>()).hash(&mut replay);
                last = Some(report.stats);
            }
            let checksum = replay.finish() & ((1_u64 << 53) - 1);
            assert!(
                previous_hash.is_none_or(|previous| previous == checksum),
                "same-build replay changed"
            );
            previous_hash = Some(checksum);
            let total_ms = times.iter().sum::<f64>();
            let raw = format!("{times:?}");
            times.sort_by(f64::total_cmp);
            let p95_ms = times[times.len() * 95 / 100];
            println!(
                "TRANSLATIONAL_WORKLOAD {{\"mode\":\"{mode}\",\"bodies\":{},\"moving\":{moving},\"trial\":{trial},\"ticks\":120,\"events\":{event_count},\"state_checksum_53\":{checksum},\"pair_checks\":{pair_checks},\"contact_resolutions\":{contact_resolutions},\"total_ms\":{total_ms},\"p95_ms\":{p95_ms},\"raw_ms\":{raw}}}",
                world.bodies().count()
            );
            println!(
                "TRANSLATIONAL_LAST_WORK {mode}/{count}/{moving}/{trial}: {:?}",
                last.unwrap()
            );
        }
    }
}
