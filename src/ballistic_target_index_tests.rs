use std::{hint::black_box, time::Instant};

use super::*;
use crate::{AngularState3d, AngularVelocity3d, RigidBody};

fn sparse_targets(count: i32) -> Vec<RigidBox3d> {
    (0..count)
        .map(|index| {
            RigidBox3d::new(
                RigidBody::fixed(
                    BodyId(index as u64 + 1),
                    Vec3i::new(0, index * 20, 0),
                    Vec3i::new(5, 4, 4),
                ),
                AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
            )
            .expect("target")
        })
        .collect()
}

fn projectile(index: i32) -> BallisticSphere3d {
    BallisticSphere3d::new(
        BodyId(10_000 + index as u64),
        Vec3i::new(-1_000, (index % 18) * 20, 0),
        Vec3i::new(120_000, 0, 0),
        2,
        1,
    )
    .expect("sphere")
}

#[test]
fn sparse_ballistic_queries_prune_unrelated_targets() {
    for target_count in [32, 256, 2_048] {
        let targets = sparse_targets(target_count);
        let scene = BallisticSphereScene3d::prepare(&targets).expect("scene");
        let step = scene.prepare_step(1, 60).expect("step");
        let mut stats = BallisticSphereQueryStats3d::default();
        let mut bound_checks = 0;
        let hit = step
            .earliest_hit_with_bound_checks(projectile(0), &mut stats, &mut bound_checks)
            .expect("query")
            .expect("hit");
        assert_eq!(hit.body, BodyId(1));
        assert_eq!(stats.toi_tests, 1);
        assert!(
            bound_checks <= 64,
            "{target_count} targets: {bound_checks} checks"
        );
    }
}

#[test]
fn indexed_queries_match_exhaustive_queries_for_moving_rotated_and_filtered_targets() {
    let mut targets = Vec::new();
    for index in 0..96 {
        let orientation = match index % 3 {
            0 => Orientation3d::IDENTITY,
            1 => Orientation3d::new(0, 0, ORIENTATION_SCALE / 2, ORIENTATION_SCALE),
            _ => Orientation3d::new(0, ORIENTATION_SCALE / 2, 0, ORIENTATION_SCALE),
        };
        targets.push(
            RigidBox3d::new(
                RigidBody::dynamic(
                    BodyId(index as u64 + 1),
                    Vec3i::new((index % 4) * 30 - 45, (index / 4) * 20 - 240, 0),
                    Vec3i::new((index % 5) * 60 - 120, (index % 7) * 90 - 270, 0),
                    Vec3i::new(5, 4, 3),
                ),
                AngularState3d::new(orientation, AngularVelocity3d::default()),
            )
            .expect("target")
            .with_collision_layers(CollisionLayers3d::new(1 << (index % 2), u32::MAX)),
        );
    }
    // Exercise input-order independence as well as an odd-sized tree and equal-time ID ties.
    let mut duplicate_geometry = targets[0].clone();
    duplicate_geometry.body.id = BodyId(999);
    targets.push(duplicate_geometry);
    targets.reverse();
    let scene = BallisticSphereScene3d::prepare(&targets).expect("scene");
    let singles = targets
        .iter()
        .map(|target| BallisticSphereScene3d::prepare([target]).expect("single target"))
        .collect::<Vec<_>>();
    let mut hits = 0;
    let mut misses = 0;
    for (numerator, denominator) in [(0, 1), (1, 60), (1, 3), (1, 1)] {
        let step = scene.prepare_step(numerator, denominator).expect("step");
        for lane in -30..=30 {
            let sphere = BallisticSphere3d::new(
                // Includes a target's own ID to exercise self exclusion.
                BodyId((lane + 31) as u64),
                Vec3i::new(-200, lane * 10, 0),
                Vec3i::new(24_000, 0, 0),
                2,
                1,
            )
            .expect("sphere")
            .with_collision_layers(CollisionLayers3d::new(u32::MAX, 1 << (lane & 1)));
            let actual = step
                .earliest_hit(sphere, &mut BallisticSphereQueryStats3d::default())
                .expect("indexed query");
            let expected = singles
                .iter()
                .filter_map(|scene| {
                    scene
                        .earliest_hit(
                            sphere,
                            numerator,
                            denominator,
                            &mut BallisticSphereQueryStats3d::default(),
                        )
                        .expect("exhaustive query")
                })
                .min_by_key(|hit| (hit.time, hit.body));
            assert_eq!(
                actual, expected,
                "lane={lane}, interval={numerator}/{denominator}"
            );
            if actual.is_some() {
                hits += 1;
            } else {
                misses += 1;
            }
        }
    }
    assert!(
        hits > 20 && misses > 20,
        "fixture must cover hits and misses"
    );
}

#[test]
fn empty_index_has_no_query_work() {
    let scene = BallisticSphereScene3d::prepare([]).expect("empty scene");
    let step = scene.prepare_step(1, 60).expect("step");
    let mut bound_checks = 0;
    assert_eq!(
        step.earliest_hit_with_bound_checks(
            projectile(0),
            &mut BallisticSphereQueryStats3d::default(),
            &mut bound_checks
        )
        .expect("query"),
        None
    );
    assert_eq!(bound_checks, 0);
}

#[test]
fn an_existing_edge_overlap_is_not_a_new_ballistic_impact() {
    let target = RigidBox3d::new(
        RigidBody::fixed(BodyId(1), Vec3i::ZERO, Vec3i::new(10, 10, 10)),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("target");
    let scene = BallisticSphereScene3d::prepare([&target]).expect("scene");
    let sphere =
        BallisticSphere3d::new(BodyId(2), Vec3i::new(11, 7, 9), Vec3i::ZERO, 2, 1).expect("sphere");
    assert_eq!(
        scene
            .earliest_hit(sphere, 1, 1, &mut BallisticSphereQueryStats3d::default())
            .expect("query"),
        None
    );
}

#[test]
#[ignore = "advisory release-mode sparse target scaling evidence"]
fn sparse_ballistic_target_benchmark() {
    for target_count in [32, 256, 2_048] {
        let targets = sparse_targets(target_count);
        let scene = BallisticSphereScene3d::prepare(&targets).expect("scene");
        let mut samples = Vec::new();
        let mut bound_checks = 0;
        for _ in 0..15 {
            let start = Instant::now();
            let step = scene.prepare_step(1, 60).expect("step");
            let mut stats = BallisticSphereQueryStats3d::default();
            bound_checks = 0;
            for index in 0..256 {
                assert!(
                    black_box(step.earliest_hit_with_bound_checks(
                        black_box(projectile(index)),
                        &mut stats,
                        &mut bound_checks
                    ))
                    .expect("query")
                    .is_some()
                );
            }
            samples.push(start.elapsed());
            assert_eq!(stats.toi_tests, 256);
            assert_eq!(stats.feature_tests, 256 * 26);
        }
        samples.sort_unstable();
        crate::performance_ratchet::record(
            &format!("ballistic-index/{target_count}"),
            &[("bound_checks", bound_checks)],
            &[("hits", 256)],
            &[("median_ms", samples[7].as_secs_f64() * 1_000.0)],
        );
        println!(
            "BALLISTIC_TARGET_SCALING targets={target_count} projectiles=256 bound_checks={bound_checks} median_ns={}",
            samples[7].as_nanos()
        );
    }
}
