use std::{hint::black_box, time::Instant};

use physics_engine::{
    AngularState3d, AngularVelocity3d, BallisticEventTimelineConfig3d, BallisticSphere3d, BodyId,
    CollisionLayers3d, Orientation3d, RepeatedRotatingEventConfig3d, RigidBody, RigidBox3d,
    RigidBoxFreeFlightConfig3d, RotatingContactSearchConfig3d, Vec3i,
    advance_ballistic_event_timeline,
};

const TARGET_COUNT: usize = 18;
const IMPACT_COUNT: usize = 5;
const PROJECTILE_COUNTS: [usize; 5] = [50, 100, 500, 1_000, 5_000];

fn target(id: u64, lane: i32) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::fixed(
            BodyId(id),
            Vec3i::new(0, lane * 20, 0),
            Vec3i::new(5, 4, 4),
        ),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid target")
    .with_collision_layers(CollisionLayers3d::ALL)
}

fn config() -> BallisticEventTimelineConfig3d {
    BallisticEventTimelineConfig3d::new(RepeatedRotatingEventConfig3d::new(
        RotatingContactSearchConfig3d::new(
            RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 60),
            8,
            2,
        ),
        4,
        16,
    ))
}

fn projectile(index: usize) -> BallisticSphere3d {
    let lane = if index < IMPACT_COUNT {
        i32::try_from(index).expect("small hit lane")
    } else {
        10_000_i32.saturating_add(i32::try_from(index).expect("small miss lane"))
    };
    BallisticSphere3d::new(
        BodyId(10_000 + u64::try_from(index).expect("projectile id")),
        Vec3i::new(-1_000, lane * 20, 0),
        Vec3i::new(120_000, 0, 0),
        2,
        1,
    )
    .expect("valid projectile")
    .with_collision_layers(CollisionLayers3d::ALL)
}

fn run(projectile_count: usize) -> physics_engine::BallisticEventTimelineWorkStats3d {
    let mut boxes = (0..TARGET_COUNT)
        .map(|index| target(u64::try_from(index + 1).expect("target id"), index as i32))
        .collect::<Vec<_>>();
    let mut projectiles = (0..projectile_count).map(projectile).collect::<Vec<_>>();
    let report = advance_ballistic_event_timeline(&mut boxes, &mut projectiles, config())
        .expect("ballistic timeline");

    let expected_impacts = IMPACT_COUNT.min(projectile_count);
    assert_eq!(report.work.ballistic_impacts, expected_impacts as u64);
    assert_eq!(projectiles.len(), projectile_count - expected_impacts);
    assert_eq!(report.work.ballistic_query_rounds, 2);
    assert_eq!(report.work.ballistic_toi_tests, expected_impacts as u64);
    assert_eq!(report.work.ballistic_feature_tests, expected_impacts as u64 * 26);
    assert_eq!(
        report.work.ballistic_target_bound_checks,
        u64::try_from(
            projectile_count * TARGET_COUNT
                + (projectile_count - expected_impacts) * TARGET_COUNT,
        )
        .expect("work count fits")
    );
    report.work
}

#[test]
fn five_thousand_projectiles_with_sparse_impacts_keep_collision_work_bounded() {
    let work = run(5_000);
    assert_eq!(work.ballistic_impacts, 5);
    assert_eq!(work.ballistic_toi_tests, 5);
    assert_eq!(work.ballistic_feature_tests, 130);
    assert_eq!(work.ballistic_query_rounds, 2);
}

#[test]
#[ignore = "advisory release-mode mixed ballistic timeline scaling evidence"]
fn ballistic_event_timeline_scaling_benchmark() {
    for projectile_count in PROJECTILE_COUNTS {
        for _ in 0..3 {
            black_box(run(projectile_count));
        }
        let mut samples = Vec::with_capacity(15);
        let mut evidence = None;
        for _ in 0..15 {
            let start = Instant::now();
            let work = black_box(run(projectile_count));
            samples.push(start.elapsed());
            evidence = Some(work);
        }
        samples.sort_unstable();
        let median = samples[samples.len() / 2];
        let work = evidence.expect("measured work");
        println!(
            "BALLISTIC_EVENT_TIMELINE projectiles={projectile_count} targets={TARGET_COUNT} impacts={} query_rounds={} target_bound_checks={} toi_tests={} feature_tests={} rigid_body_samples={} projectile_motion_samples={} median_ns={}",
            work.ballistic_impacts,
            work.ballistic_query_rounds,
            work.ballistic_target_bound_checks,
            work.ballistic_toi_tests,
            work.ballistic_feature_tests,
            work.rigid_body_motion_samples,
            work.projectile_motion_samples,
            median.as_nanos(),
        );
    }
}
