use std::{hint::black_box, time::Instant};

use physics_engine::{
    AngularState3d, AngularVelocity3d, BallisticSphere3d, BallisticSphereQueryStats3d,
    BallisticSphereScene3d, BodyId, Orientation3d, RigidBody, RigidBox3d, Vec3i,
};

const TARGET_COUNT: usize = 18;
const PROJECTILE_COUNTS: [usize; 5] = [50, 100, 500, 1_000, 5_000];

fn fixed_target(id: u64, lane: i32) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::fixed(BodyId(id), Vec3i::new(0, lane * 20, 0), Vec3i::new(5, 4, 4)),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid fixed ballistic target")
}

fn target_scene() -> (Vec<RigidBox3d>, BallisticSphereScene3d) {
    let targets = (0..TARGET_COUNT)
        .map(|index| {
            fixed_target(
                u64::try_from(index + 1).expect("small target id"),
                index as i32,
            )
        })
        .collect::<Vec<_>>();
    let scene = BallisticSphereScene3d::prepare(targets.iter()).expect("prepared ballistic scene");
    (targets, scene)
}

fn projectile(index: usize) -> BallisticSphere3d {
    let lane = i32::try_from(index % TARGET_COUNT).expect("small lane");
    BallisticSphere3d::new(
        BodyId(10_000 + u64::try_from(index).expect("projectile id")),
        Vec3i::new(-1_000, lane * 20, 0),
        Vec3i::new(120_000, 0, 0),
        2,
        1,
    )
    .expect("valid ballistic sphere")
}

fn run_queries(
    scene: &BallisticSphereScene3d,
    projectile_count: usize,
) -> (usize, BallisticSphereQueryStats3d) {
    let step = scene.prepare_step(1, 60).expect("prepared 60 Hz step");
    let mut stats = BallisticSphereQueryStats3d::default();
    let mut hit_count = 0_usize;
    for index in 0..projectile_count {
        if black_box(step.earliest_hit(black_box(projectile(index)), &mut stats))
            .expect("ballistic query")
            .is_some()
        {
            hit_count += 1;
        }
    }
    (hit_count, stats)
}

fn assert_linear_work(
    projectile_count: usize,
    hit_count: usize,
    stats: BallisticSphereQueryStats3d,
) {
    assert_eq!(hit_count, projectile_count);
    assert_eq!(stats.broad_phase_candidates, projectile_count as u64);
    assert_eq!(stats.toi_tests, projectile_count as u64);
    assert_eq!(stats.feature_tests, projectile_count as u64 * 26);
}

#[test]
fn five_thousand_projectiles_keep_narrow_phase_work_linear() {
    let (_targets, scene) = target_scene();
    assert_eq!(scene.target_count(), TARGET_COUNT);
    let (hits, stats) = run_queries(&scene, 5_000);
    assert_linear_work(5_000, hits, stats);
}

#[test]
#[ignore = "advisory release-mode ballistic projectile scaling evidence"]
fn ballistic_sphere_scaling_benchmark() {
    let (_targets, scene) = target_scene();

    for projectile_count in PROJECTILE_COUNTS {
        for _ in 0..3 {
            let (hits, stats) = black_box(run_queries(&scene, projectile_count));
            assert_linear_work(projectile_count, hits, stats);
        }

        let mut samples = Vec::with_capacity(15);
        let mut evidence = None;
        for _ in 0..15 {
            let start = Instant::now();
            let (hits, stats) = black_box(run_queries(&scene, projectile_count));
            let elapsed = start.elapsed();
            assert_linear_work(projectile_count, hits, stats);
            evidence = Some((hits, stats));
            samples.push(elapsed);
        }
        samples.sort_unstable();
        let median = samples[samples.len() / 2];
        let (hits, stats) = evidence.expect("measured evidence");
        println!(
            "BALLISTIC_SPHERE_SCALING projectiles={projectile_count} targets={} hits={hits} candidate_bounds={} toi_tests={} feature_tests={} median_ns={}",
            scene.target_count(),
            projectile_count * scene.target_count(),
            stats.toi_tests,
            stats.feature_tests,
            median.as_nanos()
        );
    }
}
