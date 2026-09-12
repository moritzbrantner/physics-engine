use std::{hint::black_box, time::Instant};

use crate::{
    AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d,
    RigidBoxFreeFlightConfig3d, Vec3i,
    rotating_broad_phase::{RotatingBroadPhase3d, rotational_sweep_candidate_pairs},
};

#[derive(Clone, Copy, Debug)]
enum Workload {
    Static,
    SingleChurn,
    SixteenChurn,
    MajorityChurn,
    Dense,
}

impl Workload {
    const fn name(self) -> &'static str {
        match self {
            Self::Static => "static",
            Self::SingleChurn => "single-churn",
            Self::SixteenChurn => "16-churn",
            Self::MajorityChurn => "majority-churn",
            Self::Dense => "dense",
        }
    }
}

fn dynamic_box(id: u64, position: Vec3i, velocity: Vec3i) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::dynamic(BodyId(id), position, velocity, Vec3i::new(2, 2, 2)),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid benchmark box")
}

fn base_scene(size: usize, dense: bool) -> Vec<RigidBox3d> {
    let side = (size as f64).sqrt().ceil() as usize;
    (0..size)
        .map(|index| {
            let x = if dense {
                0
            } else {
                i32::try_from(index % side).expect("benchmark x fits i32") * 12
            };
            let y = if dense {
                0
            } else {
                i32::try_from(index / side).expect("benchmark y fits i32") * 12
            };
            let velocity = if dense {
                Vec3i::ZERO
            } else {
                Vec3i::new(i32::try_from(index % 3).expect("small velocity") - 1, 0, 0)
            };
            dynamic_box(
                u64::try_from(index).expect("benchmark id fits u64") + 1,
                Vec3i::new(x, y, 0),
                velocity,
            )
        })
        .collect()
}

fn apply_workload(scene: &mut [RigidBox3d], workload: Workload, iteration: usize) {
    let moved = match workload {
        Workload::Static | Workload::Dense => 0,
        Workload::SingleChurn => 1,
        Workload::SixteenChurn => scene.len().min(16),
        Workload::MajorityChurn => scene.len() / 2 + 1,
    };
    for (index, rigid_box) in scene.iter_mut().take(moved).enumerate() {
        let position = Vec3i::new(
            100_000
                + i32::try_from(iteration).expect("small iteration") * 200
                + i32::try_from(index).expect("small index") * 12,
            i32::try_from(index % 17).expect("small y") * 12,
            0,
        );
        *rigid_box = dynamic_box(
            u64::try_from(index).expect("benchmark id fits u64") + 1,
            position,
            Vec3i::ZERO,
        );
    }
}

fn iterations_for(size: usize, workload: Workload) -> usize {
    if matches!(workload, Workload::Dense) {
        return 1;
    }
    match size {
        0..=256 => 24,
        257..=1024 => 8,
        _ => 2,
    }
}

fn benchmark_case(size: usize, workload: Workload) {
    let dense = matches!(workload, Workload::Dense);
    let config = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 60);
    let iterations = iterations_for(size, workload);
    let mut correctness_scene = base_scene(size, dense);
    let mut correctness_persistent = RotatingBroadPhase3d::default();

    for iteration in 0..iterations.min(3) {
        apply_workload(&mut correctness_scene, workload, iteration);
        let expected = rotational_sweep_candidate_pairs(&correctness_scene, config)
            .expect("stateless benchmark oracle");
        let actual = correctness_persistent
            .candidate_pairs(&correctness_scene, config)
            .expect("persistent benchmark query");
        assert_eq!(actual, expected, "{} {} correctness", size, workload.name());
    }

    let mut stateless_scene = base_scene(size, dense);
    let stateless_start = Instant::now();
    let mut stateless_candidates = 0_usize;
    for iteration in 0..iterations {
        apply_workload(&mut stateless_scene, workload, iteration);
        let pairs = rotational_sweep_candidate_pairs(black_box(&stateless_scene), config)
            .expect("stateless timed query");
        stateless_candidates = pairs.len();
        black_box(pairs);
    }
    let stateless_elapsed = stateless_start.elapsed();

    let mut persistent_scene = base_scene(size, dense);
    let mut persistent = RotatingBroadPhase3d::default();
    let persistent_start = Instant::now();
    let mut persistent_candidates = 0_usize;
    for iteration in 0..iterations {
        apply_workload(&mut persistent_scene, workload, iteration);
        let pairs = persistent
            .candidate_pairs(black_box(&persistent_scene), config)
            .expect("persistent timed query");
        persistent_candidates = pairs.len();
        black_box(pairs);
    }
    let persistent_elapsed = persistent_start.elapsed();

    assert_eq!(persistent_candidates, stateless_candidates);
    let speedup = stateless_elapsed.as_secs_f64() / persistent_elapsed.as_secs_f64();
    eprintln!(
        "broadphase-matrix size={size} workload={} iterations={iterations} stateless={stateless_elapsed:?} persistent={persistent_elapsed:?} ratio={speedup:.3}x candidates={persistent_candidates} stats={:?}",
        workload.name(),
        persistent.stats(),
    );
}

#[test]
#[ignore = "release-mode performance matrix; run through Performance Evidence"]
fn broad_phase_workload_matrix() {
    for size in [256_usize, 1024, 4096] {
        for workload in [
            Workload::Static,
            Workload::SingleChurn,
            Workload::SixteenChurn,
            Workload::MajorityChurn,
            Workload::Dense,
        ] {
            benchmark_case(size, workload);
        }
    }
}
