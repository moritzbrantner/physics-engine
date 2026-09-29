use std::{hint::black_box, time::Instant};

use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, Orientation3d, PhysicsWorld3dKernel, RigidBody,
    RigidBox3d, RotatingWorldConfig3d, Vec3i,
};

fn moving_body(id: u64, overlap_only: bool) -> RigidBox3d {
    let x = i32::try_from(id).expect("small fixture id") * 12;
    let body = RigidBox3d::new(
        RigidBody::dynamic(
            BodyId(id),
            Vec3i::new(x, 0, 0),
            Vec3i::new(1, 0, 0),
            Vec3i::new(1, 1, 1),
        ),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid moving body")
    .with_external_motion();
    if overlap_only {
        body.with_overlap_only()
    } else {
        body
    }
}

fn world(body_count: u64, overlap_only: bool) -> PhysicsWorld3dKernel {
    let mut world = PhysicsWorld3dKernel::new(RotatingWorldConfig3d {
        gravity: Vec3i::ZERO,
        sample_count: 8,
        refinement_steps: 2,
        solver_passes: 4,
        max_events: 8,
    });
    for id in 1..=body_count {
        world
            .add_box(moving_body(id, overlap_only))
            .expect("add benchmark body");
    }
    world
}

#[test]
fn overlap_only_partition_reports_zero_rigid_solver_work() {
    let body_count = 64_u64;
    let mut sensors = world(body_count, true);
    let report = sensors.step(1, 60).expect("partitioned sensor step");

    assert_eq!(report.stats.body_count, body_count as usize);
    assert_eq!(report.stats.solver_body_count, 0);
    assert_eq!(report.stats.solver_bypassed_body_count, body_count as usize);
    assert_eq!(report.stats.broad_phase_queries, 0);
    assert_eq!(report.stats.tail_broad_phase_queries, 0);
    assert_eq!(report.stats.sampled_events, 0);
}

#[test]
#[ignore = "release-mode deterministic performance evidence; no wall-clock gate"]
fn solver_partition_benchmark() {
    for body_count in [128_u64, 256, 512] {
        let mut sensors = world(body_count, true);
        let mut solids = world(body_count, false);

        for _ in 0..4 {
            black_box(sensors.step(1, 60).expect("warm sensor step"));
            black_box(solids.step(1, 60).expect("warm solid step"));
        }

        let iterations = 20_u32;
        let sensor_start = Instant::now();
        let mut sensor_work = None;
        for _ in 0..iterations {
            sensor_work = Some(black_box(
                sensors.step(1, 60).expect("measured sensor step").stats,
            ));
        }
        let sensor_elapsed = sensor_start.elapsed();

        let solid_start = Instant::now();
        let mut solid_work = None;
        for _ in 0..iterations {
            solid_work = Some(black_box(
                solids.step(1, 60).expect("measured solid step").stats,
            ));
        }
        let solid_elapsed = solid_start.elapsed();

        let sensor_work = sensor_work.expect("sensor evidence");
        let solid_work = solid_work.expect("solid evidence");
        println!(
            "solver partition: bodies={body_count}, iterations={iterations}, overlap_only={sensor_elapsed:?}, solid={solid_elapsed:?}, sensor_solver_bodies={}, solid_solver_bodies={}, sensor_broad_phase_queries={}, solid_broad_phase_queries={}",
            sensor_work.solver_body_count,
            solid_work.solver_body_count,
            sensor_work.broad_phase_queries,
            solid_work.broad_phase_queries,
        );
    }
}
