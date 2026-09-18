use std::{hint::black_box, time::Instant};

use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, Orientation3d, PhysicsWorld3dKernel, RigidBody,
    RigidBox3d, RotatingWorldConfig3d, Vec3i,
};

fn angular() -> AngularState3d {
    AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default())
}

fn dynamic(id: u64, position: Vec3i, velocity: Vec3i) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::dynamic(BodyId(id), position, velocity, Vec3i::new(1, 1, 1)),
        angular(),
    )
    .expect("valid dynamic body")
}

fn external(id: u64, position: Vec3i, velocity: Vec3i) -> RigidBox3d {
    dynamic(id, position, velocity).with_external_motion()
}

fn fixed(id: u64, position: Vec3i) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::fixed(BodyId(id), position, Vec3i::new(1, 1, 1)),
        angular(),
    )
    .expect("valid fixed body")
}

fn world() -> PhysicsWorld3dKernel {
    PhysicsWorld3dKernel::new(RotatingWorldConfig3d {
        gravity: Vec3i::ZERO,
        sample_count: 8,
        refinement_steps: 2,
        solver_passes: 4,
        max_events: 8,
    })
}

#[test]
fn no_response_authority_world_skips_the_rigid_pipeline() {
    let mover_id = BodyId(1);
    let mut world = world();
    world
        .add_box(external(
            mover_id.0,
            Vec3i::new(-2, 0, 0),
            Vec3i::new(6, 0, 0),
        ))
        .expect("external mover");
    world
        .add_box(fixed(2, Vec3i::ZERO))
        .expect("fixed obstacle");

    let report = world.step(1, 1).expect("authority-partitioned step");

    assert_eq!(
        world
            .box_by_id(mover_id)
            .expect("external mover remains present")
            .body()
            .position(),
        Vec3i::new(4, 0, 0),
        "external authority should advance without solver-owned blocking"
    );
    assert_eq!(report.stats.response_authority_body_count, 0);
    assert_eq!(report.stats.solver_body_count, 0);
    assert_eq!(report.stats.solver_bypassed_body_count, 2);
    assert_eq!(report.stats.broad_phase_queries, 0);
    assert_eq!(report.stats.tail_broad_phase_queries, 0);
    assert_eq!(report.stats.sampled_events, 0);
}

#[test]
fn no_authority_pairs_are_rejected_when_a_mutable_dynamic_keeps_the_solver_active() {
    let mut world = world();
    world
        .add_box(dynamic(1, Vec3i::new(1_000, 0, 0), Vec3i::ZERO))
        .expect("mutable dynamic");
    world
        .add_box(external(2, Vec3i::ZERO, Vec3i::ZERO))
        .expect("external body");
    world.add_box(fixed(3, Vec3i::ZERO)).expect("fixed body");

    let report = world.step(1, 60).expect("mixed-authority step");

    assert_eq!(report.stats.response_authority_body_count, 1);
    assert_eq!(report.stats.solver_body_count, 3);
    assert!(
        report.stats.response_authority_pair_rejections >= 1,
        "overlapping external/fixed pair should be rejected before sampled CCD"
    );
    assert_eq!(report.stats.sampled_events, 0);
}

fn populated_world(body_count: u64, external_authority: bool) -> PhysicsWorld3dKernel {
    let mut world = world();
    for id in 1..=body_count {
        let x = i32::try_from(id).expect("small fixture id") * 12;
        let body = dynamic(id, Vec3i::new(x, 0, 0), Vec3i::new(1, 0, 0));
        world
            .add_box(if external_authority {
                body.with_external_motion()
            } else {
                body
            })
            .expect("benchmark body");
    }
    world
}

#[test]
#[ignore = "release-mode deterministic performance evidence; no wall-clock gate"]
fn response_authority_partition_benchmark() {
    for body_count in [128_u64, 256, 512] {
        let mut external_world = populated_world(body_count, true);
        let mut dynamic_world = populated_world(body_count, false);

        for _ in 0..4 {
            black_box(external_world.step(1, 60).expect("warm external step"));
            black_box(dynamic_world.step(1, 60).expect("warm dynamic step"));
        }

        let iterations = 20_u32;
        let external_start = Instant::now();
        let mut external_stats = None;
        for _ in 0..iterations {
            external_stats = Some(black_box(
                external_world
                    .step(1, 60)
                    .expect("measured external step")
                    .stats,
            ));
        }
        let external_elapsed = external_start.elapsed();

        let dynamic_start = Instant::now();
        let mut dynamic_stats = None;
        for _ in 0..iterations {
            dynamic_stats = Some(black_box(
                dynamic_world
                    .step(1, 60)
                    .expect("measured dynamic step")
                    .stats,
            ));
        }
        let dynamic_elapsed = dynamic_start.elapsed();

        let external_stats = external_stats.expect("external evidence");
        let dynamic_stats = dynamic_stats.expect("dynamic evidence");
        println!(
            "response authority: bodies={body_count}, iterations={iterations}, external={external_elapsed:?}, physics_owned={dynamic_elapsed:?}, external_solver_bodies={}, dynamic_solver_bodies={}, external_broad_phase_queries={}, dynamic_broad_phase_queries={}",
            external_stats.solver_body_count,
            dynamic_stats.solver_body_count,
            external_stats.broad_phase_queries,
            dynamic_stats.broad_phase_queries,
        );

        assert_eq!(external_stats.response_authority_body_count, 0);
        assert_eq!(external_stats.solver_body_count, 0);
        assert_eq!(external_stats.broad_phase_queries, 0);
        assert_eq!(
            dynamic_stats.response_authority_body_count,
            body_count as usize
        );
    }
}
