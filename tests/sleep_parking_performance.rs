use std::{hint::black_box, time::Instant};

use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, Orientation3d, PhysicsWorld3dKernel, RigidBody,
    RigidBox3d, RotatingWorldConfig3d, RotatingWorldStepStats3d, Vec3i,
};

fn dynamic(id: u64, position: Vec3i) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::dynamic(BodyId(id), position, Vec3i::ZERO, Vec3i::new(1, 1, 1)),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid dynamic box")
}

fn parked_world(body_count: u64) -> PhysicsWorld3dKernel {
    let mut world = PhysicsWorld3dKernel::new(RotatingWorldConfig3d {
        gravity: Vec3i::ZERO,
        sample_count: 8,
        refinement_steps: 2,
        solver_passes: 4,
        max_events: 8,
    });
    for id in 1..=body_count {
        let x = i32::try_from(id).expect("small fixture") * 8;
        world
            .add_box(dynamic(id, Vec3i::new(x, 0, 0)))
            .expect("add body");
    }

    for _ in 0..60 {
        world.step(1, 60).expect("settling step");
        if world.sleeping_body_count() == usize::try_from(body_count).expect("small fixture") {
            return world;
        }
    }
    panic!("fixture did not reach parked sleep state");
}

#[test]
fn parked_world_reports_zero_collision_tail_and_tree_work() {
    let body_count = 64_u64;
    let mut world = parked_world(body_count);
    let before = world.boxes().cloned().collect::<Vec<_>>();

    let report = world.step(1, 60).expect("quiescent parked step");

    assert_eq!(
        report.stats,
        RotatingWorldStepStats3d {
            body_count: usize::try_from(body_count).expect("small fixture"),
            ..RotatingWorldStepStats3d::default()
        }
    );
    assert_eq!(world.boxes().cloned().collect::<Vec<_>>(), before);
}

#[test]
#[ignore = "release-mode performance evidence"]
fn parked_world_step_benchmark() {
    for body_count in [32_u64, 64, 128] {
        let mut world = parked_world(body_count);
        for _ in 0..100 {
            black_box(world.step(1, 60).expect("warm parked step"));
        }

        let iterations = 50_000_u32;
        let start = Instant::now();
        for _ in 0..iterations {
            black_box(world.step(1, 60).expect("measured parked step"));
        }
        let elapsed = start.elapsed();
        println!(
            "parked sleeping world: bodies={body_count}, iterations={iterations}, total={elapsed:?}, per_step={:?}",
            elapsed / iterations
        );
    }
}
