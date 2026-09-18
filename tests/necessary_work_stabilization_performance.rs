use std::{hint::black_box, time::Instant};

use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, Orientation3d, PhysicsWorld3dKernel, RigidBody,
    RigidBox3d, RotatingWorldConfig3d, Vec3i,
};

const FAR_FIXED_BODIES: u64 = 64;

fn rotating_box(body: RigidBody) -> RigidBox3d {
    RigidBox3d::new(
        body,
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("benchmark box must be valid")
}

fn representative_impact_world() -> PhysicsWorld3dKernel {
    let mut world = PhysicsWorld3dKernel::new(RotatingWorldConfig3d {
        gravity: Vec3i::ZERO,
        sample_count: 32,
        refinement_steps: 4,
        solver_passes: 8,
        max_events: 16,
    });
    world
        .add_box(rotating_box(RigidBody::dynamic(
            BodyId(1),
            Vec3i::ZERO,
            Vec3i::ZERO,
            Vec3i::new(3, 3, 3),
        )))
        .expect("target");
    world
        .add_box(rotating_box(RigidBody::dynamic(
            BodyId(2),
            Vec3i::new(-12, 2, 0),
            Vec3i::new(720, 0, 0),
            Vec3i::new(1, 1, 1),
        )))
        .expect("projectile");

    for offset in 0..FAR_FIXED_BODIES {
        let x = 10_000 + i32::try_from(offset).expect("small fixture") * 32;
        world
            .add_box(rotating_box(RigidBody::fixed(
                BodyId(100 + offset),
                Vec3i::new(x, 0, 0),
                Vec3i::new(2, 2, 2),
            )))
            .expect("far fixed geometry");
    }
    world
}

#[test]
fn stabilization_updates_only_the_active_contact_neighborhood() {
    let mut world = representative_impact_world();
    let report = world.step(1, 60).expect("impact step");
    let stats = report.stats;

    assert!(
        stats.sampled_events > 0,
        "fixture must exercise sampled contact work"
    );
    assert!(
        stats.stabilization_active_bodies > 0,
        "fixture must exercise stabilization response work"
    );
    assert_eq!(
        stats.broad_phase_partial_queries, 0,
        "velocity-only stabilization must reuse current contact evidence instead of rediscovering it"
    );
    assert_eq!(
        stats.broad_phase_partial_body_updates, 0,
        "velocity-only stabilization must not update broad-phase geometry"
    );
    assert_eq!(
        stats.stabilization_candidate_pairs, 0,
        "reused current contact evidence must not generate new candidate-pair work"
    );
    assert_eq!(
        stats.stabilization_exact_contacts, 0,
        "reused current contact evidence must not rerun exact OBB contact discovery"
    );
}

#[test]
#[ignore = "release-mode performance evidence; run with --ignored --release --nocapture"]
fn benchmark_necessary_work_stabilization() {
    const SAMPLES: usize = 128;
    let template = representative_impact_world();
    let mut worlds = (0..SAMPLES).map(|_| template.clone()).collect::<Vec<_>>();

    let started = Instant::now();
    let mut partial_queries = 0_u64;
    let mut partial_body_updates = 0_u64;
    let mut sampled_events = 0_u64;
    let mut stabilization_active_bodies = 0_u64;
    for world in &mut worlds {
        let report = black_box(world.step(1, 60).expect("impact step"));
        partial_queries = partial_queries.saturating_add(report.stats.broad_phase_partial_queries);
        partial_body_updates =
            partial_body_updates.saturating_add(report.stats.broad_phase_partial_body_updates);
        sampled_events = sampled_events
            .saturating_add(u64::try_from(report.stats.sampled_events).unwrap_or(u64::MAX));
        stabilization_active_bodies =
            stabilization_active_bodies.saturating_add(report.stats.stabilization_active_bodies);
    }
    let elapsed = started.elapsed();
    let nanos_per_step = elapsed.as_nanos() / u128::try_from(SAMPLES).expect("sample count fits");
    eprintln!(
        "necessary-work stabilization: samples={SAMPLES} ns_per_step={nanos_per_step} sampled_events={sampled_events} stabilization_active_bodies={stabilization_active_bodies} partial_queries={partial_queries} partial_body_updates={partial_body_updates}"
    );
    assert!(sampled_events > 0);
    assert!(stabilization_active_bodies > 0);
    assert_eq!(partial_queries, 0);
    assert_eq!(partial_body_updates, 0);
}
