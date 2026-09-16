use std::{hint::black_box, time::Instant};

use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d,
    RotatingWorld3d, RotatingWorldConfig3d, Vec3i,
};

const UNRELATED_DYNAMIC_BODIES: u64 = 256;

fn rotating(body: RigidBody) -> RigidBox3d {
    RigidBox3d::new(
        body,
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid box")
}

fn world(sample: u64) -> RotatingWorld3d {
    let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
        gravity: Vec3i::ZERO,
        sample_count: 8,
        refinement_steps: 2,
        solver_passes: 4,
        max_events: 8,
    });
    world
        .add_box(rotating(RigidBody::dynamic(
            BodyId(1),
            Vec3i::ZERO,
            Vec3i::new(60, 0, 0),
            Vec3i::new(1, 1, 1),
        )))
        .expect("moving body");

    let sample_offset = i32::try_from(sample).expect("small sample");
    for offset in 0..UNRELATED_DYNAMIC_BODIES {
        let id = BodyId(100 + offset);
        let x = 10_000 + i32::try_from(offset).expect("small body index") * 16 + sample_offset;
        world
            .add_box(rotating(RigidBody::dynamic(
                id,
                Vec3i::new(x, 0, 0),
                Vec3i::ZERO,
                Vec3i::new(1, 1, 1),
            )))
            .expect("unrelated stationary body");
    }
    world
}

#[test]
fn step_delta_scales_with_changed_bodies_not_world_size() {
    let mut world = world(0);
    let body_count = world.boxes().count();

    let report = world.step(1, 60).expect("precise writeback step");

    assert_eq!(
        body_count,
        1 + usize::try_from(UNRELATED_DYNAMIC_BODIES).unwrap()
    );
    assert_eq!(
        report.changed_body_ids,
        vec![BodyId(1)],
        "unrelated stationary dynamics must not enter the writeback delta"
    );
    assert_eq!(
        world
            .box_by_id(BodyId(1))
            .expect("moving body remains")
            .body()
            .position(),
        Vec3i::new(1, 0, 0)
    );
    assert_eq!(
        world
            .box_by_id(BodyId(100 + UNRELATED_DYNAMIC_BODIES - 1))
            .expect("last stationary body remains")
            .body()
            .velocity(),
        Vec3i::ZERO
    );
}

#[test]
#[ignore = "release-mode changed-body writeback evidence; run explicitly with --ignored --release --nocapture"]
fn benchmark_precise_step_writeback_fanout() {
    const SAMPLES: u64 = 24;
    let mut worlds = (0..SAMPLES).map(world).collect::<Vec<_>>();
    let bodies_per_world = worlds[0].boxes().count();

    let started = Instant::now();
    let mut changed_bodies = 0_usize;
    for world in &mut worlds {
        let report = black_box(world.step(1, 60).expect("precise writeback step"));
        assert_eq!(report.changed_body_ids, vec![BodyId(1)]);
        changed_bodies = changed_bodies.saturating_add(report.changed_body_ids.len());
    }
    let elapsed = started.elapsed();

    let prior_full_dynamic_writeback_equivalent =
        bodies_per_world.saturating_mul(usize::try_from(SAMPLES).expect("small sample count"));
    assert_eq!(
        changed_bodies,
        usize::try_from(SAMPLES).expect("small sample count")
    );
    assert!(changed_bodies < prior_full_dynamic_writeback_equivalent);

    eprintln!(
        "precise step writeback: worlds={SAMPLES}, bodies_per_world={bodies_per_world}, changed_bodies={changed_bodies}, full_dynamic_writeback_equivalent={prior_full_dynamic_writeback_equivalent}, avoided_fanout={}x, elapsed={elapsed:?}",
        prior_full_dynamic_writeback_equivalent / changed_bodies,
    );
}
