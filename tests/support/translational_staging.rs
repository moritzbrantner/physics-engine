//! Public retention/failure controls shared with the native/WASM acceptance driver.
use physics_engine::{BodyId, PhysicsError, RigidBody, StepReport, Vec3i, World, WorldConfig};

fn body(id: u64) -> RigidBody {
    RigidBody::dynamic(
        BodyId(id),
        Vec3i::new(
            (id % 32) as i32 * 100 - 1600,
            20,
            (id / 32) as i32 * 100 - 3200,
        ),
        Vec3i::ZERO,
        Vec3i::new(10, 20, 10),
    )
}

fn physical(world: &World) -> Vec<RigidBody> {
    world.bodies().cloned().collect()
}

fn semantic_report(mut report: StepReport) -> StepReport {
    // Capacity retention is the intended observable diagnostic difference.
    report.stats.work.staged_state_capacity_growths = 0;
    report.stats.work.staged_state_capacity_bytes = 0;
    report
}

fn paired_step(w: &mut World) -> StepReport {
    let mut rebuilding = World::new(w.config());
    for b in w.bodies().cloned() {
        rebuilding.add_body(b).unwrap();
    }
    let actual = w.step(1).unwrap();
    let expected = rebuilding.step(1).unwrap();
    assert_eq!(semantic_report(actual.clone()), semantic_report(expected));
    assert_eq!(physical(w), physical(&rebuilding));
    actual
}

pub fn lifecycle_and_release_keep_physics_identical() {
    let mut w = World::new(WorldConfig {
        gravity: Vec3i::ZERO,
        ..Default::default()
    });
    assert_eq!(w.retained_step_scratch_bytes(), 0);
    assert_eq!(w.step(0), Err(PhysicsError::NonPositiveTicks(0)));
    assert_eq!(w.retained_step_scratch_bytes(), 0);
    for id in 0..128 {
        w.add_body(body(id)).unwrap();
    }
    w.set_velocity(BodyId(9), Vec3i::new(1, 0, 0)).unwrap();
    let first = w.step(1).unwrap();
    assert_eq!(first.stats.work.staged_state_capacity_growths, 1);
    assert_eq!(
        first.stats.work.staged_state_capacity_bytes,
        w.retained_step_scratch_bytes()
    );
    let capacity = w.retained_step_scratch_bytes();
    for _ in 0..32 {
        let actual = paired_step(&mut w);
        assert_eq!(actual.stats.work.staged_state_capacity_growths, 0);
        assert_eq!(w.retained_step_scratch_bytes(), capacity);
    }
    for id in 128..257 {
        w.add_body(body(id)).unwrap();
    }
    assert_eq!(
        paired_step(&mut w).stats.work.staged_state_capacity_growths,
        1
    );
    let high_water = w.retained_step_scratch_bytes();
    assert!(high_water > capacity);
    for id in 128..257 {
        w.remove_body(BodyId(id)).unwrap();
    }
    let replacement = RigidBody::fixed(BodyId(50), Vec3i::new(-10_000, 0, 0), Vec3i::new(2, 2, 2));
    w.replace_body(replacement).unwrap();
    w.remove_body(BodyId(50)).unwrap();
    w.add_body(body(50)).unwrap();
    assert_eq!(
        paired_step(&mut w).stats.work.staged_state_capacity_growths,
        0
    );
    assert_eq!(w.retained_step_scratch_bytes(), high_water);
    let before = physical(&w);
    w.release_step_scratch();
    assert_eq!(w.retained_step_scratch_bytes(), 0);
    assert_eq!(physical(&w), before);
    assert_eq!(
        w.step(1).unwrap().stats.work.staged_state_capacity_growths,
        1
    );
    for id in 0..128 {
        w.remove_body(BodyId(id)).unwrap();
    }
    w.step(1).unwrap();
    let capacity = w.retained_step_scratch_bytes();
    assert!(capacity > 0);
    let cached = w.step(1).unwrap();
    assert!(cached.stats.work.cached_stationary_step);
    assert_eq!(cached.stats.work.staged_state_capacity_bytes, 0);
    assert_eq!(w.retained_step_scratch_bytes(), capacity);
    w.release_step_scratch();
    assert_eq!(w.step(1).unwrap(), cached);
}

pub fn failed_steps_retain_only_disposable_capacity_and_recover() {
    for stage in 0..3 {
        let mut w = World::new(WorldConfig {
            gravity: if stage == 0 {
                Vec3i::new(1, 0, 0)
            } else {
                Vec3i::ZERO
            },
            max_events_per_step: if stage == 1 { 0 } else { 256 },
            ..Default::default()
        });
        w.add_body(RigidBody::dynamic(
            BodyId(1),
            Vec3i::new(-10, 0, 0),
            if stage == 0 {
                Vec3i::new(i32::MAX, 0, 0)
            } else {
                Vec3i::new(40, 0, 0)
            },
            Vec3i::new(1, 1, 1),
        ))
        .unwrap();
        if stage == 1 {
            w.add_body(RigidBody::fixed(
                BodyId(2),
                Vec3i::ZERO,
                Vec3i::new(1, 8, 8),
            ))
            .unwrap();
        }
        if stage == 2 {
            w.add_body(RigidBody::dynamic(
                BodyId(99),
                Vec3i::new(i32::MAX - 1, 0, 0),
                Vec3i::new(4, 0, 0),
                Vec3i::new(1, 1, 1),
            ))
            .unwrap();
        }
        let before = physical(&w);
        let expected_error = match stage {
            0 => PhysicsError::ArithmeticOverflow(BodyId(1)),
            1 => PhysicsError::EventLimit(0),
            _ => PhysicsError::ArithmeticOverflow(BodyId(99)),
        };
        assert_eq!(w.step(1), Err(expected_error));
        let capacity = w.retained_step_scratch_bytes();
        assert!(capacity > 0);
        for _ in 0..2 {
            assert_eq!(w.step(1), Err(expected_error));
            assert_eq!(physical(&w), before);
            assert_eq!(w.retained_step_scratch_bytes(), capacity);
        }
        w.set_velocity(BodyId(1), Vec3i::ZERO).unwrap();
        if stage == 1 {
            w.remove_body(BodyId(2)).unwrap();
        }
        if stage == 2 {
            w.set_velocity(BodyId(99), Vec3i::ZERO).unwrap();
        }
        let mut uninterrupted = World::new(w.config());
        for b in w.bodies().cloned() {
            uninterrupted.add_body(b).unwrap();
        }
        let actual = w.step(1).unwrap();
        let expected = uninterrupted.step(1).unwrap();
        assert_eq!(semantic_report(actual.clone()), semantic_report(expected));
        assert_eq!(actual.stats.work.staged_state_capacity_growths, 0);
        assert_eq!(physical(&w), physical(&uninterrupted));
    }
}

pub fn run() {
    lifecycle_and_release_keep_physics_identical();
    failed_steps_retain_only_disposable_capacity_and_recover();
}
