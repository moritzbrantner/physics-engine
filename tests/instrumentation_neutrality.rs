use physics_engine::{BodyId, RigidBody, StepStats, Vec3i, World, WorldConfig};

fn fixture() -> World {
    let mut world = World::new(WorldConfig {
        gravity: Vec3i::ZERO,
        ..WorldConfig::default()
    });
    world
        .add_body(RigidBody::dynamic(
            BodyId(1),
            Vec3i::new(-20, 0, 0),
            Vec3i::new(30, 0, 0),
            Vec3i::new(1, 1, 1),
        ))
        .unwrap();
    world
        .add_body(RigidBody::fixed(
            BodyId(2),
            Vec3i::ZERO,
            Vec3i::new(1, 8, 8),
        ))
        .unwrap();
    world
        .add_body(RigidBody::dynamic(
            BodyId(3),
            Vec3i::new(20, 0, 0),
            Vec3i::new(-30, 0, 0),
            Vec3i::new(1, 1, 1),
        ))
        .unwrap();
    world
}

#[test]
fn disabling_step_stats_preserves_authoritative_simulation() {
    let mut instrumented = fixture();
    let mut uninstrumented = instrumented.clone();
    let mut observed_work = false;

    for ticks in [1, 1, 2, 1] {
        let instrumented_report = instrumented.step(ticks).unwrap();
        let uninstrumented_report = uninstrumented.step_without_stats(ticks).unwrap();

        observed_work |= instrumented_report.stats != StepStats::default();
        assert_eq!(uninstrumented_report.stats, StepStats::default());
        assert_eq!(instrumented_report.events, uninstrumented_report.events);
        assert_eq!(
            instrumented.bodies().cloned().collect::<Vec<_>>(),
            uninstrumented.bodies().cloned().collect::<Vec<_>>(),
        );
    }

    assert!(observed_work, "fixture must exercise diagnostic counter collection");
}

#[test]
fn stats_free_step_preserves_validation_errors() {
    let mut instrumented = fixture();
    let mut uninstrumented = instrumented.clone();

    assert_eq!(instrumented.step(0), uninstrumented.step_without_stats(0));
}
