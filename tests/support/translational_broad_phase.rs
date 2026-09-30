//! Public capacity and canonical-event controls shared with the existing WASM driver.
use physics_engine::{BodyId, Material, RigidBody, StepReport, Vec3i, World, WorldConfig};

fn fixture() -> World {
    let mut w = World::new(WorldConfig {
        gravity: Vec3i::ZERO,
        ..Default::default()
    });
    let elastic = Material::new(1000);
    // Two equal-time impacts and subsequent ricochets, with insertion order reversed.
    for body in [
        RigidBody::fixed(BodyId(8), Vec3i::new(10, 0, 0), Vec3i::new(1, 100, 10)),
        RigidBody::dynamic(
            BodyId(7),
            Vec3i::new(0, 20, 0),
            Vec3i::new(30, 0, 0),
            Vec3i::new(1, 1, 1),
        ),
        RigidBody::dynamic(
            BodyId(3),
            Vec3i::new(0, -20, 0),
            Vec3i::new(30, 0, 0),
            Vec3i::new(1, 1, 1),
        ),
        RigidBody::fixed(BodyId(1), Vec3i::new(-10, 0, 0), Vec3i::new(1, 100, 10)),
    ] {
        w.add_body(body.with_material(elastic)).unwrap();
    }
    w
}

fn semantic(mut report: StepReport) -> StepReport {
    report.stats.work.staged_state_capacity_growths = 0;
    report.stats.work.broad_phase_capacity_growths = 0;
    report.stats.work.staged_state_capacity_bytes = 0;
    report.stats.work.candidate_buffer_peak_capacity_bytes = 0;
    report
}

pub fn ricochets_reuse_capacity_and_preserve_canonical_events() {
    let mut w = fixture();
    let mut first = true;
    for ticks in [1, 1, 2, 1, 2, 1, 1, 2] {
        let mut fresh = World::new(w.config());
        for b in w.bodies().cloned() {
            fresh.add_body(b).unwrap();
        }
        let actual = w.step(ticks).unwrap();
        let expected = fresh.step(ticks).unwrap();
        assert_eq!(semantic(actual.clone()), semantic(expected));
        assert_eq!(
            w.bodies().collect::<Vec<_>>(),
            fresh.bodies().collect::<Vec<_>>()
        );
        assert!(actual.events.len() >= 2);
        if first {
            assert!(actual.stats.work.broad_phase_capacity_growths >= 3);
            assert_eq!(
                (actual.events[0].left, actual.events[0].right),
                (BodyId(3), BodyId(8))
            );
            assert_eq!(
                (actual.events[1].left, actual.events[1].right),
                (BodyId(7), BodyId(8))
            );
            assert_eq!(actual.events[0].time, actual.events[1].time);
            first = false;
        } else {
            assert_eq!(actual.stats.work.broad_phase_capacity_growths, 0);
        }
        assert_eq!(
            w.retained_step_scratch_bytes(),
            actual.stats.work.staged_state_capacity_bytes
                + actual.stats.work.candidate_buffer_peak_capacity_bytes,
        );
    }
    let before = w.bodies().cloned().collect::<Vec<_>>();
    w.release_step_scratch();
    assert_eq!(w.retained_step_scratch_bytes(), 0);
    assert_eq!(w.bodies().cloned().collect::<Vec<_>>(), before);
    assert!(w.step(1).unwrap().stats.work.broad_phase_capacity_growths >= 3);
}

pub fn run() {
    ricochets_reuse_capacity_and_preserve_canonical_events();
}
