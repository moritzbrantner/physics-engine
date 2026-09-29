#[path = "fixtures/legacy_tower.rs"]
mod legacy_tower;

#[cfg(not(feature = "performance-counters"))]
use physics_engine::RotatingWorldStepStats3d;
use physics_engine::{
    AngularState3d, AngularVelocity3d, BallisticSphere3d, BodyId, Material, Orientation3d,
    RepeatedRotatingEventConfig3d, RepeatedRotatingEventError3d, RigidBody, RigidBox3d,
    RigidBoxFreeFlightConfig3d, RotatingContactSearchConfig3d, RotatingWorld3d,
    RotatingWorldConfig3d, RotatingWorldError3d, Vec3i, advance_repeated_rotating_events,
};

fn box3d(body: RigidBody) -> RigidBox3d {
    RigidBox3d::new(
        body,
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid deterministic test body")
}

fn world() -> RotatingWorld3d {
    let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
        gravity: Vec3i::ZERO,
        sample_count: 8,
        refinement_steps: 2,
        solver_passes: 4,
        max_events: 8,
    });
    world
        .add_box(box3d(RigidBody::dynamic(
            BodyId(1),
            Vec3i::ZERO,
            Vec3i::new(120, 0, 0),
            Vec3i::new(1, 1, 1),
        )))
        .expect("moving body");
    world
        .add_box(box3d(RigidBody::fixed(
            BodyId(2),
            Vec3i::new(3, 0, 0),
            Vec3i::new(1, 4, 4),
        )))
        .expect("fixed obstacle");
    world
}

#[test]
fn production_world_semantics_do_not_depend_on_performance_counters() {
    let mut world = world();
    let report = world.step(1, 60).expect("deterministic production step");

    let moving = world.box_by_id(BodyId(1)).expect("moving body remains");
    assert_eq!(moving.body().position(), Vec3i::new(1, 0, 0));
    assert_eq!(moving.body().velocity(), Vec3i::ZERO);
    assert_eq!(
        world
            .box_by_id(BodyId(2))
            .expect("fixed body remains")
            .body()
            .position(),
        Vec3i::new(3, 0, 0)
    );
    assert!(report.changed_body_ids.contains(&BodyId(1)));

    #[cfg(feature = "performance-counters")]
    {
        assert!(report.stats.body_count >= 2);
        assert!(report.stats.broad_phase_queries > 0);
        assert!(report.stats.sampled_events > 0);
    }

    #[cfg(not(feature = "performance-counters"))]
    assert_eq!(report.stats, RotatingWorldStepStats3d::default());
}

fn snapshot(label: &str, world: &RotatingWorld3d) {
    let bodies = world.boxes().cloned().collect::<Vec<_>>();
    let sleeping = bodies
        .iter()
        .map(|body| (body.body().id(), world.is_sleeping(body.body().id())))
        .collect::<Vec<_>>();
    let projectiles = world.ballistic_spheres().copied().collect::<Vec<_>>();
    println!("COUNTER_SNAPSHOT {label}: {bodies:?} {sleeping:?} {projectiles:?}");
}

#[test]
fn parked_contact_lifecycle_has_identical_cross_build_snapshots() {
    for (label, y, retire, mass) in [
        ("miss", 10, true, 1.0),
        ("retire", 0, true, 1.0),
        ("bounce", 0, false, 1.0),
        ("fractional-bounce", 0, false, 0.5),
    ] {
        let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
            gravity: Vec3i::ZERO,
            ..RotatingWorldConfig3d::default()
        });
        for (id, position) in [(1, Vec3i::ZERO), (2, Vec3i::new(0, 50, 0))] {
            world
                .add_box(box3d(
                    RigidBody::dynamic(BodyId(id), position, Vec3i::ZERO, Vec3i::new(2, 2, 2))
                        .with_mass(2)
                        .with_material(Material::new(1_000)),
                ))
                .unwrap();
        }
        for _ in 0..60 {
            world.step(1, 60).unwrap();
        }
        assert_eq!(world.sleeping_body_count(), 2);
        world
            .add_ballistic_sphere(
                BallisticSphere3d::new(
                    BodyId(100),
                    Vec3i::new(-20, y, 0),
                    Vec3i::new(60, 0, 0),
                    1,
                    1,
                )
                .unwrap()
                .with_response_mass(mass)
                .unwrap()
                .with_material(Material::new(1_000)),
                retire,
            )
            .unwrap();
        world.step(0, 60).unwrap();
        assert_eq!(world.sleeping_body_count(), 2);
        for tick in 0..4 {
            let report = world.step(1, 1).unwrap();
            println!(
                "COUNTER_SNAPSHOT {label}-{tick}-changes: {:?}",
                report.changed_body_ids
            );
            snapshot(label, &world);
            assert!(world.is_sleeping(BodyId(2)));
            #[cfg(not(feature = "performance-counters"))]
            assert_eq!(report.stats, RotatingWorldStepStats3d::default());
        }
        assert_eq!(world.is_sleeping(BodyId(1)), y != 0);
        assert_eq!(
            world.ballistic_sphere_count(),
            usize::from(!retire || y != 0)
        );
    }
}

#[test]
fn collision_events_and_failures_have_identical_cross_build_snapshots() {
    let world = world();
    let advance = advance_repeated_rotating_events(
        &world.boxes().cloned().collect::<Vec<_>>(),
        RepeatedRotatingEventConfig3d::new(
            RotatingContactSearchConfig3d::new(
                RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 60),
                8,
                2,
            ),
            4,
            8,
        ),
    )
    .unwrap();
    assert!(!advance.events.is_empty());
    println!(
        "COUNTER_SNAPSHOT events: {:?} {:?} {:?}",
        advance.events, advance.boxes, advance.remaining
    );

    for event_limit in [4, 32] {
        let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
            gravity: Vec3i::ZERO,
            max_events: event_limit,
            ..RotatingWorldConfig3d::default()
        });
        for (id, x) in [(1, -10), (2, 10)] {
            world
                .add_box(box3d(
                    RigidBody::fixed(BodyId(id), Vec3i::new(x, 0, 0), Vec3i::new(1, 10, 10))
                        .with_material(Material::new(1_000)),
                ))
                .unwrap();
        }
        world
            .add_ballistic_sphere(
                BallisticSphere3d::new(BodyId(100), Vec3i::ZERO, Vec3i::new(100, 0, 0), 1, 1)
                    .unwrap()
                    .with_material(Material::new(1_000)),
                false,
            )
            .unwrap();
        for invalid in [(-1, 60), (1, 0)] {
            let error = world.step(invalid.0, invalid.1).unwrap_err();
            println!("COUNTER_SNAPSHOT invalid: {error:?}");
            snapshot("invalid", &world);
        }
        let result = world.step(1, 1);
        if event_limit == 4 {
            assert_eq!(
                result,
                Err(RotatingWorldError3d::Repeated(
                    RepeatedRotatingEventError3d::BallisticEventLimit(4)
                ))
            );
        } else {
            assert!(result.is_ok());
            assert_eq!(
                world
                    .ballistic_sphere_by_id(BodyId(100))
                    .unwrap()
                    .velocity(),
                Vec3i::new(100, 0, 0)
            );
        }
        println!(
            "COUNTER_SNAPSHOT budget-{event_limit}: {:?}",
            result.map(|report| report.changed_body_ids)
        );
        snapshot("budget", &world);
    }
}

#[test]
fn legacy_crossing_edge_contacts_have_identical_counter_build_snapshots() {
    let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
        gravity: Vec3i::new(0, -36_000, 0),
        sample_count: 8,
        refinement_steps: 4,
        solver_passes: 10,
        max_events: 64,
    });
    for body in legacy_tower::minimized_frame_38() {
        world.add_box(body).unwrap();
    }
    snapshot("legacy-edge-before", &world);
    let report = world.step(1, 60).unwrap();
    #[cfg(not(feature = "performance-counters"))]
    assert_eq!(report.stats, RotatingWorldStepStats3d::default());
    #[cfg(feature = "performance-counters")]
    assert!(report.stats.continuation_contact_evaluations > 0);
    snapshot("legacy-edge-after", &world);
}

#[test]
fn high_resolution_same_step_ricochets_have_identical_counter_build_snapshots() {
    let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
        gravity: Vec3i::ZERO,
        sample_count: 64,
        refinement_steps: 4,
        solver_passes: 4,
        max_events: 64,
    });
    for body in legacy_tower::elastic_corridor() {
        world.add_box(body).unwrap();
    }
    snapshot("legacy-ricochet-before", &world);
    world.step(1, 1).unwrap();
    snapshot("legacy-ricochet-after", &world);
}
