use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d,
    RotatingIntervalConfig3d, RotatingWorld3d, RotatingWorldConfig3d, Vec3i,
};

fn body(id: u64, x: i32, velocity: i32) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::dynamic(
            BodyId(id),
            Vec3i::new(x, 0, 0),
            Vec3i::new(velocity, 0, 0),
            Vec3i::new(1, 1, 1),
        ),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .unwrap()
}

#[cfg_attr(test, test)]
pub fn late_substep_failure_restores_the_requested_interval() {
    let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
        gravity: Vec3i::ZERO,
        ..RotatingWorldConfig3d::default()
    });
    let original = body(1, i32::MAX - 128, 7680);
    world.add_box(original.clone()).unwrap();
    let mut reports = Vec::new();
    let failure = world
        .advance_interval(
            RotatingIntervalConfig3d {
                timestep_numerator: 1,
                timestep_denominator: 60,
                substeps: 2,
                angular_damping_milli: 1000,
            },
            &mut reports,
        )
        .unwrap_err();
    assert_eq!(failure.work.completed_substeps, 1);
    assert_eq!(failure.work.motion_before_images, 1);
    assert_eq!(world.box_by_id(BodyId(1)), Some(&original));
    assert!(reports.is_empty());
    world.set_linear_velocity(BodyId(1), Vec3i::ZERO).unwrap();
    world
        .advance_interval(
            RotatingIntervalConfig3d {
                timestep_numerator: 1,
                timestep_denominator: 60,
                substeps: 2,
                angular_damping_milli: 1000,
            },
            &mut reports,
        )
        .unwrap();
    assert_eq!(reports.len(), 2);
    assert_eq!(
        world.box_by_id(BodyId(1)).unwrap().body().position(),
        original.body().position()
    );
}

fn interval() -> RotatingIntervalConfig3d {
    RotatingIntervalConfig3d {
        timestep_numerator: 1,
        timestep_denominator: 60,
        substeps: 2,
        angular_damping_milli: 1000,
    }
}

fn world() -> RotatingWorld3d {
    RotatingWorld3d::new(RotatingWorldConfig3d {
        gravity: Vec3i::ZERO,
        ..RotatingWorldConfig3d::default()
    })
}

#[cfg_attr(test, test)]
pub fn late_failure_restores_sleep_deadline_and_newly_parked_membership() {
    let mut world = world();
    world.add_box(body(2, 0, 0)).unwrap();
    world.step(195, 1000).unwrap();
    assert!(!world.is_sleeping(BodyId(2)));
    world.add_box(body(1, i32::MAX - 128, 7680)).unwrap();
    let before = world.boxes().cloned().collect::<Vec<_>>();
    let mut control = world.clone();
    control.step(1, 120).unwrap();
    assert!(control.is_sleeping(BodyId(2)));
    let mut reports = Vec::new();
    let failure = world
        .advance_interval(interval(), &mut reports)
        .unwrap_err();
    assert_eq!(failure.work.completed_substeps, 1);
    assert_eq!(failure.work.parked_before_images, 1);
    assert_eq!(world.boxes().cloned().collect::<Vec<_>>(), before);
    assert!(!world.is_sleeping(BodyId(2)));
    world.set_linear_velocity(BodyId(1), Vec3i::ZERO).unwrap();
    world.step(1, 120).unwrap();
    assert!(world.is_sleeping(BodyId(2)));
}

#[cfg_attr(test, test)]
pub fn late_failure_restores_an_activated_parked_body_and_future_impacts() {
    let mut world = world();
    world.add_box(body(2, 0, 0)).unwrap();
    world.step(1, 1).unwrap();
    assert!(world.is_sleeping(BodyId(2)));
    world.add_box(body(3, -4, 480)).unwrap();
    world.add_box(body(1, i32::MAX - 128, 7680)).unwrap();
    let mut reference = world.clone();
    let mut admitted = world.clone();
    admitted.step(1, 120).unwrap();
    assert!(!admitted.is_sleeping(BodyId(2)));
    let mut reports = Vec::new();
    let failure = world
        .advance_interval(interval(), &mut reports)
        .unwrap_err();
    assert_eq!(failure.work.completed_substeps, 1);
    assert!(failure.work.parked_before_images > 0);
    assert!(world.is_sleeping(BodyId(2)));
    assert_eq!(
        world.boxes().cloned().collect::<Vec<_>>(),
        reference.boxes().cloned().collect::<Vec<_>>()
    );
    for candidate in [&mut world, &mut reference] {
        candidate
            .set_linear_velocity(BodyId(1), Vec3i::ZERO)
            .unwrap();
    }
    for _ in 0..4 {
        world.step(1, 120).unwrap();
        reference.step(1, 120).unwrap();
        assert_eq!(
            world.boxes().cloned().collect::<Vec<_>>(),
            reference.boxes().cloned().collect::<Vec<_>>()
        );
        assert_eq!(
            world.is_sleeping(BodyId(2)),
            reference.is_sleeping(BodyId(2))
        );
    }
}

#[cfg_attr(test, test)]
pub fn successful_partition_matches_solver_commands_and_damps_once() {
    let original = RigidBox3d::new(
        RigidBody::dynamic(
            BodyId(1),
            Vec3i::ZERO,
            Vec3i::new(600, 0, 0),
            Vec3i::new(100, 100, 100),
        ),
        AngularState3d::new(
            Orientation3d::IDENTITY,
            AngularVelocity3d::new(1200, -2400, 3600),
        ),
    )
    .unwrap();
    let mut world = world();
    world.add_box(original).unwrap();
    let mut reference = world.clone();
    for _ in 0..4 {
        reference.step(1, 240).unwrap();
    }
    let mut config = interval();
    config.substeps = 4;
    config.angular_damping_milli = 750;
    let mut reports = Vec::new();
    let work = world.advance_interval(config, &mut reports).unwrap();
    assert_eq!(work.completed_substeps, 4);
    assert_eq!(work.motion_before_images, 1);
    assert_eq!(work.damping_body_visits, 1);
    assert_eq!(work.damping_changes, 1);
    let actual = world.box_by_id(BodyId(1)).unwrap();
    let expected = reference.box_by_id(BodyId(1)).unwrap();
    assert_eq!(actual.body(), expected.body());
    assert_eq!(actual.angular().orientation, expected.angular().orientation);
    assert_eq!(
        actual.angular().angular_velocity,
        AngularVelocity3d::new(900, -1800, 2700)
    );
    assert_eq!(reports.len(), 4);
    assert_eq!(reports.last().unwrap().changed_body_ids, vec![BodyId(1)]);
}

#[cfg_attr(test, test)]
pub fn invalid_partitions_and_zero_intervals_do_not_change_motion() {
    let mut world = world();
    world.add_box(body(1, 0, 600)).unwrap();
    let before = world.box_by_id(BodyId(1)).unwrap().clone();
    let mut reports = vec![world.step(0, 1).unwrap()];
    for bad in [
        RotatingIntervalConfig3d {
            substeps: 0,
            ..interval()
        },
        RotatingIntervalConfig3d {
            timestep_numerator: -1,
            ..interval()
        },
        RotatingIntervalConfig3d {
            timestep_denominator: 0,
            ..interval()
        },
        RotatingIntervalConfig3d {
            timestep_denominator: i32::MAX,
            ..interval()
        },
        RotatingIntervalConfig3d {
            angular_damping_milli: 1001,
            ..interval()
        },
    ] {
        let failure = world.advance_interval(bad, &mut reports).unwrap_err();
        assert_eq!(failure.work, Default::default());
        assert!(reports.is_empty());
        assert_eq!(world.box_by_id(BodyId(1)), Some(&before));
    }
    let work = world
        .advance_interval(
            RotatingIntervalConfig3d {
                timestep_numerator: 0,
                angular_damping_milli: 0,
                ..interval()
            },
            &mut reports,
        )
        .unwrap();
    assert_eq!(work.motion_before_images, 0);
    assert_eq!(work.sleep_before_images, 0);
    assert_eq!(work.damping_changes, 0);
    assert_eq!(world.box_by_id(BodyId(1)), Some(&before));
}

#[cfg_attr(test, test)]
pub fn fully_parked_intervals_do_not_copy_stationary_scene_state() {
    let mut world = world();
    for id in 1..=128 {
        world
            .add_box(body(id, i32::try_from(id).unwrap() * 10, 0))
            .unwrap();
    }
    world.step(1, 1).unwrap();
    let before = world.boxes().cloned().collect::<Vec<_>>();
    assert!(
        world
            .boxes()
            .all(|body| world.is_sleeping(body.body().id()))
    );
    let mut reports = Vec::new();
    let work = world
        .advance_interval(
            RotatingIntervalConfig3d {
                angular_damping_milli: 998,
                ..interval()
            },
            &mut reports,
        )
        .unwrap();
    assert_eq!(work.motion_before_images, 0);
    assert_eq!(work.sleep_before_images, 0);
    assert_eq!(work.parked_before_images, 0);
    assert_eq!(work.damping_body_visits, 0);
    assert_eq!(work.contact_work, [0; 4]);
    assert_eq!(world.boxes().cloned().collect::<Vec<_>>(), before);
}

#[cfg_attr(test, test)]
pub fn ballistic_membership_is_rejected_before_interval_work() {
    use physics_engine::{BallisticSphere3d, RotatingIntervalError3d};
    let mut world = world();
    world.add_box(body(1, 0, 600)).unwrap();
    let projectile = BallisticSphere3d::new(
        BodyId(2),
        Vec3i::new(-100, 0, 0),
        Vec3i::new(60, 0, 0),
        1,
        1,
    )
    .unwrap();
    world.add_ballistic_sphere(projectile, true).unwrap();
    let before = world.box_by_id(BodyId(1)).unwrap().clone();
    let mut reports = Vec::new();
    let failure = world
        .advance_interval(interval(), &mut reports)
        .unwrap_err();
    assert_eq!(
        failure.error,
        RotatingIntervalError3d::BallisticBodiesUnsupported
    );
    assert_eq!(failure.work, Default::default());
    assert_eq!(world.box_by_id(BodyId(1)), Some(&before));
    assert_eq!(world.ballistic_sphere_by_id(BodyId(2)), Some(&projectile));
    assert!(reports.is_empty());
}

#[cfg_attr(test, test)]
pub fn fixed_preparation_remains_equivalent_after_rollback_and_id_reuse() {
    use physics_engine::FixedGeometryPreparationMode3d;
    let mut worlds = [world(), world()];
    worlds[1].set_fixed_geometry_preparation_mode(FixedGeometryPreparationMode3d::PrepareAtLoad);
    for world in &mut worlds {
        world
            .add_box(
                RigidBox3d::new(
                    RigidBody::fixed(BodyId(2), Vec3i::ZERO, Vec3i::new(1, 1, 1)),
                    AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
                )
                .unwrap(),
            )
            .unwrap();
        world.add_box(body(3, -4, 480)).unwrap();
        world.add_box(body(1, i32::MAX - 128, 7680)).unwrap();
        let mut reports = Vec::new();
        let failure = world
            .advance_interval(interval(), &mut reports)
            .unwrap_err();
        assert_eq!(failure.work.completed_substeps, 1);
        assert!(reports.is_empty());
        world.remove_box(BodyId(1)).unwrap();
        world.add_box(body(1, 100, 60)).unwrap();
        world.advance_interval(interval(), &mut reports).unwrap();
        world.remove_box(BodyId(2)).unwrap();
        world.advance_interval(interval(), &mut reports).unwrap();
    }
    assert_eq!(
        worlds[0].boxes().cloned().collect::<Vec<_>>(),
        worlds[1].boxes().cloned().collect::<Vec<_>>()
    );
    assert_eq!(
        worlds[0].body_contacts(BodyId(3)).unwrap(),
        worlds[1].body_contacts(BodyId(3)).unwrap()
    );
}
