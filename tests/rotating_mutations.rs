use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, CollisionLayers3d, FixedGeometryPreparationMode3d,
    InteractionCategory3d, Material, MotionAuthority3d, Orientation3d, RigidBody, RigidBox3d,
    RigidBoxError3d, RotatingIntervalConfig3d, RotatingWorld3d, RotatingWorldConfig3d,
    RotatingWorldError3d, Vec3i,
};

fn world() -> RotatingWorld3d {
    RotatingWorld3d::new(RotatingWorldConfig3d {
        gravity: Vec3i::ZERO,
        ..RotatingWorldConfig3d::default()
    })
}

fn body(id: u64, position: Vec3i) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::dynamic(BodyId(id), position, Vec3i::ZERO, Vec3i::new(1, 1, 1)),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .unwrap()
}

fn floor(id: u64, x: i32) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::fixed(BodyId(id), Vec3i::new(x, -2, 0), Vec3i::new(200, 1, 10)),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .unwrap()
}

#[cfg_attr(test, test)]
pub fn descriptor_changes_keep_identity_policy_and_unrelated_state() {
    let mut world = world();
    let original = body(7, Vec3i::ZERO);
    let unrelated = body(9, Vec3i::new(100, 20, 0));
    world.add_box(original.clone()).unwrap();
    world.add_box(unrelated.clone()).unwrap();
    world
        .set_body_interaction_category(BodyId(7), InteractionCategory3d::new(42))
        .unwrap();
    let replacement = RigidBox3d::new(
        RigidBody::dynamic(
            BodyId(7),
            Vec3i::new(20, 0, 0),
            Vec3i::new(600, 0, 0),
            Vec3i::new(2, 3, 4),
        )
        .with_mass(5)
        .with_material(Material::new(250).with_friction(800)),
        AngularState3d::new(
            Orientation3d::IDENTITY,
            AngularVelocity3d::new(0, 100_000, 0),
        ),
    )
    .unwrap()
    .with_external_motion();
    assert!(world.replace_box(replacement.clone()).unwrap());
    assert_eq!(world.entity_count(), 2);
    assert_eq!(world.box_by_id(BodyId(7)), Some(&replacement));
    assert_eq!(world.box_by_id(BodyId(9)), Some(&unrelated));
    assert_eq!(
        world.body_interaction_category(BodyId(7)),
        InteractionCategory3d::new(42)
    );
    world.step(1, 60).unwrap();
    let advanced = world.box_by_id(BodyId(7)).unwrap();
    assert_eq!(advanced.body().position(), Vec3i::new(30, 0, 0));
    assert_eq!(advanced.motion_authority(), MotionAuthority3d::External);
    assert_eq!(
        advanced.angular().angular_velocity,
        replacement.angular().angular_velocity
    );
    assert_eq!(world.box_by_id(BodyId(9)), Some(&unrelated));
}

#[cfg_attr(test, test)]
pub fn equal_commands_preserve_parked_sleep_and_zero_interval_work() {
    let mut world = world();
    world.set_fixed_geometry_preparation_mode(FixedGeometryPreparationMode3d::PrepareAtLoad);
    world.add_box(floor(0, 0)).unwrap();
    for id in 1..=128 {
        world
            .add_box(body(id, Vec3i::new(i32::try_from(id).unwrap() * 4, 20, 0)))
            .unwrap();
    }
    world.step(1, 1).unwrap();
    assert_eq!(world.sleeping_body_count(), 128);
    let original = world.box_by_id(BodyId(1)).unwrap().clone();
    let preparation = world.fixed_geometry_preparation_stats();
    assert!(!world.replace_box(original).unwrap());
    assert!(
        !world
            .set_motion(BodyId(1), Vec3i::ZERO, AngularVelocity3d::default())
            .unwrap()
    );
    assert!(!world.replace_box(floor(0, 0)).unwrap());
    assert_eq!(world.sleeping_body_count(), 128);
    assert_eq!(world.fixed_geometry_preparation_stats(), preparation);
    let work = world
        .advance_interval(
            RotatingIntervalConfig3d {
                timestep_numerator: 1,
                timestep_denominator: 60,
                substeps: 4,
                angular_damping_milli: 1000,
            },
            &mut Vec::new(),
        )
        .unwrap();
    assert_eq!(work.motion_before_images, 0);
    assert_eq!(work.sleep_before_images, 0);
    assert_eq!(work.parked_before_images, 0);
}

#[cfg_attr(test, test)]
pub fn rejected_replacement_and_motion_leave_sleep_and_future_steps_unchanged() {
    let mut world = world();
    world.add_box(body(1, Vec3i::ZERO)).unwrap();
    world.add_box(floor(0, 0)).unwrap();
    world.step(1, 1).unwrap();
    let mut control = world.clone();
    let overflow = body(1, Vec3i::new(i32::MAX, 0, 0));
    assert!(world.replace_box(overflow).is_err());
    assert_eq!(
        world.replace_box(body(2, Vec3i::ZERO)),
        Err(RotatingWorldError3d::MissingBody(BodyId(2)))
    );
    assert_eq!(
        world.set_motion(BodyId(2), Vec3i::ZERO, AngularVelocity3d::default()),
        Err(RotatingWorldError3d::MissingBody(BodyId(2)))
    );
    assert_eq!(
        world.set_motion(BodyId(0), Vec3i::ZERO, AngularVelocity3d::new(1, 0, 0)),
        Err(RotatingWorldError3d::FixedBodyAngularVelocity(BodyId(0)))
    );
    assert_eq!(
        world.set_motion(BodyId(0), Vec3i::new(1, 0, 0), AngularVelocity3d::default()),
        Err(RotatingWorldError3d::FixedBodyVelocity(BodyId(0)))
    );
    assert_eq!(
        world.boxes().collect::<Vec<_>>(),
        control.boxes().collect::<Vec<_>>()
    );
    assert!(world.is_sleeping(BodyId(1)));
    for _ in 0..4 {
        assert_eq!(world.step(1, 60).unwrap(), control.step(1, 60).unwrap());
        assert_eq!(
            world.boxes().collect::<Vec<_>>(),
            control.boxes().collect::<Vec<_>>()
        );
    }
}

#[cfg_attr(test, test)]
pub fn teleporting_support_wakes_only_its_dynamic_contact_island() {
    let mut world = world();
    world.add_box(floor(0, 0)).unwrap();
    world.add_box(body(1, Vec3i::ZERO)).unwrap();
    world.add_box(body(2, Vec3i::new(0, 2, 0))).unwrap();
    world.add_box(body(3, Vec3i::new(100, 0, 0))).unwrap();
    world.step(1, 1).unwrap();
    assert_eq!(world.sleeping_body_count(), 3);
    assert!(world.replace_box(body(1, Vec3i::new(1000, 0, 0))).unwrap());
    assert!(!world.is_sleeping(BodyId(1)));
    assert!(!world.is_sleeping(BodyId(2)));
    assert!(
        world.is_sleeping(BodyId(3)),
        "a common fixed floor is not a dynamic island edge"
    );
    world.step(1, 60).unwrap();
    assert_eq!(
        world.box_by_id(BodyId(3)).unwrap().body().position(),
        Vec3i::new(100, 0, 0)
    );
}

#[cfg_attr(test, test)]
pub fn new_geometry_admits_real_contacts_and_ignores_near_misses() {
    let mut world = world();
    world.add_box(floor(0, 0)).unwrap();
    world.add_box(body(1, Vec3i::ZERO)).unwrap();
    world.add_box(body(2, Vec3i::new(100, 0, 0))).unwrap();
    world.step(1, 1).unwrap();
    // The floor's old contacts remain valid when only its material changes; both are affected.
    let changed_floor = RigidBox3d::new(
        floor(0, 0)
            .into_body()
            .with_material(Material::new(0).with_friction(700)),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .unwrap();
    world.replace_box(changed_floor).unwrap();
    assert!(!world.is_sleeping(BodyId(1)));
    assert!(!world.is_sleeping(BodyId(2)));
    world.step(1, 1).unwrap();
    world.add_box(body(4, Vec3i::new(400, 10, 0))).unwrap();
    assert!(world.replace_box(body(4, Vec3i::new(4, 0, 0))).unwrap());
    assert!(
        world.is_sleeping(BodyId(1)),
        "separated replacement is not contact evidence"
    );
    assert!(world.is_sleeping(BodyId(2)));
    assert!(world.replace_box(body(4, Vec3i::new(2, 0, 0))).unwrap());
    assert!(
        !world.is_sleeping(BodyId(1)),
        "new touching geometry admits activation"
    );
    assert!(world.is_sleeping(BodyId(2)));
}

#[cfg_attr(test, test)]
pub fn intended_motion_wakes_target_and_honors_rotation_lock() {
    let mut world = world();
    world.add_box(body(1, Vec3i::ZERO)).unwrap();
    world
        .add_box(body(2, Vec3i::new(100, 0, 0)).with_rotation_locked())
        .unwrap();
    world.step(1, 1).unwrap();
    assert!(
        world
            .set_motion(
                BodyId(1),
                Vec3i::new(600, 0, 0),
                AngularVelocity3d::new(0, 100_000, 0)
            )
            .unwrap()
    );
    assert!(!world.is_sleeping(BodyId(1)));
    assert!(world.is_sleeping(BodyId(2)));
    assert!(
        !world
            .set_motion(
                BodyId(2),
                Vec3i::ZERO,
                AngularVelocity3d::new(0, 100_000, 0)
            )
            .unwrap()
    );
    assert!(
        world
            .set_motion(
                BodyId(2),
                Vec3i::new(0, 600, 0),
                AngularVelocity3d::new(0, 100_000, 0)
            )
            .unwrap()
    );
    world.step(1, 60).unwrap();
    assert_eq!(
        world.box_by_id(BodyId(1)).unwrap().body().position(),
        Vec3i::new(10, 0, 0)
    );
    let locked = world.box_by_id(BodyId(2)).unwrap();
    assert_eq!(locked.body().position(), Vec3i::new(100, 10, 0));
    assert_eq!(locked.angular().orientation, Orientation3d::IDENTITY);
    assert!(locked.angular().angular_velocity.is_zero());
}

#[cfg_attr(test, test)]
pub fn fixed_preparation_and_kind_transitions_remain_equivalent() {
    let mut runtime = world();
    runtime.add_box(floor(0, 0)).unwrap();
    runtime.add_box(body(1, Vec3i::ZERO)).unwrap();
    let mut prepared = runtime.clone();
    prepared.set_fixed_geometry_preparation_mode(FixedGeometryPreparationMode3d::PrepareAtLoad);
    for replacement in [floor(0, 500), body(0, Vec3i::new(500, 0, 0)), floor(0, 0)] {
        assert!(runtime.replace_box(replacement.clone()).unwrap());
        assert!(prepared.replace_box(replacement).unwrap());
        assert_eq!(
            runtime.body_contacts(BodyId(1)).unwrap(),
            prepared.body_contacts(BodyId(1)).unwrap()
        );
        assert_eq!(runtime.step(1, 60).unwrap(), prepared.step(1, 60).unwrap());
        assert_eq!(
            runtime.boxes().collect::<Vec<_>>(),
            prepared.boxes().collect::<Vec<_>>()
        );
    }
    assert_eq!(
        prepared
            .fixed_geometry_preparation_stats()
            .prepared_body_count,
        1
    );
    let before = prepared
        .fixed_geometry_preparation_stats()
        .total_preparations;
    let material = RigidBox3d::new(
        floor(0, 0).into_body().with_material(Material::new(50)),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .unwrap();
    prepared.replace_box(material).unwrap();
    assert_eq!(
        prepared
            .fixed_geometry_preparation_stats()
            .total_preparations,
        before
    );
    prepared.remove_box(BodyId(0)).unwrap();
    prepared.add_box(floor(0, 300)).unwrap();
    assert_eq!(
        prepared
            .fixed_geometry_preparation_stats()
            .prepared_body_count,
        1
    );
    assert!(prepared.body_contacts(BodyId(1)).unwrap().is_empty());
}

#[cfg_attr(test, test)]
pub fn authority_changes_update_solver_partitions_without_cross_world_effects() {
    let mut first = RotatingWorld3d::new(RotatingWorldConfig3d {
        gravity: Vec3i::new(0, -600, 0),
        ..RotatingWorldConfig3d::default()
    });
    first.add_box(body(1, Vec3i::ZERO)).unwrap();
    let mut second = first.clone();
    first
        .replace_box(body(1, Vec3i::ZERO).with_external_motion())
        .unwrap();
    first
        .set_motion(
            BodyId(1),
            Vec3i::new(600, 0, 0),
            AngularVelocity3d::default(),
        )
        .unwrap();
    first.step(1, 60).unwrap();
    second.step(1, 60).unwrap();
    assert_eq!(
        first.box_by_id(BodyId(1)).unwrap().body().velocity(),
        Vec3i::new(600, 0, 0)
    );
    assert_eq!(
        second.box_by_id(BodyId(1)).unwrap().body().velocity(),
        Vec3i::new(0, -10, 0)
    );
    let external = first.box_by_id(BodyId(1)).unwrap();
    let physics_owned = external
        .clone()
        .with_motion_authority(MotionAuthority3d::Physics);
    first.replace_box(physics_owned).unwrap();
    first.step(1, 60).unwrap();
    assert_eq!(
        first.box_by_id(BodyId(1)).unwrap().body().velocity(),
        Vec3i::new(600, -10, 0)
    );
}

#[cfg_attr(test, test)]
pub fn overlap_only_edits_preserve_sleep_and_refresh_contact_eligibility() {
    let mut world = world();
    world.add_box(body(1, Vec3i::ZERO)).unwrap();
    world.step(1, 1).unwrap();
    world
        .add_box(body(2, Vec3i::new(2, 0, 0)).with_overlap_only())
        .unwrap();
    world
        .replace_box(body(2, Vec3i::ZERO).with_overlap_only())
        .unwrap();
    assert!(
        world.is_sleeping(BodyId(1)),
        "overlap-only geometry is not a contact dependency"
    );
    world.replace_box(body(2, Vec3i::new(2, 0, 0))).unwrap();
    assert!(!world.is_sleeping(BodyId(1)));
    assert_eq!(world.body_contacts(BodyId(1)).unwrap().len(), 1);
    world
        .replace_box(body(2, Vec3i::new(2, 0, 0)).with_overlap_only())
        .unwrap();
    assert!(world.body_contacts(BodyId(1)).unwrap().is_empty());
    assert_eq!(world.body_overlaps(BodyId(1)).unwrap(), vec![BodyId(2)]);
}

#[cfg_attr(test, test)]
pub fn newly_eligible_layers_wake_a_dependency_beyond_the_replacement_bounds() {
    let mut world = world();
    let eligible = CollisionLayers3d::new(2, 2);
    world
        .add_box(body(1, Vec3i::new(4, 0, 0)).with_collision_layers(eligible))
        .unwrap();
    world.step(1, 1).unwrap();
    assert!(world.is_sleeping(BodyId(1)));
    world
        .add_box(
            body(2, Vec3i::new(2, 0, 0))
                .with_collision_layers(eligible)
                .with_external_motion(),
        )
        .unwrap();
    world
        .add_box(body(3, Vec3i::ZERO).with_collision_layers(CollisionLayers3d::new(1, 1)))
        .unwrap();
    assert!(
        world
            .replace_box(body(3, Vec3i::ZERO).with_collision_layers(eligible))
            .unwrap()
    );
    assert!(
        !world.is_sleeping(BodyId(1)),
        "newly eligible contact with the awake intermediary admits its parked dependency"
    );
    assert_eq!(world.body_contacts(BodyId(3)).unwrap()[0].other, BodyId(2));
}

#[cfg_attr(test, test)]
pub fn material_edits_preserve_simulated_angular_state_and_body_policy() {
    let aggressive = body(5, Vec3i::ZERO).with_aggressive_sleep();
    assert_eq!(
        aggressive
            .clone()
            .with_motion_authority(MotionAuthority3d::Physics),
        aggressive
    );
    let mut world = world();
    world
        .add_box(
            body(1, Vec3i::ZERO)
                .with_external_motion()
                .with_transient_contacts(),
        )
        .unwrap();
    world
        .set_motion(
            BodyId(1),
            Vec3i::new(600, 0, 0),
            AngularVelocity3d::new(123_456, 789_012, 345_678),
        )
        .unwrap();
    world.step(1, 60).unwrap();
    let original = world.box_by_id(BodyId(1)).unwrap().clone();
    let replacement = original
        .clone()
        .with_body(
            original
                .body()
                .clone()
                .with_mass(7)
                .with_material(Material::new(500).with_friction(800)),
        )
        .unwrap();
    world.replace_box(replacement.clone()).unwrap();
    let changed = world.box_by_id(BodyId(1)).unwrap();
    assert_eq!(changed.angular(), original.angular());
    assert_eq!(changed.body().position(), original.body().position());
    assert_eq!(changed.body().velocity(), original.body().velocity());
    assert_eq!(changed.motion_authority(), original.motion_authority());
    assert_eq!(
        changed.contact_persistence(),
        original.contact_persistence()
    );
    assert_eq!(
        original.clone().with_body(body(2, Vec3i::ZERO).into_body()),
        Err(RigidBoxError3d::BodyIdentityChange {
            expected: BodyId(1),
            actual: BodyId(2)
        })
    );
    assert_eq!(
        original
            .clone()
            .with_body(original.body().clone().with_mass(0)),
        Err(RigidBoxError3d::ZeroMass(BodyId(1)))
    );
    assert_eq!(
        original.with_body(floor(1, 0).into_body()),
        Err(RigidBoxError3d::FixedAngularVelocity(BodyId(1)))
    );
    assert_eq!(world.box_by_id(BodyId(1)), Some(&replacement));
}

#[cfg_attr(test, test)]
pub fn interval_event_totals_include_completed_discarded_contact_work() {
    let mut world = world();
    world.add_box(body(1, Vec3i::new(-10, 0, 0))).unwrap();
    world.add_box(body(2, Vec3i::ZERO)).unwrap();
    world
        .set_motion(
            BodyId(1),
            Vec3i::new(1200, 0, 0),
            AngularVelocity3d::default(),
        )
        .unwrap();
    let interval = RotatingIntervalConfig3d {
        timestep_numerator: 1,
        timestep_denominator: 60,
        substeps: 2,
        angular_damping_milli: 1000,
    };
    let mut reports = Vec::new();
    let mut control = world.clone();
    let work = control.advance_interval(interval, &mut reports).unwrap();
    assert!(work.sampled_events > 0);
    assert_eq!(
        work.sampled_events,
        reports
            .iter()
            .map(|report| report.stats.sampled_events)
            .sum()
    );
    assert_eq!(
        work.tail_contacts,
        reports
            .iter()
            .map(|report| report.stats.tail_contacts)
            .sum()
    );
    world
        .add_box(body(3, Vec3i::new(i32::MAX - 128, 0, 0)))
        .unwrap();
    world
        .set_motion(
            BodyId(3),
            Vec3i::new(8400, 0, 0),
            AngularVelocity3d::default(),
        )
        .unwrap();
    let before = world.boxes().cloned().collect::<Vec<_>>();
    let failure = world.advance_interval(interval, &mut reports).unwrap_err();
    assert_eq!(failure.work.completed_substeps, 1);
    assert!(failure.work.sampled_events > 0);
    assert!(reports.is_empty());
    assert_eq!(world.boxes().cloned().collect::<Vec<_>>(), before);
}
