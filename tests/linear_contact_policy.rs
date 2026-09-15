use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, ContactMode3d, Material, Orientation3d, RigidBody,
    RigidBox3d, RigidBoxFreeFlightConfig3d, Vec3i, resolve_obb_contact,
    sample_rigid_box_free_flight,
};

fn body(id: u64, position: Vec3i, velocity: Vec3i, half: Vec3i) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::dynamic(BodyId(id), position, velocity, half)
            .with_mass(if id == 1 { 4 } else { 2 })
            .with_material(Material::new(0).with_friction(1000)),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .unwrap()
}

#[test]
fn off_centre_push_moves_the_object_without_adding_spin() {
    let player = body(
        1,
        Vec3i::new(-22, 30, 0),
        Vec3i::new(420, 0, 0),
        Vec3i::new(12, 20, 12),
    )
    .with_rotation_locked();
    let object = body(2, Vec3i::new(0, 10, 0), Vec3i::ZERO, Vec3i::new(10, 10, 10));
    let physical = resolve_obb_contact(player.clone(), object.clone(), true).unwrap();
    assert!(
        !physical.right.angular().angular_velocity.is_zero(),
        "fixture must exercise off-centre torque"
    );
    let linear = resolve_obb_contact(
        player.with_linear_push(Vec3i::new(0, -1, 0)),
        object.clone(),
        true,
    )
    .unwrap();
    assert!(linear.right.body().velocity().x > 0);
    assert_eq!(linear.right.angular(), object.angular());
    assert!(!linear.right.rotation_locked());
    assert_eq!(linear.right.body().kind(), object.body().kind());
}

#[test]
fn landing_projects_only_the_actuator_and_preserves_the_support() {
    for reverse in [false, true] {
        let player = body(
            if reverse { 3 } else { 1 },
            Vec3i::new(3, 39, 2),
            Vec3i::new(0, -600, 0),
            Vec3i::new(12, 20, 12),
        )
        .with_linear_push(Vec3i::new(0, -1, 0));
        let object = body(2, Vec3i::new(0, 10, 0), Vec3i::ZERO, Vec3i::new(18, 10, 18));
        let response = if reverse {
            resolve_obb_contact(object.clone(), player, true).unwrap()
        } else {
            resolve_obb_contact(player, object.clone(), true).unwrap()
        };
        let (player, support) = if reverse {
            (response.right, response.left)
        } else {
            (response.left, response.right)
        };
        assert_eq!(support, object);
        assert_eq!(player.body().position().y, 40);
        assert_eq!(player.body().velocity().y, 0);
        assert!(player.angular().angular_velocity.is_zero());
    }
}

#[test]
fn moving_support_keeps_its_velocity_and_existing_rotation() {
    let player = body(
        1,
        Vec3i::new(0, 40, 0),
        Vec3i::new(0, -600, 0),
        Vec3i::new(12, 20, 12),
    )
    .with_linear_push(Vec3i::new(0, -1, 0));
    let support = RigidBox3d::new(
        RigidBody::dynamic(
            BodyId(2),
            Vec3i::new(0, 10, 0),
            Vec3i::new(50, 20, 0),
            Vec3i::new(18, 10, 18),
        ),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::new(0, 50000, 0)),
    )
    .unwrap();
    let response = resolve_obb_contact(player, support.clone(), false).unwrap();
    assert_eq!(response.right, support);
    assert_eq!(response.left.body().velocity().y, 20);
}

#[test]
fn sampled_free_flight_retains_the_opt_in_policy() {
    let player = body(
        1,
        Vec3i::ZERO,
        Vec3i::new(0, -60, 0),
        Vec3i::new(12, 20, 12),
    )
    .with_linear_push(Vec3i::new(0, -1, 0));
    let sampled = sample_rigid_box_free_flight(
        &player,
        RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 60),
        1,
        1,
    )
    .unwrap();
    assert_eq!(
        sampled.contact_mode(),
        ContactMode3d::LinearPush {
            support_direction: Vec3i::new(0, -1, 0)
        }
    );
    assert!(sampled.rotation_locked());
}
