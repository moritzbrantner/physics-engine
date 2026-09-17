use physics_engine::{
    AngularState3d, AngularVelocity3d, BallisticSphere3d, BallisticSphereQueryStats3d,
    BallisticSphereScene3d, BodyId, ORIENTATION_SCALE, Orientation3d, RigidBody, RigidBox3d, Vec3i,
};

fn target(id: u64, orientation: Orientation3d) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::fixed(BodyId(id), Vec3i::ZERO, Vec3i::new(40, 4, 4)),
        AngularState3d::new(orientation, AngularVelocity3d::default()),
    )
    .expect("valid target")
}

#[test]
fn prepared_ballistic_scene_respects_target_orientation() {
    let orientation = Orientation3d::new(0, 0, ORIENTATION_SCALE / 2, ORIENTATION_SCALE)
        .normalized()
        .expect("valid non-axis-aligned orientation");
    let rotated = target(1, orientation);
    let unrotated = target(2, Orientation3d::IDENTITY);
    let projectile = BallisticSphere3d::new(
        BodyId(1_000),
        Vec3i::new(-100, 20, 0),
        Vec3i::new(12_000, 0, 0),
        2,
        1,
    )
    .expect("valid sphere");

    let rotated_scene = BallisticSphereScene3d::prepare([&rotated]).expect("rotated scene");
    let unrotated_scene = BallisticSphereScene3d::prepare([&unrotated]).expect("unrotated scene");
    let mut rotated_stats = BallisticSphereQueryStats3d::default();
    let mut unrotated_stats = BallisticSphereQueryStats3d::default();

    let rotated_hit = rotated_scene
        .earliest_hit(projectile, 1, 60, &mut rotated_stats)
        .expect("rotated query");
    let unrotated_hit = unrotated_scene
        .earliest_hit(projectile, 1, 60, &mut unrotated_stats)
        .expect("unrotated query");

    assert!(rotated_hit.is_some());
    assert!(unrotated_hit.is_none());
    assert_eq!(rotated_stats.broad_phase_candidates, 1);
    assert_eq!(rotated_stats.toi_tests, 1);
    assert_eq!(rotated_stats.feature_tests, 26);
    assert_eq!(unrotated_stats.broad_phase_candidates, 0);
    assert_eq!(unrotated_stats.toi_tests, 0);
}
