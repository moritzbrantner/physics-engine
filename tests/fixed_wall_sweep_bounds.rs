use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d,
    RigidBoxFreeFlightConfig3d, RigidBoxFreeFlightError3d, RotatingContactSearchConfig3d,
    RotationalSweepPair3d, Vec3i, oriented_box_vertices, rigid_box_free_flight_sweep_bounds,
    rotational_sweep_candidate_pairs, sample_rigid_box_free_flight,
    sampled_rotating_contact_search, sampled_rotating_recontact_search,
};

fn wall(orientation: Orientation3d) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::fixed(BodyId(11), Vec3i::new(0, 72, -520), Vec3i::new(520, 72, 8)),
        AngularState3d::new(orientation, AngularVelocity3d::default()),
    )
    .expect("valid fixed wall")
}

fn projectile(position: Vec3i, velocity: Vec3i) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::dynamic(BodyId(1000), position, velocity, Vec3i::new(3, 3, 3)),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid projectile")
}

#[test]
fn immovable_thin_wall_keeps_its_actual_sixteen_unit_thickness() {
    let wall = wall(Orientation3d::IDENTITY);
    let bounds = rigid_box_free_flight_sweep_bounds(
        &wall,
        RigidBoxFreeFlightConfig3d::new(Vec3i::new(0, -3600, 0), 1, 60),
    )
    .unwrap();
    assert_eq!(bounds.minimum, [-520, 0, -528]);
    assert_eq!(bounds.maximum, [520, 144, -512]);
}

#[test]
fn rotated_fixed_bounds_cover_every_canonical_sample_and_equal_current_vertices() {
    let orientations = [
        Orientation3d::IDENTITY,
        Orientation3d::new(0, 0, 410_903_207, 992_008_094),
        Orientation3d::new(-1_875_283_248, 1_103_306_364, -807_599_983, -607_366_902),
    ];
    for orientation in orientations {
        let wall = wall(orientation.normalized().unwrap());
        for (numerator, denominator) in [(1, 60), (3, 2), (i32::MAX, 1)] {
            let config = RigidBoxFreeFlightConfig3d::new(
                Vec3i::new(i32::MAX, i32::MIN, i32::MAX),
                numerator,
                denominator,
            );
            let bounds = rigid_box_free_flight_sweep_bounds(&wall, config).unwrap();
            let vertices = oriented_box_vertices(wall.oriented_box()).unwrap();
            for axis in 0..3 {
                let components = vertices.map(|v| i64::from([v.x, v.y, v.z][axis]));
                assert_eq!(bounds.minimum[axis], *components.iter().min().unwrap());
                assert_eq!(bounds.maximum[axis], *components.iter().max().unwrap());
            }
            for sample in 0..=32 {
                let sampled = sample_rigid_box_free_flight(&wall, config, sample, 32).unwrap();
                assert_eq!(sampled, wall);
                for vertex in oriented_box_vertices(sampled.oriented_box()).unwrap() {
                    assert!(bounds.contains([
                        i64::from(vertex.x),
                        i64::from(vertex.y),
                        i64::from(vertex.z),
                    ]));
                }
            }
        }
    }
}

#[test]
fn remote_wall_is_pruned_but_swept_projectile_and_touching_pairs_are_retained() {
    let wall = wall(Orientation3d::IDENTITY);
    let config = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 60);
    let far = projectile(Vec3i::new(0, 72, 0), Vec3i::ZERO);
    let far_pairs = rotational_sweep_candidate_pairs(&[wall.clone(), far], config).unwrap();
    assert!(far_pairs.is_empty());
    let pair = RotationalSweepPair3d {
        left: BodyId(11),
        right: BodyId(1000),
    };
    let search = RotatingContactSearchConfig3d::new(config, 32, 4);
    for projectile in [
        projectile(Vec3i::new(0, 72, -480), Vec3i::new(0, 0, -5760)),
        projectile(Vec3i::new(0, 72, -509), Vec3i::ZERO),
    ] {
        let boxes = [wall.clone(), projectile];
        let pairs = rotational_sweep_candidate_pairs(&boxes, config).unwrap();
        assert_eq!(pairs, vec![pair]);
        let hit = sampled_rotating_contact_search(&boxes, search).unwrap().unwrap();
        assert_eq!(hit.pair, pair);
    }
    let boxes = [
        wall,
        projectile(Vec3i::new(0, 72, -480), Vec3i::new(0, 0, -5760)),
    ];
    let hit = sampled_rotating_recontact_search(&boxes, search).unwrap().unwrap();
    assert_eq!(hit.pair, pair);
}

#[test]
fn fixed_bound_fast_path_does_not_bypass_invalid_time() {
    let wall = wall(Orientation3d::IDENTITY);
    let negative = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, -1, 60);
    assert_eq!(
        rigid_box_free_flight_sweep_bounds(&wall, negative),
        Err(RigidBoxFreeFlightError3d::NegativeTimestepNumerator(-1)),
    );
    let zero_denominator = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 0);
    assert_eq!(
        rigid_box_free_flight_sweep_bounds(&wall, zero_denominator),
        Err(RigidBoxFreeFlightError3d::NonPositiveTimestepDenominator(0)),
    );
}
