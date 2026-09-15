use super::{CoarseSampleCache3d, PairContactCache3d};
use crate::oriented_box::PreparedObb3d;
use crate::{
    AngularState3d, AngularVelocity3d, BodyId, Orientation3d, OrientedBox3d, OrientedBoxError3d,
    RigidBody, RigidBox3d, RigidBoxFreeFlightConfig3d, RigidBoxFreeFlightError3d,
    RotatingContactSearchConfig3d, RotatingContactSearchError3d, Vec3i, obb_contact_seed,
};

fn shape(center: Vec3i) -> OrientedBox3d {
    OrientedBox3d::new(center, Vec3i::new(2, 2, 2), Orientation3d::IDENTITY)
}

#[test]
fn stationary_grid_reuses_two_preparations_and_one_sat_evaluation() {
    let left = RigidBox3d::new(
        RigidBody::dynamic(
            BodyId(1),
            Vec3i::new(-4, 0, 0),
            Vec3i::ZERO,
            Vec3i::new(2, 2, 2),
        ),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid moving-kind stationary body");
    let right = RigidBox3d::new(
        RigidBody::fixed(BodyId(2), Vec3i::ZERO, Vec3i::new(2, 2, 2)),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid fixed body");
    let search = RotatingContactSearchConfig3d::new(
        RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 60),
        32,
        4,
    );
    let expected = obb_contact_seed(left.oriented_box(), right.oriented_box()).unwrap();
    assert!(expected.is_some());
    let mut geometry = CoarseSampleCache3d::default();
    let mut contacts = PairContactCache3d::default();
    for numerator in 1..=32 {
        let left = geometry
            .sample_geometry(&left, search, numerator, 32)
            .unwrap();
        let right = geometry
            .sample_geometry(&right, search, numerator, 32)
            .unwrap();
        assert_eq!(contacts.contact(&left, &right).unwrap(), expected);
    }
    assert_eq!(geometry.preparations, 2);
    assert_eq!(contacts.evaluations, 1);
}

#[test]
fn every_shape_component_and_pair_order_invalidates_the_exact_result() {
    let left = shape(Vec3i::ZERO);
    let right = shape(Vec3i::new(4, 0, 0));
    let rotated = OrientedBox3d {
        orientation: Orientation3d::new(0, 0, 410_903_207, 992_008_094),
        ..left
    };
    let enlarged = OrientedBox3d {
        half_extents: Vec3i::new(3, 2, 2),
        ..left
    };
    let pairs = [
        (left, right),
        (shape(Vec3i::new(-10, 0, 0)), right),
        (enlarged, right),
        (rotated, right),
        (right, rotated),
        (left, right),
    ];
    let mut contacts = PairContactCache3d::default();
    for (index, (left, right)) in pairs.into_iter().enumerate() {
        let expected = obb_contact_seed(left, right).unwrap();
        let left = PreparedObb3d::new(left);
        let right = PreparedObb3d::new(right);
        assert_eq!(contacts.contact(&left, &right).unwrap(), expected);
        assert_eq!(contacts.contact(&left, &right).unwrap(), expected);
        assert_eq!(contacts.evaluations, index + 1);
    }
}

#[test]
fn geometry_cache_tracks_full_shape_instead_of_only_body_identity() {
    let mut cache = CoarseSampleCache3d::default();
    let original = shape(Vec3i::ZERO);
    let moved = shape(Vec3i::new(1, 0, 0));
    assert_eq!(cache.prepare_geometry(BodyId(7), original).shape, original);
    assert_eq!(cache.prepare_geometry(BodyId(7), moved).shape, moved);
    assert_eq!(cache.prepare_geometry(BodyId(7), original).shape, original);
    assert_eq!(cache.preparations, 3);
    assert_eq!(cache.latest_geometry.len(), 1);
}

#[test]
fn cached_valid_contact_does_not_hide_invalid_geometry_or_error_order() {
    let valid = shape(Vec3i::ZERO);
    let invalid = OrientedBox3d {
        half_extents: Vec3i::new(0, 2, 2),
        ..valid
    };
    let invalid_orientation = OrientedBox3d {
        orientation: Orientation3d::new(0, 0, 0, 0),
        ..valid
    };
    let overflowing = shape(Vec3i::new(i32::MAX, 0, 0));
    let mut contacts = PairContactCache3d::default();
    contacts
        .contact(&PreparedObb3d::new(valid), &PreparedObb3d::new(valid))
        .unwrap();
    for (left, right) in [
        (invalid, invalid_orientation),
        (invalid_orientation, invalid),
        (valid, overflowing),
        (overflowing, valid),
    ] {
        assert_eq!(
            contacts.contact(&PreparedObb3d::new(left), &PreparedObb3d::new(right)),
            obb_contact_seed(left, right).map_err(RotatingContactSearchError3d::Contact),
        );
    }
    assert_eq!(
        contacts.contact(&PreparedObb3d::new(invalid), &PreparedObb3d::new(valid)),
        Err(RotatingContactSearchError3d::Contact(
            OrientedBoxError3d::InvalidHalfExtents
        )),
    );
}

#[test]
fn fixed_geometry_reuse_still_validates_each_uncached_sample_fraction() {
    let fixed = RigidBox3d::new(
        RigidBody::fixed(BodyId(2), Vec3i::ZERO, Vec3i::new(2, 2, 2)),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .unwrap();
    let search = RotatingContactSearchConfig3d::new(
        RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 60),
        32,
        4,
    );
    let mut cache = CoarseSampleCache3d::default();
    cache.sample_geometry(&fixed, search, 1, 32).unwrap();
    assert_eq!(
        cache.sample_geometry(&fixed, search, 1, 0),
        Err(RotatingContactSearchError3d::FreeFlight(
            RigidBoxFreeFlightError3d::ZeroFractionDenominator,
        )),
    );
}
