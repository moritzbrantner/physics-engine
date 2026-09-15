use crate::{
    AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d,
    RigidBoxFreeFlightConfig3d, RotatingContactSearchConfig3d, Vec3i,
    rotating_broad_phase::RotatingBroadPhase3d,
    rotating_contact_search_ordered::sampled_rotating_contact_search_with_broad_phase as ordered_search,
    rotating_contact_search_reference::sampled_rotating_contact_search_with_broad_phase as reference_search,
};

fn dynamic(id: u64, position: Vec3i, velocity: Vec3i) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::dynamic(BodyId(id), position, velocity, Vec3i::new(1, 1, 1)),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid dynamic box")
}

fn fixed(id: u64, position: Vec3i) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::fixed(BodyId(id), position, Vec3i::new(1, 1, 1)),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid fixed box")
}

fn config(samples: u16, refinement_steps: u8) -> RotatingContactSearchConfig3d {
    RotatingContactSearchConfig3d::new(
        RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1),
        samples,
        refinement_steps,
    )
}

fn assert_exact_reference_result(boxes: &[RigidBox3d], config: RotatingContactSearchConfig3d) {
    let mut reference_broad_phase = RotatingBroadPhase3d::default();
    let reference = reference_search(boxes, config, &mut reference_broad_phase);
    let mut ordered_broad_phase = RotatingBroadPhase3d::default();
    let ordered = ordered_search(boxes, config, &mut ordered_broad_phase);
    assert_eq!(ordered, reference);
}

#[test]
fn ordered_search_exactly_matches_reference_when_later_pair_refines_earlier() {
    let moving = dynamic(10, Vec3i::new(-10, 0, 0), Vec3i::new(20, 0, 0));
    let later_hit_first_pair = fixed(1, Vec3i::ZERO);
    let earlier_hit_later_pair = fixed(2, Vec3i::new(-1, 0, 0));

    assert_exact_reference_result(
        &[moving, later_hit_first_pair, earlier_hit_later_pair],
        config(4, 3),
    );
}

#[test]
fn ordered_search_exactly_matches_reference_for_resolution_failures() {
    let boxes = [
        dynamic(1, Vec3i::ZERO, Vec3i::ZERO),
        fixed(2, Vec3i::new(2, 0, 0)),
    ];

    assert_exact_reference_result(&boxes, config(0, 0));
    assert_exact_reference_result(&boxes, config(u16::MAX, 17));
}

#[test]
fn ordered_search_exactly_matches_reference_for_duplicate_identity_failure() {
    let boxes = [
        dynamic(7, Vec3i::new(-4, 0, 0), Vec3i::new(8, 0, 0)),
        fixed(7, Vec3i::new(4, 0, 0)),
    ];

    assert_exact_reference_result(&boxes, config(8, 2));
}
