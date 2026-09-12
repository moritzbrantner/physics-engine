use std::{
    collections::{BTreeMap, BTreeSet},
    hint::black_box,
    time::Instant,
};

use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, ORIENTATION_SCALE, Orientation3d, RigidBody,
    RigidBox3d, RigidBoxFreeFlightConfig3d, Vec3i, obb_contact_seed,
    rotational_sweep_candidate_pairs,
};

fn dynamic_box(
    id: u64,
    center: Vec3i,
    half_extents: Vec3i,
    orientation: Orientation3d,
) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::dynamic(BodyId(id), center, Vec3i::ZERO, half_extents),
        AngularState3d::new(orientation, AngularVelocity3d::default()),
    )
    .expect("valid deterministic test box")
}

fn legacy_all_pairs_contacts(boxes: &[RigidBox3d]) -> BTreeSet<(BodyId, BodyId)> {
    let mut contacts = BTreeSet::new();
    for left_index in 0..boxes.len() {
        for right_index in (left_index + 1)..boxes.len() {
            if obb_contact_seed(
                boxes[left_index].oriented_box(),
                boxes[right_index].oriented_box(),
            )
            .expect("valid oracle OBB query")
            .is_none()
            {
                continue;
            }
            let left = boxes[left_index].body().id();
            let right = boxes[right_index].body().id();
            contacts.insert(if left < right {
                (left, right)
            } else {
                (right, left)
            });
        }
    }
    contacts
}

fn broad_phase_contacts(boxes: &[RigidBox3d]) -> BTreeSet<(BodyId, BodyId)> {
    let zero_time = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 0, 1);
    let candidates = rotational_sweep_candidate_pairs(boxes, zero_time)
        .expect("valid zero-time broad-phase query");
    let by_id = boxes
        .iter()
        .map(|rigid_box| (rigid_box.body().id(), rigid_box))
        .collect::<BTreeMap<_, _>>();
    candidates
        .into_iter()
        .filter_map(|pair| {
            let left = by_id.get(&pair.left).expect("candidate left body");
            let right = by_id.get(&pair.right).expect("candidate right body");
            obb_contact_seed(left.oriented_box(), right.oriented_box())
                .expect("valid exact OBB query")
                .is_some()
                .then_some((pair.left, pair.right))
        })
        .collect()
}

fn deterministic_orientation(index: u64) -> Orientation3d {
    match index % 5 {
        0 => Orientation3d::IDENTITY,
        1 => Orientation3d::new(
            ORIENTATION_SCALE / 3,
            0,
            ORIENTATION_SCALE / 7,
            ORIENTATION_SCALE,
        )
        .normalized()
        .expect("valid orientation one"),
        2 => Orientation3d::new(
            0,
            ORIENTATION_SCALE / 2,
            ORIENTATION_SCALE / 5,
            ORIENTATION_SCALE,
        )
        .normalized()
        .expect("valid orientation two"),
        3 => Orientation3d::new(
            -ORIENTATION_SCALE / 4,
            ORIENTATION_SCALE / 6,
            ORIENTATION_SCALE / 3,
            ORIENTATION_SCALE,
        )
        .normalized()
        .expect("valid orientation three"),
        _ => Orientation3d::new(
            ORIENTATION_SCALE / 8,
            -ORIENTATION_SCALE / 3,
            ORIENTATION_SCALE / 9,
            ORIENTATION_SCALE,
        )
        .normalized()
        .expect("valid orientation four"),
    }
}

#[test]
fn review_counterexample_is_retained_by_zero_time_broad_phase() {
    let orientation = Orientation3d::new(-1_875_283_248, 1_103_306_364, -807_599_983, -607_366_902)
        .normalized()
        .expect("valid review orientation");
    let boxes = [
        dynamic_box(1, Vec3i::ZERO, Vec3i::new(426, 340, 538), orientation),
        dynamic_box(
            2,
            Vec3i::new(-10, 1533, -10),
            Vec3i::new(426, 340, 538),
            orientation,
        ),
    ];

    let expected = legacy_all_pairs_contacts(&boxes);
    assert_eq!(expected, BTreeSet::from([(BodyId(1), BodyId(2))]));
    assert_eq!(broad_phase_contacts(&boxes), expected);
}

#[test]
fn zero_time_broad_phase_matches_all_pairs_oracle_across_rotated_scene() {
    let boxes = (0..96_u64)
        .map(|index| {
            let x = i32::try_from(index % 12).expect("small x") * 5;
            let y = i32::try_from((index / 12) % 4).expect("small y") * 5;
            let z = i32::try_from(index / 48).expect("small z") * 7;
            let z_extent = if index % 3 == 0 { 3 } else { 2 };
            let half_extents = Vec3i::new(
                2 + i32::try_from(index % 2).expect("small extent"),
                2,
                z_extent,
            );
            dynamic_box(
                index + 1,
                Vec3i::new(x, y, z),
                half_extents,
                deterministic_orientation(index),
            )
        })
        .collect::<Vec<_>>();

    let expected = legacy_all_pairs_contacts(&boxes);
    let actual = broad_phase_contacts(&boxes);
    assert!(!expected.is_empty(), "scene must exercise actual contacts");
    assert_eq!(actual, expected);
}

#[test]
#[ignore = "release-mode performance evidence; run explicitly with --ignored --nocapture"]
fn zero_time_broad_phase_sparse_scene_benchmark() {
    let boxes = (0..1024_u64)
        .map(|index| {
            let x = i32::try_from(index % 32).expect("small x") * 40;
            let y = i32::try_from(index / 32).expect("small y") * 40;
            dynamic_box(
                index + 1,
                Vec3i::new(x, y, i32::try_from(index % 7).expect("small z") * 40),
                Vec3i::new(3, 2, 4),
                deterministic_orientation(index),
            )
        })
        .collect::<Vec<_>>();

    let expected = legacy_all_pairs_contacts(&boxes);
    let actual = broad_phase_contacts(&boxes);
    assert_eq!(actual, expected, "benchmark algorithms must be equivalent");

    let iterations = 8_u32;
    let legacy_start = Instant::now();
    for _ in 0..iterations {
        black_box(legacy_all_pairs_contacts(black_box(&boxes)));
    }
    let legacy_elapsed = legacy_start.elapsed();

    let optimized_start = Instant::now();
    for _ in 0..iterations {
        black_box(broad_phase_contacts(black_box(&boxes)));
    }
    let optimized_elapsed = optimized_start.elapsed();
    let speedup = legacy_elapsed.as_secs_f64() / optimized_elapsed.as_secs_f64();

    eprintln!(
        "zero-time current-contact 1024 bodies × {iterations}: all_pairs={legacy_elapsed:?}, broad_phase={optimized_elapsed:?}, speedup={speedup:.3}x, contacts={}",
        expected.len()
    );
}
