use std::{hint::black_box, time::Instant};

use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d,
    RigidBoxFreeFlightConfig3d, RotatingContactSearchConfig3d, RotationalSweepPair3d, Vec3i,
    sampled_rotating_contact_search,
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

fn config() -> RotatingContactSearchConfig3d {
    RotatingContactSearchConfig3d::new(RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1), 64, 4)
}

fn projectile_lane(body_count: u64) -> Vec<RigidBox3d> {
    assert!((30..=40).contains(&body_count));
    let mut boxes = vec![dynamic(1, Vec3i::new(-40, 0, 0), Vec3i::new(120, 0, 0))];
    for id in 2..body_count {
        let offset = i32::try_from(id - 2).expect("small fixture id");
        boxes.push(fixed(id, Vec3i::new(10 + offset * 2, 0, 0)));
    }
    boxes.push(fixed(body_count, Vec3i::new(-20, 0, 0)));
    boxes
}

#[test]
fn thirty_six_body_first_contact_lane_selects_front_loaded_contact() {
    let boxes = projectile_lane(36);
    let hit = sampled_rotating_contact_search(&boxes, config())
        .expect("valid 36-body first-contact search")
        .expect("projectile lane should contact");
    assert_eq!(
        hit.pair,
        RotationalSweepPair3d {
            left: BodyId(1),
            right: BodyId(36),
        }
    );
}

#[test]
#[ignore = "advisory release-mode performance evidence"]
fn thirty_to_forty_body_first_contact_lane_benchmark() {
    for body_count in [30_u64, 36, 40] {
        let boxes = projectile_lane(body_count);
        let expected = RotationalSweepPair3d {
            left: BodyId(1),
            right: BodyId(body_count),
        };
        let mut samples = Vec::with_capacity(25);
        for _ in 0..5 {
            let hit = black_box(sampled_rotating_contact_search(black_box(&boxes), config()))
                .expect("warm first-contact search")
                .expect("warm fixture contact");
            assert_eq!(hit.pair, expected);
        }
        for _ in 0..25 {
            let start = Instant::now();
            let hit = black_box(sampled_rotating_contact_search(black_box(&boxes), config()))
                .expect("measured first-contact search")
                .expect("measured fixture contact");
            samples.push(start.elapsed());
            assert_eq!(hit.pair, expected);
        }
        samples.sort_unstable();
        let median = samples[samples.len() / 2];
        println!(
            "sample-major first-contact lane: bodies={body_count}, median={median:?}, selected={}-{}",
            expected.left.0, expected.right.0
        );
    }
}
