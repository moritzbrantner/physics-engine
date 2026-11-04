//! Distance joints (#453): lifecycle validation, stable ids, canonical order, removal with a
//! body, island sleep and wake through a joint, rollback and the checkpoint boundary. The Rapier
//! comparison is the `rope-joints` oracle scenario.
use physics_engine::approximate::{
    Body, BodyStorage, Config, DistanceJoint, Error, Shape, Vector, World,
};

const DT: f64 = 1.0 / 60.0;

fn world(storage: BodyStorage) -> World {
    World::new(Config {
        body_storage: storage,
        ..Config::default()
    })
    .unwrap()
}
fn anchor(w: &mut World, key: u64, at: Vector) -> physics_engine::BodyId {
    let mut b = Body::keyed(key, Shape::Sphere(0.05), at, 0.0);
    b.sensor = true;
    w.add_body(b).unwrap()
}
fn load(w: &mut World, key: u64, at: Vector) -> physics_engine::BodyId {
    w.add_body(Body::keyed(
        key,
        Shape::Box(Vector(0.25, 0.25, 0.25)),
        at,
        10.0,
    ))
    .unwrap()
}

#[test]
fn add_joint_validates_its_bodies_and_numbers() {
    let mut w = world(BodyStorage::Ordered);
    let a = anchor(&mut w, 1, Vector(0.0, 4.0, 0.0));
    let b = load(&mut w, 2, Vector(0.0, 2.0, 0.0));
    let fixed = anchor(&mut w, 3, Vector(5.0, 4.0, 0.0));
    let rope = DistanceJoint::rope(a, b, 2.0);
    for bad in [
        DistanceJoint { b: a, ..rope },
        DistanceJoint {
            max_length: 0.0,
            ..rope
        },
        DistanceJoint {
            max_length: -1.0,
            ..rope
        },
        DistanceJoint {
            max_length: f32::NAN,
            ..rope
        },
        DistanceJoint {
            anchor_a: Vector(f32::INFINITY, 0.0, 0.0),
            ..rope
        },
        // Neither side dynamic.
        DistanceJoint::rope(a, fixed, 1.0),
    ] {
        assert_eq!(w.add_joint(bad), Err(Error::InvalidInput));
    }
    let mut projectile = Body::new(Shape::Sphere(0.1), Vector(9.0, 1.0, 0.0), 1.0);
    projectile.retire_on_impact = true;
    let projectile = w.add_body(projectile).unwrap();
    assert_eq!(
        w.add_joint(DistanceJoint::rope(a, projectile, 1.0)),
        Err(Error::InvalidInput)
    );
    let removed = load(&mut w, 4, Vector(3.0, 1.0, 0.0));
    w.remove_body(removed).unwrap();
    assert_eq!(
        w.add_joint(DistanceJoint::rope(a, removed, 1.0)),
        Err(Error::MissingBody(removed))
    );
    assert_eq!(w.joint_count(), 0);
    let id = w.add_joint(rope).unwrap();
    assert_eq!(w.joint(id), Some(&rope));
    assert_eq!(w.joint_count(), 1);
}

#[test]
fn ids_are_never_reused_and_removal_follows_the_body() {
    let mut w = world(BodyStorage::Ordered);
    let a = anchor(&mut w, 1, Vector(0.0, 4.0, 0.0));
    let b = load(&mut w, 2, Vector(0.0, 2.0, 0.0));
    let first = w.add_joint(DistanceJoint::rope(a, b, 2.0)).unwrap();
    assert!(w.remove_joint(first).is_some());
    assert!(w.remove_joint(first).is_none());
    assert!(w.joint(first).is_none());
    let second = w.add_joint(DistanceJoint::rope(a, b, 2.0)).unwrap();
    assert_ne!(first, second);
    // Removing a body removes its joints; a new body in the same slot gets none of them.
    w.remove_body(b).unwrap();
    assert!(w.joint(second).is_none());
    assert_eq!(w.joint_count(), 0);
    let c = load(&mut w, 2, Vector(0.0, 2.0, 0.0));
    assert_eq!(c.slot(), b.slot());
    let third = w.add_joint(DistanceJoint::rope(a, c, 2.0)).unwrap();
    assert!(third != first && third != second);
    assert_eq!(
        physics_engine::approximate::JointId::from_bits(third.to_bits()),
        third
    );
}

#[test]
fn rows_follow_body_order_not_insertion_order() {
    let build = |reverse: bool| {
        let mut w = world(BodyStorage::Ordered);
        let mut ids = Vec::new();
        let keys: Vec<u64> = if reverse {
            (0..4).rev().collect()
        } else {
            (0..4).collect()
        };
        for &k in &keys {
            let a = anchor(&mut w, 100 + k, Vector(k as f32 * 3.0, 4.0, 0.0));
            let b = load(&mut w, 200 + k, Vector(k as f32 * 3.0 - 1.0, 2.5, 0.0));
            ids.push((k, a, b));
        }
        for &(k, a, b) in &ids {
            w.add_joint(DistanceJoint {
                anchor_b: Vector(0.0, 0.25, 0.0),
                ..DistanceJoint::rope(a, b, 1.5 + 0.1 * k as f32)
            })
            .unwrap();
        }
        for _ in 0..90 {
            w.step(DT).unwrap();
        }
        let order: Vec<f32> = w.joints().map(|(_, j)| j.max_length).collect();
        let state: Vec<_> = (0..4)
            .map(|k| w.body(w.body_id(200 + k).unwrap()).unwrap().position)
            .collect();
        (order, state)
    };
    let (order, state) = build(false);
    let (reversed_order, reversed_state) = build(true);
    assert_eq!(order, vec![1.5, 1.6, 1.7, 1.8]);
    assert_eq!(order, reversed_order);
    assert_eq!(state, reversed_state);
}

#[test]
fn both_storage_modes_give_identical_results() {
    let run = |storage| {
        let mut w = world(storage);
        let a = anchor(&mut w, 1, Vector(0.0, 4.0, 0.0));
        let b = load(&mut w, 2, Vector(-1.5, 3.0, 0.0));
        let j = w.add_joint(DistanceJoint::rope(a, b, 2.0)).unwrap();
        let mut out = Vec::new();
        for _ in 0..120 {
            w.step(DT).unwrap();
            let body = w.body(b).unwrap();
            out.push((body.position, body.velocity, w.joint_impulse(j).unwrap()));
        }
        out
    };
    assert_eq!(run(BodyStorage::Ordered), run(BodyStorage::Indexed));
}

#[test]
fn a_hanging_load_holds_its_weight_and_sleeps() {
    let mut w = world(BodyStorage::Ordered);
    let a = anchor(&mut w, 1, Vector(0.0, 4.0, 0.0));
    let b = load(&mut w, 2, Vector(0.0, 1.75, 0.0));
    let j = w
        .add_joint(DistanceJoint {
            anchor_b: Vector(0.0, 0.25, 0.0),
            ..DistanceJoint::rope(a, b, 2.0)
        })
        .unwrap();
    for _ in 0..120 {
        w.step(DT).unwrap();
    }
    let body = w.body(b).unwrap();
    assert!(body.is_sleeping(), "a still hanging load sleeps");
    assert!((body.position.1 - 1.75).abs() < 1e-3, "{:?}", body.position);
    let h = DT as f32 / f32::from(Config::default().substeps);
    let tension = w.joint_impulse(j).unwrap() / h;
    assert!((tension - 10.0 * 9.81).abs() < 0.01 * 98.1, "{tension}");
    // Removing the joint wakes the load, which falls.
    w.remove_joint(j).unwrap();
    assert!(!w.body(b).unwrap().is_sleeping());
    w.step(DT).unwrap();
    assert!(w.body(b).unwrap().position.1 < 1.75);
}

#[test]
fn a_slack_rope_lets_the_bodies_close_and_a_taut_one_stops_them() {
    let mut w = World::new(Config {
        gravity: Vector::ZERO,
        ..Config::default()
    })
    .unwrap();
    let a = w
        .add_body(Body::keyed(1, Shape::Sphere(0.1), Vector::ZERO, 1.0))
        .unwrap();
    let b = w
        .add_body(Body::keyed(
            2,
            Shape::Sphere(0.1),
            Vector(1.0, 0.0, 0.0),
            3.0,
        ))
        .unwrap();
    w.add_joint(DistanceJoint::rope(a, b, 2.0)).unwrap();
    // Closing: the rope does nothing.
    w.set_velocity(b, Vector(-1.0, 0.0, 0.0)).unwrap();
    w.step(DT).unwrap();
    assert_eq!(w.body(b).unwrap().velocity, Vector(-1.0, 0.0, 0.0));
    assert_eq!(w.body(a).unwrap().velocity, Vector::ZERO);
    // Separating: the rope stops them at its length, momentum kept.
    w.set_velocity(b, Vector(5.0, 0.0, 0.0)).unwrap();
    for _ in 0..30 {
        w.step(DT).unwrap();
    }
    let (pa, pb) = (w.body(a).unwrap(), w.body(b).unwrap());
    let gap = pb.position.0 - pa.position.0;
    assert!(gap <= 2.0 + 1e-3, "{gap}");
    let momentum = pa.velocity.0 * 1.0 + pb.velocity.0 * 3.0;
    assert!((momentum - 15.0).abs() < 1e-3, "{momentum}");
    assert!((pa.velocity.0 - pb.velocity.0).abs() < 1e-3);
}

#[test]
fn a_moving_kinematic_anchor_wakes_its_sleeping_crate() {
    let mut w = world(BodyStorage::Ordered);
    w.add_body(Body::new(Shape::HalfSpace, Vector::ZERO, 0.0))
        .unwrap();
    let mut driver = Body::keyed(1, Shape::Sphere(0.05), Vector(1.0, 1.0, 0.0), 1.0);
    driver.external = true;
    driver.sensor = true;
    let driver = w.add_body(driver).unwrap();
    let mut crate_ = Body::keyed(
        2,
        Shape::Box(Vector(0.35, 0.35, 0.35)),
        Vector(0.0, 0.35, 0.0),
        50.0,
    );
    crate_.rotation_locked = true;
    let crate_ = w.add_body(crate_).unwrap();
    w.add_joint(DistanceJoint {
        anchor_b: Vector(0.35, 0.0, 0.0),
        ..DistanceJoint::rope(driver, crate_, 1.5)
    })
    .unwrap();
    for _ in 0..120 {
        w.step(DT).unwrap();
    }
    assert!(w.body(crate_).unwrap().is_sleeping());
    w.set_velocity(driver, Vector(2.0, 0.0, 0.0)).unwrap();
    for _ in 0..90 {
        w.step(DT).unwrap();
    }
    let c = w.body(crate_).unwrap();
    assert!(!c.is_sleeping());
    assert!(c.position.0 > 1.5, "dragged: {:?}", c.position);
    assert!((c.velocity.0 - 2.0).abs() < 0.05, "{:?}", c.velocity);
}

#[test]
fn a_failed_step_keeps_the_joint_history() {
    // The f32 spacing near 1e12 is 2^16: move by many of them.
    const EDGE: f32 = 16_777_216.0;
    let mut w = World::new(Config {
        substeps: 1,
        ..Config::default()
    })
    .unwrap();
    let a = anchor(&mut w, 1, Vector(0.0, 4.0, 0.0));
    // Swinging, so its impulse changes every step.
    let b = load(&mut w, 2, Vector(-1.5, 4.0 - 1.75_f32.sqrt(), 0.0));
    let j = w.add_joint(DistanceJoint::rope(a, b, 2.0)).unwrap();
    for _ in 0..10 {
        w.step(DT).unwrap();
    }
    let (impulse, body) = (w.joint_impulse(j), w.body(b).unwrap().clone());
    // A body that leaves the finite range fails the step after the joint was solved.
    let mut runaway = Body::keyed(9, Shape::Sphere(1.0), Vector(1e12 - EDGE, 0.0, 0.0), 1.0);
    runaway.velocity = Vector(20.0 * EDGE, 0.0, 0.0);
    w.add_body(runaway).unwrap();
    assert!(matches!(w.step(0.1), Err(Error::NonFiniteState(_))));
    assert_eq!(w.joint_impulse(j), impulse);
    assert_eq!(w.body(b), Some(&body));
    // The same step without the runaway continues from the kept history.
    let runaway = w.body_id(9).unwrap();
    w.remove_body(runaway).unwrap();
    w.step(DT).unwrap();
    assert_ne!(w.joint_impulse(j), impulse);
}

#[cfg(feature = "checkpoints")]
#[test]
fn worlds_with_joints_are_not_checkpointed() {
    use physics_engine::approximate::{CheckpointContext, CheckpointError};
    let context = CheckpointContext {
        build: [1; 32],
        content: [2; 32],
    };
    let mut w = World::new(Config::default()).unwrap();
    let a = w
        .add_body(Body::keyed(
            1,
            Shape::Sphere(0.1),
            Vector(0.0, 4.0, 0.0),
            0.0,
        ))
        .unwrap();
    let b = load(&mut w, 2, Vector(0.0, 2.0, 0.0));
    let j = w.add_joint(DistanceJoint::rope(a, b, 2.0)).unwrap();
    assert_eq!(
        w.checkpoint(context).err(),
        Some(CheckpointError::UnsupportedJoints)
    );
    w.remove_joint(j).unwrap();
    assert!(w.checkpoint(context).is_ok());
}
