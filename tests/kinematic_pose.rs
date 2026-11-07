//! Rotating kinematic bodies (#520): `set_next_kinematic_pose` moves an external body to a pose
//! over the next step with the velocity and angular velocity of that change.
use physics_engine::BodyId;
use physics_engine::approximate::{Body, Config, Error, Quaternion, Shape, Vector as V, World};

const DT: f64 = 1.0 / 60.0;

fn yaw(angle: f32) -> Quaternion {
    let (s, c) = (0.5 * angle).sin_cos();
    Quaternion(0.0, s, 0.0, c)
}

fn world(gravity: bool) -> World {
    let mut w = World::new(Config {
        gravity: if gravity { V(0.0, -9.81, 0.0) } else { V::ZERO },
        substeps: 2,
        velocity_iterations: 4,
        ..Config::default()
    })
    .unwrap();
    if gravity {
        w.add_body(Body::keyed(1, Shape::HalfSpace, V::ZERO, 0.0))
            .unwrap();
    }
    w
}

/// A 2 m blade (half extents 1 × 0.25 × 0.1) whose centre sits 1 m from a pivot at the
/// origin, at yaw 0 along +X.
fn blade(w: &mut World, height: f32) -> BodyId {
    let mut body = Body::keyed(2, Shape::Box(V(1.0, 0.25, 0.1)), V(1.0, height, 0.0), 1.0);
    body.external = true;
    w.add_body(body).unwrap()
}

/// The blade's pose at `angle` about the pivot (yaw: +X turns towards -Z).
fn blade_pose(angle: f32, height: f32) -> (V, Quaternion) {
    let (s, c) = angle.sin_cos();
    (V(c, height, -s), yaw(angle))
}

#[test]
fn invalid_targets_are_rejected() {
    let mut w = world(false);
    let fixed = w
        .add_body(Body::keyed(
            3,
            Shape::Box(V(1.0, 1.0, 1.0)),
            V(5.0, 0.0, 0.0),
            0.0,
        ))
        .unwrap();
    let dynamic = w
        .add_body(Body::keyed(
            4,
            Shape::Box(V(1.0, 1.0, 1.0)),
            V(9.0, 0.0, 0.0),
            1.0,
        ))
        .unwrap();
    let mut massless = Body::keyed(5, Shape::Box(V(1.0, 1.0, 1.0)), V(13.0, 0.0, 0.0), 0.0);
    massless.external = true;
    let massless = w.add_body(massless).unwrap();
    let blade = blade(&mut w, 0.0);
    let q = Quaternion::IDENTITY;
    for id in [fixed, dynamic, massless] {
        assert_eq!(
            w.set_next_kinematic_pose(id, V::ZERO, q),
            Err(Error::InvalidInput)
        );
    }
    for (p, q) in [
        (V(f32::NAN, 0.0, 0.0), q),
        (V(1e12, 0.0, 0.0), q),
        (V::ZERO, Quaternion(0.0, 0.0, 0.0, 0.9)),
        (V::ZERO, Quaternion(f32::NAN, 0.0, 0.0, 1.0)),
    ] {
        assert_eq!(
            w.set_next_kinematic_pose(blade, p, q),
            Err(Error::InvalidInput)
        );
    }
    let mut capsule = Body::keyed(6, Shape::capsule(0.5, 0.3), V(0.0, 5.0, 9.0), 1.0);
    capsule.external = true;
    let character = w.add_body(capsule).unwrap();
    let settings = w.character_settings();
    w.add_character(character, settings).unwrap();
    assert_eq!(
        w.set_next_kinematic_pose(character, V(0.0, 5.0, 9.0), yaw(0.1)),
        Err(Error::InvalidInput)
    );
}

#[test]
fn the_step_ends_exactly_at_the_target_and_at_rest() {
    let mut w = world(false);
    let blade = blade(&mut w, 0.0);
    let (p, q) = blade_pose(0.3, 0.0);
    w.set_next_kinematic_pose(blade, p, q).unwrap();
    // `step(0.0)` keeps the target.
    w.step(0.0).unwrap();
    assert_eq!(w.body(blade).unwrap().position, V(1.0, 0.0, 0.0));
    w.step(DT).unwrap();
    let b = w.body(blade).unwrap();
    assert_eq!((b.position, b.orientation), (p, q));
    assert_eq!((b.velocity, b.angular_velocity), (V::ZERO, V::ZERO));
    // Without a new target the body stays.
    w.step(DT).unwrap();
    let b = w.body(blade).unwrap();
    assert_eq!((b.position, b.orientation), (p, q));
}

#[test]
fn a_removed_body_drops_its_target() {
    let mut w = world(false);
    let blade = blade(&mut w, 0.0);
    let (p, q) = blade_pose(0.3, 0.0);
    w.set_next_kinematic_pose(blade, p, q).unwrap();
    w.remove_body(blade).unwrap();
    w.step(DT).unwrap();
}

/// A turning blade strikes a free crate at its tip: the crate leaves at about the tip's speed
/// along the blade's motion, not just pushed out of overlap.
#[test]
fn a_turning_blade_gives_a_struck_crate_its_point_velocity() {
    let mut w = world(false);
    let blade = blade(&mut w, 0.0);
    // 3 rad/s about +Y; the tip (2 m out) moves at 6 m/s, towards -Z at yaw 0.
    let rate = 3.0;
    let crate_body = w
        .add_body(Body::keyed(
            10,
            Shape::Box(V(0.2, 0.2, 0.2)),
            V(1.7, 0.0, -0.45),
            1.0,
        ))
        .unwrap();
    let mut angle = 0.0;
    let mut speed: f32 = 0.0;
    for _ in 0..12 {
        angle += rate * DT as f32;
        let (p, q) = blade_pose(angle, 0.0);
        w.set_next_kinematic_pose(blade, p, q).unwrap();
        w.step(DT).unwrap();
        speed = speed.max(-w.body(crate_body).unwrap().velocity.2);
    }
    // The contact point at about 1.7 m moves at about 5.1 m/s; a frictionless-enough
    // inelastic hit gives the crate at least that order of speed.
    assert!(speed > 3.0, "crate speed {speed}");
    assert!(speed < 9.0, "crate speed {speed}");
}

#[test]
fn a_sleeping_crate_in_the_path_wakes() {
    let mut w = world(true);
    let blade = blade(&mut w, 0.5);
    let crate_body = w
        .add_body(Body::keyed(
            10,
            Shape::Box(V(0.25, 0.25, 0.25)),
            V(1.5, 0.25, -1.0),
            1.0,
        ))
        .unwrap();
    for _ in 0..120 {
        w.step(DT).unwrap();
    }
    assert!(w.body(crate_body).unwrap().is_sleeping());
    let start = w.body(crate_body).unwrap().position;
    let mut angle = 0.0;
    for _ in 0..60 {
        angle += 2.0 * DT as f32;
        let (p, q) = blade_pose(angle, 0.5);
        w.set_next_kinematic_pose(blade, p, q).unwrap();
        w.step(DT).unwrap();
    }
    let end = w.body(crate_body).unwrap().position;
    assert!(
        (end - start).length() > 0.2,
        "crate did not move: {start:?} → {end:?}"
    );
}

/// A fast thin blade spinning about its own centre sweeps past a small sphere within one
/// substep: the sphere is struck, not passed. A sweep that held the blade's orientation would
/// see no motion at all.
#[test]
fn a_fast_turning_blade_does_not_pass_through_a_small_body() {
    for ccd in [false, true] {
        let mut w = world(false);
        let mut body = Body::keyed(2, Shape::Box(V(1.0, 0.25, 0.02)), V::ZERO, 1.0);
        body.external = true;
        let blade = w.add_body(body).unwrap();
        let mut ball = Body::keyed(10, Shape::Sphere(0.05), V(0.9, 0.0, -0.3), 1.0);
        ball.ccd = ccd;
        let ball = w.add_body(ball).unwrap();
        // 60 rad/s: the blade passes the ball (0.23 m away, 0.9 m out) within the first
        // substep, sweeping 0.45 m there.
        w.set_next_kinematic_pose(blade, V::ZERO, yaw(1.0)).unwrap();
        w.step(DT).unwrap();
        let b = w.body(ball).unwrap();
        assert!(b.velocity.2 < -10.0, "ccd {ccd}: ball not struck: {b:?}");
    }
}

#[cfg(feature = "checkpoints")]
#[test]
fn a_pending_target_is_not_checkpointed() {
    use physics_engine::approximate::{CheckpointContext, CheckpointError};
    let mut w = world(false);
    let blade = blade(&mut w, 0.0);
    let (p, q) = blade_pose(0.3, 0.0);
    w.set_next_kinematic_pose(blade, p, q).unwrap();
    let context = CheckpointContext {
        build: [1; 32],
        content: [2; 32],
    };
    assert_eq!(
        w.checkpoint(context).err(),
        Some(CheckpointError::PendingKinematicPose)
    );
    w.step(DT).unwrap();
    let restored = w.checkpoint(context).unwrap().restore(&[]).unwrap();
    let b = restored.body(blade).unwrap();
    assert_eq!((b.position, b.orientation), (p, q));
}
