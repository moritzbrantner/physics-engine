//! Revolute joints (#568, moenarch-games/shooter#73): hinges, servos and torque-limited rotors.
//! Lifecycle and validation, driving on motored wheels (forward, sideways, spinning in place),
//! stalls, servo turrets and a chained two-axis turret, limits, compound chassis, determinism
//! across storage modes, rollback and exact continuation from consumer-set state. The Rapier
//! comparison is the `revolute-robots` oracle scenario.
use physics_engine::BodyId;
use physics_engine::approximate::{
    Body, BodyStorage, CombineRule, CompoundChild, Config, DistanceJoint, Error, JointId, Motor,
    PreparedCompound, Quaternion, RevoluteJoint, Shape, Vector as V, World,
};

const DT: f64 = 1.0 / 60.0;
const WHEEL: f32 = 0.15;
/// The chassis' half extents.
const CHASSIS: V = V(0.4, 0.1, 0.25);

fn world(storage: BodyStorage, warm_start: bool) -> World {
    let mut w = World::new(Config {
        body_storage: storage,
        warm_start,
        ..Config::default()
    })
    .unwrap();
    w.add_body(Body::keyed(1, Shape::HalfSpace, V::ZERO, 0.0))
        .unwrap();
    w
}

struct Car {
    chassis: BodyId,
    wheels: [BodyId; 4],
    joints: [JointId; 4],
}

/// A 20 kg chassis on four 2 kg ball wheels at its corners. `sideways` mounts the wheels turned
/// 90 degrees round (spinning about X, so they drive along Z).
fn car(w: &mut World, key: u64, at: V, sideways: bool) -> Car {
    let mut body = Body::keyed(
        key,
        Shape::Box(CHASSIS),
        at + V(0.0, WHEEL + 0.05, 0.0),
        20.0,
    );
    body.sleep_allowed = false;
    let chassis = w.add_body(body).unwrap();
    let axis = if sideways { V::X } else { V::Z };
    let mut wheels = [chassis; 4];
    let mut joints = [JointId::from_bits(0); 4];
    for (k, (x, z)) in [(-0.3, -0.35), (-0.3, 0.35), (0.3, -0.35), (0.3, 0.35)]
        .into_iter()
        .enumerate()
    {
        let mount = V(x, -0.05, z);
        let mut wheel = Body::keyed(
            key + 1 + k as u64,
            Shape::Sphere(WHEEL),
            at + V(x, WHEEL, z),
            2.0,
        );
        wheel.friction = 1.0;
        wheel.sleep_allowed = false;
        wheels[k] = w.add_body(wheel).unwrap();
        joints[k] = w
            .add_revolute_joint(
                RevoluteJoint::new(chassis, wheels[k], axis).anchors(mount, V::ZERO),
            )
            .unwrap();
    }
    Car {
        chassis,
        wheels,
        joints,
    }
}

fn drive(w: &mut World, car: &Car, rates: [f32; 4], max_torque: f32) {
    for (j, rate) in car.joints.iter().zip(rates) {
        w.set_revolute_motor(*j, Motor::Velocity { rate, max_torque })
            .unwrap();
    }
}

#[test]
fn a_car_drives_forward_on_its_motored_wheels() {
    let mut w = world(BodyStorage::Ordered, true);
    let c = car(&mut w, 10, V::ZERO, false);
    // Rolling along +X turns a wheel about −Z: 10 rad/s is 1.5 m/s.
    drive(&mut w, &c, [-10.0; 4], 20.0);
    for _ in 0..120 {
        w.step(DT).unwrap();
    }
    let body = w.body(c.chassis).unwrap();
    assert!(
        (body.velocity.0 - 1.5).abs() < 0.1,
        "{:?} {:?}",
        body.velocity,
        body.position
    );
    assert!(body.position.0 > 1.5, "{:?}", body.position);
    assert!(body.position.2.abs() < 0.02 && body.velocity.2.abs() < 0.02);
    // Upright, at its ride height, wheels still on their mounts.
    assert!(body.orientation.rotate(V::Y).1 > 0.99);
    for (k, &wheel) in c.wheels.iter().enumerate() {
        let s = w.revolute_state(c.joints[k]).unwrap();
        assert!((s.rate + 10.0).abs() < 0.2, "{s:?}");
        let joint = w.revolute_joint(c.joints[k]).unwrap();
        let mount = body.position + body.orientation.rotate(joint.anchor_a);
        let gap = (w.body(wheel).unwrap().position - mount).length();
        assert!(gap < 0.01, "wheel {k} {gap}");
    }
}

#[test]
fn sideways_wheels_strafe_and_a_mismatched_drive_spins_in_place() {
    let mut w = world(BodyStorage::Ordered, true);
    let side = car(&mut w, 10, V::ZERO, true);
    let spin = car(&mut w, 20, V(5.0, 0.0, 0.0), false);
    // Turning about −X, a wheel rolls along +Z: the same command drives the car sideways.
    drive(&mut w, &side, [-10.0; 4], 20.0);
    // Left wheels (z < 0) forward, right wheels backward: the chassis turns about Y.
    drive(&mut w, &spin, [-10.0, 10.0, -10.0, 10.0], 20.0);
    for _ in 0..120 {
        w.step(DT).unwrap();
    }
    let s = w.body(side.chassis).unwrap();
    assert!(
        (s.velocity.2 + 1.5).abs() < 0.1 && s.velocity.0.abs() < 0.05,
        "{s:?}"
    );
    let t = w.body(spin.chassis).unwrap();
    assert!(
        (t.position - V(5.0, t.position.1, 0.0)).length() < 0.1,
        "{t:?}"
    );
    assert!(t.angular_velocity.1.abs() > 1.0, "{t:?}");
}

#[test]
fn a_torque_limited_rotor_stalls_against_a_wall() {
    let mut w = world(BodyStorage::Ordered, true);
    // A frictionless wall: front wheels spinning against a rough one would climb it.
    let mut wall = Body::keyed(2, Shape::Box(V(0.2, 1.0, 3.0)), V(1.2, 1.0, 0.0), 0.0);
    wall.friction = 0.0;
    wall.friction_combine = CombineRule::Min;
    w.add_body(wall).unwrap();
    let c = car(&mut w, 10, V::ZERO, false);
    drive(&mut w, &c, [-10.0; 4], 4.0);
    for _ in 0..180 {
        w.step(DT).unwrap();
    }
    let body = w.body(c.chassis).unwrap();
    // Stopped at the wall, wheels stalled at their torque limit.
    assert!(
        body.position.0 < 0.65 && body.velocity.0.abs() < 0.05,
        "{body:?}"
    );
    for &j in &c.joints {
        let s = w.revolute_state(j).unwrap();
        assert!(s.saturated, "{s:?}");
        assert!((s.motor_torque.abs() - 4.0).abs() < 0.05, "{s:?}");
        assert!(s.rate.abs() < 3.0, "{s:?}");
    }
}

#[test]
fn a_servo_turret_turns_to_its_angle_and_holds_within_limits() {
    let mut w = world(BodyStorage::Ordered, true);
    let mut base = Body::keyed(10, Shape::Box(V(0.4, 0.2, 0.4)), V(0.0, 0.2, 0.0), 40.0);
    base.sleep_allowed = false;
    let base = w.add_body(base).unwrap();
    // A gun offset from the yaw axis: 0.6 m long along X.
    let mut gun = Body::keyed(11, Shape::Box(V(0.3, 0.05, 0.05)), V(0.25, 0.5, 0.0), 3.0);
    gun.sleep_allowed = false;
    let gun = w.add_body(gun).unwrap();
    let j = w
        .add_revolute_joint(
            RevoluteJoint::new(base, gun, V::Y)
                .anchors(V(0.0, 0.3, 0.0), V(-0.25, 0.0, 0.0))
                .limits(-1.0, 1.0)
                .motor(Motor::Position {
                    angle: 0.8,
                    max_rate: 2.0,
                    max_torque: 50.0,
                }),
        )
        .unwrap();
    for _ in 0..60 {
        w.step(DT).unwrap();
    }
    let s = w.revolute_state(j).unwrap();
    assert!((s.angle - 0.8).abs() < 0.01 && s.rate.abs() < 0.05, "{s:?}");
    // Commanded past the limit: it stops at the limit.
    w.set_revolute_motor(
        j,
        Motor::Position {
            angle: 2.5,
            max_rate: 2.0,
            max_torque: 50.0,
        },
    )
    .unwrap();
    for _ in 0..60 {
        w.step(DT).unwrap();
        assert!(w.revolute_state(j).unwrap().angle < 1.02);
    }
    let s = w.revolute_state(j).unwrap();
    assert!((s.angle - 1.0).abs() < 0.01, "{s:?}");
    // The gun turned about the base's +Y: its far end is towards −Z (right-handed).
    let g = w.body(gun).unwrap();
    assert!(g.position.2 < -0.1, "{g:?}");
}

#[test]
fn a_chained_two_axis_turret_aims_in_yaw_and_pitch() {
    let mut w = world(BodyStorage::Ordered, true);
    let mut base = Body::keyed(10, Shape::Box(V(0.4, 0.2, 0.4)), V(0.0, 0.2, 0.0), 40.0);
    base.sleep_allowed = false;
    let base = w.add_body(base).unwrap();
    let mut ring = Body::keyed(11, Shape::Box(V(0.15, 0.05, 0.15)), V(0.0, 0.45, 0.0), 4.0);
    ring.sleep_allowed = false;
    let ring = w.add_body(ring).unwrap();
    let mut barrel = Body::keyed(12, Shape::Box(V(0.3, 0.04, 0.04)), V(0.3, 0.6, 0.0), 2.0);
    barrel.sleep_allowed = false;
    let barrel = w.add_body(barrel).unwrap();
    let servo = |angle| Motor::Position {
        angle,
        max_rate: 3.0,
        max_torque: 80.0,
    };
    let yaw = w
        .add_revolute_joint(
            RevoluteJoint::new(base, ring, V::Y)
                .anchors(V(0.0, 0.25, 0.0), V(0.0, 0.0, 0.0))
                .motor(servo(1.2)),
        )
        .unwrap();
    // Pitch about the ring's Z: positive raises the barrel's far end.
    let pitch = w
        .add_revolute_joint(
            RevoluteJoint::new(ring, barrel, V::Z)
                .anchors(V(0.0, 0.15, 0.0), V(-0.3, 0.0, 0.0))
                .limits(-0.2, 0.7)
                .motor(servo(0.5)),
        )
        .unwrap();
    for _ in 0..90 {
        w.step(DT).unwrap();
    }
    let (y, p) = (
        w.revolute_state(yaw).unwrap(),
        w.revolute_state(pitch).unwrap(),
    );
    assert!((y.angle - 1.2).abs() < 0.01, "{y:?}");
    assert!((p.angle - 0.5).abs() < 0.01, "{p:?}");
    // The barrel points along yaw 1.2 about +Y, raised 0.5.
    let dir = w.body(barrel).unwrap().orientation.rotate(V::X);
    let expected = V(
        1.2f32.cos() * 0.5f32.cos(),
        0.5f32.sin(),
        -1.2f32.sin() * 0.5f32.cos(),
    );
    assert!((dir - expected).length() < 0.02, "{dir:?} {expected:?}");
}

#[test]
fn add_validates_and_ids_are_shared_with_distance_joints() {
    let mut w = world(BodyStorage::Ordered, true);
    let a = w
        .add_body(Body::keyed(10, Shape::Sphere(0.2), V(0.0, 1.0, 0.0), 1.0))
        .unwrap();
    let b = w
        .add_body(Body::keyed(11, Shape::Sphere(0.2), V(1.0, 1.0, 0.0), 1.0))
        .unwrap();
    let fixed = w
        .add_body(Body::keyed(12, Shape::Sphere(0.2), V(3.0, 1.0, 0.0), 0.0))
        .unwrap();
    let fixed2 = w
        .add_body(Body::keyed(13, Shape::Sphere(0.2), V(4.0, 1.0, 0.0), 0.0))
        .unwrap();
    let hinge = RevoluteJoint::new(a, b, V::Z);
    for bad in [
        RevoluteJoint { b: a, ..hinge },
        RevoluteJoint {
            axis_a: V::ZERO,
            ..hinge
        },
        RevoluteJoint {
            reference_b: V::Z,
            ..hinge
        },
        RevoluteJoint {
            anchor_a: V(f32::NAN, 0.0, 0.0),
            ..hinge
        },
        hinge.limits(0.5, 0.1),
        hinge.limits(-4.0, 0.0),
        hinge.motor(Motor::Velocity {
            rate: 1.0,
            max_torque: -1.0,
        }),
        RevoluteJoint::new(fixed, fixed2, V::Z),
    ] {
        assert_eq!(w.add_revolute_joint(bad), Err(Error::InvalidInput));
    }
    let rope = w.add_joint(DistanceJoint::rope(a, b, 2.0)).unwrap();
    let id = w
        .add_revolute_joint(RevoluteJoint {
            axis_a: V(0.0, 0.0, 2.0),
            reference_a: V(1.0, 0.0, 1.0),
            ..hinge
        })
        .unwrap();
    assert_ne!(id, rope);
    let stored = *w.revolute_joint(id).unwrap();
    assert_eq!(stored.axis_a, V::Z);
    assert_eq!(stored.reference_a, V::X);
    assert!(w.joint(id).is_none() && w.revolute_joint(rope).is_none());
    assert_eq!(w.revolute_joints().count(), 1);
    assert_eq!(
        w.set_revolute_limits(id, Some([1.0, 0.0])),
        Err(Error::InvalidInput)
    );
    w.set_revolute_limits(id, Some([-0.5, 0.5])).unwrap();
    assert_eq!(w.revolute_joint(id).unwrap().limits, Some([-0.5, 0.5]));
    // Removing a body removes its revolute joints; ids are never reused.
    w.remove_body(b).unwrap();
    assert!(w.revolute_joint(id).is_none());
    assert!(w.remove_revolute_joint(id).is_none());
    let c = w
        .add_body(Body::keyed(11, Shape::Sphere(0.2), V(1.0, 1.0, 0.0), 1.0))
        .unwrap();
    let again = w
        .add_revolute_joint(RevoluteJoint::new(a, c, V::Z))
        .unwrap();
    assert!(again != id && again != rope);
    assert!(w.remove_revolute_joint(again).is_some());
    assert_eq!(w.revolute_joints().count(), 0);
}

#[test]
fn perpendicular_matches_rapiers_revolute_frame() {
    assert_eq!(RevoluteJoint::perpendicular(V::X), V::Y);
    let p = RevoluteJoint::perpendicular(V::Y);
    assert!((p - V(-1.0, 0.0, 0.0)).length() < 1e-6, "{p:?}");
    let p = RevoluteJoint::perpendicular(V::Z);
    assert!((p - V::Y).length() < 1e-6, "{p:?}");
}

#[test]
fn jointed_bodies_sleep_together_and_a_motor_command_wakes_them() {
    let mut w = world(BodyStorage::Ordered, true);
    let mut base = Body::keyed(10, Shape::Box(V(0.4, 0.2, 0.4)), V(0.0, 0.2, 0.0), 40.0);
    base.sleep_allowed = true;
    let base = w.add_body(base).unwrap();
    let arm = w
        .add_body(Body::keyed(
            11,
            Shape::Box(V(0.3, 0.05, 0.05)),
            V(0.25, 0.5, 0.0),
            3.0,
        ))
        .unwrap();
    let j = w
        .add_revolute_joint(
            RevoluteJoint::new(base, arm, V::Y)
                .anchors(V(0.0, 0.3, 0.0), V(-0.25, 0.0, 0.0))
                .motor(Motor::Position {
                    angle: 0.0,
                    max_rate: 2.0,
                    max_torque: 50.0,
                }),
        )
        .unwrap();
    for _ in 0..120 {
        w.step(DT).unwrap();
    }
    assert!(w.body(base).unwrap().is_sleeping());
    assert!(w.body(arm).unwrap().is_sleeping(), "held by its joint");
    w.step(DT).unwrap();
    assert_eq!(w.revolute_state(j).unwrap().motor_torque, 0.0);
    w.set_revolute_motor(
        j,
        Motor::Position {
            angle: 0.5,
            max_rate: 2.0,
            max_torque: 50.0,
        },
    )
    .unwrap();
    assert!(!w.body(arm).unwrap().is_sleeping());
    for _ in 0..30 {
        w.step(DT).unwrap();
    }
    assert!((w.revolute_state(j).unwrap().angle - 0.5).abs() < 0.01);
}

/// The compound chassis of #567 with two motored wheels on its sides and a caster ball.
fn compound_car(w: &mut World) -> (BodyId, [JointId; 2]) {
    let c = PreparedCompound::new(&[
        CompoundChild::new(Shape::Box(V(0.3, 0.1, 0.2)), V::ZERO, 15.0),
        // A heavy block mounted off-centre at the back.
        CompoundChild::new(Shape::Box(V(0.1, 0.1, 0.1)), V(-0.2, 0.2, 0.0), 10.0),
        CompoundChild::new(Shape::Sphere(0.08), V(0.25, -0.12, 0.0), 1.0),
    ])
    .unwrap();
    let (position, orientation) = c.body_pose(V(0.0, 0.2, 0.0), Quaternion::IDENTITY);
    let mut body = Body::keyed(10, Shape::Sphere(1.0), position, 1.0);
    body.orientation = orientation;
    body.sleep_allowed = false;
    let chassis = w.add_compound(&c, body).unwrap();
    let mut joints = [JointId::from_bits(0); 2];
    for (k, z) in [-0.3f32, 0.3].into_iter().enumerate() {
        let mut wheel = Body::keyed(20 + k as u64, Shape::Sphere(0.2), V(-0.15, 0.2, z), 2.0);
        wheel.friction = 1.0;
        wheel.sleep_allowed = false;
        let wheel = w.add_body(wheel).unwrap();
        joints[k] = w
            .add_revolute_joint(
                RevoluteJoint::new(chassis, wheel, V::Z)
                    .axes(c.body_direction(V::Z), V::Z)
                    .anchors(c.body_point(V(-0.15, 0.0, z)), V::ZERO),
            )
            .unwrap();
    }
    (chassis, joints)
}

#[test]
fn a_compound_chassis_drives_on_its_wheels() {
    let mut w = world(BodyStorage::Ordered, true);
    let (chassis, joints) = compound_car(&mut w);
    for &j in &joints {
        w.set_revolute_motor(
            j,
            Motor::Velocity {
                rate: -5.0,
                max_torque: 20.0,
            },
        )
        .unwrap();
    }
    for _ in 0..120 {
        w.step(DT).unwrap();
    }
    let b = w.body(chassis).unwrap();
    assert!(b.velocity.0 > 0.8, "{b:?}");
    assert!(b.velocity.2.abs() < 0.05, "{b:?}");
}

/// Two cars, a turret, a stalled rotor: every step's state.
fn scene(storage: BodyStorage, warm_start: bool) -> (World, Vec<BodyId>, Vec<JointId>) {
    let mut w = world(storage, warm_start);
    w.add_body(Body::keyed(
        2,
        Shape::Box(V(0.2, 1.0, 3.0)),
        V(3.0, 1.0, 0.0),
        0.0,
    ))
    .unwrap();
    let a = car(&mut w, 10, V::ZERO, false);
    let b = car(&mut w, 20, V(0.0, 0.0, 4.0), true);
    drive(&mut w, &a, [-12.0; 4], 6.0);
    drive(&mut w, &b, [-10.0, 10.0, 5.0, -5.0], 20.0);
    let mut ids = vec![a.chassis, b.chassis];
    ids.extend(a.wheels);
    ids.extend(b.wheels);
    let mut joints = a.joints.to_vec();
    joints.extend(b.joints);
    (w, ids, joints)
}

type State = Vec<(V, Quaternion, V, V)>;
fn state(w: &World, ids: &[BodyId]) -> State {
    ids.iter()
        .map(|&id| {
            let b = w.body(id).unwrap();
            (b.position, b.orientation, b.velocity, b.angular_velocity)
        })
        .collect()
}

#[test]
fn storage_modes_and_insertion_order_give_identical_results() {
    let (mut ordered, ids, joints) = scene(BodyStorage::Ordered, true);
    let (mut indexed, same, _) = scene(BodyStorage::Indexed, true);
    for _ in 0..120 {
        ordered.step(DT).unwrap();
        indexed.step(DT).unwrap();
        assert_eq!(state(&ordered, &ids), state(&indexed, &same));
    }
    let a: Vec<_> = joints
        .iter()
        .map(|&j| ordered.revolute_state(j).unwrap())
        .collect();
    let b: Vec<_> = joints
        .iter()
        .map(|&j| indexed.revolute_state(j).unwrap())
        .collect();
    assert_eq!(a, b);
}

#[test]
fn a_failed_step_restores_the_joint_states() {
    let (mut w, ids, joints) = scene(BodyStorage::Ordered, true);
    for _ in 0..30 {
        w.step(DT).unwrap();
    }
    let mut reference = w.clone();
    let before: Vec<_> = joints.iter().map(|&j| w.revolute_state(j)).collect();
    // A body that leaves the finite range fails the step after the joints were solved (the
    // f32 spacing near 1e12 is 2^16: it moves by many of them).
    const EDGE: f32 = 16_777_216.0;
    let mut runaway = Body::keyed(99, Shape::Sphere(1.0), V(1e12 - EDGE, 0.0, 0.0), 1.0);
    runaway.velocity = V(20.0 * EDGE, 0.0, 0.0);
    let runaway = w.add_body(runaway).unwrap();
    assert!(matches!(w.step(0.1), Err(Error::NonFiniteState(_))));
    assert_eq!(
        joints
            .iter()
            .map(|&j| w.revolute_state(j))
            .collect::<Vec<_>>(),
        before
    );
    w.remove_body(runaway).unwrap();
    for _ in 0..30 {
        reference.step(DT).unwrap();
        w.step(DT).unwrap();
        assert_eq!(state(&w, &ids), state(&reference, &ids));
    }
}

#[test]
fn a_rebuilt_world_continues_bit_identically() {
    // The exact-continuation contract (warm starting off): a world rebuilt from read-back body
    // states, the same joints and the same motor commands steps exactly as the original.
    let (mut w, ids, joints) = scene(BodyStorage::Ordered, false);
    for _ in 0..40 {
        w.step(DT).unwrap();
    }
    let saved: Vec<Body> = ids.iter().map(|&id| w.body(id).unwrap().clone()).collect();
    let defs: Vec<RevoluteJoint> = joints
        .iter()
        .map(|&j| *w.revolute_joint(j).unwrap())
        .collect();
    let mut rebuilt = world(BodyStorage::Indexed, false);
    rebuilt
        .add_body(Body::keyed(
            2,
            Shape::Box(V(0.2, 1.0, 3.0)),
            V(3.0, 1.0, 0.0),
            0.0,
        ))
        .unwrap();
    // Bodies added in reverse: order follows their keys, not insertion.
    let mut again: Vec<BodyId> = saved
        .iter()
        .rev()
        .map(|b| rebuilt.add_body(b.clone()).unwrap())
        .collect();
    again.reverse();
    let map = |id: BodyId| again[ids.iter().position(|&x| x == id).unwrap()];
    let rejoined: Vec<JointId> = defs
        .iter()
        .map(|d| {
            rebuilt
                .add_revolute_joint(RevoluteJoint {
                    a: map(d.a),
                    b: map(d.b),
                    ..*d
                })
                .unwrap()
        })
        .collect();
    for _ in 0..60 {
        w.step(DT).unwrap();
        rebuilt.step(DT).unwrap();
        assert_eq!(state(&rebuilt, &again), state(&w, &ids));
    }
    for (j, r) in joints.iter().zip(&rejoined) {
        assert_eq!(w.revolute_state(*j), rebuilt.revolute_state(*r));
    }
}

#[cfg(feature = "checkpoints")]
#[test]
fn worlds_with_revolute_joints_are_not_checkpointed() {
    use physics_engine::approximate::{CheckpointContext, CheckpointError};
    let context = CheckpointContext {
        build: [1; 32],
        content: [2; 32],
    };
    let mut w = world(BodyStorage::Ordered, false);
    let c = car(&mut w, 10, V::ZERO, false);
    assert_eq!(
        w.checkpoint(context).err(),
        Some(CheckpointError::UnsupportedJoints)
    );
    for j in c.joints {
        w.remove_revolute_joint(j).unwrap();
    }
    assert!(w.checkpoint(context).is_ok());
}
