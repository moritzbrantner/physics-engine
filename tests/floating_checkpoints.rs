use physics_engine::{
    BodyId, CollisionLayers3d,
    approximate::{
        Body, Checkpoint, CheckpointContext, CheckpointError, CheckpointLimits, Config,
        ConvergenceScope, PositionCorrection, Quaternion, Report, Shape, Vector as V, World,
    },
};
use physics_engine::{approximate::Real, numeric::Scalar};
use sha2::{Digest, Sha256};

const CONTEXT: CheckpointContext = CheckpointContext {
    build: [17; 32],
    content: [29; 32],
};

fn bytes(world: &World) -> Vec<u8> {
    world.checkpoint(CONTEXT).unwrap().to_bytes()
}
fn restore(bytes: &[u8]) -> World {
    Checkpoint::from_bytes(bytes, CONTEXT, CheckpointLimits::default())
        .unwrap()
        .restore()
}
fn same(actual: &World, expected: &World) {
    assert_eq!(
        bytes(actual),
        bytes(expected),
        "exact physical continuation/history bits"
    );
    assert_eq!(
        format!("{:?}", actual.bodies().collect::<Vec<_>>()),
        format!("{:?}", expected.bodies().collect::<Vec<_>>()),
        "including rebuilt bounds"
    );
    assert_eq!(actual.is_quiescent(), expected.is_quiescent());
}
fn semantic(report: &Report) -> String {
    format!(
        "{:?}",
        (
            report.substeps,
            report.contact_points,
            report.impulse_iterations,
            report.integrated_bodies,
            report.woken_bodies,
            report.swept_contacts,
            &report.retired,
            report.max_penetration.to_bits(),
            report.position.corrections
        )
    )
}
fn fixture(config: Config) -> World {
    let mut world = World::new(config).unwrap();
    world
        .add_body(Body::new(
            BodyId(0),
            Shape::Box(V(100.0, 0.1, 100.0)),
            V(0.0, -0.1, 0.0),
            0.0,
        ))
        .unwrap();
    for id in 1..=6 {
        let mut b = Body::new(
            BodyId(id),
            Shape::Box(V(0.5, 0.5, 0.5)),
            V(id as Real * 3.0, 0.5, 0.0),
            1.0,
        );
        b.rotation_locked = true;
        world.add_body(b).unwrap();
    }
    for (id, shape, x) in [
        (20, Shape::Sphere(0.5), -20.0),
        (21, Shape::capsule(0.3, 0.25), -10.0),
        (22, Shape::Wedge(V(0.5, 0.5, 0.5)), -30.0),
    ] {
        let mut b = Body::new(BodyId(id), shape, V(x, 1.0, 0.0), 0.75);
        b.rotation_locked = id == 22;
        b.friction = 0.43;
        b.restitution = 0.12;
        b.layers = CollisionLayers3d::new(2, 2);
        world.add_body(b).unwrap();
    }
    let mut rotating = Body::new(
        BodyId(30),
        Shape::Box(V(0.4, 0.6, 0.7)),
        V(0.0, 4.0, 10.0),
        1.25,
    );
    rotating.orientation = Quaternion(1.0, 2.0, 3.0, 4.0);
    rotating.angular_velocity = V(0.3, 0.2, -0.1);
    rotating.sleep_allowed = false;
    world.add_body(rotating).unwrap();
    let mut external = Body::new(BodyId(31), Shape::capsule(0.3, 0.3), V(20.0, 0.6, 0.0), 2.5);
    external.external = true;
    external.velocity = V(0.01, 0.0, 0.0);
    external.linear_support = Some(V::Y);
    external.rotation_locked = true;
    world.add_body(external).unwrap();
    let mut sensor = Body::new(BodyId(32), Shape::Sphere(0.7), V(3.0, 0.5, 0.0), 0.0);
    sensor.sensor = true;
    world.add_body(sensor).unwrap();
    world
}
fn config() -> Config {
    Config {
        gravity: V(0.0, -10.0, 0.0),
        substeps: 2,
        fixed_position_iterations: 2,
        position_correction: PositionCorrection::AdmittedContacts,
        sleep_speed: 0.1,
        sleep_seconds: 0.08,
        ..Config::default()
    }
}

#[test]
fn destroyed_world_restores_pending_input_contacts_and_exact_future_state() {
    for (warm_start, convergence, scope) in [
        (
            true,
            Config::default().convergence,
            ConvergenceScope::ContactIslands,
        ),
        (
            true,
            Config::default().convergence,
            ConvergenceScope::WholeWorld,
        ),
        (false, None, ConvergenceScope::ContactIslands),
    ] {
        let make = || {
            fixture(Config {
                warm_start,
                convergence,
                convergence_scope: scope,
                ..config()
            })
        };
        let mut uninterrupted = make();
        let mut captured = make();
        for w in [&mut uninterrupted, &mut captured] {
            for _ in 0..30 {
                w.step(0.01).unwrap();
            }
            assert!(w.has_support(BodyId(1)));
            w.add_force(BodyId(1), V(0.2, 0.1, 0.0)).unwrap();
            let position = w.body(BodyId(30)).unwrap().position;
            w.apply_impulse(BodyId(30), V(0.3, 0.1, -0.2), position + V(0.3, 0.0, 0.0))
                .unwrap();
        }
        let saved = bytes(&captured);
        drop(captured);
        let mut continued = restore(&saved);
        same(&continued, &uninterrupted);
        assert!(continued.last_report.retired.is_empty());
        assert_eq!(continued.last_report.substeps, 0);
        assert_eq!(continued.last_step_transaction().vector_capacity_bytes, 0);
        for tick in 0..80 {
            for w in [&mut uninterrupted, &mut continued] {
                if tick == 10 {
                    w.remove_body(BodyId(0)).unwrap();
                }
                if tick == 15 {
                    w.remove_body(BodyId(3)).unwrap();
                    w.add_body(Body::new(
                        BodyId(3),
                        Shape::Sphere(0.37),
                        V(-4.0, 3.0, 0.0),
                        0.8,
                    ))
                    .unwrap();
                }
                if tick == 20 {
                    w.add_force(BodyId(3), V(0.0, 0.2, 0.1)).unwrap();
                }
            }
            if tick == 32 {
                let saved = bytes(&continued);
                drop(continued);
                continued = restore(&saved);
            }
            let dt = [0.01, 0.02, 0.005, 0.0][tick % 4];
            let expected = uninterrupted.step(dt as Scalar).unwrap();
            let actual = continued.step(dt as Scalar).unwrap();
            assert_eq!(semantic(&actual), semantic(&expected), "tick {tick}");
            same(&continued, &uninterrupted);
        }
    }
}

#[test]
fn partial_quiet_timers_sleepers_and_support_removal_continue_identically() {
    let make = || {
        let mut w = World::new(config()).unwrap();
        w.add_body(Body::new(
            BodyId(0),
            Shape::Box(V(10.0, 0.1, 10.0)),
            V(0.0, -0.1, 0.0),
            0.0,
        ))
        .unwrap();
        w.add_body(Body::new(
            BodyId(1),
            Shape::Box(V(0.5, 0.5, 0.5)),
            V(0.0, 0.5, 0.0),
            1.0,
        ))
        .unwrap();
        w
    };
    let mut expected = make();
    let mut captured = make();
    for w in [&mut expected, &mut captured] {
        for _ in 0..3 {
            w.step(0.01).unwrap();
        }
    }
    assert!(!captured.body(BodyId(1)).unwrap().is_sleeping());
    let saved = bytes(&captured);
    drop(captured);
    let mut actual = restore(&saved);
    for _ in 0..12 {
        let e = expected.step(0.01).unwrap();
        let a = actual.step(0.01).unwrap();
        assert_eq!(semantic(&a), semantic(&e));
        same(&actual, &expected);
    }
    assert!(actual.is_quiescent());
    let saved = bytes(&actual);
    drop(actual);
    let mut actual = restore(&saved);
    assert!(actual.is_quiescent());
    assert!(actual.has_support(BodyId(1)));
    for w in [&mut actual, &mut expected] {
        w.remove_body(BodyId(0)).unwrap();
    }
    assert!(!actual.is_quiescent());
    for _ in 0..20 {
        actual.step(0.02).unwrap();
        expected.step(0.02).unwrap();
        same(&actual, &expected);
    }
}

#[test]
fn failed_steps_retirement_and_repeated_restores_keep_the_same_boundary() {
    let make = || {
        let mut w = World::new(Config {
            gravity: V::ZERO,
            substeps: 2,
            ..Config::default()
        })
        .unwrap();
        w.add_body(Body::new(
            BodyId(0),
            Shape::Box(V(0.1, 2.0, 2.0)),
            V::ZERO,
            0.0,
        ))
        .unwrap();
        let mut shot = Body::new(BodyId(1), Shape::Sphere(0.1), V(-1.0, 0.0, 0.0), 0.3);
        shot.ccd = true;
        shot.retire_on_impact = true;
        shot.velocity = V(40.0, 0.0, 0.0);
        w.add_body(shot).unwrap();
        let mut bad = Body::new(
            BodyId(99),
            Shape::Sphere(0.1),
            V(1e12 - 0.075, 0.0, 0.0),
            1.0,
        );
        bad.velocity = V(1.0, 0.0, 0.0);
        w.add_body(bad).unwrap();
        w.add_force(BodyId(99), V(0.1, 0.0, 0.0)).unwrap();
        w
    };
    let mut expected = make();
    let captured = make();
    let saved = bytes(&captured);
    drop(captured);
    let mut actual = restore(&saved);
    for _ in 0..3 {
        assert_eq!(
            actual.step(0.1).unwrap_err(),
            expected.step(0.1).unwrap_err()
        );
        assert!(actual.last_failed_step_work().unwrap().retirement_attempts > 0);
        assert_eq!(bytes(&actual), saved);
        same(&actual, &expected);
        let boundary = bytes(&actual);
        drop(actual);
        actual = restore(&boundary);
    }
    for w in [&mut expected, &mut actual] {
        w.remove_body(BodyId(99)).unwrap();
    }
    let a = actual.step(0.1).unwrap();
    let e = expected.step(0.1).unwrap();
    assert_eq!(a.retired, vec![BodyId(1)]);
    assert_eq!(semantic(&a), semantic(&e));
    same(&actual, &expected);
    let saved = bytes(&actual);
    for _ in 0..3 {
        let mut again = restore(&saved);
        assert!(again.last_report.retired.is_empty());
        again.step(0.02).unwrap();
    }
}

#[cfg(feature = "experimental-soft-contact")]
#[test]
fn experimental_policy_and_warm_history_are_preserved_explicitly() {
    let c = Config {
        soft_contact: Some(physics_engine::approximate::SoftContact::default()),
        ..config()
    };
    let mut expected = fixture(c);
    let mut captured = fixture(c);
    for w in [&mut expected, &mut captured] {
        for _ in 0..8 {
            w.step(0.01).unwrap();
        }
    }
    let saved = bytes(&captured);
    drop(captured);
    let mut actual = restore(&saved);
    for tick in 0..40 {
        let dt = if tick % 2 == 0 { 0.01 } else { 0.02 };
        let a = actual.step(dt as Scalar).unwrap();
        let e = expected.step(dt as Scalar).unwrap();
        assert_eq!(a.correction.softened_points, e.correction.softened_points);
        assert_eq!(semantic(&a), semantic(&e));
        same(&actual, &expected);
    }
}

fn rehash(bytes: &mut [u8]) {
    let len = bytes.len() - 32;
    let digest = Sha256::digest(&bytes[..len]);
    bytes[len..].copy_from_slice(&digest);
}
fn simple() -> World {
    let mut w = World::new(Config {
        gravity: V::ZERO,
        convergence: None,
        ..Config::default()
    })
    .unwrap();
    for id in 1..=2 {
        w.add_body(Body::new(
            BodyId(id),
            Shape::Sphere(0.5),
            V(id as Real * 3.0, 0.0, 0.0),
            1.0,
        ))
        .unwrap();
    }
    w
}
// Offsets follow the documented v1 layout, not the implementation's Reader.
fn config_start() -> usize {
    80 + 1 + std::env::consts::ARCH.len() + 1 + std::env::consts::OS.len()
}
fn body_start() -> usize {
    config_start() + 55 + 16 + 8
}

#[test]
fn damaged_truncated_unknown_and_incompatible_bytes_reject_without_touching_a_world() {
    let mut live = simple();
    let saved = bytes(&live);
    for length in 0..saved.len() {
        assert!(
            Checkpoint::from_bytes(&saved[..length], CONTEXT, CheckpointLimits::default()).is_err()
        );
    }
    for index in 0..saved.len() {
        let mut damaged = saved.clone();
        damaged[index] ^= 1;
        assert!(
            Checkpoint::from_bytes(&damaged, CONTEXT, CheckpointLimits::default()).is_err(),
            "bit flip {index}"
        );
    }
    for (offset, value, error) in [
        (8, 3u32, CheckpointError::UnsupportedFormat(3)),
        (12, 8u32, CheckpointError::UnsupportedAlgorithm(8)),
        (12, 6u32, CheckpointError::UnsupportedAlgorithm(6)),
        (12, 5u32, CheckpointError::UnsupportedAlgorithm(5)),
        (12, 4u32, CheckpointError::UnsupportedAlgorithm(4)),
        (12, 3u32, CheckpointError::UnsupportedAlgorithm(3)),
        (12, 2u32, CheckpointError::UnsupportedAlgorithm(2)),
        (8, 1u32, CheckpointError::UnsupportedFormat(1)),
        (12, 1u32, CheckpointError::UnsupportedAlgorithm(1)),
    ] {
        let mut altered = saved.clone();
        altered[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        rehash(&mut altered);
        assert_eq!(
            Checkpoint::from_bytes(&altered, CONTEXT, CheckpointLimits::default()).unwrap_err(),
            error
        );
    }
    for offset in [16, 48] {
        let mut altered = saved.clone();
        altered[offset] ^= 1;
        rehash(&mut altered);
        assert_eq!(
            Checkpoint::from_bytes(&altered, CONTEXT, CheckpointLimits::default()).unwrap_err(),
            CheckpointError::IncompatibleContext
        );
    }
    let mut altered = saved.clone();
    altered[81] ^= 1;
    rehash(&mut altered);
    assert_eq!(
        Checkpoint::from_bytes(&altered, CONTEXT, CheckpointLimits::default()).unwrap_err(),
        CheckpointError::IncompatibleTarget
    );
    let mut altered = saved.clone();
    altered.insert(altered.len() - 32, 0);
    rehash(&mut altered);
    assert_eq!(
        Checkpoint::from_bytes(&altered, CONTEXT, CheckpointLimits::default()).unwrap_err(),
        CheckpointError::InvalidData
    );
    assert_eq!(bytes(&live), saved);
    live.step(0.02).unwrap();
}

#[test]
fn bounded_parser_rejects_semantically_invalid_authoritative_data() {
    let saved = bytes(&simple());
    let b = body_start();
    let c = config_start();
    for (offset, invalid) in [
        (b + 8, Real::NAN),
        (b + 32, 1e12),
        (b + 56, Real::INFINITY),
        (b + 56, Real::MAX),
        (b + 80, 2.1),
        (b + 121, -1.0),
        (b + 129, 11.0),
        (b + 137, 1.1),
        (b + 156, -0.1),
        (b + 164, Real::NAN),
        (c + 27, -0.1),
    ] {
        let mut bad = saved.clone();
        bad[offset..offset + 8].copy_from_slice(&invalid.to_bits().to_le_bytes());
        rehash(&mut bad);
        assert_eq!(
            Checkpoint::from_bytes(&bad, CONTEXT, CheckpointLimits::default()).unwrap_err(),
            CheckpointError::InvalidData,
            "offset {offset}"
        );
    }
    for (offset, value) in [
        (b + 112, 99),
        (b + 146, 128),
        (b + 155, 2),
        (c + 52, 2),
        (c + 53, 3),
    ] {
        let mut bad = saved.clone();
        bad[offset] = value;
        rehash(&mut bad);
        assert_eq!(
            Checkpoint::from_bytes(&bad, CONTEXT, CheckpointLimits::default()).unwrap_err(),
            CheckpointError::InvalidData
        );
    }
    let mut duplicate = saved.clone();
    duplicate[b + 260..b + 268].copy_from_slice(&1u64.to_le_bytes());
    rehash(&mut duplicate);
    assert_eq!(
        Checkpoint::from_bytes(&duplicate, CONTEXT, CheckpointLimits::default()).unwrap_err(),
        CheckpointError::InvalidData
    );
    let mut huge = saved.clone();
    huge[b - 8..b].copy_from_slice(&u64::MAX.to_le_bytes());
    rehash(&mut huge);
    assert_eq!(
        Checkpoint::from_bytes(&huge, CONTEXT, CheckpointLimits::default()).unwrap_err(),
        CheckpointError::ResourceLimit
    );
    for limits in [
        CheckpointLimits {
            bytes: saved.len() - 1,
            ..CheckpointLimits::default()
        },
        CheckpointLimits {
            bodies: 1,
            ..CheckpointLimits::default()
        },
    ] {
        assert_eq!(
            Checkpoint::from_bytes(&saved, CONTEXT, limits).unwrap_err(),
            CheckpointError::ResourceLimit
        );
    }
}

#[test]
fn warm_start_endpoints_points_and_resource_budgets_are_validated() {
    let mut world = World::new(Config {
        convergence: None,
        ..config()
    })
    .unwrap();
    world
        .add_body(Body::new(
            BodyId(0),
            Shape::Box(V(10.0, 0.1, 10.0)),
            V(0.0, -0.1, 0.0),
            0.0,
        ))
        .unwrap();
    world
        .add_body(Body::new(
            BodyId(1),
            Shape::Box(V(0.5, 0.5, 0.5)),
            V(0.0, 0.5, 0.0),
            1.0,
        ))
        .unwrap();
    world.step(0.01).unwrap();
    let checkpoint = world.checkpoint(CONTEXT).unwrap();
    assert_eq!(checkpoint.stats().pairs, 1);
    assert!(checkpoint.stats().contact_points > 0);
    let saved = checkpoint.to_bytes();
    let pair = body_start() + 2 * 276 + 8;
    for (offset, value) in [
        (pair + 72, Real::NAN),
        (pair + 72, 65.0),
        (pair + 96, -0.1),
        (pair + 104, Real::INFINITY),
    ] {
        let mut bad = saved.clone();
        bad[offset..offset + 8].copy_from_slice(&value.to_bits().to_le_bytes());
        rehash(&mut bad);
        assert_eq!(
            Checkpoint::from_bytes(&bad, CONTEXT, CheckpointLimits::default()).unwrap_err(),
            CheckpointError::InvalidData
        );
    }
    for (offset, value) in [(pair + 8, 999u64), (pair + 16, 0), (pair + 16, u64::MAX)] {
        let mut bad = saved.clone();
        bad[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
        rehash(&mut bad);
        assert!(Checkpoint::from_bytes(&bad, CONTEXT, CheckpointLimits::default()).is_err());
    }
    for limits in [
        CheckpointLimits {
            pairs: 0,
            ..CheckpointLimits::default()
        },
        CheckpointLimits {
            contact_points: 0,
            ..CheckpointLimits::default()
        },
    ] {
        assert_eq!(
            Checkpoint::from_bytes(&saved, CONTEXT, limits).unwrap_err(),
            CheckpointError::ResourceLimit
        );
    }
    same(&checkpoint.restore(), &world);
}

#[cfg(not(feature = "experimental-soft-contact"))]
#[test]
fn diagnostic_soft_policy_is_rejected_by_the_ordinary_build() {
    let mut saved = bytes(&simple());
    let c = config_start();
    saved[c + 54] = 1;
    let mut policy = Vec::new();
    policy.extend_from_slice(&60f64.to_le_bytes());
    policy.extend_from_slice(&1f64.to_le_bytes());
    policy.push(2);
    saved.splice(c + 55..c + 55, policy);
    rehash(&mut saved);
    assert_eq!(
        Checkpoint::from_bytes(&saved, CONTEXT, CheckpointLimits::default()).unwrap_err(),
        CheckpointError::UnsupportedPolicy
    );
}

#[cfg(all(target_arch = "x86_64", target_os = "linux"))]
#[test]
fn format_two_algorithm_seven_empty_world_wire_fixture_is_stable() {
    let world = World::new(Config {
        gravity: V::ZERO,
        ..Config::default()
    })
    .unwrap();
    let encoded = bytes(&world);
    assert_eq!(
        format!("{:x}", Sha256::digest(&encoded)),
        "c924619923ef7c9791f45ade050711ea0854d5dc6a065abc47c22fafd4e2cc81"
    );
}

#[test]
fn capture_rejects_nonfinite_state_and_preserves_finite_inputs_that_fail_on_step() {
    let mut world = simple();
    world
        .add_force(BodyId(1), V(Real::MAX / 2.0, 0.0, 0.0))
        .unwrap();
    let saved = bytes(&world);
    let mut restored = restore(&saved);
    assert_eq!(
        restored.step(0.1).unwrap_err(),
        world.step(0.1).unwrap_err()
    );
    assert_eq!(bytes(&restored), saved);
    same(&restored, &world);

    let mut zero_orientation = simple();
    let mut legacy = Body::new(BodyId(u64::MAX), Shape::Sphere(0.5), V(0.0, 0.0, 10.0), 1.0);
    legacy.orientation = Quaternion(Real::MAX, 0.0, 0.0, 0.0);
    zero_orientation.add_body(legacy).unwrap();
    let saved = bytes(&zero_orientation);
    let mut restored = restore(&saved);
    same(&restored, &zero_orientation);
    assert_eq!(
        restored.step(0.01).unwrap_err(),
        zero_orientation.step(0.01).unwrap_err()
    );
    assert_eq!(bytes(&restored), saved);

    let mut tiny_orientation = simple();
    let mut tiny = Body::new(
        BodyId(u64::MAX),
        Shape::Box(V(0.5, 0.5, 0.5)),
        V(0.0, 0.0, 10.0),
        1.0,
    );
    tiny.orientation = Quaternion(1e-160, 0.0, 0.0, 1e-160);
    tiny.rotation_locked = true;
    tiny_orientation.add_body(tiny).unwrap();
    let saved = bytes(&tiny_orientation);
    let mut restored = restore(&saved);
    same(&restored, &tiny_orientation);
    for _ in 0..16 {
        restored.step(0.01).unwrap();
        tiny_orientation.step(0.01).unwrap();
        same(&restored, &tiny_orientation);
    }

    let mut invalid = World::new(config()).unwrap();
    let mut body = Body::new(BodyId(u64::MAX), Shape::Sphere(0.5), V::ZERO, 1.0);
    // This compatibility input is accepted by the existing body construction API;
    // a physical checkpoint must reject it rather than serialize NaN authority.
    body.linear_support = Some(V(Real::NAN, 0.0, 0.0));
    invalid.add_body(body).unwrap();
    assert_eq!(
        invalid.checkpoint(CONTEXT).unwrap_err(),
        CheckpointError::InvalidData
    );
    assert_eq!(invalid.elapsed_seconds(), 0.0);
    assert!(
        invalid
            .body(BodyId(u64::MAX))
            .unwrap()
            .linear_support
            .unwrap()
            .0
            .is_nan()
    );
}
