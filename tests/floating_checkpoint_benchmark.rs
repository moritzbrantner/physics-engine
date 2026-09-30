//! Explicit product work/cost; no capture/encoding is part of ordinary stepping.
use std::time::Instant;

use physics_engine::{
    BodyId,
    approximate::{
        Body, Checkpoint, CheckpointContext, CheckpointLimits, Config, Shape, Vector as V, World,
    },
};

const CONTEXT: CheckpointContext = CheckpointContext {
    build: [17; 32],
    content: [29; 32],
};
fn fixture(kind: &str, count: u64) -> World {
    let supported = kind != "free";
    let mut world = World::new(Config {
        gravity: if supported {
            V(0.0, -10.0, 0.0)
        } else {
            V::ZERO
        },
        substeps: 2,
        sleep_speed: 0.1,
        sleep_seconds: 0.05,
        ..Config::default()
    })
    .unwrap();
    if supported {
        world
            .add_body(Body::new(
                BodyId(0),
                Shape::Box(V(10_000.0, 0.1, 10.0)),
                V(0.0, -0.1, 0.0),
                0.0,
            ))
            .unwrap();
    }
    for id in 1..=count {
        let mut b = Body::new(
            BodyId(id),
            Shape::Box(V(0.5, 0.5, 0.5)),
            V(id as f64 * 3.0, if supported { 0.5 } else { 10.0 }, 0.0),
            1.0,
        );
        if !supported {
            b.velocity = V(0.1, 0.0, 0.0);
            b.sleep_allowed = false;
        }
        world.add_body(b).unwrap();
    }
    if supported {
        for _ in 0..100 {
            world.step(0.01).unwrap();
        }
        assert!(world.is_quiescent());
    }
    world
}

#[test]
#[ignore = "advisory explicit checkpoint product cost and continuation matrix"]
fn explicit_checkpoint_products() {
    for kind in ["free", "quiet", "local"] {
        for count in [128, 512, 2048] {
            for trial in 0..3 {
                let mut world = fixture(kind, count);
                // Independent uninterrupted test control; setup is outside all timers.
                let mut reference = world.clone();
                let (mut capture_ms, mut encode_ms, mut decode_ms, mut restore_ms, mut step_ms) =
                    (0.0, 0.0, 0.0, 0.0, 0.0);
                let (
                    mut body_copies,
                    mut point_copies,
                    mut max_vector_bytes,
                    mut max_encoded_bytes,
                ) = (0, 0, 0, 0);
                for boundary in 0..16 {
                    if kind == "local" {
                        let v = V(if boundary % 2 == 0 { 0.2 } else { -0.2 }, 0.0, 0.0);
                        world.set_velocity(BodyId(1), v).unwrap();
                        reference.set_velocity(BodyId(1), v).unwrap();
                    }
                    let start = Instant::now();
                    let checkpoint = world.checkpoint(CONTEXT).unwrap();
                    capture_ms += start.elapsed().as_secs_f64() * 1000.0;
                    let stats = checkpoint.stats();
                    body_copies += stats.bodies;
                    point_copies += stats.contact_points;
                    max_vector_bytes = max_vector_bytes.max(stats.vector_capacity_bytes);
                    let start = Instant::now();
                    let encoded = checkpoint.to_bytes();
                    encode_ms += start.elapsed().as_secs_f64() * 1000.0;
                    max_encoded_bytes = max_encoded_bytes.max(encoded.len());
                    drop(checkpoint);
                    drop(world);
                    let start = Instant::now();
                    let decoded =
                        Checkpoint::from_bytes(&encoded, CONTEXT, CheckpointLimits::default())
                            .unwrap();
                    decode_ms += start.elapsed().as_secs_f64() * 1000.0;
                    let start = Instant::now();
                    world = decoded.restore();
                    restore_ms += start.elapsed().as_secs_f64() * 1000.0;
                    assert_eq!(
                        world.checkpoint(CONTEXT).unwrap().to_bytes(),
                        reference.checkpoint(CONTEXT).unwrap().to_bytes()
                    );
                    let start = Instant::now();
                    let report = world.step(0.01).unwrap();
                    step_ms += start.elapsed().as_secs_f64() * 1000.0;
                    let expected = reference.step(0.01).unwrap();
                    assert_eq!(
                        (
                            report.substeps,
                            report.contact_points,
                            report.woken_bodies,
                            report.swept_contacts,
                            report.retired
                        ),
                        (
                            expected.substeps,
                            expected.contact_points,
                            expected.woken_bodies,
                            expected.swept_contacts,
                            expected.retired
                        )
                    );
                    assert_eq!(
                        world.checkpoint(CONTEXT).unwrap().to_bytes(),
                        reference.checkpoint(CONTEXT).unwrap().to_bytes()
                    );
                }
                println!(
                    "CHECKPOINT_PRODUCT {{\"kind\":\"{kind}\",\"dynamic_bodies\":{count},\"trial\":{trial},\"boundaries\":16,\"completed_ticks\":16,\"capture_ms\":{capture_ms},\"encode_ms\":{encode_ms},\"decode_ms\":{decode_ms},\"restore_ms\":{restore_ms},\"cold_step_ms\":{step_ms},\"body_copies\":{body_copies},\"point_copies\":{point_copies},\"max_vector_bytes\":{max_vector_bytes},\"max_encoded_bytes\":{max_encoded_bytes}}}"
                );
            }
        }
    }
}
