use super::*;
use crate::approximate::{Config, Error, Shape};
use std::time::Instant;

fn supported_world(count: u64) -> World {
    let mut world = World::new(Config {
        gravity: Vector(0.0, -10.0, 0.0),
        substeps: 2,
        sleep_speed: 0.1,
        sleep_seconds: 0.05,
        ..Config::default()
    })
    .unwrap();
    world
        .add_body(Body::new(
            BodyId(0),
            Shape::Box(Vector(2_000.0, 0.1, 10.0)),
            Vector(0.0, -0.1, 0.0),
            0.0,
        ))
        .unwrap();
    for index in 0..count {
        world
            .add_body(Body::new(
                BodyId(index + 1),
                Shape::Box(Vector(0.5, 0.5, 0.5)),
                Vector(index as Real * 3.0, 0.5, 0.0),
                1.0,
            ))
            .unwrap();
    }
    for _ in 0..100 {
        world.step(0.01).unwrap();
    }
    assert!(world.is_quiescent());
    world.release_transaction_scratch();
    world
}

#[test]
fn stamp_epoch_wrap_still_captures_the_first_before_image() {
    let body = Body::new(BodyId(7), Shape::Sphere(1.0), Vector::ZERO, 1.0);
    let mut journal = Journal {
        epoch: u64::MAX,
        stamps: vec![1],
        ..Journal::default()
    };
    journal.begin(1);
    journal.body(0, &body);
    journal.body(0, &body);
    assert_eq!(journal.stats(false).journaled_bodies, 1);
}

#[test]
#[ignore = "deterministic transaction work ratchet runs explicitly"]
fn transaction_work_ratchet() {
    for count in [128, 512] {
        for local in [false, true] {
            let mut world = supported_world(count);
            if local {
                world
                    .set_velocity(BodyId(1), Vector(0.2, 0.0, 0.0))
                    .unwrap();
                world.step(0.01).unwrap();
            }
            let unaffected = world
                .bodies()
                .filter(|b| !local || b.id.0 > 1)
                .cloned()
                .collect::<Vec<_>>();
            let mut totals = TransactionStats::default();
            let mut milliseconds = 0.0;
            for tick in 0..64 {
                if local {
                    let speed = if tick % 2 == 0 { -0.2 } else { 0.2 };
                    world
                        .set_velocity(BodyId(1), Vector(speed, 0.0, 0.0))
                        .unwrap();
                }
                let start = Instant::now();
                let report = world.step(0.01).unwrap();
                milliseconds += start.elapsed().as_secs_f64() * 1_000.0;
                assert!(report.retired.is_empty());
                let work = world.last_step_transaction();
                assert_eq!(work.journaled_bodies, if local { 2 } else { 0 });
                assert_eq!(work.journaled_pairs, u64::from(local));
                assert!(!work.rolled_back);
                totals.journaled_bodies += work.journaled_bodies;
                totals.journaled_pairs += work.journaled_pairs;
                totals.copied_contact_points += work.copied_contact_points;
                totals.vector_growths += work.vector_growths;
                totals.vector_capacity_bytes =
                    totals.vector_capacity_bytes.max(work.vector_capacity_bytes);
            }
            for body in &unaffected {
                assert_eq!(world.body(body.id), Some(body));
            }
            let name = if local { "local" } else { "quiet" };
            crate::performance_ratchet::record(
                &format!("floating-transaction/{name}/{count}"),
                &[
                    ("journaled_bodies", totals.journaled_bodies),
                    ("journaled_pairs", totals.journaled_pairs),
                    ("copied_contact_points", totals.copied_contact_points),
                    ("vector_growths", totals.vector_growths),
                    ("vector_capacity_bytes", totals.vector_capacity_bytes),
                ],
                &[
                    ("admitted_bodies", count + 1),
                    ("completed_ticks", 64),
                    ("rolled_back_steps", 0),
                ],
                &[("whole_step_ms", milliseconds)],
            );
        }
    }
    let mut world = World::new(Config {
        gravity: Vector::ZERO,
        substeps: 1,
        ..Config::default()
    })
    .unwrap();
    let id = BodyId(1);
    let position = Vector(1e12 - 1.0, 0.0, 0.0);
    let mut body = Body::new(id, Shape::Sphere(1.0), position, 1.0);
    body.velocity = Vector(20.0, 0.0, 0.0);
    world.add_body(body).unwrap();
    world
        .apply_impulse(id, Vector(1.0, 0.0, 0.0), position)
        .unwrap();
    let before = world.body(id).unwrap().clone();
    let mut copied = 0;
    let mut milliseconds = 0.0;
    for _ in 0..32 {
        let start = Instant::now();
        assert!(matches!(
            world.step(0.1),
            Err(Error::NonFiniteState(BodyId(1)))
        ));
        milliseconds += start.elapsed().as_secs_f64() * 1_000.0;
        assert_eq!(world.body(id), Some(&before));
        assert_eq!(world.elapsed_seconds(), 0.0);
        let work = world.last_step_transaction();
        assert!(work.rolled_back);
        copied += work.journaled_bodies;
    }
    crate::performance_ratchet::record(
        "floating-transaction/rollback",
        &[("journaled_bodies", copied)],
        &[
            ("completed_ticks", 0),
            ("failed_steps", 32),
            ("rolled_back_steps", 32),
        ],
        &[("failed_call_ms", milliseconds)],
    );
}
