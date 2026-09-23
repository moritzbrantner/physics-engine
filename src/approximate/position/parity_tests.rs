//! Frozen scalar scan from ca394b3f, independent of prepared views/frames/clipping buffers.
use super::*;
use crate::{
    BodyId, CollisionLayers3d,
    approximate::{Config, Quaternion},
};

pub(super) fn reference(w: &mut World, h: f64, report: &mut PositionReport) -> Result<(), Error> {
    for _ in 0..w.config.fixed_position_iterations {
        report.passes += 1;
        let mut changed = false;
        for i in 0..w.bodies.len() {
            if !w.bodies[i].movable() || w.bodies[i].sleeping || w.bodies[i].sensor {
                continue;
            }
            for j in 0..w.bodies.len() {
                if w.bodies[j].mass != 0.0
                    || w.bodies[j].sensor
                    || !w.bodies[i].layers.collides_with(w.bodies[j].layers)
                {
                    continue;
                }
                report.bounds_tests += 1;
                let (lo, hi) = w.bodies[i].cached_bounds;
                let (bl, bh) = w.bodies[j].cached_bounds;
                if lo.0 > bh.0
                    || hi.0 < bl.0
                    || lo.1 > bh.1
                    || hi.1 < bl.1
                    || lo.2 > bh.2
                    || hi.2 < bl.2
                {
                    continue;
                }
                report.contact_tests += 1;
                // Fresh post-integration manifold: no old normals or swept TOI may substitute.
                let Some(m) = super::contact::current(&w.bodies[j], &w.bodies[i], 0.0) else {
                    continue;
                };
                let depth = (&m.points)
                    .into_iter()
                    .map(|p| -p.separation)
                    .fold(0.0, f64::max);
                let distance = (depth - w.config.contact_slop).max(0.0);
                if distance <= 0.0 {
                    continue;
                }
                let body = &mut w.bodies[i];
                body.position += m.normal * distance;
                if !body.valid() {
                    return Err(Error::NonFiniteState(body.id));
                }
                body.cached_bounds = super::contact::bounds(body);
                // A body still receiving material correction is not ready to sleep.
                if distance > w.config.sleep_speed * h {
                    body.quiet_time = 0.0;
                }
                report.corrections += 1;
                report.max_distance = report.max_distance.max(distance);
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }
    Ok(())
}

fn vector_bits(v: Vector) -> [u64; 3] {
    [v.0.to_bits(), v.1.to_bits(), v.2.to_bits()]
}
fn body_bits(b: &Body) -> Vec<u64> {
    let mut values = vec![
        b.mass.to_bits(),
        b.friction.to_bits(),
        b.restitution.to_bits(),
        b.quiet_time.to_bits(),
    ];
    for v in [
        b.position,
        b.velocity,
        b.angular_velocity,
        b.shape.half_extents(),
        b.force,
        b.torque,
        b.impulse,
        b.angular_impulse,
        b.cached_bounds.0,
        b.cached_bounds.1,
    ] {
        values.extend(vector_bits(v));
    }
    values.extend([
        b.orientation.0.to_bits(),
        b.orientation.1.to_bits(),
        b.orientation.2.to_bits(),
        b.orientation.3.to_bits(),
    ]);
    values
}
fn same(a: &World, b: &World, ar: &PositionReport, br: &PositionReport) {
    assert_eq!(a.bodies, b.bodies);
    for (a, b) in a.bodies.iter().zip(&b.bodies) {
        assert_eq!(body_bits(a), body_bits(b));
    }
    assert_eq!(
        (
            ar.passes,
            ar.bounds_tests,
            ar.contact_tests,
            ar.corrections,
            ar.max_distance.to_bits()
        ),
        (
            br.passes,
            br.bounds_tests,
            br.contact_tests,
            br.corrections,
            br.max_distance.to_bits()
        )
    );
    assert_eq!(a.elapsed.to_bits(), b.elapsed.to_bits());
}
fn pair(w: World) -> (World, World) {
    let mut reference = w.clone();
    reference.bookkeeping.position.reference = true;
    (w, reference)
}
fn correct_pair(a: &mut World, b: &mut World) -> PositionReport {
    let mut ar = PositionReport::default();
    let mut br = PositionReport::default();
    assert_eq!(
        a.correct_fixed_positions(1.0 / 240.0, &mut ar),
        b.correct_fixed_positions(1.0 / 240.0, &mut br)
    );
    same(a, b, &ar, &br);
    ar
}
fn world() -> World {
    let mut w = World::new(Config {
        fixed_position_iterations: 2,
        ..Config::default()
    })
    .unwrap();
    w.add_body(Body::new(
        BodyId(10),
        Shape::Box(Vector(50.0, 1.0, 50.0)),
        Vector(0.0, -1.0, 0.0),
        0.0,
    ))
    .unwrap();
    w
}
fn random(seed: &mut u64) -> f64 {
    *seed = seed
        .wrapping_mul(6364136223846793005)
        .wrapping_add(1442695040888963407);
    ((*seed >> 11) as f64 / (1u64 << 53) as f64) * 2.0 - 1.0
}
#[test]
fn prepared_position_matches_original_scan_over_rotated_shapes_and_filters() {
    let mut seed = 91624u64;
    for case in 0..1200 {
        let mut w = world();
        let mut fixed = Body::new(
            BodyId(30),
            Shape::Box(Vector(3.0, 1.0, 2.0)),
            Vector(0.0, 2.5, 0.0),
            0.0,
        );
        fixed.orientation =
            Quaternion(random(&mut seed), random(&mut seed), random(&mut seed), 1.0).normalized();
        if case % 5 == 0 {
            fixed.shape = Shape::Sphere(2.0)
        }
        if case % 13 == 0 {
            fixed.sensor = true
        }
        w.add_body(fixed).unwrap();
        for id in [1, 20, 40] {
            let v = Vector(random(&mut seed), random(&mut seed), random(&mut seed));
            let shape = if (case + id) % 3 == 0 {
                Shape::Sphere(1.5)
            } else {
                Shape::Box(Vector(2.0, 1.0, 0.75))
            };
            let mut b = Body::new(BodyId(id as u64), shape, v * 2.0, 3.0);
            b.orientation = Quaternion(v.0, v.1, v.2, 1.0).normalized();
            b.velocity = v;
            b.angular_velocity = v * 0.3;
            if case % 7 == 0 {
                b.layers = CollisionLayers3d::new(0, 0)
            }
            if case % 11 == 0 {
                b.external = true
            }
            if case % 17 == 0 {
                b.sleeping = true
            }
            if case % 19 == 0 {
                b.sensor = true
            }
            w.add_body(b).unwrap();
        }
        let (mut a, mut b) = pair(w);
        correct_pair(&mut a, &mut b);
        correct_pair(&mut a, &mut b);
    }
}
#[test]
fn correction_discovers_a_later_collider_entered_during_the_same_pass() {
    let mut w = world();
    w.config.fixed_position_iterations = 1;
    w.add_body(Body::new(
        BodyId(30),
        Shape::Box(Vector(50.0, 1.0, 50.0)),
        Vector(0.0, 2.6, 0.0),
        0.0,
    ))
    .unwrap();
    w.add_body(Body::new(
        BodyId(20),
        Shape::Sphere(1.0),
        Vector(0.0, 0.5, 0.0),
        1.0,
    ))
    .unwrap();
    assert!(w.bodies[1].cached_bounds.1.1 < w.bodies[2].cached_bounds.0.1);
    let (mut a, mut b) = pair(w);
    let r = correct_pair(&mut a, &mut b);
    assert_eq!(r.corrections, 2);
    assert_eq!(r.contact_tests, 2);
    assert!((a.body(BodyId(20)).unwrap().position.1 - 0.62).abs() < 1e-12);
}
#[test]
fn fixed_index_invalidates_for_id_reuse_shape_rotation_sensor_and_index_shifts() {
    let mut w = world();
    w.add_body(Body::new(
        BodyId(20),
        Shape::Box(Vector(1.0, 1.0, 1.0)),
        Vector(0.0, 0.4, 0.0),
        1.0,
    ))
    .unwrap();
    let (mut a, mut b) = pair(w);
    correct_pair(&mut a, &mut b);
    for variant in 0..4 {
        for w in [&mut a, &mut b] {
            w.remove_body(BodyId(10)).unwrap();
            let mut f = Body::new(
                BodyId(10),
                Shape::Box(Vector(3.0, 1.0, 4.0)),
                Vector(0.0, 0.0, 0.0),
                0.0,
            );
            match variant {
                0 => f.orientation = Quaternion(0.0, 0.0, 0.3, 1.0).normalized(),
                1 => f.shape = Shape::Sphere(1.5),
                2 => f.sensor = true,
                _ => f.layers = CollisionLayers3d::new(0, 0),
            }
            w.add_body(f).unwrap();
            w.add_body(Body::new(
                BodyId(0),
                Shape::Sphere(0.5),
                Vector(100.0, 0.0, 0.0),
                1.0,
            ))
            .unwrap();
        }
        assert_eq!(correct_pair(&mut a, &mut b).fixed_index_rebuilds, 1);
        for w in [&mut a, &mut b] {
            w.remove_body(BodyId(0)).unwrap();
        }
        assert_eq!(correct_pair(&mut a, &mut b).fixed_index_rebuilds, 1);
    }
}
#[test]
fn complete_steps_preserve_rotation_impulses_contacts_sleep_and_projectile_retirement() {
    let mut w = world();
    for id in 0..4 {
        let mut b = Body::new(
            BodyId(20 + id),
            Shape::Box(Vector(2.0, 1.0, 1.0)),
            Vector(0.0, 1.0 + 2.0 * id as f64, 0.0),
            3.0,
        );
        b.orientation = Quaternion(0.0, 0.0, 0.03, 1.0).normalized();
        w.add_body(b).unwrap();
    }
    // A projectile with an earlier ID removes a row before every retained fixed/dynamic index.
    let mut shot = Body::new(BodyId(0), Shape::Sphere(0.4), Vector(0.0, 12.0, 0.0), 1.0);
    shot.velocity = Vector(0.0, -100.0, 0.0);
    shot.ccd = true;
    shot.retire_on_impact = true;
    w.add_body(shot).unwrap();
    let (mut a, mut b) = pair(w);
    let mut retired = false;
    for tick in 0..600 {
        if tick == 100 {
            for w in [&mut a, &mut b] {
                w.apply_impulse(BodyId(20), Vector(10.0, 3.0, -2.0), Vector(1.0, 0.0, 0.0))
                    .unwrap();
            }
        }
        let dt = if tick % 2 == 0 {
            1.0 / 60.0
        } else {
            1.0 / 120.0
        };
        let ar = a.step(dt).unwrap();
        let br = b.step(dt).unwrap();
        same(&a, &b, &ar.position, &br.position);
        assert_eq!(ar.retired, br.retired);
        retired |= ar.retired.contains(&BodyId(0));
        assert_eq!(ar.bookkeeping, br.bookkeeping);
        assert_eq!(ar.geometry, br.geometry);
        assert_eq!(
            a.cache.keys().collect::<Vec<_>>(),
            b.cache.keys().collect::<Vec<_>>()
        );
        for (ap, bp) in a.cache.values().zip(b.cache.values()) {
            assert_eq!(ap.len(), bp.len());
            for (ap, bp) in ap.iter().zip(bp) {
                for (av, bv) in [
                    (ap.a, bp.a),
                    (ap.b, bp.b),
                    (ap.normal, bp.normal),
                    (ap.tangent, bp.tangent),
                ] {
                    assert_eq!(vector_bits(av), vector_bits(bv));
                }
                assert_eq!(ap.impulse.to_bits(), bp.impulse.to_bits());
            }
        }
    }
    assert!(retired);
}
#[test]
fn warmed_position_work_ignores_sleeping_bodies_and_reuses_fixed_frames() {
    for count in [32, 128, 512] {
        let mut w = world();
        for id in 0..count {
            let mut b = Body::new(
                BodyId(100 + id),
                Shape::Box(Vector(1.0, 1.0, 1.0)),
                Vector(100.0 + id as f64 * 3.0, 1.0, 0.0),
                1.0,
            );
            b.sleeping = true;
            w.add_body(b).unwrap();
        }
        w.add_body(Body::new(
            BodyId(20),
            Shape::Box(Vector(1.0, 1.0, 1.0)),
            Vector(0.0, 0.99, 0.0),
            1.0,
        ))
        .unwrap();
        let mut r = PositionReport::default();
        w.correct_fixed_positions(1.0 / 240.0, &mut r).unwrap();
        assert_eq!(r.fixed_index_rebuilds, 1);
        assert_eq!(r.index_body_scans, count + 2);
        let before = w.bodies.clone();
        r = PositionReport::default();
        w.correct_fixed_positions(1.0 / 240.0, &mut r).unwrap();
        assert_eq!(w.bodies, before);
        assert_eq!((r.body_visits, r.bounds_tests, r.contact_tests), (1, 1, 1));
        assert_eq!(
            (
                r.fixed_index_rebuilds,
                r.index_body_scans,
                r.fixed_frame_preparations,
                r.scratch_growths
            ),
            (0, 0, 0, 0)
        );
        assert!(r.scratch_retained_bytes > 0 && r.scratch_retained_bytes < 4096);
    }
}
#[test]
fn zero_quiescent_and_disabled_steps_preserve_work_and_report_retained_payload() {
    let mut w = world();
    w.add_body(Body::new(
        BodyId(20),
        Shape::Box(Vector(1.0, 1.0, 1.0)),
        Vector(0.0, 0.99, 0.0),
        1.0,
    ))
    .unwrap();
    for _ in 0..180 {
        w.step(1.0 / 60.0).unwrap();
    }
    assert!(w.is_quiescent());
    let bytes = w.bookkeeping.position.retained_bytes();
    assert!(bytes > 0);
    for dt in [0.0, 1.0 / 60.0] {
        let p = w.step(dt).unwrap().position;
        assert_eq!(
            (
                p.passes,
                p.bounds_tests,
                p.body_visits,
                p.fixed_index_rebuilds
            ),
            (0, 0, 0, 0)
        );
        assert_eq!(p.scratch_retained_bytes, bytes);
    }
    let mut w = World::new(Config::default()).unwrap();
    w.add_body(Body::new(BodyId(1), Shape::Sphere(1.0), Vector::ZERO, 1.0))
        .unwrap();
    assert_eq!(
        w.step(1.0 / 60.0).unwrap().position.scratch_retained_bytes,
        0
    );
}

#[test]
#[ignore = "advisory phase timing; work counts and equivalence are gated independently"]
fn position_preparation_scaling_benchmark() {
    use std::{hint::black_box, time::Instant};
    for sleeping in [32, 128, 512] {
        let mut w = world();
        for id in 0..sleeping {
            let mut b = Body::new(
                BodyId(100 + id),
                Shape::Box(Vector(1.0, 1.0, 1.0)),
                Vector(100.0 + id as f64 * 3.0, 1.0, 0.0),
                1.0,
            );
            b.sleeping = true;
            w.add_body(b).unwrap();
        }
        for id in 0..32 {
            w.add_body(Body::new(
                BodyId(1000 + id),
                Shape::Box(Vector(1.0, 1.0, 1.0)),
                Vector((id % 8) as f64 * 3.0, 0.99, (id / 8) as f64 * 3.0),
                1.0,
            ))
            .unwrap();
        }
        let (mut candidate, mut old) = pair(w);
        correct_pair(&mut candidate, &mut old);
        for trial in 0..6 {
            for variant in 0..2 {
                let prepared = (trial + variant) % 2 == 0;
                let w = if prepared { &mut candidate } else { &mut old };
                let start = Instant::now();
                for _ in 0..200 {
                    let mut r = PositionReport::default();
                    w.correct_fixed_positions(black_box(1.0 / 240.0), &mut r)
                        .unwrap();
                    black_box(r);
                }
                println!(
                    "{{\"sleepers\":{sleeping},\"active\":32,\"trial\":{trial},\"prepared\":{prepared},\"calls\":200,\"total_ms\":{}}}",
                    start.elapsed().as_secs_f64() * 1000.0
                );
            }
        }
        correct_pair(&mut candidate, &mut old);
    }
}
