use super::super::{Config, Report, World};
use super::*;

fn pair() -> [Body; 2] {
    [
        Body::new(
            BodyId(1),
            Shape::Box(Vector(3.0, 1.0, 2.0)),
            Vector::ZERO,
            0.0,
        ),
        Body::new(
            BodyId(2),
            Shape::Box(Vector(1.0, 1.0, 1.0)),
            Vector(0.0, 1.99, 0.0),
            2.0,
        ),
    ]
}
fn bits(m: &Option<contact::Manifold>) -> Option<(Vec<u64>, bool)> {
    m.as_ref().map(|m| {
        let mut out = vector_bits(m.normal).to_vec();
        out.push(m.time.to_bits());
        for p in &m.points {
            out.extend(vector_bits(p.ra));
            out.extend(vector_bits(p.rb));
            out.push(p.separation.to_bits());
        }
        (out, m.swept)
    })
}
fn query(
    cache: &mut GeometryCache,
    bodies: &[Body; 2],
    margin: Scalar,
) -> (Option<contact::Manifold>, GeometryStats) {
    let mut work = GeometryStats::default();
    cache.begin(2);
    let result = cache.current([0, 1], [&bodies[0], &bodies[1]], margin, &mut work);
    cache.finish(&mut work);
    assert_eq!(
        bits(&result),
        bits(&contact::current(&bodies[0], &bodies[1], margin))
    );
    (result, work)
}

#[test]
fn identical_poses_reuse_points_without_sat_or_clipping() {
    let mut cache = GeometryCache::default();
    let mut b = pair();
    let (initial, cold) = query(&mut cache, &b, 0.02);
    assert_eq!(initial.as_ref().unwrap().points.len(), 4);
    assert_eq!(cold.frame_preparations, 2);
    assert_eq!(cold.projection_preparations, 1);
    assert_eq!(cold.sat_queries, 1);
    assert_eq!(cold.clip_passes, 4);
    // Response and velocity changes do not change current contact geometry.
    b[1].velocity = Vector(3.0, -4.0, 0.0);
    b[1].mass = 7.0;
    for _ in 0..20 {
        let (m, work) = query(&mut cache, &b, 0.02);
        assert_eq!(bits(&m), bits(&initial));
        assert_eq!(work.manifold_hits, 1);
        assert_eq!(
            work.sat_queries + work.clip_passes + work.frame_preparations,
            0
        );
    }
}

#[test]
fn translations_reuse_projections_but_refresh_clipped_points_and_separation() {
    let mut cache = GeometryCache::default();
    let mut b = pair();
    query(&mut cache, &b, 0.02);
    for (x, y, z) in [
        (0.0, 1.98, 0.0),
        (0.75, 1.995, 0.3),
        (2.9, 2.0, 1.4),
        (4.2, 2.0, 1.4),
        (0.0, -1.99, 0.0),
        (0.0, 1.99, 0.0),
    ] {
        b[1].position = Vector(x, y, z);
        let (_, work) = query(&mut cache, &b, 0.02);
        assert_eq!(work.manifold_hits, 0);
        assert_eq!(work.projection_reuses, 1);
        assert_eq!(work.frame_preparations + work.projection_preparations, 0);
        assert_eq!(work.sat_queries, 1);
    }
    // Moving supports retain correct world-space rounding too.
    for offset in [0.1, 123.456, -150.1] {
        b[0].position += Vector(offset, offset, offset);
        b[1].position += Vector(offset, offset, offset);
        query(&mut cache, &b, 0.02);
    }
}

#[test]
fn rotation_shape_margin_and_signed_zero_are_dependency_changes() {
    let mut cache = GeometryCache::default();
    let mut b = pair();
    query(&mut cache, &b, 0.02);
    for axis in [Vector::X, Vector::Y, Vector::Z] {
        b[1].orientation = b[1].orientation.integrate(axis, 0.08);
        let (_, w) = query(&mut cache, &b, 0.02);
        assert_eq!(w.frame_preparations, 1);
        assert_eq!(w.projection_preparations, 1);
        assert_eq!(w.manifold_hits, 0);
    }
    b[1].shape = Shape::Box(Vector(0.4, 0.7, 1.9));
    assert_eq!(query(&mut cache, &b, 0.02).1.projection_preparations, 1);
    assert_eq!(query(&mut cache, &b, 0.08).1.manifold_hits, 0);
    b[1].shape = Shape::Sphere(0.9);
    assert_eq!(query(&mut cache, &b, 0.02).1.manifold_hits, 0);
    b[1].shape = Shape::Box(Vector(1.0, 1.0, 1.0));
    assert_eq!(query(&mut cache, &b, 0.02).1.projection_preparations, 1);
    b[0].position.0 = -0.0;
    assert_eq!(query(&mut cache, &b, 0.02).1.manifold_hits, 0);
}

#[test]
fn cached_negative_contact_does_not_suppress_changed_velocity_or_dt_ccd() {
    let mut w = World::new(Config::default()).unwrap();
    w.add_body(Body::new(
        BodyId(1),
        Shape::Box(Vector(1.0, 1.0, 1.0)),
        Vector::ZERO,
        0.0,
    ))
    .unwrap();
    let mut projectile = Body::new(BodyId(2), Shape::Sphere(0.1), Vector(-3.0, 1.12, 0.0), 1.0);
    projectile.ccd = true;
    projectile.rotation_locked = true; // Exercise a cache-eligible projectile.
    projectile.velocity = Vector(40.0, 0.0, 0.0);
    w.add_body(projectile).unwrap();
    let mut out = Vec::new();
    w.manifolds::<true>(0.1, &mut Report::default(), &mut out);
    assert!(out.is_empty(), "rounded corner near miss must stay a miss");
    w.set_velocity(BodyId(2), Vector(40.0, -2.0, 0.0)).unwrap();
    let mut report = Report::default();
    w.manifolds::<true>(0.1, &mut report, &mut out);
    assert_eq!(report.geometry.negative_hits, 1);
    assert_eq!(report.geometry.sweep_queries, 1);
    assert_eq!(out.len(), 1, "new trajectory must still be swept");
    assert!(out[0].2.swept);
    let mut reference = Vec::new();
    w.manifolds::<false>(0.1, &mut Report::default(), &mut reference);
    assert_eq!(out, reference);
    w.manifolds::<true>(0.001, &mut Report::default(), &mut out);
    assert!(
        out.is_empty(),
        "shorter requested interval must not reuse old impact time"
    );
}

#[test]
fn absent_pairs_removal_reused_ids_and_epoch_wrap_cannot_resurrect_geometry() {
    let mut cache = GeometryCache::default();
    let b = pair();
    query(&mut cache, &b, 0.02);
    cache.begin(2);
    let mut work = GeometryStats::default();
    cache.finish(&mut work);
    assert_eq!(work.pair_invalidations, 1);
    assert!(cache.pairs.is_empty());
    query(&mut cache, &b, 0.02);
    cache.remove(BodyId(2));
    assert!(cache.pairs.is_empty());
    assert_eq!(query(&mut cache, &b, 0.02).1.manifold_hits, 0);
    cache.epoch = u64::MAX;
    assert_eq!(query(&mut cache, &b, 0.02).1.manifold_hits, 0);
    let mut changed = b.clone();
    changed[0].id = BodyId(100);
    changed[1].id = BodyId(101);
    assert_eq!(query(&mut cache, &changed, 0.02).1.frame_preparations, 2);
    assert_eq!(
        cache.pairs.len(),
        1,
        "cache retains only current candidate pairs"
    );
}

#[test]
fn prepared_axes_match_uncached_geometry_through_tumbling_sliding_and_edge_transitions() {
    let mut cache = GeometryCache::default();
    let mut b = pair();
    // Deterministic, varied poses: no external RNG or wall-clock-dependent coverage.
    let mut seed = 7_u64;
    let mut value = || {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
        (seed >> 32) as f64 / u32::MAX as f64 * 2.0 - 1.0
    };
    for tick in 0..600 {
        b[0].orientation = Quaternion(value(), value(), value(), value()).normalized();
        b[1].orientation = Quaternion(value(), value(), value(), value()).normalized();
        b[1].position = Vector(value() * 4.0, value() * 3.0, value() * 3.0);
        if tick % 4 == 0 {
            b[0].shape = Shape::Sphere(1.5);
        } else {
            b[0].shape = Shape::Box(Vector(3.0, 1.0, 2.0));
        }
        query(&mut cache, &b, 0.02);
        query(&mut cache, &b, 0.02);
        b[1].position += Vector(0.02, -0.001, 0.017);
        query(&mut cache, &b, 0.02);
    }
}

#[test]
fn cached_and_uncached_worlds_preserve_complete_motion_and_contact_impulses() {
    for locked in [false, true] {
        let mut candidate = World::new(Config::default()).unwrap();
        candidate
            .add_body(Body::new(
                BodyId(1),
                Shape::Box(Vector(50.0, 1.0, 50.0)),
                Vector(0.0, -1.0, 0.0),
                0.0,
            ))
            .unwrap();
        for i in 0..4 {
            let mut b = Body::new(
                BodyId(10 + i),
                Shape::Box(Vector(2.0, 2.0, 2.0)),
                Vector(0.0, 2.0 + 4.0 * i as f64, 0.0),
                2.0 + i as f64,
            );
            b.rotation_locked = locked;
            candidate.add_body(b).unwrap();
        }
        let mut reference = candidate.clone();
        for tick in 0..400 {
            if tick == 200 {
                for w in [&mut candidate, &mut reference] {
                    w.set_velocity(BodyId(12), Vector(10.0, 0.0, 0.0)).unwrap();
                }
            }
            if tick == 300 {
                candidate.remove_body(BodyId(10));
                reference.remove_body(BodyId(10));
            }
            let mut actual = candidate
                .step_with_geometry::<true, true>(1.0 / 60.0)
                .unwrap();
            let mut expected = reference
                .step_with_geometry::<true, false>(1.0 / 60.0)
                .unwrap();
            assert_eq!(candidate.bodies, reference.bodies, "body state tick {tick}");
            assert_eq!(
                format!("{:?}", candidate.cache),
                format!("{:?}", reference.cache),
                "warm-start impulses tick {tick}"
            );
            actual.geometry = GeometryStats::default();
            expected.geometry = GeometryStats::default();
            assert_eq!(
                format!("{actual:?}"),
                format!("{expected:?}"),
                "old work counters tick {tick}"
            );
        }
    }
}

#[test]
fn zero_and_quiescent_steps_do_no_geometry_work_but_report_retained_memory() {
    let mut w = World::new(Config::default()).unwrap();
    for b in pair() {
        w.add_body(b).unwrap();
    }
    for _ in 0..240 {
        w.step(1.0 / 60.0).unwrap();
    }
    assert!(w.is_quiescent());
    let retained = w.geometry.retained_bytes();
    assert!(retained > 0);
    for dt in [0.0, 1.0 / 60.0] {
        let r = w.step(dt).unwrap();
        assert_eq!(
            r.geometry.current_queries + r.geometry.sat_queries + r.geometry.clip_passes,
            0
        );
        assert_eq!(r.geometry.retained_bytes, retained as u64);
    }
}

#[test]
fn rotating_pairs_share_frames_without_retaining_pair_payloads() {
    let mut cache = GeometryCache::default();
    let mut bodies = (0..4)
        .map(|i| {
            let mut b = Body::new(
                BodyId(i + 1),
                Shape::Box(Vector(3.0, 1.0, 2.0)),
                Vector(i as f64, 0.0, 0.0),
                2.0,
            );
            b.orientation = Quaternion(0.1 * i as f64, 0.2, 0.3, 0.9).normalized();
            b
        })
        .collect::<Vec<_>>();
    for pass in 0..3 {
        if pass == 2 {
            bodies[1].orientation = bodies[1].orientation.integrate(Vector::X, 0.01);
        }
        cache.begin(bodies.len());
        let mut work = GeometryStats::default();
        for i in 0..bodies.len() {
            for j in i + 1..bodies.len() {
                let actual = cache.query([i, j], [&bodies[i], &bodies[j]], 0.02, &mut work);
                assert_eq!(
                    bits(&actual),
                    bits(&contact::current(&bodies[i], &bodies[j], 0.02))
                );
            }
        }
        cache.finish(&mut work);
        assert_eq!(work.frame_preparations, [4, 0, 1][pass]);
        assert_eq!(work.frame_preparations + work.frame_reuses, 12);
        assert_eq!(work.manifold_hits + work.projection_preparations, 0);
        assert!(
            cache.pairs.is_empty(),
            "rotating pairs must not allocate pair entries"
        );
    }
    cache.remove(BodyId(2));
    bodies[1].shape = Shape::Box(Vector(1.1, 0.8, 2.5));
    cache.begin(bodies.len());
    let mut work = GeometryStats::default();
    let actual = cache.query([0, 1], [&bodies[0], &bodies[1]], 0.02, &mut work);
    assert_eq!(
        bits(&actual),
        bits(&contact::current(&bodies[0], &bodies[1], 0.02))
    );
    assert_eq!(
        work.frame_preparations, 2,
        "reused IDs must not reuse old indexed frames"
    );
}

#[test]
fn separated_rotating_query_short_circuits_without_pair_or_polygon_storage() {
    let mut cache = GeometryCache::default();
    let mut b = pair();
    b[1].position = Vector(100.0, 0.0, 0.0);
    cache.begin(2);
    let mut work = GeometryStats::default();
    assert!(
        cache
            .query([0, 1], [&b[0], &b[1]], 0.02, &mut work)
            .is_none()
    );
    assert_eq!(work.sat_axes_tested, 1);
    assert_eq!(work.clip_passes, 0);
    assert_eq!(cache.clipping.retained_bytes(), 0);
    assert!(cache.pairs.is_empty());
}

#[test]
fn rotating_frame_and_sweep_reuse_matches_uncached_oracle() {
    let mut cache = GeometryCache::default();
    let mut b = pair();
    let mut seed = 911_u64;
    let mut value = || {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
        (seed >> 32) as f64 / u32::MAX as f64 * 2.0 - 1.0
    };
    let mut contacts = 0;
    let mut misses = 0;
    let mut swept_hits = 0;
    for _ in 0..1200 {
        b[0].orientation = Quaternion(value(), value(), value(), value()).normalized();
        b[1].orientation = Quaternion(value(), value(), value(), value()).normalized();
        b[1].position = Vector(value() * 6.0, value() * 4.0, value() * 4.0);
        b[1].velocity = Vector(value() * 400.0, value() * 400.0, value() * 400.0);
        cache.begin(2);
        let mut work = GeometryStats::default();
        let actual = cache.query([0, 1], [&b[0], &b[1]], 0.02, &mut work);
        assert_eq!(bits(&actual), bits(&contact::current(&b[0], &b[1], 0.02)));
        contacts += usize::from(actual.is_some());
        misses += usize::from(actual.is_none());
        for dt in [1.0 / 240.0, 1.0 / 60.0, 0.001] {
            // Changing trajectory/time within a read-only pose pass must not reuse a time of hit.
            b[1].velocity = -b[1].velocity;
            let actual = cache.swept([0, 1], [&b[0], &b[1]], dt, 0.02, &mut work);
            let mut reference_work = GeometryStats::default();
            let expected = contact::swept(&b[0], &b[1], dt, 0.02, &mut reference_work);
            assert_eq!(bits(&actual), bits(&expected), "dt={dt}");
            swept_hits += usize::from(actual.is_some());
        }
        assert_eq!(work.frame_preparations, 2);
        cache.finish(&mut work);
        assert!(cache.pairs.is_empty());
    }
    assert!(contacts > 50 && misses > 50 && swept_hits > 50);
}

#[test]
fn prepared_sweeps_preserve_thin_wall_and_changed_interval() {
    let mut cache = GeometryCache::default();
    let wall = Body::new(
        BodyId(1),
        Shape::Box(Vector(0.01, 10.0, 10.0)),
        Vector::ZERO,
        0.0,
    );
    let mut projectile = Body::new(
        BodyId(2),
        Shape::Box(Vector(0.1, 0.1, 0.5)),
        Vector(-3.0, 0.0, 0.0),
        1.0,
    );
    projectile.velocity = Vector(1000.0, 0.0, 0.0);
    projectile.ccd = true;
    let mut work = GeometryStats::default();
    cache.begin(2);
    let first = cache.swept([0, 1], [&wall, &projectile], 0.01, 0.02, &mut work);
    assert!(first.is_some());
    assert!(
        cache
            .swept([0, 1], [&wall, &projectile], 0.0001, 0.02, &mut work)
            .is_none()
    );
    projectile.velocity = -projectile.velocity;
    assert!(
        cache
            .swept([0, 1], [&wall, &projectile], 0.01, 0.02, &mut work)
            .is_none()
    );
    assert_eq!(work.frame_preparations, 2);
    assert_eq!(work.sweep_queries, 3);
}
