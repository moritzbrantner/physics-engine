use super::*;
use crate::approximate::{CheckpointContext, Config};

fn world() -> World {
    World::new(Config {
        gravity: V::ZERO,
        ..Default::default()
    })
    .unwrap()
}

#[test]
fn native_contract_has_stable_ties_features_support_velocity_and_no_state_changes() {
    let mut w = world();
    for id in [9, 3] {
        let mut b = Body::new(BodyId(id), Shape::Box(V(0.01, 2.0, 2.0)), V::ZERO, 1.0);
        b.velocity = V::Y;
        b.angular_velocity = V(0.0, 0.0, 2.0);
        w.add_body(b).unwrap();
    }
    w.add_force(BodyId(3), V::X).unwrap();
    let context = CheckpointContext {
        build: [19; 32],
        content: [94; 32],
    };
    let before = w.checkpoint(context).unwrap().to_bytes();
    let cast = RayCast::new(V(-100.0, 0.0, 0.0), V(200.0, 0.0, 0.0));
    let mut hits = Vec::new();
    let stats = w.cast_ray(cast, QueryFilter::default(), &mut hits).unwrap();
    assert_eq!(stats.ray_queries, 2);
    assert_eq!(stats.ray_planes_tested, 12);
    assert_eq!(
        hits.iter().map(|h| h.body).collect::<Vec<_>>(),
        [BodyId(3), BodyId(9)]
    );
    for h in &hits {
        assert!((h.fraction - 0.49995).abs() < 1e-14);
        assert!((h.distance - 99.99).abs() < 1e-12);
        assert_eq!(h.normal, -V::X);
        assert_eq!(h.feature, RayFeature::BoxFace(0));
        assert!((h.support_velocity - V(0.0, 0.98, 0.0)).length() < 1e-12);
    }
    let expected = hits.clone();
    for _ in 0..64 {
        let stats = w.cast_ray(cast, QueryFilter::default(), &mut hits).unwrap();
        assert_eq!(hits, expected);
        assert_eq!(stats.output_capacity_growths, 0);
    }
    assert_eq!(w.checkpoint(context).unwrap().to_bytes(), before);
    let inside = RayCast::new(V::ZERO, V::X);
    w.cast_ray(inside, QueryFilter::default(), &mut hits)
        .unwrap();
    assert!(
        hits.iter().all(|h| h.starts_overlapping
            && h.feature == RayFeature::Interior
            && h.fraction == 0.0)
    );
    w.cast_ray(
        RayCast::new(V(-0.01, 0.0, 0.0), -V::X),
        QueryFilter::default(),
        &mut hits,
    )
    .unwrap();
    assert!(
        hits.iter()
            .all(|h| !h.starts_overlapping && h.fraction == 0.0)
    );
}

#[test]
fn filters_lifecycle_and_invalid_requests_cannot_reuse_stale_results() {
    let mut w = world();
    let mut b = Body::new(BodyId(1), Shape::Sphere(1.0), V::ZERO, 0.0);
    b.sensor = true;
    b.layers = CollisionLayers3d::new(2, 2);
    w.add_body(b).unwrap();
    let cast = RayCast::new(V(-3.0, 0.0, 0.0), V(6.0, 0.0, 0.0));
    let mut hits = Vec::new();
    let filter = QueryFilter {
        include_sensors: true,
        ..Default::default()
    };
    w.cast_ray(cast, QueryFilter::default(), &mut hits).unwrap();
    assert!(hits.is_empty());
    w.cast_ray(cast, filter, &mut hits).unwrap();
    assert!(hits[0].sensor);
    for filter in [
        QueryFilter {
            exclude: Some(BodyId(1)),
            ..filter
        },
        QueryFilter {
            layers: CollisionLayers3d::new(1, 1),
            ..filter
        },
    ] {
        w.cast_ray(cast, filter, &mut hits).unwrap();
        assert!(hits.is_empty());
    }
    w.remove_body(BodyId(1)).unwrap();
    w.add_body(Body::new(
        BodyId(1),
        Shape::wedge(V(1.0, 1.0, 1.0)),
        V(30.0, 0.0, 0.0),
        0.0,
    ))
    .unwrap();
    w.cast_ray(cast, filter, &mut hits).unwrap();
    assert!(hits.is_empty());
    w.cast_ray(
        RayCast::new(V(27.0, -0.5, 0.0), V(6.0, 0.0, 0.0)),
        filter,
        &mut hits,
    )
    .unwrap();
    assert_eq!(hits[0].feature, RayFeature::WedgeFace(1));
    for cast in [
        RayCast::new(V::ZERO, V::ZERO),
        RayCast::new(V(f64::NAN, 0.0, 0.0), V::X),
        RayCast::new(V(1e12, 0.0, 0.0), V::X),
    ] {
        let error = w.cast_ray(cast, filter, &mut hits).unwrap_err();
        assert_eq!(error.reason, QueryFailureReason::InvalidInput);
        assert!(hits.is_empty());
    }
}

#[test]
fn partial_work_and_hits_are_reported_when_support_velocity_fails() {
    let mut w = world();
    w.add_body(Body::new(
        BodyId(1),
        Shape::Sphere(1.0),
        V(-3.0, 0.0, 0.0),
        0.0,
    ))
    .unwrap();
    w.add_body(Body::new(BodyId(2), Shape::Sphere(2.0), V::ZERO, 0.0))
        .unwrap();
    // Deliberately corrupt otherwise validated state to test the numerical error boundary.
    w.bodies[1].angular_velocity = V(0.0, f64::MAX, 0.0);
    let mut hits = Vec::new();
    let error = w
        .cast_ray(
            RayCast::new(V(-10.0, 0.0, 0.0), V(20.0, 0.0, 0.0)),
            QueryFilter::default(),
            &mut hits,
        )
        .unwrap_err();
    assert_eq!(
        error.reason,
        QueryFailureReason::Search {
            body: BodyId(2),
            reason: SweepFailure::NonFiniteComputation
        }
    );
    assert_eq!(error.stats.hits_admitted, 1);
    assert_eq!(error.stats.ray_queries, 2);
    assert_eq!(error.stats.ray_quadratic_tests, 2);
    assert!(hits.is_empty());
}

#[test]
fn seeded_pruned_rays_match_exhaustive_rotated_geometry() {
    let mut seed = 0x194_0930_u64;
    let mut next = || {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
        (seed >> 32) as Scalar / u32::MAX as Scalar * 40.0 - 20.0
    };
    let mut pruned = Vec::new();
    let mut exhaustive = Vec::new();
    for _ in 0..20 {
        let mut w = world();
        for id in 0..32 {
            let shape = match id % 4 {
                0 => Shape::Sphere(0.7),
                1 => Shape::Box(V(0.5, 1.0, 0.3)),
                2 => Shape::capsule(0.7, 0.3),
                _ => Shape::wedge(V(1.0, 0.7, 0.5)),
            };
            let mut b = Body::new(BodyId(id), shape, V(next(), next(), next()), 0.0);
            let angle = next() * 0.1;
            b.orientation = Quaternion(0.0, (angle * 0.5).sin(), 0.0, (angle * 0.5).cos());
            b.sensor = id % 5 == 0;
            w.add_body(b).unwrap();
        }
        for i in 0..32 {
            let cast = RayCast::new(
                V(next(), next(), next()),
                V(next() * 2.0, next() * 2.0, next() * 2.0),
            );
            let filter = QueryFilter {
                include_sensors: true,
                exclude: Some(BodyId(i)),
                ..Default::default()
            };
            w.cast_ray(cast, filter, &mut pruned).unwrap();
            w.ray_query(cast, filter, &mut exhaustive, false).unwrap();
            assert_eq!(pruned, exhaustive);
        }
    }
}

#[test]
fn sparse_work_is_bounded_and_warmed_output_retains_capacity() {
    let mut w = world();
    for id in 0..512 {
        w.add_body(Body::new(
            BodyId(id),
            Shape::Sphere(0.5),
            V(0.0, id as Scalar * 4.0, 0.0),
            0.0,
        ))
        .unwrap();
    }
    let mut hits = Vec::new();
    let cast = RayCast::new(V(-3.0, 0.0, 0.0), V(6.0, 0.0, 0.0));
    w.cast_ray(cast, QueryFilter::default(), &mut hits).unwrap();
    let expected = hits.clone();
    for _ in 0..32 {
        let s = w.cast_ray(cast, QueryFilter::default(), &mut hits).unwrap();
        assert_eq!(hits, expected);
        assert_eq!(s.bodies_visited, 512);
        assert_eq!(s.bound_tests, 512);
        assert_eq!(s.exact_candidates, 1);
        assert_eq!(s.geometry_preparations, 1);
        assert_eq!(s.ray_quadratic_tests, 1);
        assert_eq!(s.output_capacity_growths, 0);
    }
}

#[test]
#[ignore = "advisory paired whole-call evidence; correctness/work are asserted in ordinary tests"]
fn paired_ray_workload_evidence() {
    use std::{hint::black_box, time::Instant};
    for (kind, shape) in [
        ("sphere", Shape::Sphere(0.5)),
        ("box", Shape::Box(V(0.5, 0.5, 0.5))),
        ("capsule", Shape::capsule(1.0, 0.5)),
        ("wedge", Shape::wedge(V(0.5, 0.5, 0.5))),
    ] {
        for scenario in ["small_hit", "sparse_hit", "sparse_miss", "crowded_hit"] {
            let count = if scenario == "small_hit" { 4 } else { 512 };
            let mut w = world();
            for id in 0..count {
                let position = if scenario == "crowded_hit" {
                    V(id as Scalar * 2.0, 0.0, 0.0)
                } else {
                    V(0.0, id as Scalar * 4.0, 0.0)
                };
                w.add_body(Body::new(BodyId(id), shape, position, 0.0))
                    .unwrap();
            }
            let y = if scenario == "sparse_miss" {
                -10.0
            } else {
                0.0
            };
            let dx = if scenario == "crowded_hit" {
                2000.0
            } else {
                6.0
            };
            let cast = RayCast::new(V(-3.0, y, 0.0), V(dx, 0.0, 0.0));
            let mut pruned = Vec::new();
            let mut exhaustive = Vec::new();
            w.cast_ray(cast, QueryFilter::default(), &mut pruned)
                .unwrap();
            w.ray_query(cast, QueryFilter::default(), &mut exhaustive, false)
                .unwrap();
            assert_eq!(pruned, exhaustive);
            for trial in 0..3 {
                // Alternate timing order; equality/correctness checks stay outside timed calls.
                let mut timings = [0u128; 2];
                let mut work = [QueryStats::default(); 2];
                for prune in if trial % 2 == 0 {
                    [true, false]
                } else {
                    [false, true]
                } {
                    let index = usize::from(!prune);
                    let output = if prune { &mut pruned } else { &mut exhaustive };
                    let start = Instant::now();
                    for _ in 0..256 {
                        work[index] = black_box(
                            w.ray_query(black_box(cast), QueryFilter::default(), output, prune)
                                .unwrap(),
                        );
                    }
                    timings[index] = start.elapsed().as_nanos();
                    assert_eq!(work[index].output_capacity_growths, 0);
                }
                assert_eq!(pruned, exhaustive);
                println!(
                    "RAY_EVIDENCE {{\"shape\":\"{kind}\",\"scenario\":\"{scenario}\",\"trial\":{trial},\"bodies\":{count},\"calls\":256,\"hits\":{},\"pruned_ns\":{},\"exhaustive_ns\":{},\"body_visits\":{},\"pruned_exact\":{},\"exhaustive_exact\":{},\"geometry_preparations\":{},\"planes\":{},\"quadratics\":{},\"output_capacity_bytes\":{}}}",
                    pruned.len(),
                    timings[0],
                    timings[1],
                    work[0].bodies_visited,
                    work[0].exact_candidates,
                    work[1].exact_candidates,
                    work[0].geometry_preparations,
                    work[0].ray_planes_tested,
                    work[0].ray_quadratic_tests,
                    work[0].output_capacity_bytes
                );
            }
        }
    }
}
