use super::*;

fn next(seed: &mut u64) -> u64 {
    *seed = seed
        .wrapping_mul(6364136223846793005)
        .wrapping_add(1442695040888963407);
    *seed
}
fn coordinate(seed: &mut u64) -> Real {
    (next(seed) >> 32) as Real / u32::MAX as Real * 40.0 - 20.0
}
fn shape(index: u64) -> Shape {
    match index % 4 {
        0 => Shape::Sphere(0.6),
        1 => Shape::Box(V(0.5, 1.0, 0.3)),
        2 => Shape::capsule(0.7, 0.3),
        _ => Shape::wedge(V(1.0, 0.7, 0.5)),
    }
}
#[test]
fn conservative_bounds_match_exhaustive_current_geometry_for_seeded_rotated_shapes() {
    let mut seed = 0x194_2026_0930;
    let mut pruned = Vec::new();
    let mut exhaustive = Vec::new();
    for _ in 0..20 {
        let mut world = World::new(super::super::Config {
            gravity: V::ZERO,
            ..Default::default()
        })
        .unwrap();
        for id in 0..32 {
            let mut body = Body::new(
                BodyId(id),
                shape(id),
                V(
                    coordinate(&mut seed),
                    coordinate(&mut seed),
                    coordinate(&mut seed),
                ),
                0.0,
            );
            let angle = coordinate(&mut seed) * 0.1;
            body.orientation = Quaternion(0.0, (angle / 2.0).sin(), 0.0, (angle / 2.0).cos());
            body.sensor = id % 5 == 0;
            world.add_body(body).unwrap();
        }
        for index in 0..16 {
            let pose = QueryPose {
                shape: shape(index),
                position: V(
                    coordinate(&mut seed),
                    coordinate(&mut seed),
                    coordinate(&mut seed),
                ),
                orientation: Quaternion(0.0, 0.0, 0.6, 0.8),
            };
            let filter = QueryFilter {
                include_sensors: true,
                exclude: Some(BodyId(index)),
                ..Default::default()
            };
            for cast in [
                None,
                Some(ShapeCast::new(
                    pose,
                    V(
                        coordinate(&mut seed),
                        coordinate(&mut seed),
                        coordinate(&mut seed),
                    ),
                )),
            ] {
                let fast = world.shape_query(pose, cast, filter, &mut pruned, true);
                let reference = world.shape_query(pose, cast, filter, &mut exhaustive, false);
                assert_eq!(
                    fast.as_ref().err().map(|error| error.reason),
                    reference.as_ref().err().map(|error| error.reason)
                );
                assert_eq!(pruned, exhaustive, "query {index}, cast {cast:?}");
                assert!(
                    fast.is_ok(),
                    "bounded seeded control unexpectedly failed: {fast:?}"
                );
                assert!(fast.unwrap().exact_candidates <= reference.unwrap().exact_candidates);
            }
        }
    }
}

#[test]
fn warmed_query_outputs_reuse_capacity_and_prune_unrelated_geometry() {
    let mut world = World::new(super::super::Config {
        gravity: V::ZERO,
        ..Default::default()
    })
    .unwrap();
    for id in 0..512 {
        world
            .add_body(Body::new(
                BodyId(id),
                Shape::Sphere(0.5),
                V(id as Real * 10.0, 0.0, 0.0),
                0.0,
            ))
            .unwrap();
    }
    let pose = QueryPose::new(Shape::Sphere(0.5), V::ZERO);
    let mut hits = Vec::new();
    world
        .overlap_shape(pose, QueryFilter::default(), &mut hits)
        .unwrap();
    let capacity = hits.capacity();
    for _ in 0..128 {
        let stats = world
            .overlap_shape(pose, QueryFilter::default(), &mut hits)
            .unwrap();
        assert_eq!(stats.bodies_visited, 512);
        assert_eq!(stats.bound_tests, 512);
        assert_eq!(stats.exact_candidates, 1);
        assert_eq!(stats.current_queries, 1);
        assert_eq!(stats.output_capacity_growths, 0);
        assert_eq!(hits.capacity(), capacity);
        assert_eq!(hits.len(), 1);
        assert_eq!(hits[0].body, BodyId(0));
    }
}

#[test]
#[ignore = "explicit paired query work/timing evidence; semantic reference controls run normally"]
fn paired_query_workload_evidence() {
    use std::time::Instant;
    for case in ["sparse-overlap", "sparse-cast", "miss"] {
        let mut world = World::new(super::super::Config {
            gravity: V::ZERO,
            ..Default::default()
        })
        .unwrap();
        for id in 0..512 {
            world
                .add_body(Body::new(
                    BodyId(id),
                    shape(id),
                    V(id as Real * 10.0, 0.0, 0.0),
                    0.0,
                ))
                .unwrap();
        }
        for index in 0..4 {
            let pose = QueryPose::new(
                shape(index),
                match case {
                    "sparse-cast" => V(-5.0, 0.0, 0.0),
                    "miss" => V(0.0, 100.0, 0.0),
                    _ => V::ZERO,
                },
            );
            let cast = (case == "sparse-cast").then(|| ShapeCast::new(pose, V(10.0, 0.0, 0.0)));
            let mut hits = Vec::new();
            let mut reference = Vec::new();
            world
                .shape_query(pose, cast, QueryFilter::default(), &mut hits, true)
                .unwrap();
            world
                .shape_query(pose, cast, QueryFilter::default(), &mut reference, false)
                .unwrap();
            assert_eq!(hits, reference);
            for trial in 0..3 {
                for prune in [false, true] {
                    let out = if prune { &mut hits } else { &mut reference };
                    let start = Instant::now();
                    let mut last = QueryStats::default();
                    for _ in 0..256 {
                        last = world
                            .shape_query(pose, cast, QueryFilter::default(), out, prune)
                            .unwrap();
                        assert_eq!(last.output_capacity_growths, 0);
                    }
                    let elapsed = start.elapsed().as_secs_f64() * 1000.0;
                    println!(
                        "QUERY_EVIDENCE {{\"case\":\"{case}\",\"shape\":{index},\"trial\":{trial},\"pruned\":{prune},\"queries\":256,\"ms\":{elapsed},\"visited\":{},\"exact\":{},\"primitive_queries\":{},\"iterations\":{},\"axes\":{},\"output_bytes\":{}}}",
                        last.bodies_visited,
                        last.exact_candidates,
                        last.primitive_queries,
                        last.sweep_iterations,
                        last.axes_tested,
                        last.output_capacity_bytes
                    );
                }
                assert_eq!(hits, reference);
            }
        }
    }
}
