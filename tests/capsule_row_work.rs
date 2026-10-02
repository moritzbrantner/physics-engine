//! Capsule row work must reach public world/query reports on native and WASM.
use physics_engine::approximate::Real;
use physics_engine::{
    BodyId,
    approximate::{
        Body, CheckpointContext, Config, Error, QueryFailureReason, QueryFilter, QueryPose, Shape,
        ShapeCast, SweepFailure, Vector as V, World,
    },
};

/// Rounding-only bound: the f64 reference, or 16 ulps of `scale` when larger (f32 state).
fn rounding(reference: Real, scale: Real) -> Real {
    reference.max(16.0 * Real::EPSILON * scale)
}

fn world(shape: Shape) -> World {
    let mut world = World::new(Config {
        gravity: V::ZERO,
        ..Config::default()
    })
    .unwrap();
    world
        .add_body(Body::new(BodyId(17), shape, V::ZERO, 1.0))
        .unwrap();
    world.add_force(BodyId(17), V::X).unwrap();
    world
}
fn context() -> CheckpointContext {
    CheckpointContext {
        build: [173; 32],
        content: [18; 32],
    }
}

#[cfg_attr(test, test)]
pub fn capsule_row_public_queries_report_actual_distance_and_feature_work() {
    for shape in [
        Shape::Sphere(0.5),
        Shape::Box(V(0.5, 0.5, 0.5)),
        Shape::capsule(1.0, 0.5),
    ] {
        let world = world(shape);
        let before = world.checkpoint(context()).unwrap().to_bytes();
        let mut hits = Vec::new();
        let overlap = QueryPose::new(Shape::capsule(2.0, 0.25), V(0.6, 0.0, 0.0));
        world
            .overlap_shape(overlap, QueryFilter::default(), &mut hits)
            .unwrap();
        assert_eq!(hits.len(), 1);
        let expected = hits.clone();
        for _ in 0..32 {
            let stats = world
                .overlap_shape(overlap, QueryFilter::default(), &mut hits)
                .unwrap();
            assert_eq!(hits, expected);
            assert_eq!(stats.output_capacity_growths, 0);
            assert_eq!(stats.primitive_queries, 1);
            assert_eq!(stats.segment_distance_evaluations, 1);
            assert!(stats.segment_feature_tests > 0 && stats.segment_feature_tests <= 46);
            assert_eq!(stats.support_evaluations, 0);
            assert_eq!(stats.vertex_tests, 0);
        }
        let cast = ShapeCast::new(
            QueryPose::new(Shape::capsule(0.5, 0.2), V(-10.0, 0.0, 0.0)),
            V(20.0, 0.0, 0.0),
        );
        let stats = world
            .cast_shape(cast, QueryFilter::default(), &mut hits)
            .unwrap();
        assert_eq!(hits.len(), 1);
        // All three shapes share a 0.5 X core/radius extent in this control.
        assert!((hits[0].fraction - 0.465).abs() < rounding(1e-8, 4.0));
        assert!((hits[0].normal + V::X).length() < rounding(1e-8, 1.0));
        assert!(stats.sweep_iterations > 0);
        assert_eq!(stats.segment_distance_evaluations, stats.primitive_queries);
        assert!(stats.segment_feature_tests <= 46 * stats.primitive_queries);
        assert_eq!(world.checkpoint(context()).unwrap().to_bytes(), before);
    }
}

#[cfg_attr(test, test)]
pub fn failed_capsule_search_retains_discarded_distance_work() {
    let world = world(Shape::Sphere(0.5));
    let before = world.checkpoint(context()).unwrap().to_bytes();
    let mut cast = ShapeCast::new(
        QueryPose::new(Shape::capsule(0.5, 0.2), V(-10.0, 0.0, 0.0)),
        V(20.0, 0.0, 0.0),
    );
    cast.max_iterations = 1;
    let mut hits = Vec::new();
    world
        .overlap_shape(
            QueryPose::new(Shape::Sphere(0.5), V::ZERO),
            QueryFilter::default(),
            &mut hits,
        )
        .unwrap();
    assert_eq!(hits.len(), 1);
    let failure = world
        .cast_shape(cast, QueryFilter::default(), &mut hits)
        .unwrap_err();
    assert_eq!(
        failure.reason,
        QueryFailureReason::Search {
            body: BodyId(17),
            reason: SweepFailure::IterationLimit
        }
    );
    assert!(hits.is_empty());
    assert_eq!(failure.stats.sweep_iterations, 1);
    assert_eq!(failure.stats.primitive_queries, 2);
    assert_eq!(failure.stats.segment_distance_evaluations, 2);
    assert_eq!(failure.stats.segment_feature_tests, 2);
    assert_eq!(failure.stats.hits_admitted, 0);
    assert_eq!(world.checkpoint(context()).unwrap().to_bytes(), before);
}

#[cfg_attr(test, test)]
pub fn capsule_contacts_report_distance_work_through_physical_steps() {
    for shape in [
        Shape::Sphere(0.5),
        Shape::Box(V(0.5, 0.5, 0.5)),
        Shape::capsule(1.0, 0.5),
    ] {
        let mut world = World::new(Config {
            gravity: V::ZERO,
            ..Config::default()
        })
        .unwrap();
        world
            .add_body(Body::new(BodyId(1), shape, V::ZERO, 0.0))
            .unwrap();
        let mut moving = Body::new(BodyId(2), Shape::capsule(0.5, 0.25), V(0.74, 0.0, 0.0), 1.0);
        moving.rotation_locked = true;
        world.add_body(moving).unwrap();
        let report = world.step(1.0 / 60.0).unwrap();
        assert!(report.geometry.primitive_queries > 0);
        assert!(report.geometry.primitive_segment_distance_evaluations > 0);
        assert!(report.geometry.primitive_segment_feature_tests > 0);
        assert!(
            report.geometry.primitive_segment_distance_evaluations
                <= 6 * report.geometry.primitive_queries
        );
        assert!(
            report.geometry.primitive_segment_feature_tests
                <= 46 * report.geometry.primitive_queries
        );
        assert_eq!(report.geometry.generic_fallback_calls, 0);
        assert_eq!(report.geometry.primitive_vertex_tests, 0);
        assert_eq!(report.geometry.support_evaluations, 0);
        assert_eq!(world.elapsed_seconds(), 1.0 / 60.0);
    }
}

#[cfg_attr(test, test)]
pub fn failed_capsule_step_retains_discarded_distance_work() {
    let mut world = World::new(Config {
        gravity: V::ZERO,
        substeps: 1,
        ..Config::default()
    })
    .unwrap();
    world
        .add_body(Body::new(BodyId(1), Shape::Sphere(1.0), V::ZERO, 0.0))
        .unwrap();
    world
        .add_body(Body::new(
            BodyId(2),
            Shape::capsule(0.5, 1.0),
            V(-4.0, 0.0, 0.0),
            0.1,
        ))
        .unwrap();
    world.step(0.01).unwrap();
    world.add_force(BodyId(2), V(Real::MAX, 0.0, 0.0)).unwrap();
    let before = world.checkpoint(context()).unwrap().to_bytes();
    let prior_report = format!("{:?}", world.last_report);
    for _ in 0..3 {
        assert_eq!(
            world.step(0.1).unwrap_err(),
            Error::CollisionSearchFailed {
                bodies: [BodyId(1), BodyId(2)],
                reason: SweepFailure::InvalidGeometryInput,
            }
        );
        assert_eq!(world.checkpoint(context()).unwrap().to_bytes(), before);
        assert_eq!(format!("{:?}", world.last_report), prior_report);
        let work = world.last_failed_step_work().unwrap();
        assert!(work.geometry.primitive_segment_distance_evaluations > 0);
        assert!(work.geometry.primitive_segment_feature_tests > 0);
        assert_eq!(work.geometry.primitive_sweep_failures, 1);
        assert_eq!(work.integration_attempts, 0);
        assert_eq!(work.contact_points, 0);
        assert!(work.transaction.rolled_back);
    }
}
