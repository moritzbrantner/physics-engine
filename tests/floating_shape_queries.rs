use physics_engine::approximate::Real;
use physics_engine::{
    BodyId, CollisionLayers3d,
    approximate::{
        Body, Config, QueryFailureReason, QueryFilter, QueryPose, Shape, ShapeCast, Vector as V,
        World,
    },
};

fn world() -> World {
    World::new(Config {
        gravity: V::ZERO,
        ..Config::default()
    })
    .unwrap()
}
fn sphere(position: V) -> QueryPose {
    QueryPose::new(Shape::Sphere(0.5), position)
}

#[test]
fn sphere_cast_hits_a_thin_wall_across_the_full_interval_with_outward_normal() {
    let mut world = world();
    world
        .add_body(Body::new(
            BodyId(7),
            Shape::Box(V(0.01, 2.0, 2.0)),
            V::ZERO,
            0.0,
        ))
        .unwrap();
    let mut hits = Vec::new();
    let stats = world
        .cast_shape(
            ShapeCast::new(sphere(V(-10.0, 0.0, 0.0)), V(20.0, 0.0, 0.0)),
            QueryFilter::default(),
            &mut hits,
        )
        .unwrap();
    assert_eq!(hits.len(), 1);
    let hit = hits[0];
    assert_eq!(hit.body, BodyId(7));
    assert!((hit.fraction - 0.4745).abs() < 1e-8);
    assert!((hit.distance - 9.49).abs() < 1e-7);
    assert!((hit.normal + V::X).length() < 1e-12);
    assert!((hit.target_point.0 + 0.01).abs() < 1e-8);
    assert!(!hit.starts_overlapping);
    assert_eq!(stats.bodies_visited, 1);
    assert_eq!(stats.exact_candidates, 1);
}

#[test]
fn initial_overlap_touching_and_zero_motion_are_explicit_and_queries_do_not_step() {
    let mut world = world();
    world
        .add_body(Body::new(BodyId(3), Shape::Sphere(0.5), V::ZERO, 1.0))
        .unwrap();
    world.add_force(BodyId(3), V(2.0, 0.0, 0.0)).unwrap();
    let before = world.bodies().cloned().collect::<Vec<_>>();
    let mut hits = Vec::new();
    world
        .overlap_shape(sphere(V(0.75, 0.0, 0.0)), QueryFilter::default(), &mut hits)
        .unwrap();
    assert_eq!(hits.len(), 1);
    assert!(hits[0].starts_overlapping);
    assert_eq!(hits[0].separation, -0.25);
    world
        .cast_shape(
            ShapeCast::new(sphere(V::X), V::ZERO),
            QueryFilter::default(),
            &mut hits,
        )
        .unwrap();
    assert_eq!(hits.len(), 1);
    assert_eq!(hits[0].fraction, 0.0);
    assert!(!hits[0].starts_overlapping);
    assert_eq!(world.elapsed_seconds(), 0.0);
    assert_eq!(world.bodies().cloned().collect::<Vec<_>>(), before);
}

#[test]
fn ties_filter_sensors_and_same_id_replacement_use_current_authority() {
    let mut world = world();
    for id in [9, 2] {
        let mut body = Body::new(BodyId(id), Shape::Sphere(0.5), V::ZERO, 0.0);
        body.layers = CollisionLayers3d::new(2, 1);
        body.sensor = id == 9;
        world.add_body(body).unwrap();
    }
    let mut hits = Vec::new();
    let filter = QueryFilter {
        layers: CollisionLayers3d::new(1, 2),
        include_sensors: true,
        ..QueryFilter::default()
    };
    let cast = ShapeCast::new(sphere(V(-4.0, 0.0, 0.0)), V(8.0, 0.0, 0.0));
    world.cast_shape(cast, filter, &mut hits).unwrap();
    assert_eq!(
        hits.iter().map(|hit| hit.body).collect::<Vec<_>>(),
        [BodyId(2), BodyId(9)]
    );
    world
        .cast_shape(
            cast,
            QueryFilter {
                exclude: Some(BodyId(2)),
                ..filter
            },
            &mut hits,
        )
        .unwrap();
    assert_eq!(hits.len(), 1);
    assert!(hits[0].sensor);
    world
        .cast_shape(
            cast,
            QueryFilter {
                include_sensors: false,
                ..filter
            },
            &mut hits,
        )
        .unwrap();
    assert_eq!(hits.len(), 1);
    assert_eq!(hits[0].body, BodyId(2));
    world
        .cast_shape(
            cast,
            QueryFilter {
                layers: CollisionLayers3d::new(4, 4),
                ..filter
            },
            &mut hits,
        )
        .unwrap();
    assert!(hits.is_empty());
    world.remove_body(BodyId(2)).unwrap();
    world
        .add_body(Body::new(
            BodyId(2),
            Shape::Sphere(0.5),
            V(40.0, 0.0, 0.0),
            0.0,
        ))
        .unwrap();
    world
        .cast_shape(cast, QueryFilter::default(), &mut hits)
        .unwrap();
    assert!(hits.is_empty());
}

#[test]
fn failed_search_clears_partial_hits_and_reports_discarded_work() {
    let mut world = world();
    world
        .add_body(Body::new(
            BodyId(1),
            Shape::Sphere(0.5),
            V(-4.0, 0.0, 0.0),
            0.0,
        ))
        .unwrap();
    world
        .add_body(Body::new(BodyId(2), Shape::Sphere(0.5), V::ZERO, 0.0))
        .unwrap();
    let mut cast = ShapeCast::new(sphere(V(-4.0, 0.0, 0.0)), V(8.0, 0.0, 0.0));
    cast.max_iterations = 1;
    let mut hits = Vec::new();
    let failure = world
        .cast_shape(cast, QueryFilter::default(), &mut hits)
        .unwrap_err();
    assert_eq!(
        failure.reason,
        QueryFailureReason::Search {
            body: BodyId(2),
            reason: physics_engine::approximate::SweepFailure::IterationLimit
        }
    );
    assert!(hits.is_empty());
    assert_eq!(failure.stats.bodies_visited, 2);
    assert_eq!(failure.stats.exact_candidates, 2);
    assert_eq!(failure.stats.sweep_iterations, 1);
    assert_eq!(failure.stats.hits_admitted, 1);
    assert!(failure.stats.current_queries >= 2);
}

#[test]
fn all_supported_query_shapes_have_thin_obstacle_sweeps_and_rotated_capsule_bounds() {
    let shapes = [
        Shape::Sphere(0.2),
        Shape::Box(V(0.2, 0.3, 0.2)),
        Shape::capsule(0.5, 0.2),
        Shape::wedge(V(0.2, 0.3, 0.2)),
    ];
    for shape in shapes {
        let mut world = world();
        world
            .add_body(Body::new(
                BodyId(1),
                Shape::wedge(V(2.0, 2.0, 0.01)),
                V::ZERO,
                0.0,
            ))
            .unwrap();
        let pose = QueryPose::new(shape, V(-0.5, -0.5, 10.0));
        let mut hits = Vec::new();
        world
            .cast_shape(
                ShapeCast::new(pose, V(0.0, 0.0, -20.0)),
                QueryFilter::default(),
                &mut hits,
            )
            .unwrap();
        assert_eq!(hits.len(), 1, "{shape:?}");
        assert!(hits[0].fraction > 0.0 && hits[0].fraction < 0.5);
        assert!((hits[0].normal - V::Z).length() < 1e-8);
        assert!((hits[0].target_point.2 - 0.01).abs() < 1e-8);
        assert!(hits[0].separation.abs() < 1e-7);
    }
    let mut world = world();
    world
        .add_body(Body::new(
            BodyId(1),
            Shape::Sphere(0.2),
            V(1.0, 0.0, 0.0),
            0.0,
        ))
        .unwrap();
    let mut pose = QueryPose::new(Shape::capsule(1.0, 0.2), V::ZERO);
    pose.orientation = physics_engine::approximate::Quaternion(
        0.0,
        0.0,
        std::f64::consts::FRAC_1_SQRT_2 as Real,
        std::f64::consts::FRAC_1_SQRT_2 as Real,
    );
    let mut hits = Vec::new();
    world
        .overlap_shape(pose, QueryFilter::default(), &mut hits)
        .unwrap();
    assert_eq!(hits.len(), 1);
    assert!(hits[0].starts_overlapping);
}

#[test]
fn ramp_normal_and_support_velocity_come_from_current_physical_geometry() {
    let mut ramp_world = world();
    ramp_world
        .add_body(Body::new(
            BodyId(1),
            Shape::wedge(V(2.0, 2.0, 2.0)),
            V::ZERO,
            0.0,
        ))
        .unwrap();
    let mut hits = Vec::new();
    ramp_world
        .cast_shape(
            ShapeCast::new(
                QueryPose::new(Shape::Sphere(0.2), V(0.0, 3.0, 0.0)),
                V(0.0, -6.0, 0.0),
            ),
            QueryFilter::default(),
            &mut hits,
        )
        .unwrap();
    assert_eq!(hits.len(), 1);
    let normal = V(1.0, 1.0, 0.0) / (2.0 as Real).sqrt();
    assert!((hits[0].normal - normal).length() < 1e-8);
    assert!((hits[0].fraction - (3.0 - 0.2 * (2.0 as Real).sqrt()) / 6.0).abs() < 1e-8);
    assert!((hits[0].target_point.0 + hits[0].target_point.1).abs() < 1e-8);

    let mut moving_world = world();
    let mut body = Body::new(BodyId(2), Shape::Sphere(1.0), V::ZERO, 1.0);
    body.velocity = V(3.0, 0.0, 0.0);
    body.angular_velocity = V(0.0, 0.0, 2.0);
    moving_world.add_body(body).unwrap();
    moving_world
        .cast_shape(
            ShapeCast::new(sphere(V(4.0, 0.0, 0.0)), V(-8.0, 0.0, 0.0)),
            QueryFilter::default(),
            &mut hits,
        )
        .unwrap();
    assert_eq!(hits.len(), 1);
    assert!((hits[0].fraction - 0.3125).abs() < 1e-8);
    assert!((hits[0].normal - V::X).length() < 1e-8);
    assert!((hits[0].support_velocity - V(3.0, 2.0, 0.0)).length() < 1e-8);
}

#[test]
fn invalid_inputs_clear_output_without_touching_pending_commands_or_contact_history() {
    use physics_engine::approximate::{CheckpointContext, Quaternion};
    let mut world = world();
    world
        .add_body(Body::new(BodyId(1), Shape::Sphere(0.5), V::ZERO, 1.0))
        .unwrap();
    world.add_force(BodyId(1), V::Y).unwrap();
    let context = CheckpointContext {
        build: [7; 32],
        content: [9; 32],
    };
    let before = world.checkpoint(context).unwrap().to_bytes();
    let mut hits = Vec::new();
    for pose in [
        QueryPose::new(Shape::Sphere(-1.0), V::ZERO),
        QueryPose::new(Shape::Sphere(0.5), V(Real::NAN, 0.0, 0.0)),
        QueryPose {
            orientation: Quaternion(0.0, 0.0, 0.0, 0.0),
            ..sphere(V::ZERO)
        },
    ] {
        world
            .overlap_shape(sphere(V::ZERO), QueryFilter::default(), &mut hits)
            .unwrap();
        assert!(!hits.is_empty());
        let failure = world
            .overlap_shape(pose, QueryFilter::default(), &mut hits)
            .unwrap_err();
        assert_eq!(failure.reason, QueryFailureReason::InvalidInput);
        assert_eq!(failure.stats.bodies_visited, 0);
        assert!(hits.is_empty());
    }
    for (displacement, budget) in [
        (V(Real::INFINITY, 0.0, 0.0), 128),
        (V(1e12, 0.0, 0.0), 128),
        (V::ZERO, 0),
        (V::ZERO, 129),
    ] {
        let mut cast = ShapeCast::new(sphere(V::ZERO), displacement);
        cast.max_iterations = budget;
        let failure = world
            .cast_shape(cast, QueryFilter::default(), &mut hits)
            .unwrap_err();
        assert_eq!(failure.reason, QueryFailureReason::InvalidInput);
        assert!(hits.is_empty());
    }
    assert_eq!(world.checkpoint(context).unwrap().to_bytes(), before);
}

#[test]
fn translated_scale_controls_preserve_hits_and_real_near_misses() {
    for scale in [1e-4, 1.0, 1e6] {
        let center = V(-100.0 * scale, 37.0 * scale, -12.0 * scale);
        let mut world = world();
        world
            .add_body(Body::new(
                BodyId(1),
                Shape::Sphere(0.5 * scale),
                center,
                0.0,
            ))
            .unwrap();
        let shape = Shape::capsule(0.5 * scale, 0.5 * scale);
        let pose = QueryPose::new(shape, center + V(-4.0 * scale, 0.0, 0.0));
        let mut hits = Vec::new();
        world
            .cast_shape(
                ShapeCast::new(pose, V(8.0 * scale, 0.0, 0.0)),
                QueryFilter::default(),
                &mut hits,
            )
            .unwrap();
        assert_eq!(hits.len(), 1);
        assert!((hits[0].fraction - 0.375).abs() < 1e-8);
        assert!((hits[0].normal + V::X).length() < 1e-8);
        let miss = QueryPose::new(shape, pose.position + V(0.0, 0.0, 1.01 * scale));
        world
            .cast_shape(
                ShapeCast::new(miss, V(8.0 * scale, 0.0, 0.0)),
                QueryFilter::default(),
                &mut hits,
            )
            .unwrap();
        assert!(hits.is_empty(), "near miss at scale {scale}");
    }
}
