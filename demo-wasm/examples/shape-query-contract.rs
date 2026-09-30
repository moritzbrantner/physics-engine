//! Native/WASM query acceptance; never linked into the production Pages module.
use physics_engine::{
    BodyId,
    approximate::{
        Body, CheckpointContext, Config, QueryFailureReason, QueryFilter, QueryPose, Shape,
        ShapeCast, SweepFailure, Vector as V, World,
    },
};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
fn run() -> Result<()> {
    let mut world = World::new(Config {
        gravity: V::ZERO,
        ..Default::default()
    })?;
    world.add_body(Body::new(
        BodyId(7),
        Shape::Box(V(0.01, 2.0, 2.0)),
        V::ZERO,
        0.0,
    ))?;
    let mut moving = Body::new(BodyId(8), Shape::Sphere(0.5), V(3.0, 0.0, 0.0), 1.0);
    moving.velocity = V(0.0, 1.0, 0.0);
    world.add_body(moving)?;
    world.add_force(BodyId(8), V::X)?;
    let context = CheckpointContext {
        build: [94; 32],
        content: [19; 32],
    };
    let physical = world.checkpoint(context)?.to_bytes();
    let mut hits = Vec::new();
    for shape in [
        Shape::Sphere(0.5),
        Shape::Box(V(0.5, 0.5, 0.5)),
        Shape::capsule(0.5, 0.5),
        Shape::wedge(V(0.5, 0.5, 0.5)),
    ] {
        let pose = QueryPose::new(shape, V(-10.0, 0.0, 0.0));
        let cast = ShapeCast::new(pose, V(20.0, 0.0, 0.0));
        world.cast_shape(cast, QueryFilter::default(), &mut hits)?;
        assert_eq!(hits.len(), 2);
        assert_eq!(hits[0].body, BodyId(7));
        assert!((hits[0].fraction - 0.4745).abs() < 1e-8);
        assert!((hits[0].normal + V::X).length() < 1e-8);
        assert_eq!(hits[1].body, BodyId(8));
        assert_eq!(hits[1].support_velocity, V::Y);
        let expected = hits.clone();
        let capacity = hits.capacity();
        for _ in 0..16 {
            let work = world.cast_shape(cast, QueryFilter::default(), &mut hits)?;
            assert_eq!(hits, expected);
            assert_eq!(hits.capacity(), capacity);
            assert_eq!(work.output_capacity_growths, 0);
            assert_eq!(work.exact_candidates, 2);
        }
    }
    let pose = QueryPose::new(Shape::Sphere(0.5), V(4.0, 0.0, 0.0));
    world.overlap_shape(pose, QueryFilter::default(), &mut hits)?;
    assert_eq!(hits.len(), 1);
    assert!(!hits[0].starts_overlapping);
    assert_eq!(hits[0].fraction, 0.0);
    world.overlap_shape(
        QueryPose::new(Shape::Sphere(0.5), V(3.0, 0.0, 0.0)),
        QueryFilter::default(),
        &mut hits,
    )?;
    assert!(hits[0].starts_overlapping);
    assert_eq!(world.checkpoint(context)?.to_bytes(), physical);

    let mut limited = World::new(Config {
        gravity: V::ZERO,
        ..Default::default()
    })?;
    for (id, x) in [(1, -4.0), (2, 0.0)] {
        limited.add_body(Body::new(
            BodyId(id),
            Shape::Sphere(0.5),
            V(x, 0.0, 0.0),
            0.0,
        ))?;
    }
    let mut cast = ShapeCast::new(
        QueryPose::new(Shape::Sphere(0.5), V(-4.0, 0.0, 0.0)),
        V(8.0, 0.0, 0.0),
    );
    cast.max_iterations = 1;
    let failed = limited
        .cast_shape(cast, QueryFilter::default(), &mut hits)
        .unwrap_err();
    assert_eq!(
        failed.reason,
        QueryFailureReason::Search {
            body: BodyId(2),
            reason: SweepFailure::IterationLimit
        }
    );
    assert_eq!(failed.stats.hits_admitted, 1);
    assert!(hits.is_empty());
    cast.max_iterations = 128;
    cast.displacement = V(f64::NAN, 0.0, 0.0);
    assert_eq!(
        limited
            .cast_shape(cast, QueryFilter::default(), &mut hits)
            .unwrap_err()
            .reason,
        QueryFailureReason::InvalidInput
    );
    assert!(hits.is_empty());
    Ok(())
}

#[unsafe(no_mangle)]
pub extern "C" fn shape_query_contract() -> i32 {
    run().map_or(-1, |()| 0)
}
#[cfg(test)]
mod tests {
    #[test]
    fn native_shape_queries_cross_the_real_public_api() {
        super::run().unwrap();
        super::run().unwrap();
    }
}
