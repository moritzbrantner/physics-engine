//! Public capsule query normals must separate the solids by their reported depth.
use physics_engine::approximate::Real;
use physics_engine::{
    BodyId,
    approximate::{
        Body, CheckpointContext, Config, Quaternion, QueryFilter, QueryPose, Shape, Vector as V,
        World,
    },
};

// Independent capsule/sphere support formula, without contact-kernel helpers.
fn support(pose: QueryPose, direction: V) -> V {
    let (half, radius) = match pose.shape {
        Shape::Sphere(radius) => (0.0, radius),
        Shape::Capsule {
            half_segment,
            radius,
        } => (half_segment, radius),
        _ => panic!("capsule/sphere fixture only"),
    };
    let axis = pose.orientation.rotate(V::Y);
    let endpoint = if axis.dot(direction) >= 0.0 {
        half
    } else {
        -half
    };
    pose.position + axis * endpoint + direction * (radius / direction.length())
}

fn certify_query_clearance(target: QueryPose, mut query: QueryPose, scale: Real) {
    let mut world = World::new(Config {
        gravity: V::ZERO,
        ..Config::default()
    })
    .unwrap();
    let mut body = Body::new(BodyId(17), target.shape, target.position, 1.0);
    body.orientation = target.orientation;
    world.add_body(body).unwrap();
    world.add_force(BodyId(17), V::X).unwrap();
    let context = CheckpointContext {
        build: [173; 32],
        content: [17; 32],
    };
    let before = world.checkpoint(context).unwrap().to_bytes();
    let mut hits = Vec::new();
    world
        .overlap_shape(query, QueryFilter::default(), &mut hits)
        .unwrap();
    assert_eq!(hits.len(), 1);
    let hit = hits[0];
    assert!(hit.starts_overlapping && hit.separation < 0.0);
    assert!((hit.normal.length() - 1.0).abs() <= 64.0 * Real::EPSILON);
    assert!(
        ((hit.query_point - hit.target_point).dot(hit.normal) - hit.separation).abs()
            <= 128.0 * Real::EPSILON * scale
    );
    query.position += hit.normal * (-hit.separation + scale * 1e-5);
    let gap = (support(query, -hit.normal) - support(target, hit.normal)).dot(hit.normal);
    assert!(
        gap > scale * 0.99e-5,
        "reported clearance left support-plane gap {gap}"
    );
    world
        .overlap_shape(query, QueryFilter::default(), &mut hits)
        .unwrap();
    assert!(hits.is_empty());
    assert_eq!(world.checkpoint(context).unwrap().to_bytes(), before);
}

#[cfg_attr(test, test)]
pub fn sphere_on_capsule_skeleton_has_a_certified_public_clearance() {
    for scale in [1e-4, 1.0, 1e6] {
        let target = QueryPose::new(Shape::capsule(5.0 * scale, 0.5 * scale), V::ZERO);
        let query = QueryPose::new(Shape::Sphere(0.25 * scale), V(0.0, 2.0 * scale, 0.0));
        certify_query_clearance(target, query, scale);
        certify_query_clearance(query, target, scale);
    }
}

#[cfg_attr(test, test)]
pub fn crossing_capsules_have_a_certified_public_clearance() {
    let quarter_turn = Quaternion(
        0.0,
        0.0,
        std::f64::consts::FRAC_1_SQRT_2 as Real,
        std::f64::consts::FRAC_1_SQRT_2 as Real,
    );
    for scale in [1e-4, 1.0, 1e6] {
        let position = V(-13.0 * scale, 7.0 * scale, -2.0 * scale);
        let a = QueryPose::new(Shape::capsule(5.0 * scale, 0.5 * scale), position);
        let b = QueryPose {
            orientation: quarter_turn,
            ..a
        };
        certify_query_clearance(a, b, scale);
        certify_query_clearance(b, a, scale);
    }
}

#[cfg_attr(test, test)]
pub fn collapsed_capsules_retain_the_sphere_clearance_contract() {
    for scale in [1e-4, 1.0, 1e6] {
        let a = QueryPose::new(Shape::capsule(5.0 * scale, 0.5 * scale), V::ZERO);
        let b = QueryPose::new(Shape::capsule(0.0, 0.25 * scale), V(0.0, 2.0 * scale, 0.0));
        certify_query_clearance(a, b, scale);
        certify_query_clearance(b, a, scale);
    }
}
