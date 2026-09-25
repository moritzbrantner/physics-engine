//! Physics-engine adapter for reusable f64 primitive collision kernels.
//!
//! Reusable shape math lives in `geometry-kernels`. This module retains engine-local body,
//! quaternion and work-counter boundaries; simulation policy remains owned by Physics Engine.

use super::{Body, Shape, Vector as V, geometry::GeometryStats};
use geometry_kernels::primitive3::{
    PrimitiveBody3, PrimitiveContact3, PrimitiveShape3, PrimitiveWork3,
    bounds_extents as kernel_bounds, query_canonical as kernel_query_canonical,
    support_point as kernel_support_point,
    swept_time as kernel_swept_time,
};

pub(super) use geometry_kernels::primitive3::PrimitivePair3 as PrimitivePair;
#[cfg(test)]
pub(super) use geometry_kernels::primitive3::PrimitiveKind3 as PrimitiveKind;
#[cfg(test)]
use geometry_kernels::primitive3::query as kernel_query;

#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct PrimitiveContact {
    pub normal: V,
    pub separation: f64,
    pub point_a: V,
    pub point_b: V,
}

pub(super) fn canonical_pair(left: Shape, right: Shape) -> (PrimitivePair, bool) {
    PrimitivePair::canonical(shape(left).kind(), shape(right).kind())
}

pub(super) fn bounds_extents(body: &Body) -> V {
    from_array(kernel_bounds(kernel_body(body)))
}

pub(super) fn support_point_counted(body: &Body, direction: V, work: &mut GeometryStats) -> V {
    let mut kernel_work = PrimitiveWork3::default();
    let point = kernel_support_point(kernel_body(body), to_array(direction), &mut kernel_work);
    accumulate_work(work, kernel_work);
    from_array(point)
}

#[cfg(test)]
pub(super) fn query(a: &Body, b: &Body, work: &mut GeometryStats) -> Option<PrimitiveContact> {
    let (pair, reversed) = canonical_pair(a.shape, b.shape);
    let (left, right) = if reversed { (b, a) } else { (a, b) };
    let contact = query_canonical(pair, left, right, work)?;
    Some(if reversed { flip(contact) } else { contact })
}

pub(super) fn query_canonical(
    pair: PrimitivePair,
    a: &Body,
    b: &Body,
    work: &mut GeometryStats,
) -> Option<PrimitiveContact> {
    work.primitive_queries += 1;
    let mut kernel_work = PrimitiveWork3::default();
    let contact = kernel_query_canonical(pair, kernel_body(a), kernel_body(b), &mut kernel_work);
    accumulate_work(work, kernel_work);
    Some(from_contact(contact))
}

#[cfg(test)]
pub(super) fn reference_query(
    a: &Body,
    b: &Body,
    work: &mut GeometryStats,
) -> Option<PrimitiveContact> {
    if matches!(
        canonical_pair(a.shape, b.shape).0,
        PrimitivePair::SphereSphere | PrimitivePair::SphereBox | PrimitivePair::BoxBox
    ) {
        return None;
    }

    let mut kernel_work = PrimitiveWork3::default();
    let contact = kernel_query(kernel_body(a), kernel_body(b), &mut kernel_work);
    accumulate_work(work, kernel_work);
    Some(from_contact(contact))
}

pub(super) fn swept_time(
    a: &Body,
    b: &Body,
    dt: f64,
    margin: f64,
    work: &mut GeometryStats,
) -> Option<f64> {
    let mut kernel_work = PrimitiveWork3::default();
    let result = kernel_swept_time(kernel_body(a), kernel_body(b), dt, margin, &mut kernel_work);
    accumulate_work(work, kernel_work);
    result
}

fn shape(shape: Shape) -> PrimitiveShape3 {
    match shape {
        Shape::Sphere(radius) => PrimitiveShape3::sphere(radius),
        Shape::Box(half) => PrimitiveShape3::cuboid(to_array(half)),
        Shape::Capsule {
            half_segment,
            radius,
        } => PrimitiveShape3::capsule(half_segment, radius),
        Shape::Wedge(half) => PrimitiveShape3::wedge(to_array(half)),
    }
}

fn kernel_body(body: &Body) -> PrimitiveBody3 {
    let axes = match body.shape {
        Shape::Sphere(_) => identity_axes(),
        Shape::Capsule { .. } => [
            [1.0, 0.0, 0.0],
            to_array(body.orientation.rotate(V::Y)),
            [0.0, 0.0, 1.0],
        ],
        Shape::Box(_) | Shape::Wedge(_) => body.orientation.axes().map(to_array),
    };
    PrimitiveBody3::new(
        shape(body.shape),
        to_array(body.position),
        axes,
        to_array(body.velocity),
    )
}

const fn identity_axes() -> [[f64; 3]; 3] {
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
}

const fn to_array(value: V) -> [f64; 3] {
    [value.0, value.1, value.2]
}

const fn from_array(value: [f64; 3]) -> V {
    V(value[0], value[1], value[2])
}

fn from_contact(contact: PrimitiveContact3) -> PrimitiveContact {
    PrimitiveContact {
        normal: from_array(contact.normal),
        separation: contact.separation,
        point_a: from_array(contact.point_a),
        point_b: from_array(contact.point_b),
    }
}

#[cfg(test)]
fn flip(contact: PrimitiveContact) -> PrimitiveContact {
    PrimitiveContact {
        normal: -contact.normal,
        separation: contact.separation,
        point_a: contact.point_b,
        point_b: contact.point_a,
    }
}

fn accumulate_work(work: &mut GeometryStats, kernel: PrimitiveWork3) {
    work.support_evaluations += kernel.support_evaluations;
    work.primitive_axes_tested += kernel.axes_tested;
    work.primitive_vertex_tests += kernel.vertex_tests;
    work.primitive_sweep_iterations += kernel.sweep_iterations;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::approximate::Quaternion;

    fn body(id: u64, shape: Shape, position: V) -> Body {
        Body::new(crate::BodyId(id), shape, position, 0.0)
    }

    #[test]
    fn adapter_preserves_rotated_capsule_wedge_contact_and_work() {
        let wedge = body(1, Shape::wedge(V(2.0, 2.0, 2.0)), V::ZERO);
        let mut capsule = body(2, Shape::capsule(1.5, 0.4), V(0.0, 0.5, 0.0));
        capsule.orientation = Quaternion(
            0.0,
            0.0,
            std::f64::consts::FRAC_1_SQRT_2,
            std::f64::consts::FRAC_1_SQRT_2,
        );
        let mut work = GeometryStats::default();
        let forward = query(&capsule, &wedge, &mut work).unwrap();
        let reverse = query(&wedge, &capsule, &mut work).unwrap();
        assert!((forward.separation - reverse.separation).abs() < 1e-12);
        assert!((forward.normal + reverse.normal).length() < 1e-12);
        assert!(work.primitive_queries >= 2);
    }

    #[test]
    fn fast_capsule_shared_sweep_reaches_thin_wedge() {
        let mut capsule = Body::new(
            crate::BodyId(1),
            Shape::capsule(0.5, 0.1),
            V(-1.0, -1.0, 10.0),
            1.0,
        );
        capsule.velocity = V(0.0, 0.0, -10_000.0);
        let wedge = body(2, Shape::wedge(V(4.0, 4.0, 0.02)), V::ZERO);
        let mut work = GeometryStats::default();
        let time = swept_time(&capsule, &wedge, 1.0 / 60.0, 0.02, &mut work).unwrap();
        assert!((0.0..=1.0).contains(&time));
        assert!(work.primitive_sweep_iterations > 0);
        assert!(work.primitive_sweep_iterations < 128);
    }

    #[test]
    #[ignore = "advisory timing; deterministic work counters are tested separately"]
    fn capsule_and_wedge_query_benchmark() {
        use std::{hint::black_box, time::Instant};
        let capsule = body(1, Shape::capsule(2.0, 0.5), V::ZERO);
        let wedge = body(2, Shape::wedge(V(2.0, 2.0, 2.0)), V(1.0, 0.5, 0.0));
        let started = Instant::now();
        let mut work = GeometryStats::default();
        for _ in 0..100_000 {
            black_box(query(&capsule, &wedge, &mut work));
        }
        eprintln!(
            "primitive capsule-wedge: elapsed={:?} queries={} axes={} wedge_vertex_tests={}",
            started.elapsed(),
            work.primitive_queries,
            work.primitive_axes_tested,
            work.primitive_vertex_tests
        );
    }
}
