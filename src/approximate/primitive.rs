//! Physics-engine adapter for reusable f64 primitive collision kernels.
//!
//! Reusable shape math lives in `geometry-kernels`. This module retains engine-local body,
//! quaternion and work-counter boundaries; simulation policy remains owned by Physics Engine.
//!
//! The kernels stay f64 in every build. [`Real`] inputs widen exactly to f64. Results narrow
//! conservatively: time of impact and sweep fractions toward zero (earlier), bounds/extents
//! outward, separation toward smaller; directions and witnesses round to nearest. In the
//! default f64 build every conversion is the identity.

use super::{
    Body, Quaternion, REAL_BITS, Real, Shape, SweepFailure, Vector as V, geometry::GeometryStats,
};
use geometry_kernels::primitive3::{
    PrimitiveBody3, PrimitiveContact3, PrimitiveShape3, PrimitiveSweepError3, PrimitiveWork3,
    bounds_extents as kernel_bounds, query_canonical as kernel_query_canonical,
    try_swept_time as kernel_swept_time,
};

#[cfg(test)]
pub(super) use geometry_kernels::primitive3::PrimitiveKind3 as PrimitiveKind;
pub(super) use geometry_kernels::primitive3::PrimitivePair3 as PrimitivePair;
use geometry_kernels::primitive3::query as kernel_query;

#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct PrimitiveContact {
    pub normal: V,
    pub separation: Real,
    pub point_a: V,
    pub point_b: V,
}

pub(super) fn canonical_pair(left: Shape, right: Shape) -> (PrimitivePair, bool) {
    PrimitivePair::canonical(shape(left).kind(), shape(right).kind())
}

pub(super) fn bounds_extents(body: &Body) -> V {
    let [x, y, z] = kernel_bounds(kernel_body(body));
    V(round_up(x), round_up(y), round_up(z))
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
    dt: Real,
    margin: Real,
    work: &mut GeometryStats,
) -> Result<Option<Real>, SweepFailure> {
    // The f64 kernel would absorb a relative motion that overflows `Real`, but the solver
    // consumes the impact in `Real`. With f32 state, fail the search as the f64 build's
    // kernel does for f64 overflow. Never evaluated in the f64 build.
    // Non-finite inputs stay the kernel's InvalidInput.
    if REAL_BITS < 64
        && a.velocity.finite()
        && b.velocity.finite()
        && dt.is_finite()
        && !((b.velocity - a.velocity) * dt).finite()
    {
        work.primitive_sweep_failures += 1;
        return Err(SweepFailure::NonFiniteComputation);
    }
    let mut kernel_work = PrimitiveWork3::default();
    let result = kernel_swept_time(
        kernel_body(a),
        kernel_body(b),
        widen(dt),
        widen(margin),
        128,
        &mut kernel_work,
    );
    accumulate_work(work, kernel_work);
    work.primitive_sweep_failures += u64::from(result.is_err());
    result
        .map(|time| time.map(toward_zero))
        .map_err(sweep_failure)
}

fn sweep_failure(error: PrimitiveSweepError3) -> SweepFailure {
    match error {
        PrimitiveSweepError3::InvalidInput => SweepFailure::InvalidGeometryInput,
        PrimitiveSweepError3::NonFiniteComputation => SweepFailure::NonFiniteComputation,
        PrimitiveSweepError3::IterationLimit => SweepFailure::IterationLimit,
    }
}

pub(super) fn snapshot_ray(
    target: &Body,
    origin: V,
    displacement: V,
    work: &mut geometry_kernels::primitive3::PrimitiveRayWork3,
) -> Result<Option<geometry_kernels::primitive3::PrimitiveRayHit3>, SweepFailure> {
    use geometry_kernels::primitive3::{PrimitiveRayError3, try_ray_cast};
    try_ray_cast(
        kernel_body(target),
        to_array(origin),
        to_array(displacement),
        work,
    )
    .map_err(|error| match error {
        PrimitiveRayError3::InvalidInput => SweepFailure::InvalidGeometryInput,
        PrimitiveRayError3::NonFiniteComputation => SweepFailure::NonFiniteComputation,
    })
}

/// Snapshot queries ignore target motion; physical velocities remain available to the caller.
pub(super) fn snapshot_query(a: &Body, b: &Body, work: &mut PrimitiveWork3) -> PrimitiveContact {
    from_contact(kernel_query(kernel_body(a), kernel_body(b), work))
}

pub(super) fn snapshot_sweep(
    query: &Body,
    target: &Body,
    displacement: V,
    max_iterations: u32,
    work: &mut PrimitiveWork3,
) -> Result<Option<Real>, SweepFailure> {
    let mut a = kernel_body(query);
    let mut b = kernel_body(target);
    a.velocity = to_array(displacement);
    b.velocity = [0.0; 3];
    kernel_swept_time(a, b, 1.0, 0.0, max_iterations, work)
        .map(|fraction| fraction.map(toward_zero))
        .map_err(sweep_failure)
}

pub(super) fn volume_properties(
    value: Shape,
) -> Result<
    geometry_kernels::primitive3::PrimitiveVolumeProperties3,
    geometry_kernels::primitive3::PrimitiveVolumeError3,
> {
    geometry_kernels::primitive3::try_volume_properties(shape(value))
}

fn shape(shape: Shape) -> PrimitiveShape3 {
    match shape {
        Shape::Sphere(radius) => PrimitiveShape3::sphere(widen(radius)),
        Shape::Box(half) => PrimitiveShape3::cuboid(to_array(half)),
        Shape::Capsule {
            half_segment,
            radius,
        } => PrimitiveShape3::capsule(widen(half_segment), widen(radius)),
        Shape::Wedge(half) => PrimitiveShape3::wedge(to_array(half)),
    }
}

fn kernel_body(body: &Body) -> PrimitiveBody3 {
    let axes = match body.shape {
        Shape::Sphere(_) => identity_axes(),
        Shape::Capsule { .. } => [
            [1.0, 0.0, 0.0],
            kernel_axes(body.orientation)[1],
            [0.0, 0.0, 1.0],
        ],
        Shape::Box(_) | Shape::Wedge(_) => kernel_axes(body.orientation),
    };
    // Force integration can produce a non-finite velocity before the final body check.
    // The checked sweep reports that failure; the constructor's debug assertion would panic.
    PrimitiveBody3 {
        shape: shape(body.shape),
        position: to_array(body.position),
        axes,
        velocity: to_array(body.velocity),
    }
}

/// Kernel frames must be orthonormal to f64 tolerances. The default build passes the solver's
/// own axes unchanged; the f32 build widens the stored quaternion, renormalizes it and
/// rotates in f64 so the frame is orthonormal at kernel precision.
fn kernel_axes(q: Quaternion) -> [[f64; 3]; 3] {
    if REAL_BITS == 64 {
        return q.axes().map(to_array);
    }
    let [x, y, z, w] = [q.0, q.1, q.2, q.3].map(widen);
    let length = (x * x + y * y + z * z + w * w).sqrt();
    let [x, y, z, w] = [x, y, z, w].map(|c| c / length);
    [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y + w * z),
            2.0 * (x * z - w * y),
        ],
        [
            2.0 * (x * y - w * z),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z + w * x),
        ],
        [
            2.0 * (x * z + w * y),
            2.0 * (y * z - w * x),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]
}

const fn identity_axes() -> [[f64; 3]; 3] {
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
}

/// Exact widening of a solver value to the f64 kernel domain.
#[allow(clippy::unnecessary_cast)]
pub(super) const fn widen(value: Real) -> f64 {
    value as f64
}

/// Round-to-nearest narrowing for directions and witness points.
#[allow(clippy::unnecessary_cast)]
pub(super) const fn nearest(value: f64) -> Real {
    value as Real
}

/// Largest `Real` not above `value` (exact in the f64 build; NaN stays NaN).
pub(super) fn round_down(value: f64) -> Real {
    let narrowed = nearest(value);
    if widen(narrowed) > value {
        narrowed.next_down()
    } else {
        narrowed
    }
}

/// Smallest `Real` not below `value` (exact in the f64 build; NaN stays NaN).
pub(super) fn round_up(value: f64) -> Real {
    let narrowed = nearest(value);
    if widen(narrowed) < value {
        narrowed.next_up()
    } else {
        narrowed
    }
}

/// Narrowing toward zero: an impact time or fraction never moves later.
pub(super) fn toward_zero(value: f64) -> Real {
    if value >= 0.0 {
        round_down(value)
    } else {
        round_up(value)
    }
}

const fn to_array(value: V) -> [f64; 3] {
    [widen(value.0), widen(value.1), widen(value.2)]
}

const fn from_array(value: [f64; 3]) -> V {
    V(nearest(value[0]), nearest(value[1]), nearest(value[2]))
}

fn from_contact(contact: PrimitiveContact3) -> PrimitiveContact {
    PrimitiveContact {
        normal: from_array(contact.normal),
        separation: round_down(contact.separation),
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
    work.primitive_segment_distance_evaluations += kernel.segment_distance_evaluations;
    work.primitive_segment_feature_tests += kernel.segment_feature_tests;
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
            crate::approximate::real_consts::FRAC_1_SQRT_2,
            crate::approximate::real_consts::FRAC_1_SQRT_2,
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
        let time = swept_time(&capsule, &wedge, 1.0 / 60.0, 0.02, &mut work)
            .unwrap()
            .unwrap();
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
