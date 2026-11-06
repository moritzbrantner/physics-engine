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
    // Pair dispatch needs only the shape tags. Constructing validated f64 shapes
    // here repeats dimension conversion and validation before the actual query.
    fn kind(shape: Shape) -> PrimitiveKind {
        match shape {
            Shape::Sphere(_) => PrimitiveKind::Sphere,
            Shape::Box(_) => PrimitiveKind::Box,
            Shape::Capsule { .. } => PrimitiveKind::Capsule,
            Shape::Wedge(_) => PrimitiveKind::Wedge,
            Shape::Heightfield { .. }
            | Shape::ConvexHull { .. }
            | Shape::TriangleMesh { .. }
            | Shape::HalfSpace => {
                unreachable!("terrain and halfspaces are queried through their closed forms")
            }
        }
    }
    PrimitivePair::canonical(kind(left), kind(right))
}

pub(super) fn bounds_extents(body: &Body) -> V {
    let [x, y, z] = kernel_bounds(kernel_body(body));
    V(round_up(x), round_up(y), round_up(z))
}

pub(super) fn box_hull_manifold(
    hull: &geometry_kernels::segment_hull::PreparedSegmentHull,
    a: &Body,
    b: &Body,
    margin: Real,
    stats: &mut super::GeometryStats,
) -> Result<Option<super::contact::Manifold>, SweepFailure> {
    let hull_first = matches!(a.shape, Shape::ConvexHull { .. });
    let (target, moving) = if hull_first { (a, b) } else { (b, a) };
    let Shape::Box(half) = moving.shape else {
        return Err(SweepFailure::InvalidGeometryInput);
    };
    let mut work = geometry_kernels::segment_hull::BoxHullWork::default();
    let result = hull
        .box_contact(
            geometry_kernels::math3::sub(to_array(moving.position), to_array(target.position)),
            to_array(half),
            kernel_axes(moving.orientation),
            f64::from(margin),
            &mut work,
        )
        .map_err(|error| match error {
            geometry_kernels::segment_hull::BoxHullError::InvalidInput => {
                SweepFailure::InvalidGeometryInput
            }
            geometry_kernels::segment_hull::BoxHullError::NonFiniteComputation => {
                SweepFailure::NonFiniteComputation
            }
        })?;
    stats.sat_queries += 1;
    stats.sat_axes_tested += work.axes_tested;
    stats.primitive_vertex_tests += work.vertex_tests;
    stats.clip_passes += work.clipping_planes;
    Ok(result.map(|contact| {
        let normal = if hull_first {
            from_array(contact.normal)
        } else {
            -from_array(contact.normal)
        };
        let origin = to_array(target.position);
        let mut points = [super::contact::Point::default(); 4];
        for (out, p) in points.iter_mut().zip(&contact.points[..contact.len]) {
            let world = |v| from_array(geometry_kernels::math3::add(v, origin));
            let (pa, pb) = if hull_first {
                (world(p.point_hull), world(p.point_box))
            } else {
                (world(p.point_box), world(p.point_hull))
            };
            *out = super::contact::Point {
                ra: pa - a.position,
                rb: pb - b.position,
                separation: round_down(p.separation),
            };
        }
        super::contact::Manifold {
            normal,
            points: super::contact::Points::selected(&points[..contact.len]),
            swept: false,
            time: 0.0,
        }
    }))
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
    // Avoid generic shape/frame materialization for the checked analytic moving-sphere path.
    // Other inputs retain the primitive kernel's validation and failure behavior.
    if let (Shape::Sphere(radius_a), Shape::Sphere(radius_b)) = (a.shape, b.shape)
        && radius_a.is_finite()
        && radius_a > 0.0
        && radius_b.is_finite()
        && radius_b > 0.0
        && a.position.finite()
        && b.position.finite()
        && a.velocity.finite()
        && b.velocity.finite()
        && dt.is_finite()
        && dt >= 0.0
        && margin.is_finite()
        && margin >= 0.0
    {
        let velocity_a = to_array(a.velocity);
        let velocity_b = to_array(b.velocity);
        let displacement = std::array::from_fn(|i| (velocity_b[i] - velocity_a[i]) * widen(dt));
        // Stationary pairs retain the generic contact-query validation and work accounting.
        if displacement != [0.0; 3] {
            let position_a = to_array(a.position);
            let position_b = to_array(b.position);
            let delta = std::array::from_fn(|i| position_b[i] - position_a[i]);
            work.primitive_sweep_iterations += 1;
            let result = geometry_kernels::try_sphere_sphere_fraction_f64(
                delta,
                displacement,
                widen(radius_a) + widen(radius_b) + widen(margin),
            );
            work.primitive_sweep_failures += u64::from(result.is_err());
            return result
                .map(|time| time.map(toward_zero))
                .map_err(|_| SweepFailure::NonFiniteComputation);
        }
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
    // Sweep in the query's frame. Conservative advance creeps one time ulp at a time when the
    // gap stalls just above contact, and the separation's rounding grows with the coordinates'
    // magnitude: far from the origin (a capsule reaching a box edge at x = 15853,
    // physics-engine#349) the creep cannot outrun it and exhausts the iteration budget.
    // Widened f32 coordinates subtract exactly in f64, so the fraction is unchanged.
    let origin = a.position;
    a.position = [0.0; 3];
    b.position = [0, 1, 2].map(|k| b.position[k] - origin[k]);
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
        Shape::Heightfield { .. }
        | Shape::ConvexHull { .. }
        | Shape::TriangleMesh { .. }
        | Shape::HalfSpace => {
            unreachable!("terrain and halfspaces are queried through their closed forms")
        }
    }
}

/// The point of `body`'s shape deepest along `-normal` (its support point), in f64.
pub(super) fn deepest_along(body: &Body, normal: [f64; 3]) -> [f64; 3] {
    let mut work = PrimitiveWork3::default();
    geometry_kernels::primitive3::support_point(
        kernel_body(body),
        [-normal[0], -normal[1], -normal[2]],
        &mut work,
    )
}

/// A box's or wedge's world-space vertices, outward face normals and edge directions, for
/// separating-axis tests against terrain triangles. `None` for other shapes.
pub(super) struct Polytope {
    pub vertices: Vec<[f64; 3]>,
    pub face_normals: Vec<[f64; 3]>,
    pub edges: Vec<[f64; 3]>,
}
pub(super) fn polytope_of(body: &Body) -> Option<Polytope> {
    let axes = kernel_axes(body.orientation);
    let c = to_array(body.position);
    let world = |l: [f64; 3]| -> [f64; 3] {
        std::array::from_fn(|i| c[i] + axes[0][i] * l[0] + axes[1][i] * l[1] + axes[2][i] * l[2])
    };
    let direction = |l: [f64; 3]| -> [f64; 3] {
        std::array::from_fn(|i| axes[0][i] * l[0] + axes[1][i] * l[1] + axes[2][i] * l[2])
    };
    let Polytope {
        vertices: local,
        face_normals: normals,
        edges,
    } = match body.shape {
        Shape::Box(h) => {
            let h = to_array(h);
            let corners = (0..8)
                .map(|k| std::array::from_fn(|i| if k >> i & 1 == 0 { -h[i] } else { h[i] }))
                .collect();
            Polytope {
                vertices: corners,
                face_normals: identity_axes().to_vec(),
                edges: identity_axes().to_vec(),
            }
        }
        Shape::Wedge(h) => {
            // The ramp rises toward -X: corners (-x, ±y), (+x, -y) at both z.
            let [x, y, z] = to_array(h);
            let corners = vec![
                [-x, -y, -z],
                [-x, y, -z],
                [x, -y, -z],
                [-x, -y, z],
                [-x, y, z],
                [x, -y, z],
            ];
            let ramp = (y * y + x * x).sqrt();
            let normals = vec![
                [0.0, -1.0, 0.0],
                [-1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
                [y / ramp, x / ramp, 0.0],
            ];
            let edges = vec![
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
                [x, -y, 0.0],
            ];
            Polytope {
                vertices: corners,
                face_normals: normals,
                edges,
            }
        }
        _ => return None,
    };
    Some(Polytope {
        vertices: local.into_iter().map(world).collect(),
        face_normals: normals.into_iter().map(direction).collect(),
        edges: edges.into_iter().map(direction).collect(),
    })
}

fn kernel_body(body: &Body) -> PrimitiveBody3 {
    let axes = match body.shape {
        Shape::Sphere(_) => identity_axes(),
        Shape::Capsule { .. } => [
            [1.0, 0.0, 0.0],
            kernel_axes(body.orientation)[1],
            [0.0, 0.0, 1.0],
        ],
        Shape::Box(_)
        | Shape::Wedge(_)
        | Shape::ConvexHull { .. }
        | Shape::Heightfield { .. }
        | Shape::TriangleMesh { .. }
        | Shape::HalfSpace => kernel_axes(body.orientation),
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
    if q == Quaternion::IDENTITY {
        return identity_axes();
    }
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
        Body::fixture(id, shape, position, 0.0)
    }

    #[test]
    fn sphere_sweep_adapter_matches_generic_results_and_work() {
        let compare = |a: &Body, b: &Body, dt: Real, margin: Real| {
            let mut actual_work = GeometryStats::default();
            let actual = swept_time(a, b, dt, margin, &mut actual_work);
            let mut kernel_work = PrimitiveWork3::default();
            let expected = kernel_swept_time(
                kernel_body(a),
                kernel_body(b),
                widen(dt),
                widen(margin),
                128,
                &mut kernel_work,
            )
            .map(|time| time.map(toward_zero))
            .map_err(sweep_failure);
            let mut expected_work = GeometryStats::default();
            accumulate_work(&mut expected_work, kernel_work);
            expected_work.primitive_sweep_failures += u64::from(expected.is_err());
            assert_eq!(actual, expected, "a={a:?} b={b:?} dt={dt} margin={margin}");
            assert_eq!(actual_work, expected_work);
        };
        let mut a = body(1, Shape::Sphere(1.0), V::ZERO);
        let mut b = body(2, Shape::Sphere(1.0), V(4.0, 0.0, 0.0));
        for position in [
            V::ZERO,
            V(2.0, 0.0, 0.0),
            V(4.0, 2.0, 0.0),
            V(4.0, 0.0, 0.0),
        ] {
            b.position = position;
            for velocity in [V::ZERO, V(-4.0, 0.0, 0.0), V(4.0, 0.0, 0.0)] {
                b.velocity = velocity;
                for dt in [0.0, 0.25, 1.0, -1.0, Real::NAN, Real::INFINITY] {
                    for margin in [0.0, 0.1, -0.1, Real::NAN, Real::INFINITY] {
                        compare(&a, &b, dt, margin);
                    }
                }
            }
        }
        // Deterministic varied magnitudes and trajectories exercise moving/initial-overlap
        // decisions without depending on the optimized adapter for the reference result.
        let mut seed = 0x5eed_u64;
        let mut value = || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            ((seed >> 32) as u32 as Real / u32::MAX as Real - 0.5) * 100.0
        };
        for _ in 0..2048 {
            a.position = V(value(), value(), value());
            b.position = V(value(), value(), value());
            a.velocity = V(value(), value(), value());
            b.velocity = V(value(), value(), value());
            a.shape = Shape::Sphere(value().abs() + 0.001);
            b.shape = Shape::Sphere(value().abs() + 0.001);
            compare(&a, &b, value().abs(), value().abs());
        }
        for invalid in [Real::NAN, Real::INFINITY, Real::NEG_INFINITY] {
            a.velocity = V(invalid, 0.0, 0.0);
            compare(&a, &b, 1.0, 0.0);
            a.velocity = V::ZERO;
            b.position = V(0.0, invalid, 0.0);
            compare(&a, &b, 1.0, 0.0);
        }
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
        let mut capsule = Body::fixture(1, Shape::capsule(0.5, 0.1), V(-1.0, -1.0, 10.0), 1.0);
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
    fn far_snapshot_sweep_reaches_a_box_edge_within_rounding() {
        // physics-engine#349 (mmorpg#427): an offset-inflated character capsule walking into
        // a crate's vertical edge far from the origin. The gap stalled at 7e-15 and the
        // conservative advance exhausted its 128 iterations one ulp at a time.
        let capsule = body(
            1,
            Shape::capsule(45.0, 35.25),
            V(15811.787, 385.7618, 2472.5217),
        );
        let crate_box = body(
            2,
            Shape::Box(V(41.0, 250.0, 41.0)),
            V(15894.0, 527.0, 2519.0),
        );
        let mut work = PrimitiveWork3::default();
        let fraction = snapshot_sweep(&capsule, &crate_box, V(7.0, 0.0, 1.0), 128, &mut work)
            .unwrap()
            .unwrap();
        assert!((fraction - 0.894_596_6).abs() < 1e-6, "{fraction}");
        assert!(work.sweep_iterations < 8, "{}", work.sweep_iterations);
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
