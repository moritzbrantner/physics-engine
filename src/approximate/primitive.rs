//! Specialized f64 contact geometry for capsules and right triangular-prism wedges.
//!
//! Capsules use analytic segment/radius support and distance queries. Wedges retain only six local
//! vertices and four unique face/edge direction classes. The fixed-step solver keeps orientation
//! constant during each translation CCD substep, so polyhedral swept SAT and conservative
//! advancement operate on immutable local geometry rather than materialized meshes.
use super::{Body, Shape, Vector as V, geometry::GeometryStats};
use crate::numeric::Scalar;

#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct PrimitiveContact {
    pub normal: V,
    pub separation: Scalar,
    pub point_a: V,
    pub point_b: V,
}

fn flip(contact: PrimitiveContact) -> PrimitiveContact {
    PrimitiveContact {
        normal: -contact.normal,
        separation: contact.separation,
        point_a: contact.point_b,
        point_b: contact.point_a,
    }
}

pub(super) fn bounds_extents(body: &Body) -> V {
    match body.shape {
        Shape::Sphere(radius) => V(radius, radius, radius),
        Shape::Box(half) | Shape::Wedge(half) => {
            let axes = body.orientation.axes();
            axes[0].abs() * half.0 + axes[1].abs() * half.1 + axes[2].abs() * half.2
        }
        Shape::Capsule {
            half_segment,
            radius,
        } => {
            let axis = body.orientation.rotate(V::Y).abs();
            V(radius, radius, radius) + axis * half_segment
        }
    }
}

pub(super) fn support_point(body: &Body, direction: V) -> V {
    support_counted(body, direction, None)
}

fn support_counted(body: &Body, direction: V, work: Option<&mut GeometryStats>) -> V {
    let n = direction.unit();
    match body.shape {
        Shape::Sphere(radius) => body.position + n * radius,
        Shape::Box(half) => {
            let axes = body.orientation.axes();
            let mut point = body.position;
            for (index, axis) in axes.into_iter().enumerate() {
                let dot = direction.dot(axis);
                if dot.abs() > 1e-12 {
                    point += axis * (half.at(index) * dot.signum());
                }
            }
            point
        }
        Shape::Capsule {
            half_segment,
            radius,
        } => {
            let axis = body.orientation.rotate(V::Y);
            let end = axis * (half_segment * if direction.dot(axis) < 0.0 { -1.0 } else { 1.0 });
            body.position + end + n * radius
        }
        Shape::Wedge(half) => {
            let vertices = wedge_vertices(half);
            if let Some(work) = work {
                work.primitive_vertex_tests += vertices.len() as u64;
            }
            let mut best = vertices[0];
            let mut best_dot = direction.dot(body.orientation.rotate(best));
            for &vertex in &vertices[1..] {
                let dot = direction.dot(body.orientation.rotate(vertex));
                if dot > best_dot {
                    best = vertex;
                    best_dot = dot;
                }
            }
            body.position + body.orientation.rotate(best)
        }
    }
}

pub(super) fn query(a: &Body, b: &Body, work: &mut GeometryStats) -> Option<PrimitiveContact> {
    work.primitive_queries += 1;
    match (a.shape, b.shape) {
        (
            Shape::Capsule {
                half_segment,
                radius,
            },
            Shape::Sphere(sphere_radius),
        ) => Some(capsule_sphere(a, b, half_segment, radius, sphere_radius)),
        (
            Shape::Sphere(sphere_radius),
            Shape::Capsule {
                half_segment,
                radius,
            },
        ) => Some(flip(capsule_sphere(
            b,
            a,
            half_segment,
            radius,
            sphere_radius,
        ))),
        (
            Shape::Capsule {
                half_segment: ah,
                radius: ar,
            },
            Shape::Capsule {
                half_segment: bh,
                radius: br,
            },
        ) => Some(capsule_capsule(a, b, ah, ar, bh, br)),
        (
            Shape::Capsule {
                half_segment,
                radius,
            },
            Shape::Box(_),
        ) => Some(capsule_box(a, b, half_segment, radius)),
        (
            Shape::Box(_),
            Shape::Capsule {
                half_segment,
                radius,
            },
        ) => Some(flip(capsule_box(b, a, half_segment, radius))),
        (
            Shape::Capsule {
                half_segment,
                radius,
            },
            Shape::Wedge(_),
        ) => Some(capsule_wedge(a, b, half_segment, radius)),
        (
            Shape::Wedge(_),
            Shape::Capsule {
                half_segment,
                radius,
            },
        ) => Some(flip(capsule_wedge(b, a, half_segment, radius))),
        (Shape::Sphere(radius), Shape::Wedge(_)) => Some(sphere_wedge(a, b, radius)),
        (Shape::Wedge(_), Shape::Sphere(radius)) => Some(flip(sphere_wedge(b, a, radius))),
        (Shape::Wedge(_), Shape::Box(_))
        | (Shape::Box(_), Shape::Wedge(_))
        | (Shape::Wedge(_), Shape::Wedge(_)) => Some(poly_poly(a, b, work)),
        _ => None,
    }
}

pub(super) fn swept_time(
    a: &Body,
    b: &Body,
    dt: Scalar,
    margin: Scalar,
    work: &mut GeometryStats,
) -> Option<Scalar> {
    if matches!(
        (a.shape, b.shape),
        (Shape::Wedge(_), Shape::Box(_))
            | (Shape::Box(_), Shape::Wedge(_))
            | (Shape::Wedge(_), Shape::Wedge(_))
    ) {
        return poly_sweep_time(a, b, dt, margin, work);
    }

    let displacement = (b.velocity - a.velocity) * dt;
    if displacement.dot(displacement) <= 1e-28 {
        return None;
    }
    let target = margin.max(1e-6);
    let tolerance = 1e-9 * (1.0 + a.shape.radius() + b.shape.radius());
    let mut time = 0.0;
    let mut aa = a.clone();
    let mut bb = b.clone();

    for _ in 0..128 {
        work.primitive_sweep_iterations += 1;
        aa.position = a.position + a.velocity * (dt * time);
        bb.position = b.position + b.velocity * (dt * time);
        let contact = query(&aa, &bb, work)?;
        let gap = contact.separation - target;
        if gap <= 0.0 {
            return Some(time);
        }
        // The current closest-feature normal is a separating direction for these convex
        // fixed-orientation shapes. Advancing by total speed can make arbitrarily little progress
        // when motion is mostly tangential, eventually exhausting the iteration bound and
        // tunnelling. Use only the distance closed along that separating normal.
        let closing = -displacement.dot(contact.normal);
        if closing <= 1e-14 {
            return None;
        }
        let remaining = 1.0 - time;
        if gap > closing * remaining + tolerance {
            return None;
        }
        let advance = gap / closing;
        let next = time + advance;
        if next > 1.0 {
            return None;
        }
        if next <= time {
            let bumped = Scalar::from_bits(time.to_bits() + 1);
            if bumped > 1.0 {
                return None;
            }
            time = bumped;
        } else {
            time = next;
        }
    }
    None
}

fn capsule_segment(body: &Body, half_segment: Scalar) -> (V, V) {
    let axis = body.orientation.rotate(V::Y) * half_segment;
    (body.position - axis, body.position + axis)
}

fn capsule_sphere(
    capsule: &Body,
    sphere: &Body,
    half_segment: Scalar,
    capsule_radius: Scalar,
    sphere_radius: Scalar,
) -> PrimitiveContact {
    let (a, b) = capsule_segment(capsule, half_segment);
    let core = closest_point_segment(sphere.position, a, b);
    let delta = sphere.position - core;
    let distance = delta.length();
    let normal = fallback_normal(delta, sphere.position - capsule.position);
    PrimitiveContact {
        normal,
        separation: distance - capsule_radius - sphere_radius,
        point_a: core + normal * capsule_radius,
        point_b: sphere.position - normal * sphere_radius,
    }
}

fn capsule_capsule(
    a: &Body,
    b: &Body,
    a_half: Scalar,
    a_radius: Scalar,
    b_half: Scalar,
    b_radius: Scalar,
) -> PrimitiveContact {
    let (a0, a1) = capsule_segment(a, a_half);
    let (b0, b1) = capsule_segment(b, b_half);
    let (pa, pb) = closest_segment_segment(a0, a1, b0, b1);
    let delta = pb - pa;
    let distance = delta.length();
    let normal = fallback_normal(delta, b.position - a.position);
    PrimitiveContact {
        normal,
        separation: distance - a_radius - b_radius,
        point_a: pa + normal * a_radius,
        point_b: pb - normal * b_radius,
    }
}

fn capsule_box(
    capsule: &Body,
    box_body: &Body,
    half_segment: Scalar,
    radius: Scalar,
) -> PrimitiveContact {
    let Shape::Box(half) = box_body.shape else {
        unreachable!()
    };
    let (world_a, world_b) = capsule_segment(capsule, half_segment);
    let local_a = box_body
        .orientation
        .inverse_rotate(world_a - box_body.position);
    let local_b = box_body
        .orientation
        .inverse_rotate(world_b - box_body.position);

    if let Some((enter, exit)) = segment_aabb_interval(local_a, local_b, half) {
        let local_core = local_a + (local_b - local_a) * ((enter + exit) * 0.5);
        let (outward, depth) = nearest_box_face(local_core, half);
        let core = box_body.position + box_body.orientation.rotate(local_core);
        let outward = box_body.orientation.rotate(outward);
        let normal = -outward;
        return PrimitiveContact {
            normal,
            separation: -depth - radius,
            point_a: core + normal * radius,
            point_b: core + outward * depth,
        };
    }

    let (local_core, local_box) = closest_segment_aabb(local_a, local_b, half);
    let core = box_body.position + box_body.orientation.rotate(local_core);
    let point_b = box_body.position + box_body.orientation.rotate(local_box);
    let delta = point_b - core;
    let distance = delta.length();
    let normal = fallback_normal(delta, box_body.position - capsule.position);
    PrimitiveContact {
        normal,
        separation: distance - radius,
        point_a: core + normal * radius,
        point_b,
    }
}

fn sphere_wedge(sphere: &Body, wedge: &Body, radius: Scalar) -> PrimitiveContact {
    let Shape::Wedge(half) = wedge.shape else {
        unreachable!()
    };
    let local = wedge
        .orientation
        .inverse_rotate(sphere.position - wedge.position);
    if let Some((outward, depth)) = wedge_inside_depth(local, half) {
        let outward_world = wedge.orientation.rotate(outward);
        let normal = -outward_world;
        return PrimitiveContact {
            normal,
            separation: -depth - radius,
            point_a: sphere.position + normal * radius,
            point_b: sphere.position + outward_world * depth,
        };
    }

    let local_wedge = closest_point_wedge(local, half);
    let point_b = wedge.position + wedge.orientation.rotate(local_wedge);
    let delta = point_b - sphere.position;
    let distance = delta.length();
    let normal = fallback_normal(delta, wedge.position - sphere.position);
    PrimitiveContact {
        normal,
        separation: distance - radius,
        point_a: sphere.position + normal * radius,
        point_b,
    }
}

fn capsule_wedge(
    capsule: &Body,
    wedge: &Body,
    half_segment: Scalar,
    radius: Scalar,
) -> PrimitiveContact {
    let Shape::Wedge(half) = wedge.shape else {
        unreachable!()
    };
    let (world_a, world_b) = capsule_segment(capsule, half_segment);
    let local_a = wedge.orientation.inverse_rotate(world_a - wedge.position);
    let local_b = wedge.orientation.inverse_rotate(world_b - wedge.position);

    if let Some((enter, exit)) = segment_wedge_interval(local_a, local_b, half) {
        let local_core = local_a + (local_b - local_a) * ((enter + exit) * 0.5);
        let (outward, depth) = wedge_inside_depth(local_core, half).unwrap_or((V::Y, 0.0));
        let core = wedge.position + wedge.orientation.rotate(local_core);
        let outward = wedge.orientation.rotate(outward);
        let normal = -outward;
        return PrimitiveContact {
            normal,
            separation: -depth - radius,
            point_a: core + normal * radius,
            point_b: core + outward * depth,
        };
    }

    let (local_core, local_wedge) = closest_segment_wedge(local_a, local_b, half);
    let core = wedge.position + wedge.orientation.rotate(local_core);
    let point_b = wedge.position + wedge.orientation.rotate(local_wedge);
    let delta = point_b - core;
    let distance = delta.length();
    let normal = fallback_normal(delta, wedge.position - capsule.position);
    PrimitiveContact {
        normal,
        separation: distance - radius,
        point_a: core + normal * radius,
        point_b,
    }
}

fn poly_poly(a: &Body, b: &Body, work: &mut GeometryStats) -> PrimitiveContact {
    let (axes, len) = poly_axes(a, b);
    let mut best_separation = -Scalar::INFINITY;
    let mut best_normal = V::X;

    for axis in axes[..len].iter().copied() {
        work.primitive_axes_tested += 1;
        let (a_min, a_max) = projection_interval(a, axis, work);
        let (b_min, b_max) = projection_interval(b, axis, work);
        let forward = b_min - a_max;
        let backward = a_min - b_max;
        let (separation, normal) = if forward >= backward {
            (forward, axis)
        } else {
            (backward, -axis)
        };
        if separation > best_separation {
            best_separation = separation;
            best_normal = normal;
        }
    }

    PrimitiveContact {
        normal: best_normal,
        separation: best_separation,
        point_a: support_counted(a, best_normal, Some(work)),
        point_b: support_counted(b, -best_normal, Some(work)),
    }
}

fn poly_sweep_time(
    a: &Body,
    b: &Body,
    dt: Scalar,
    margin: Scalar,
    work: &mut GeometryStats,
) -> Option<Scalar> {
    let (axes, len) = poly_axes(a, b);
    let displacement = (b.velocity - a.velocity) * dt;
    let mut enter: Scalar = 0.0;
    let mut exit: Scalar = 1.0;

    for axis in axes[..len].iter().copied() {
        work.primitive_axes_tested += 1;
        work.primitive_sweep_iterations += 1;
        let (a_min, a_max) = projection_interval(a, axis, work);
        let (b_min, b_max) = projection_interval(b, axis, work);
        let velocity = displacement.dot(axis);
        if velocity.abs() <= 1e-14 {
            if b_min > a_max + margin || a_min > b_max + margin {
                return None;
            }
            continue;
        }
        let first = (a_min - margin - b_max) / velocity;
        let second = (a_max + margin - b_min) / velocity;
        enter = enter.max(first.min(second));
        exit = exit.min(first.max(second));
        if enter > exit {
            return None;
        }
    }

    (exit >= 0.0 && enter <= 1.0).then_some(enter.max(0.0))
}

fn projection_interval(body: &Body, axis: V, work: &mut GeometryStats) -> (Scalar, Scalar) {
    let maximum = support_counted(body, axis, Some(work)).dot(axis);
    let minimum = support_counted(body, -axis, Some(work)).dot(axis);
    (minimum, maximum)
}

fn poly_axes(a: &Body, b: &Body) -> ([V; 32], usize) {
    let mut axes = [V::ZERO; 32];
    let mut len = 0;
    append_face_axes(a, &mut axes, &mut len);
    append_face_axes(b, &mut axes, &mut len);

    let (a_edges, a_len) = edge_axes(a);
    let (b_edges, b_len) = edge_axes(b);
    for left in a_edges[..a_len].iter().copied() {
        for right in b_edges[..b_len].iter().copied() {
            push_axis(&mut axes, &mut len, left.cross(right));
        }
    }
    (axes, len)
}

fn append_face_axes(body: &Body, axes: &mut [V; 32], len: &mut usize) {
    match body.shape {
        Shape::Box(_) => {
            for axis in body.orientation.axes() {
                push_axis(axes, len, axis);
            }
        }
        Shape::Wedge(half) => {
            for local in [V::X, V::Y, V::Z, V(half.1, half.0, 0.0).unit()] {
                push_axis(axes, len, body.orientation.rotate(local));
            }
        }
        _ => unreachable!(),
    }
}

fn edge_axes(body: &Body) -> ([V; 4], usize) {
    let mut out = [V::ZERO; 4];
    match body.shape {
        Shape::Box(_) => {
            let axes = body.orientation.axes();
            out[..3].copy_from_slice(&axes);
            (out, 3)
        }
        Shape::Wedge(half) => {
            for (index, local) in [V::X, V::Y, V::Z, V(half.0, -half.1, 0.0).unit()]
                .into_iter()
                .enumerate()
            {
                out[index] = body.orientation.rotate(local);
            }
            (out, 4)
        }
        _ => unreachable!(),
    }
}

fn push_axis(axes: &mut [V; 32], len: &mut usize, axis: V) {
    if axis.dot(axis) <= 1e-12 {
        return;
    }
    let axis = axis.unit();
    if axes[..*len]
        .iter()
        .any(|existing| existing.dot(axis).abs() >= 1.0 - 1e-10)
    {
        return;
    }
    axes[*len] = axis;
    *len += 1;
}

fn wedge_vertices(half: V) -> [V; 6] {
    [
        V(-half.0, -half.1, -half.2),
        V(-half.0, half.1, -half.2),
        V(half.0, -half.1, -half.2),
        V(-half.0, -half.1, half.2),
        V(-half.0, half.1, half.2),
        V(half.0, -half.1, half.2),
    ]
}

fn wedge_planes(half: V) -> [(V, Scalar); 5] {
    [
        (-V::Y, half.1),
        (-V::X, half.0),
        (V(half.1, half.0, 0.0).unit(), 0.0),
        (V::Z, half.2),
        (-V::Z, half.2),
    ]
}

fn wedge_triangles(half: V) -> [[V; 3]; 8] {
    let v = wedge_vertices(half);
    [
        [v[0], v[2], v[1]],
        [v[3], v[4], v[5]],
        [v[0], v[3], v[5]],
        [v[0], v[5], v[2]],
        [v[0], v[1], v[4]],
        [v[0], v[4], v[3]],
        [v[1], v[2], v[5]],
        [v[1], v[5], v[4]],
    ]
}

fn wedge_inside_depth(point: V, half: V) -> Option<(V, Scalar)> {
    let mut best = (V::Y, Scalar::INFINITY);
    for (normal, limit) in wedge_planes(half) {
        let depth = limit - normal.dot(point);
        if depth < -1e-10 {
            return None;
        }
        if depth < best.1 {
            best = (normal, depth.max(0.0));
        }
    }
    Some(best)
}

fn segment_wedge_interval(a: V, b: V, half: V) -> Option<(Scalar, Scalar)> {
    let delta = b - a;
    let mut enter: Scalar = 0.0;
    let mut exit: Scalar = 1.0;
    for (normal, limit) in wedge_planes(half) {
        let start = normal.dot(a) - limit;
        let velocity = normal.dot(delta);
        if velocity.abs() <= 1e-14 {
            if start > 0.0 {
                return None;
            }
            continue;
        }
        let time = -start / velocity;
        if velocity > 0.0 {
            exit = exit.min(time);
        } else {
            enter = enter.max(time);
        }
        if enter > exit {
            return None;
        }
    }
    (exit >= 0.0 && enter <= 1.0).then_some((enter.max(0.0), exit.min(1.0)))
}

fn closest_point_wedge(point: V, half: V) -> V {
    let mut best = wedge_vertices(half)[0];
    let mut best_distance = Scalar::INFINITY;
    for triangle in wedge_triangles(half) {
        let candidate = closest_point_triangle(point, triangle[0], triangle[1], triangle[2]);
        let distance = (candidate - point).dot(candidate - point);
        if distance < best_distance {
            best = candidate;
            best_distance = distance;
        }
    }
    best
}

fn closest_segment_wedge(a: V, b: V, half: V) -> (V, V) {
    let mut best = (a, wedge_vertices(half)[0]);
    let mut best_distance = Scalar::INFINITY;
    for triangle in wedge_triangles(half) {
        for endpoint in [a, b] {
            let candidate = closest_point_triangle(endpoint, triangle[0], triangle[1], triangle[2]);
            update_pair(endpoint, candidate, &mut best, &mut best_distance);
        }
        for edge in [
            (triangle[0], triangle[1]),
            (triangle[1], triangle[2]),
            (triangle[2], triangle[0]),
        ] {
            let candidate = closest_segment_segment(a, b, edge.0, edge.1);
            update_pair(candidate.0, candidate.1, &mut best, &mut best_distance);
        }
    }
    best
}

fn update_pair(a: V, b: V, best: &mut (V, V), best_distance: &mut Scalar) {
    let distance = (b - a).dot(b - a);
    if distance < *best_distance {
        *best = (a, b);
        *best_distance = distance;
    }
}

fn closest_point_triangle(point: V, a: V, b: V, c: V) -> V {
    let ab = b - a;
    let ac = c - a;
    let ap = point - a;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }

    let bp = point - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }

    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return a + ab * (d1 / (d1 - d3));
    }

    let cp = point - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }

    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return a + ac * (d2 / (d2 - d6));
    }

    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    }

    let denominator = 1.0 / (va + vb + vc);
    let v = vb * denominator;
    let w = vc * denominator;
    a + ab * v + ac * w
}

fn closest_point_segment(point: V, a: V, b: V) -> V {
    let delta = b - a;
    let denominator = delta.dot(delta);
    if denominator <= 1e-20 {
        return a;
    }
    let time = ((point - a).dot(delta) / denominator).clamp(0.0, 1.0);
    a + delta * time
}

fn closest_segment_segment(p1: V, q1: V, p2: V, q2: V) -> (V, V) {
    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let r = p1 - p2;
    let a = d1.dot(d1);
    let e = d2.dot(d2);
    let epsilon = 1e-20;

    if a <= epsilon && e <= epsilon {
        return (p1, p2);
    }
    if a <= epsilon {
        let t = (d2.dot(r) / e).clamp(0.0, 1.0);
        return (p1, p2 + d2 * t);
    }
    if e <= epsilon {
        let s = (-d1.dot(r) / a).clamp(0.0, 1.0);
        return (p1 + d1 * s, p2);
    }

    let b = d1.dot(d2);
    let c = d1.dot(r);
    let f = d2.dot(r);
    let denominator = a * e - b * b;
    let mut s = if denominator.abs() > epsilon {
        ((b * f - c * e) / denominator).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let mut t = (b * s + f) / e;
    if t < 0.0 {
        t = 0.0;
        s = (-c / a).clamp(0.0, 1.0);
    } else if t > 1.0 {
        t = 1.0;
        s = ((b - c) / a).clamp(0.0, 1.0);
    }
    (p1 + d1 * s, p2 + d2 * t)
}

fn segment_aabb_interval(a: V, b: V, half: V) -> Option<(Scalar, Scalar)> {
    let delta = b - a;
    let mut enter: Scalar = 0.0;
    let mut exit: Scalar = 1.0;
    for axis in 0..3 {
        let start = a.at(axis);
        let velocity = delta.at(axis);
        let extent = half.at(axis);
        if velocity.abs() <= 1e-14 {
            if start < -extent || start > extent {
                return None;
            }
            continue;
        }
        let first = (-extent - start) / velocity;
        let second = (extent - start) / velocity;
        enter = enter.max(first.min(second));
        exit = exit.min(first.max(second));
        if enter > exit {
            return None;
        }
    }
    (exit >= 0.0 && enter <= 1.0).then_some((enter.max(0.0), exit.min(1.0)))
}

fn closest_segment_aabb(a: V, b: V, half: V) -> (V, V) {
    let delta = b - a;
    let mut times = [0.0; 8];
    times[0] = 0.0;
    times[1] = 1.0;
    let mut len = 2;
    for axis in 0..3 {
        let velocity = delta.at(axis);
        if velocity.abs() <= 1e-14 {
            continue;
        }
        for bound in [-half.at(axis), half.at(axis)] {
            let time = (bound - a.at(axis)) / velocity;
            if time > 0.0 && time < 1.0 {
                times[len] = time;
                len += 1;
            }
        }
    }
    times[..len].sort_by(Scalar::total_cmp);
    let mut unique = 1;
    for index in 1..len {
        if (times[index] - times[unique - 1]).abs() > 1e-14 {
            times[unique] = times[index];
            unique += 1;
        }
    }
    len = unique;

    let mut best = (a, clamp_aabb(a, half));
    let mut best_distance = (best.1 - best.0).dot(best.1 - best.0);
    let mut evaluate = |time: Scalar| {
        let point = a + delta * time.clamp(0.0, 1.0);
        let clamped = clamp_aabb(point, half);
        update_pair(point, clamped, &mut best, &mut best_distance);
    };
    for &time in &times[..len] {
        evaluate(time);
    }
    for window in times[..len].windows(2) {
        let low = window[0];
        let high = window[1];
        let middle = (low + high) * 0.5;
        let point = a + delta * middle;
        let mut numerator = 0.0;
        let mut denominator = 0.0;
        for axis in 0..3 {
            let coordinate = point.at(axis);
            let bound = if coordinate < -half.at(axis) {
                -half.at(axis)
            } else if coordinate > half.at(axis) {
                half.at(axis)
            } else {
                continue;
            };
            let velocity = delta.at(axis);
            numerator += velocity * (a.at(axis) - bound);
            denominator += velocity * velocity;
        }
        if denominator > 1e-20 {
            evaluate((-numerator / denominator).clamp(low, high));
        }
    }
    best
}

fn clamp_aabb(point: V, half: V) -> V {
    V(
        point.0.clamp(-half.0, half.0),
        point.1.clamp(-half.1, half.1),
        point.2.clamp(-half.2, half.2),
    )
}

fn nearest_box_face(point: V, half: V) -> (V, Scalar) {
    let mut axis = 0;
    let mut depth = half.0 - point.0.abs();
    for candidate in 1..3 {
        let candidate_depth = half.at(candidate) - point.at(candidate).abs();
        if candidate_depth < depth {
            axis = candidate;
            depth = candidate_depth;
        }
    }
    let sign = if point.at(axis) < 0.0 { -1.0 } else { 1.0 };
    let normal = match axis {
        0 => V::X,
        1 => V::Y,
        _ => V::Z,
    } * sign;
    (normal, depth.max(0.0))
}

fn fallback_normal(delta: V, fallback: V) -> V {
    if delta.dot(delta) > 1e-20 {
        delta.unit()
    } else if fallback.dot(fallback) > 1e-20 {
        fallback.unit()
    } else {
        V::X
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::approximate::{Quaternion, Shape};

    fn body(id: u64, shape: Shape, position: V) -> Body {
        Body::new(crate::BodyId(id), shape, position, 0.0)
    }

    #[test]
    fn capsule_sphere_and_capsule_capsule_use_analytic_segment_distance() {
        let capsule = body(1, Shape::capsule(2.0, 0.5), V::ZERO);
        let sphere = body(2, Shape::Sphere(0.5), V(0.0, 3.0, 0.0));
        let mut work = GeometryStats::default();
        let contact = query(&capsule, &sphere, &mut work).unwrap();
        assert!(contact.separation.abs() < 1e-12);
        assert_eq!(work.primitive_vertex_tests, 0);

        let other = body(3, Shape::capsule(2.0, 0.5), V(0.9, 0.0, 0.0));
        let contact = query(&capsule, &other, &mut work).unwrap();
        assert!(contact.separation < 0.0);
        assert_eq!(work.primitive_vertex_tests, 0);
    }

    #[test]
    fn sphere_wedge_respects_sloped_face_instead_of_bounding_box() {
        let wedge = body(1, Shape::wedge(V(2.0, 2.0, 2.0)), V::ZERO);
        let near = body(2, Shape::Sphere(0.8), V(1.0, 0.0, 0.0));
        let far = body(3, Shape::Sphere(0.2), V(1.5, 1.5, 0.0));
        let mut work = GeometryStats::default();
        assert!(query(&near, &wedge, &mut work).unwrap().separation < 0.0);
        assert!(query(&far, &wedge, &mut work).unwrap().separation > 0.0);
    }

    #[test]
    fn capsule_wedge_is_symmetric_and_handles_rotated_capsules() {
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
    }

    #[test]
    fn wedge_sat_has_a_fixed_small_work_ceiling() {
        let a = body(1, Shape::wedge(V(2.0, 1.0, 3.0)), V::ZERO);
        let mut b = body(2, Shape::wedge(V(1.5, 2.0, 1.0)), V(3.0, 0.0, 0.0));
        b.orientation = Quaternion(0.0, 0.0, 0.25881904510252074, 0.9659258262890683);
        let mut work = GeometryStats::default();
        let contact = query(&a, &b, &mut work).unwrap();
        assert!(contact.separation.is_finite());
        assert!(work.primitive_axes_tested <= 24, "{work:?}");
        assert!(work.primitive_vertex_tests <= 600, "{work:?}");
    }

    #[test]
    fn glancing_capsule_sweep_uses_normal_closing_speed() {
        let mut capsule = Body::new(
            crate::BodyId(1),
            Shape::capsule(0.0, 0.1),
            V(-400.0, 1.0, 0.0),
            1.0,
        );
        capsule.velocity = V(800.0, -2.0, 0.0);
        let target = body(2, Shape::Box(V(500.0, 0.01, 10.0)), V::ZERO);
        let mut work = GeometryStats::default();

        let time = swept_time(&capsule, &target, 1.0, 0.02, &mut work).unwrap();

        assert!((time - 0.435).abs() < 1e-9, "time={time}");
        assert!(
            work.primitive_sweep_iterations <= 3,
            "glancing motion must not burn the bounded sweep budget: {work:?}"
        );
    }

    #[test]
    fn fast_capsule_conservative_advancement_reaches_thin_wedge() {
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
    #[ignore = "advisory timing; deterministic axis/vertex ceilings are tested separately"]
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
