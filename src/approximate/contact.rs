use super::{Body, Shape, Vector as V, numeric::Scalar};

#[derive(Clone, Copy, Debug)]
pub(super) struct Point {
    pub ra: V,
    pub rb: V,
    pub separation: Scalar,
}
#[derive(Clone, Debug)]
pub(super) struct Manifold {
    pub normal: V,
    pub points: Vec<Point>,
    pub swept: bool,
    /// First translation-only time of contact in [0, 1] of this substep.
    pub time: Scalar,
}

pub(super) fn bounds(b: &Body) -> (V, V) {
    let e = match b.shape {
        Shape::Sphere(r) => V(r, r, r),
        Shape::Box(h) => {
            let a = b.orientation.axes();
            a[0].abs() * h.0 + a[1].abs() * h.1 + a[2].abs() * h.2
        }
    };
    (b.position - e, b.position + e)
}
fn radius(b: &Body, n: V) -> Scalar {
    match b.shape {
        Shape::Sphere(r) => r,
        Shape::Box(h) => {
            let a = b.orientation.axes();
            n.dot(a[0]).abs() * h.0 + n.dot(a[1]).abs() * h.1 + n.dot(a[2]).abs() * h.2
        }
    }
}
fn axes(a: &Body, b: &Body) -> Vec<(V, usize)> {
    let aa = a.orientation.axes();
    let bb = b.orientation.axes();
    let mut out = Vec::with_capacity(15);
    for (i, n) in aa.into_iter().chain(bb).enumerate() {
        out.push((n, i));
    }
    for u in aa {
        for v in bb {
            let n = u.cross(v);
            if n.dot(n) > 1e-12 {
                out.push((n.unit(), 6));
            }
        }
    }
    out
}
fn clip(input: &[V], n: V, limit: Scalar) -> Vec<V> {
    let mut out = Vec::with_capacity(8);
    if input.is_empty() {
        return out;
    }
    let mut a = *input.last().unwrap();
    let mut da = a.dot(n) - limit;
    for &b in input {
        let db = b.dot(n) - limit;
        if (da <= 0.0) != (db <= 0.0) {
            out.push(a + (b - a) * (da / (da - db)));
        }
        if db <= 0.0 {
            out.push(b);
        }
        a = b;
        da = db;
    }
    out
}
fn support(b: &Body, n: V) -> V {
    match b.shape {
        Shape::Sphere(r) => b.position + n * r,
        Shape::Box(h) => {
            let a = b.orientation.axes();
            let mut p = b.position;
            for (i, axis) in a.iter().enumerate() {
                let d = n.dot(*axis);
                if d.abs() > 1e-8 {
                    p += *axis * (h.at(i) * d.signum());
                }
            }
            p
        }
    }
}
fn box_manifold(a: &Body, b: &Body, margin: Scalar) -> Option<Manifold> {
    let d = b.position - a.position;
    let mut best = (-Scalar::INFINITY, V::X, 0);
    for (axis, feature) in axes(a, b) {
        let projected = d.dot(axis);
        let sep = projected.abs() - radius(a, axis) - radius(b, axis);
        if sep > margin {
            return None;
        }
        // Prefer face normals at near ties; the cross-axis fallback is one point.
        if sep
            > best.0
                + if feature >= 6 {
                    margin * 2.0 + 0.01
                } else {
                    1e-7
                }
        {
            best = (
                sep,
                axis * if projected < 0.0 { -1.0 } else { 1.0 },
                feature,
            );
        }
    }
    let (separation, n, feature) = best;
    let mut points = Vec::new();
    if feature < 6 {
        let swap = feature >= 3;
        let (reference, incident, rn, face) = if swap {
            (b, a, -n, feature - 3)
        } else {
            (a, b, n, feature)
        };
        let Shape::Box(rh) = reference.shape else {
            unreachable!()
        };
        let Shape::Box(ih) = incident.shape else {
            unreachable!()
        };
        let ra = reference.orientation.axes();
        let ia = incident.orientation.axes();
        let ref_center = reference.position + rn * rh.at(face);
        let inc_face = (0..3)
            .max_by(|&i, &j| rn.dot(ia[i]).abs().total_cmp(&rn.dot(ia[j]).abs()))
            .unwrap();
        let ic =
            incident.position - ia[inc_face] * (ih.at(inc_face) * rn.dot(ia[inc_face]).signum());
        let u = (inc_face + 1) % 3;
        let v = (inc_face + 2) % 3;
        let mut poly = vec![
            ic + ia[u] * ih.at(u) + ia[v] * ih.at(v),
            ic - ia[u] * ih.at(u) + ia[v] * ih.at(v),
            ic - ia[u] * ih.at(u) - ia[v] * ih.at(v),
            ic + ia[u] * ih.at(u) - ia[v] * ih.at(v),
        ];
        for (i, basis) in ra.iter().enumerate() {
            if i != face {
                for sign in [-1.0, 1.0] {
                    let axis = *basis * sign;
                    poly = clip(&poly, axis, reference.position.dot(axis) + rh.at(i));
                }
            }
        }
        for p in poly {
            let sep = (p - ref_center).dot(rn);
            if sep <= margin {
                let projected = p - rn * sep;
                let (pa, pb) = if swap { (p, projected) } else { (projected, p) };
                points.push(Point {
                    ra: pa - a.position,
                    rb: pb - b.position,
                    separation: sep,
                });
            }
        }
        // Bound manifold cost. Spread retained anchors around the clipped polygon.
        if points.len() > 4 {
            let n = points.len();
            points = (0..4).map(|i| points[i * n / 4]).collect();
        }
    }
    if points.is_empty() {
        let p = (support(a, n) + support(b, -n)) * 0.5;
        points.push(Point {
            ra: p - a.position,
            rb: p - b.position,
            separation,
        });
    }
    Some(Manifold {
        normal: n,
        points,
        swept: false,
        time: 0.0,
    })
}
fn sphere_box(s: &Body, b: &Body, r: Scalar, margin: Scalar) -> Option<Manifold> {
    let Shape::Box(h) = b.shape else {
        unreachable!()
    };
    let local = b.orientation.inverse_rotate(s.position - b.position);
    let closest = V(
        local.0.clamp(-h.0, h.0),
        local.1.clamp(-h.1, h.1),
        local.2.clamp(-h.2, h.2),
    );
    let delta = closest - local;
    let distance = delta.length();
    let (n, separation, p) = if distance > 1e-10 {
        (
            b.orientation.rotate(delta / distance),
            distance - r,
            b.position + b.orientation.rotate(closest),
        )
    } else {
        let axis = (0..3)
            .min_by(|&i, &j| {
                (h.at(i) - local.at(i).abs()).total_cmp(&(h.at(j) - local.at(j).abs()))
            })
            .unwrap();
        let basis = [V::X, V::Y, V::Z][axis] * if local.at(axis) < 0.0 { -1.0 } else { 1.0 };
        let depth = h.at(axis) - local.at(axis).abs();
        let out = b.orientation.rotate(basis);
        (-out, -depth - r, s.position + out * depth)
    };
    if separation > margin {
        return None;
    }
    Some(Manifold {
        normal: n,
        points: vec![Point {
            ra: n * r,
            rb: p - b.position,
            separation,
        }],
        swept: false,
        time: 0.0,
    })
}
pub(super) fn current(a: &Body, b: &Body, margin: Scalar) -> Option<Manifold> {
    match (a.shape, b.shape) {
        (Shape::Box(_), Shape::Box(_)) => box_manifold(a, b, margin),
        (Shape::Sphere(r), Shape::Box(_)) => sphere_box(a, b, r, margin),
        (Shape::Box(_), Shape::Sphere(r)) => sphere_box(b, a, r, margin).map(|mut m| {
            m.normal = -m.normal;
            for p in &mut m.points {
                std::mem::swap(&mut p.ra, &mut p.rb);
            }
            m
        }),
        (Shape::Sphere(ra), Shape::Sphere(rb)) => {
            let d = b.position - a.position;
            let l = d.length();
            let sep = l - ra - rb;
            if sep > margin {
                return None;
            }
            let n = if l > 1e-10 { d / l } else { V::X };
            Some(Manifold {
                normal: n,
                points: vec![Point {
                    ra: n * ra,
                    rb: -n * rb,
                    separation: sep,
                }],
                swept: false,
                time: 0.0,
            })
        }
    }
}

// Earliest ray intersection with the actual rounded box: faces, edge cylinders and corner spheres.
// Unlike an expanded AABB alone, a diagonal near miss does not become a hit.
fn sphere_box_time(s: &Body, b: &Body, r: Scalar, dt: Scalar) -> Option<Scalar> {
    let Shape::Box(h) = b.shape else {
        unreachable!()
    };
    let p = b.orientation.inverse_rotate(s.position - b.position);
    let v = b.orientation.inverse_rotate((s.velocity - b.velocity) * dt);
    let mut best: Scalar = 2.0;
    let mut admit = |t: Scalar| {
        if (0.0..=1.0).contains(&t) {
            let q = p + v * t;
            let c = V(
                q.0.clamp(-h.0, h.0),
                q.1.clamp(-h.1, h.1),
                q.2.clamp(-h.2, h.2),
            );
            if ((q - c).length() - r).abs() <= 1e-6 * (1.0 + r) {
                best = best.min(t);
            }
        }
    };
    for axis in 0..3 {
        for sign in [-1.0, 1.0] {
            if v.at(axis).abs() > 1e-14 {
                admit((sign * (h.at(axis) + r) - p.at(axis)) / v.at(axis));
            }
        }
    }
    for axis in 0..3 {
        let u = (axis + 1) % 3;
        let w = (axis + 2) % 3;
        for su in [-1.0, 1.0] {
            for sw in [-1.0, 1.0] {
                let x = p.at(u) - su * h.at(u);
                let y = p.at(w) - sw * h.at(w);
                let dx = v.at(u);
                let dy = v.at(w);
                for t in roots(
                    dx * dx + dy * dy,
                    2.0 * (x * dx + y * dy),
                    x * x + y * y - r * r,
                )
                .into_iter()
                .flatten()
                {
                    if (p.at(axis) + t * v.at(axis)).abs() <= h.at(axis) + 1e-8 {
                        admit(t);
                    }
                }
            }
        }
    }
    for sx in [-1.0, 1.0] {
        for sy in [-1.0, 1.0] {
            for sz in [-1.0, 1.0] {
                let d = p - V(h.0 * sx, h.1 * sy, h.2 * sz);
                for t in roots(v.dot(v), 2.0 * d.dot(v), d.dot(d) - r * r)
                    .into_iter()
                    .flatten()
                {
                    admit(t);
                }
            }
        }
    }
    (best <= 1.0).then_some(best)
}
fn roots(a: Scalar, b: Scalar, c: Scalar) -> [Option<Scalar>; 2] {
    if a <= 1e-20 {
        return [None, None];
    }
    let d = b * b - 4.0 * a * c;
    if d < 0.0 {
        return [None, None];
    }
    let q = -0.5 * (b + if b < 0.0 { -d.sqrt() } else { d.sqrt() });
    if q.abs() < 1e-20 {
        return [Some(-b / (2.0 * a)), None];
    }
    [Some(q / a), Some(c / q)]
}
/// Translation-only CCD for the current substep. Rotation is integrated between substeps,
/// not analytically swept: this is an explicit approximation, not general rotational CCD.
pub(super) fn swept(a: &Body, b: &Body, dt: Scalar, margin: Scalar) -> Option<Manifold> {
    let time = match (a.shape, b.shape) {
        (Shape::Sphere(r), Shape::Box(_)) => sphere_box_time(a, b, r, dt)?,
        (Shape::Box(_), Shape::Sphere(r)) => sphere_box_time(b, a, r, dt)?,
        (Shape::Sphere(ra), Shape::Sphere(rb)) => {
            let d = b.position - a.position;
            let v = (b.velocity - a.velocity) * dt;
            roots(v.dot(v), 2.0 * d.dot(v), d.dot(d) - (ra + rb) * (ra + rb))
                .into_iter()
                .flatten()
                .filter(|t| (0.0..=1.0).contains(t))
                .min_by(Scalar::total_cmp)?
        }
        (Shape::Box(_), Shape::Box(_)) => {
            let d = b.position - a.position;
            let v = (b.velocity - a.velocity) * dt;
            let mut enter: Scalar = 0.0;
            let mut exit: Scalar = 1.0;
            for (axis, _) in axes(a, b) {
                let r = radius(a, axis) + radius(b, axis);
                let x = d.dot(axis);
                let dx = v.dot(axis);
                if dx.abs() < 1e-14 {
                    if x.abs() > r {
                        return None;
                    }
                    continue;
                }
                let t1 = (-r - x) / dx;
                let t2 = (r - x) / dx;
                enter = enter.max(t1.min(t2));
                exit = exit.min(t1.max(t2));
                if enter > exit {
                    return None;
                }
            }
            if !(0.0..=1.0).contains(&enter) {
                return None;
            }
            enter
        }
    };
    let mut aa = a.clone();
    let mut bb = b.clone();
    aa.position += a.velocity * (dt * time);
    bb.position += b.velocity * (dt * time);
    let mut m = current(&aa, &bb, margin.max(1e-6))?;
    for p in &mut m.points {
        p.separation -= (b.velocity - a.velocity).dot(m.normal) * dt * time;
    }
    m.swept = true;
    m.time = time;
    Some(m)
}
