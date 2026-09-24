use super::{Body, Shape, Vector as V, geometry::GeometryStats, numeric::Scalar, primitive};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(super) struct Point {
    pub ra: V,
    pub rb: V,
    pub separation: Scalar,
}
/// A generated manifold already has at most four points. Keep that bounded result inline,
/// so copying a cache hit into solver scratch never allocates another point vector.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct Points {
    values: [Point; 4],
    len: usize,
}
impl Points {
    fn selected(input: &[Point]) -> Self {
        let mut out = Self {
            values: [Point::default(); 4],
            len: input.len().min(4),
        };
        for i in 0..out.len {
            out.values[i] = input[if input.len() > 4 {
                i * input.len() / 4
            } else {
                i
            }];
        }
        out
    }
    fn one(p: Point) -> Self {
        Self::selected(&[p])
    }
    pub fn len(&self) -> usize {
        self.len
    }
}
impl IntoIterator for Points {
    type Item = Point;
    type IntoIter = std::iter::Take<std::array::IntoIter<Point, 4>>;
    fn into_iter(self) -> Self::IntoIter {
        self.values.into_iter().take(self.len)
    }
}
impl<'a> IntoIterator for &'a Points {
    type Item = &'a Point;
    type IntoIter = std::slice::Iter<'a, Point>;
    fn into_iter(self) -> Self::IntoIter {
        self.values[..self.len].iter()
    }
}
impl<'a> IntoIterator for &'a mut Points {
    type Item = &'a mut Point;
    type IntoIter = std::slice::IterMut<'a, Point>;
    fn into_iter(self) -> Self::IntoIter {
        self.values[..self.len].iter_mut()
    }
}

#[derive(Clone, Debug, PartialEq)]
pub(super) struct Manifold {
    pub normal: V,
    pub points: Points,
    pub swept: bool,
    /// First translation-only time of contact in [0, 1] of this substep.
    pub time: Scalar,
}

pub(super) fn bounds(b: &Body) -> (V, V) {
    let e = primitive::bounds_extents(b);
    (b.position - e, b.position + e)
}
fn radius(b: &Body, n: V) -> Scalar {
    match b.shape {
        Shape::Sphere(r) => r,
        Shape::Capsule {
            half_segment,
            radius,
        } => n.dot(b.orientation.rotate(V::Y)).abs() * half_segment + radius,
        Shape::Box(h) | Shape::Wedge(h) => {
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
/// Polygon buffers belong to the geometry cache, but never carry geometric authority:
/// every fresh manifold starts with freshly calculated incident vertices.
#[derive(Clone, Debug, Default)]
pub(super) struct ClipScratch {
    polygon: Vec<V>,
    clipped: Vec<V>,
    points: Vec<Point>,
}
impl ClipScratch {
    pub fn retained_bytes(&self) -> usize {
        (self.polygon.capacity() + self.clipped.capacity()) * std::mem::size_of::<V>()
            + self.points.capacity() * std::mem::size_of::<Point>()
    }
}
fn clip_into(input: &[V], n: V, limit: Scalar, out: &mut Vec<V>) {
    out.clear();
    if input.is_empty() {
        return;
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
}

fn support(b: &Body, n: V) -> V {
    primitive::support_point(b, n)
}
/// Shape/orientation-dependent support projections. Centers are deliberately not cached here.
#[derive(Clone, Debug, Default)]
pub(super) struct BoxProjections {
    axes: Vec<(V, usize, Scalar, Scalar)>,
}
impl BoxProjections {
    pub fn refresh(&mut self, a: &Body, b: &Body, aa: [V; 3], bb: [V; 3]) {
        self.axes.clear();
        let Shape::Box(ha) = a.shape else {
            unreachable!()
        };
        let Shape::Box(hb) = b.shape else {
            unreachable!()
        };
        let project = |n: V, axes: [V; 3], h: V| {
            // Same products, additions and ordering as radius(), not a regrouped expression.
            n.dot(axes[0]).abs() * h.0 + n.dot(axes[1]).abs() * h.1 + n.dot(axes[2]).abs() * h.2
        };
        for (feature, n) in aa.into_iter().chain(bb).enumerate() {
            self.axes
                .push((n, feature, project(n, aa, ha), project(n, bb, hb)));
        }
        for u in aa {
            for v in bb {
                let n = u.cross(v);
                if n.dot(n) > 1e-12 {
                    let n = n.unit();
                    self.axes
                        .push((n, 6, project(n, aa, ha), project(n, bb, hb)));
                }
            }
        }
    }
    pub fn retained_bytes(&self) -> usize {
        self.axes.capacity() * std::mem::size_of::<(V, usize, Scalar, Scalar)>()
    }
}

fn select_axis(
    a: &Body,
    b: &Body,
    margin: Scalar,
    axes: impl IntoIterator<Item = (V, usize, Scalar, Scalar)>,
    work: &mut GeometryStats,
) -> Option<(Scalar, V, usize)> {
    work.sat_queries += 1;
    let d = b.position - a.position;
    let mut best = (-Scalar::INFINITY, V::X, 0);
    for (axis, feature, ar, br) in axes {
        work.sat_axes_tested += 1;
        let projected = d.dot(axis);
        let sep = projected.abs() - ar - br;
        if sep > margin {
            return None;
        }
        // Prefer face normals at near ties; preserve the original feature order and thresholds.
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
    Some(best)
}

fn box_manifold(a: &Body, b: &Body, margin: Scalar, work: &mut GeometryStats) -> Option<Manifold> {
    let best = select_axis(
        a,
        b,
        margin,
        axes(a, b)
            .into_iter()
            .map(|(n, feature)| (n, feature, radius(a, n), radius(b, n))),
        work,
    )?;
    box_points(
        a,
        b,
        margin,
        best,
        [a.orientation.axes(), b.orientation.axes()],
        work,
        &mut ClipScratch::default(),
    )
}

/// Lazy SAT projections with a small explicit cursor rather than nested iterator state.
/// Axis/feature order and each dot-product expression match the uncached reference.
struct FrameProjections {
    frames: [[V; 3]; 2],
    halves: [V; 2],
    next_axis: usize,
}
impl Iterator for FrameProjections {
    type Item = (V, usize, Scalar, Scalar);

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        while self.next_axis < 15 {
            let index = self.next_axis;
            self.next_axis += 1;
            let (n, feature) = if index < 6 {
                (self.frames[index / 3][index % 3], index)
            } else {
                let edge = index - 6;
                let n = self.frames[0][edge / 3].cross(self.frames[1][edge % 3]);
                if n.dot(n) <= 1e-12 {
                    continue;
                }
                (n.unit(), 6)
            };
            let project = |axes: [V; 3], h: V| {
                n.dot(axes[0]).abs() * h.0 + n.dot(axes[1]).abs() * h.1 + n.dot(axes[2]).abs() * h.2
            };
            return Some((
                n,
                feature,
                project(self.frames[0], self.halves[0]),
                project(self.frames[1], self.halves[1]),
            ));
        }
        None
    }
}
fn frame_projection_axes(a: &Body, b: &Body, frames: [[V; 3]; 2]) -> FrameProjections {
    let (Shape::Box(ha), Shape::Box(hb)) = (a.shape, b.shape) else {
        unreachable!()
    };
    FrameProjections {
        frames,
        halves: [ha, hb],
        next_axis: 0,
    }
}

/// Reuses frames within one read-only pass, without retaining a rotating pair's contact result.
pub(super) fn box_current_with_frames(
    a: &Body,
    b: &Body,
    margin: Scalar,
    frames: [[V; 3]; 2],
    work: &mut GeometryStats,
    scratch: &mut ClipScratch,
) -> Option<Manifold> {
    let projected_axes = frame_projection_axes(a, b, frames);
    let best = select_axis(a, b, margin, projected_axes, work)?;
    box_points(a, b, margin, best, frames, work, scratch)
}

pub(super) fn box_prepared(
    a: &Body,
    b: &Body,
    margin: Scalar,
    projections: &BoxProjections,
    frames: [[V; 3]; 2],
    work: &mut GeometryStats,
    scratch: &mut ClipScratch,
) -> Option<Manifold> {
    let best = select_axis(a, b, margin, projections.axes.iter().copied(), work)?;
    box_points(a, b, margin, best, frames, work, scratch)
}

fn box_points(
    a: &Body,
    b: &Body,
    margin: Scalar,
    best: (Scalar, V, usize),
    frames: [[V; 3]; 2],
    work: &mut GeometryStats,
    scratch: &mut ClipScratch,
) -> Option<Manifold> {
    let (separation, n, feature) = best;
    scratch.points.clear();
    let points = &mut scratch.points;
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
        let [ra, ia] = if swap { [frames[1], frames[0]] } else { frames };
        let ref_center = reference.position + rn * rh.at(face);
        let inc_face = (0..3)
            .max_by(|&i, &j| rn.dot(ia[i]).abs().total_cmp(&rn.dot(ia[j]).abs()))
            .unwrap();
        let ic =
            incident.position - ia[inc_face] * (ih.at(inc_face) * rn.dot(ia[inc_face]).signum());
        let u = (inc_face + 1) % 3;
        let v = (inc_face + 2) % 3;
        scratch.polygon.clear();
        scratch.polygon.extend([
            ic + ia[u] * ih.at(u) + ia[v] * ih.at(v),
            ic - ia[u] * ih.at(u) + ia[v] * ih.at(v),
            ic - ia[u] * ih.at(u) - ia[v] * ih.at(v),
            ic + ia[u] * ih.at(u) - ia[v] * ih.at(v),
        ]);
        for (i, basis) in ra.iter().enumerate() {
            if i != face {
                for sign in [-1.0, 1.0] {
                    let axis = *basis * sign;
                    work.clip_passes += 1;
                    clip_into(
                        &scratch.polygon,
                        axis,
                        reference.position.dot(axis) + rh.at(i),
                        &mut scratch.clipped,
                    );
                    std::mem::swap(&mut scratch.polygon, &mut scratch.clipped);
                }
            }
        }
        for &p in &scratch.polygon {
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
        points: Points::selected(points),
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
        points: Points::one(Point {
            ra: n * r,
            rb: p - b.position,
            separation,
        }),
        swept: false,
        time: 0.0,
    })
}
pub(super) fn current(a: &Body, b: &Body, margin: Scalar) -> Option<Manifold> {
    current_counted(a, b, margin, &mut GeometryStats::default())
}

pub(super) fn current_counted(
    a: &Body,
    b: &Body,
    margin: Scalar,
    work: &mut GeometryStats,
) -> Option<Manifold> {
    work.current_queries += 1;
    work.manifold_refreshes += 1;
    match (a.shape, b.shape) {
        (Shape::Box(_), Shape::Box(_)) => box_manifold(a, b, margin, work),
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
                points: Points::one(Point {
                    ra: n * ra,
                    rb: -n * rb,
                    separation: sep,
                }),
                swept: false,
                time: 0.0,
            })
        }
        _ => primitive::query(a, b, work).and_then(|contact| {
            if contact.separation > margin {
                return None;
            }
            Some(Manifold {
                normal: contact.normal,
                points: Points::one(Point {
                    ra: contact.point_a - a.position,
                    rb: contact.point_b - b.position,
                    separation: contact.separation,
                }),
                swept: false,
                time: 0.0,
            })
        }),
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
pub(super) fn swept(
    a: &Body,
    b: &Body,
    dt: Scalar,
    margin: Scalar,
    work: &mut GeometryStats,
) -> Option<Manifold> {
    work.sweep_queries += 1;
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
        (Shape::Box(_), Shape::Box(_)) => box_sweep_time(
            a,
            b,
            dt,
            axes(a, b)
                .into_iter()
                .map(|(axis, feature)| (axis, feature, radius(a, axis), radius(b, axis))),
        )?,
        _ => primitive::swept_time(a, b, dt, margin, work)?,
    };
    finish_sweep(a, b, dt, time, |aa, bb| {
        current_counted(aa, bb, margin.max(1e-6), work)
    })
}

fn box_sweep_time(
    a: &Body,
    b: &Body,
    dt: Scalar,
    projections: impl IntoIterator<Item = (V, usize, Scalar, Scalar)>,
) -> Option<Scalar> {
    let d = b.position - a.position;
    let v = (b.velocity - a.velocity) * dt;
    let mut enter: Scalar = 0.0;
    let mut exit: Scalar = 1.0;
    for (axis, _, ar, br) in projections {
        let r = ar + br;
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
    Some(enter)
}

/// Only orientation-derived axes are reused; requested time, trajectory and first hit are fresh.
pub(super) fn box_swept_with_frames(
    a: &Body,
    b: &Body,
    dt: Scalar,
    margin: Scalar,
    frames: [[V; 3]; 2],
    work: &mut GeometryStats,
    scratch: &mut ClipScratch,
) -> Option<Manifold> {
    work.sweep_queries += 1;
    let time = box_sweep_time(a, b, dt, frame_projection_axes(a, b, frames))?;
    finish_sweep(a, b, dt, time, |aa, bb| {
        work.current_queries += 1;
        work.manifold_refreshes += 1;
        box_current_with_frames(aa, bb, margin.max(1e-6), frames, work, scratch)
    })
}

fn finish_sweep(
    a: &Body,
    b: &Body,
    dt: Scalar,
    time: Scalar,
    mut current: impl FnMut(&Body, &Body) -> Option<Manifold>,
) -> Option<Manifold> {
    let mut aa = a.clone();
    let mut bb = b.clone();
    aa.position += a.velocity * (dt * time);
    bb.position += b.velocity * (dt * time);
    let mut m = current(&aa, &bb)?;
    for p in &mut m.points {
        p.separation -= (b.velocity - a.velocity).dot(m.normal) * dt * time;
    }
    m.swept = true;
    m.time = time;
    Some(m)
}

#[cfg(test)]
mod tests;
