//! Deterministic regular icosahedron and dodecahedron fixtures (#465), in f64 and independent
//! of the engine: vertices for an edge length, face planes and edge directions found by brute
//! force, and an exact translational first contact against an axis-aligned box from the
//! complete separating-axis set of two convex polyhedra. These are test references only; the
//! engine collides the solids' circumsphere proxies as `Shape::Sphere`.

pub(crate) type V3 = [f64; 3];

pub(crate) const PHI: f64 = std::f64::consts::GOLDEN_RATIO;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Solid {
    Icosahedron,
    Dodecahedron,
}

impl Solid {
    pub(crate) const ALL: [Self; 2] = [Self::Icosahedron, Self::Dodecahedron];

    /// Closed-form circumradius for edge length `a`.
    pub(crate) fn circumradius(self, a: f64) -> f64 {
        match self {
            Self::Icosahedron => a / 4.0 * (10.0 + 2.0 * 5f64.sqrt()).sqrt(),
            Self::Dodecahedron => a * 3f64.sqrt() / 4.0 * (1.0 + 5f64.sqrt()),
        }
    }

    /// Closed-form inradius for edge length `a`.
    pub(crate) fn inradius(self, a: f64) -> f64 {
        match self {
            Self::Icosahedron => a * 3f64.sqrt() / 12.0 * (3.0 + 5f64.sqrt()),
            Self::Dodecahedron => a / 2.0 * (2.5 + 1.1 * 5f64.sqrt()).sqrt(),
        }
    }

    pub(crate) fn faces(self) -> usize {
        match self {
            Self::Icosahedron => 20,
            Self::Dodecahedron => 12,
        }
    }
}

/// A regular solid centred at the origin of its body frame.
#[derive(Clone, Debug)]
pub(crate) struct Polyhedron {
    pub(crate) solid: Solid,
    pub(crate) edge: f64,
    pub(crate) vertices: Vec<V3>,
    /// Outward unit face normals, one per face.
    pub(crate) normals: Vec<V3>,
    /// Distance of each face plane from the centre (the same for every face).
    pub(crate) plane_offsets: Vec<f64>,
    /// Unit edge directions, one per edge.
    pub(crate) edges: Vec<V3>,
}

pub(crate) fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
pub(crate) fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
pub(crate) fn scale(a: V3, s: f64) -> V3 {
    [a[0] * s, a[1] * s, a[2] * s]
}
pub(crate) fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
pub(crate) fn cross(a: V3, b: V3) -> V3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
pub(crate) fn length(a: V3) -> f64 {
    dot(a, a).sqrt()
}
pub(crate) fn unit(a: V3) -> V3 {
    scale(a, 1.0 / length(a))
}

/// Unit quaternion `[x, y, z, w]`.
pub(crate) type Quat = [f64; 4];

pub(crate) fn axis_angle(axis: V3, angle: f64) -> Quat {
    let a = unit(axis);
    let (s, c) = (angle * 0.5).sin_cos();
    [a[0] * s, a[1] * s, a[2] * s, c]
}

pub(crate) fn mul(a: Quat, b: Quat) -> Quat {
    [
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    ]
}

pub(crate) fn rotate(q: Quat, v: V3) -> V3 {
    let u = [q[0], q[1], q[2]];
    let t = scale(cross(u, v), 2.0);
    add(add(v, scale(t, q[3])), cross(u, t))
}

/// The rotation taking unit vector `from` onto unit vector `to`.
pub(crate) fn between(from: V3, to: V3) -> Quat {
    let (from, to) = (unit(from), unit(to));
    let c = dot(from, to);
    if c < -1.0 + 1e-12 {
        let other = if from[0].abs() < 0.9 {
            [1.0, 0.0, 0.0]
        } else {
            [0.0, 1.0, 0.0]
        };
        return axis_angle(cross(from, other), std::f64::consts::PI);
    }
    let axis = cross(from, to);
    let w = 1.0 + c;
    let n = (dot(axis, axis) + w * w).sqrt();
    [axis[0] / n, axis[1] / n, axis[2] / n, w / n]
}

/// Deterministic xorshift for orientation sweeps.
pub(crate) struct Rng(pub(crate) u64);
impl Rng {
    pub(crate) fn unit(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
    /// A uniformly distributed unit quaternion (Shoemake).
    pub(crate) fn rotation(&mut self) -> Quat {
        let (u1, u2, u3) = (self.unit(), self.unit(), self.unit());
        let tau = std::f64::consts::TAU;
        let (a, b) = ((1.0 - u1).sqrt(), u1.sqrt());
        [
            a * (tau * u2).sin(),
            a * (tau * u2).cos(),
            b * (tau * u3).sin(),
            b * (tau * u3).cos(),
        ]
    }
}

fn cyclic(v: V3) -> [V3; 3] {
    [v, [v[2], v[0], v[1]], [v[1], v[2], v[0]]]
}

fn signs(v: V3) -> Vec<V3> {
    let mut out = Vec::new();
    for sx in [1.0, -1.0] {
        for sy in [1.0, -1.0] {
            for sz in [1.0, -1.0] {
                let p = [v[0] * sx, v[1] * sy, v[2] * sz];
                if !out.contains(&p) {
                    out.push(p);
                }
            }
        }
    }
    out
}

impl Polyhedron {
    /// The regular solid with edge length `edge`, in its canonical (golden-rectangle) frame.
    pub(crate) fn new(solid: Solid, edge: f64) -> Self {
        let (raw, raw_edge): (Vec<V3>, f64) = match solid {
            Solid::Icosahedron => (
                cyclic([0.0, 1.0, PHI])
                    .into_iter()
                    .flat_map(signs)
                    .collect(),
                2.0,
            ),
            Solid::Dodecahedron => (
                signs([1.0, 1.0, 1.0])
                    .into_iter()
                    .chain(cyclic([0.0, 1.0 / PHI, PHI]).into_iter().flat_map(signs))
                    .collect(),
                2.0 / PHI,
            ),
        };
        let vertices: Vec<V3> = raw.iter().map(|&v| scale(v, edge / raw_edge)).collect();
        let tol = 1e-9 * edge;
        let mut normals: Vec<V3> = Vec::new();
        let mut plane_offsets = Vec::new();
        let n = vertices.len();
        for i in 0..n {
            for j in i + 1..n {
                for k in j + 1..n {
                    let c = cross(sub(vertices[j], vertices[i]), sub(vertices[k], vertices[i]));
                    if length(c) < tol * edge {
                        continue;
                    }
                    let mut normal = unit(c);
                    if dot(normal, vertices[i]) < 0.0 {
                        normal = scale(normal, -1.0);
                    }
                    let offset = dot(normal, vertices[i]);
                    if vertices.iter().all(|&v| dot(normal, v) <= offset + tol)
                        && !normals.iter().any(|&m| dot(m, normal) > 1.0 - 1e-9)
                    {
                        normals.push(normal);
                        plane_offsets.push(offset);
                    }
                }
            }
        }
        let mut edges = Vec::new();
        for i in 0..n {
            for j in i + 1..n {
                let d = sub(vertices[j], vertices[i]);
                if (length(d) - edge).abs() < tol {
                    edges.push(unit(d));
                }
            }
        }
        Self {
            solid,
            edge,
            vertices,
            normals,
            plane_offsets,
            edges,
        }
    }

    pub(crate) fn circumradius(&self) -> f64 {
        self.vertices.iter().map(|&v| length(v)).fold(0.0, f64::max)
    }

    pub(crate) fn inradius(&self) -> f64 {
        self.plane_offsets
            .iter()
            .copied()
            .fold(f64::INFINITY, f64::min)
    }

    /// Support distance of the solid rotated by `q` along unit direction `n`.
    pub(crate) fn support(&self, q: Quat, n: V3) -> f64 {
        self.vertices
            .iter()
            .map(|&v| dot(rotate(q, v), n))
            .fold(f64::NEG_INFINITY, f64::max)
    }

    /// The orientation whose face (`Face`), vertex (`Vertex`) or edge midpoint (`Edge`) leads
    /// along unit direction `dir`.
    pub(crate) fn leading(&self, feature: Feature, dir: V3) -> Quat {
        let local = match feature {
            Feature::Face => self.normals[0],
            Feature::Vertex => unit(self.vertices[0]),
            Feature::Edge => {
                let a = self.vertices[0];
                let b = *self
                    .vertices
                    .iter()
                    .find(|&&b| (length(sub(b, a)) - self.edge).abs() < 1e-9 * self.edge)
                    .unwrap();
                unit(add(a, b))
            }
        };
        between(local, dir)
    }

    /// Separating axes of this solid (rotated by `q`) against an axis-aligned box: the face
    /// normals of both and every cross product of their edge directions. Complete for two
    /// convex polyhedra.
    fn axes_against_box(&self, q: Quat) -> Vec<V3> {
        let boxes = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut axes: Vec<V3> = self.normals.iter().map(|&n| rotate(q, n)).collect();
        axes.extend(boxes);
        for &e in &self.edges {
            let e = rotate(q, e);
            for b in boxes {
                let c = cross(e, b);
                if length(c) > 1e-9 {
                    axes.push(unit(c));
                }
            }
        }
        axes
    }

    /// Exact first contact (fraction of `displacement` in `0..=1`, touching included) of the
    /// solid at orientation `q`, translating from `start` by `displacement`, with the box
    /// `center ± half`. `None` when the swept solid never touches the box in the interval.
    /// Per axis the overlap condition is linear in time, so the contact set is the
    /// intersection of the axes' time intervals.
    pub(crate) fn box_toi(
        &self,
        q: Quat,
        start: V3,
        displacement: V3,
        center: V3,
        half: V3,
    ) -> Option<f64> {
        self.box_contact(q, start, displacement, center, half)
            .map(|(t, _)| t)
    }

    /// [`Self::box_toi`] with the contact normal (outward from the box towards the solid): the
    /// separating axis whose interval is entered last. Zero for an initial overlap.
    pub(crate) fn box_contact(
        &self,
        q: Quat,
        start: V3,
        displacement: V3,
        center: V3,
        half: V3,
    ) -> Option<(f64, V3)> {
        let (mut enter, mut exit) = (0.0f64, 1.0f64);
        let mut normal = [0.0; 3];
        for n in self.axes_against_box(q) {
            let hmax = self.support(q, n);
            let hmin = -self.support(q, scale(n, -1.0));
            let radius = half[0] * n[0].abs() + half[1] * n[1].abs() + half[2] * n[2].abs();
            let (bmin, bmax) = (dot(n, center) - radius, dot(n, center) + radius);
            // s(t) = n·start + t·n·d must lie in [bmin − hmax, bmax − hmin].
            let (lo, hi) = (bmin - hmax, bmax - hmin);
            let s0 = dot(n, start);
            let ds = dot(n, displacement);
            if ds.abs() < 1e-15 {
                if s0 < lo || s0 > hi {
                    return None;
                }
                continue;
            }
            let (a, b) = ((lo - s0) / ds, (hi - s0) / ds);
            if a.min(b) > enter {
                enter = a.min(b);
                // Entering through `lo` approaches from the box's −n side.
                normal = if a < b { scale(n, -1.0) } else { n };
            }
            exit = exit.min(a.max(b));
            if enter > exit {
                return None;
            }
        }
        Some((enter, normal))
    }

    /// Whether the solid at `position` / `q` touches or overlaps the box (static SAT).
    pub(crate) fn overlaps_box(&self, q: Quat, position: V3, center: V3, half: V3) -> bool {
        self.box_toi(q, position, [0.0; 3], center, half).is_some()
    }

    /// First contact of a solid that rotates about its centre with constant angular velocity
    /// `omega` (rad/s) while translating with constant `velocity` over `dt`, against the box.
    /// Dense sampling followed by bisection on the exact static SAT overlap: an independent
    /// reference for the rotating-visual case (which only ever makes the solid's swept volume
    /// smaller than its circumsphere's).
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn rotating_box_toi(
        &self,
        q0: Quat,
        omega: V3,
        start: V3,
        velocity: V3,
        dt: f64,
        center: V3,
        half: V3,
        samples: usize,
    ) -> Option<f64> {
        let speed = length(omega);
        let pose = |t: f64| {
            let q = if speed == 0.0 {
                q0
            } else {
                mul(axis_angle(omega, speed * t), q0)
            };
            (q, add(start, scale(velocity, t)))
        };
        let touches = |t: f64| {
            let (q, p) = pose(t);
            self.overlaps_box(q, p, center, half)
        };
        if touches(0.0) {
            return Some(0.0);
        }
        let mut before = 0.0;
        for i in 1..=samples {
            let t = dt * i as f64 / samples as f64;
            if touches(t) {
                let (mut lo, mut hi) = (before, t);
                for _ in 0..60 {
                    let mid = 0.5 * (lo + hi);
                    if touches(mid) { hi = mid } else { lo = mid }
                }
                return Some(hi);
            }
            before = t;
        }
        None
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) enum Feature {
    Face,
    Edge,
    Vertex,
}
