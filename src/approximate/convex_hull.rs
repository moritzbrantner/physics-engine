//! Static authored convex rocks (#180). Hull construction and support-map algorithms are
//! provided by geometry-kernels; the world owns admission, IDs and query policy.

use super::{
    Body, BodyId, Error, Material, Real, Shape, SweepFailure, Vector as V, World, primitive,
    terrain,
};
use geometry_kernels::{
    epa3::{Epa3Status, epa_penetration_3d},
    gjk_distance::{GjkDistanceConfig, GjkDistanceStatus, gjk_distance_with_config},
    hull::{Hull3, HullDimension, QuickHullWork, quickhull3},
    math3::{Vec3, cross, dot, length, scale, sub},
    segment_hull::{PreparedSegmentHull, SegmentHullWork},
    support::{ConvexHull3, SupportMap3},
};

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct ConvexHullId(pub(super) u32);

/// Local vertices relative to `position` (same scene units as character and rigid bodies).
/// The input may contain interior and duplicate points; admission retains only the canonical
/// 3D convex boundary. Hulls are static; no dynamic inertia or hull×hull solver is implied.
#[derive(Clone, Debug, PartialEq)]
pub struct ConvexHullDesc {
    pub vertices: Vec<V>,
    pub position: V,
    pub material: Material,
}

#[derive(Clone, Copy, Debug)]
struct Plane {
    normal: Vec3,
    offset: f64,
    face: u32,
}

#[derive(Debug)]
pub(super) struct PreparedHull {
    hull: PreparedSegmentHull,
    planes: Vec<Plane>,
}

/// Contact from the moving shape toward the static hull. The query API reverses this normal
/// to give the outward target normal, just like its other primitive kernels.
#[derive(Clone, Copy, Debug)]
pub(super) struct Contact {
    pub separation: Real,
    pub normal: V,
    pub point_query: V,
    pub point_hull: V,
}

pub(super) struct RayContact {
    pub fraction: f64,
    pub normal: Vec3,
    pub face: Option<u32>,
    pub starts_inside: bool,
}

enum CapsuleClip {
    Miss,
    Face(Real, Contact),
    /// Necessary (conservative) Minkowski halfspace-entry fraction; an edge/corner
    /// still needs the exact support-map kernel from this point onward.
    LowerBound(f64),
}

#[derive(Clone, Copy, Debug, Default)]
pub(super) struct HullWork {
    pub segment_queries: u64,
    pub segment_plane_tests: u64,
    pub segment_triangles_considered: u64,
    pub segment_triangles_pruned: u64,
    pub segment_edges_considered: u64,
    pub segment_edges_pruned: u64,
    pub distance_queries: u64,
    pub gjk_iterations: u64,
    pub gjk_intersection_iterations: u64,
    pub gjk_limit_queries: u64,
    pub epa_iterations: u64,
    pub face_planes_tested: u64,
    pub face_hits: u64,
}

impl World {
    /// Adds a *fixed* convex collision rock, with a game key for stable query tie-breaking.
    pub fn add_convex_hull(
        &mut self,
        desc: ConvexHullDesc,
        key: Option<u64>,
    ) -> Result<BodyId, Error> {
        if !desc.position.finite() || desc.position.abs().max_component() >= 1e12 {
            return Err(Error::InvalidInput);
        }
        let (prepared, half) = prepare(&desc.vertices)?;
        let id = ConvexHullId(u32::try_from(self.hulls.len()).map_err(|_| Error::InvalidInput)?);
        self.hulls.push(Some(std::sync::Arc::new(prepared)));
        let mut body = Body::new(
            Shape::ConvexHull {
                hull: id,
                half_extents: half,
            },
            desc.position,
            0.0,
        );
        body.key = key;
        body.set_material(desc.material);
        self.insert_body(body).inspect_err(|_| {
            self.hulls.pop();
        })
    }

    pub(super) fn convex_hull(&self, id: ConvexHullId) -> &PreparedHull {
        self.hulls[id.0 as usize]
            .as_deref()
            .expect("live convex-hull body has geometry")
    }
}

/// Canonical preparation shared by live admission and checkpoint reconstruction.
pub(super) fn prepare(vertices: &[V]) -> Result<(PreparedHull, V), Error> {
    if vertices.len() < 4
        || vertices.len() > 65_536
        || vertices
            .iter()
            .any(|v| !v.finite() || v.abs().max_component() >= 1e12)
    {
        return Err(Error::InvalidInput);
    }
    let points: Vec<Vec3> = vertices.iter().copied().map(to64).collect();
    let mut hull = Hull3::default();
    quickhull3(&points, &mut QuickHullWork::default(), &mut hull);
    if hull.dimension != HullDimension::Polytope || hull.triangles.is_empty() {
        return Err(Error::InvalidInput);
    }
    let mut maximum = V::ZERO;
    for p in &hull.vertices {
        maximum = maximum.max(V(
            p[0].abs() as Real,
            p[1].abs() as Real,
            p[2].abs() as Real,
        ));
    }
    // The authored centre is retained for asymmetric hulls. Bounds are conservative.
    let half = V(
        primitive::round_up(f64::from(maximum.0)),
        primitive::round_up(f64::from(maximum.1)),
        primitive::round_up(f64::from(maximum.2)),
    );
    if !half.finite() || half.min_component() < 1e-6 {
        return Err(Error::InvalidInput);
    }
    let mut planes = Vec::with_capacity(hull.triangles.len());
    for (face, &triangle) in hull.triangles.iter().enumerate() {
        let [a, b, c] = triangle.map(|i| hull.vertices[i as usize]);
        let n = cross(sub(b, a), sub(c, a));
        let magnitude = length(n);
        if !magnitude.is_finite() || magnitude <= 0.0 {
            return Err(Error::InvalidInput);
        }
        let normal = scale(n, magnitude.recip());
        planes.push(Plane {
            normal,
            offset: dot(normal, a),
            face: face as u32,
        });
    }
    let hull = PreparedSegmentHull::new(hull).ok_or(Error::InvalidInput)?;
    Ok((PreparedHull { hull, planes }, half))
}

impl PreparedHull {
    /// A capsule–hull Minkowski sum lies *inside* the intersection of the hull
    /// planes offset by the capsule support radius. The clip therefore safely
    /// rejects misses and finds an earliest possible TOI. If the capsule's
    /// supporting point on the entering plane lies on the original face,
    /// the TOI is exact; corner and edge cases go to GJK/EPA below.
    fn capsule_face_cast(
        &self,
        pose: &Body,
        target: &Body,
        displacement: V,
        work: &mut HullWork,
    ) -> Option<CapsuleClip> {
        let (p, q, radius) = terrain::capsule_of(pose)?;
        let origin = to64(target.position);
        let p = sub(p, origin);
        let q = sub(q, origin);
        let d = to64(displacement);
        let mut enter = 0.0_f64;
        let mut exit = 1.0_f64;
        let mut entering_face = None;
        let mut initially_outside = false;
        for plane in &self.planes {
            work.face_planes_tested += 1;
            let signed = dot(plane.normal, p).min(dot(plane.normal, q)) - radius - plane.offset;
            let motion = dot(plane.normal, d);
            if signed > 0.0 {
                initially_outside = true;
            }
            if motion == 0.0 {
                if signed > 0.0 {
                    return Some(CapsuleClip::Miss);
                }
                continue;
            }
            let t = -signed / motion;
            if motion < 0.0 && t > enter {
                enter = t;
                entering_face = Some(plane);
            } else if motion > 0.0 {
                exit = exit.min(t);
            }
            if enter > exit {
                return Some(CapsuleClip::Miss);
            }
        }
        if enter > 1.0 || exit < 0.0 {
            return Some(CapsuleClip::Miss);
        }
        if !initially_outside || enter == 0.0 {
            return Some(CapsuleClip::LowerBound(0.0));
        }
        let plane = entering_face?;
        let endpoint = if dot(plane.normal, p) <= dot(plane.normal, q) {
            p
        } else {
            q
        };
        let surface = sub(
            std::array::from_fn(|i| endpoint[i] + d[i] * enter),
            scale(plane.normal, radius),
        );
        // Point-on-face, not merely point-on-its-infinite-plane. The other
        // hull halfspaces also handle triangulated coplanar face polygons.
        for other in &self.planes {
            work.face_planes_tested += 1;
            if dot(other.normal, surface) > other.offset + 1.0e-8 {
                return Some(CapsuleClip::LowerBound(enter));
            }
        }
        work.face_hits += 1;
        let world = to_real(std::array::from_fn(|i| surface[i] + origin[i]));
        Some(CapsuleClip::Face(
            primitive::toward_zero(enter),
            Contact {
                separation: 0.0,
                normal: -to_real(plane.normal),
                point_query: world,
                point_hull: world,
            },
        ))
    }

    #[cfg(feature = "checkpoints")]
    pub(super) fn checkpoint_vertices(&self) -> Vec<V> {
        self.hull
            .hull()
            .vertices
            .iter()
            .copied()
            .map(to_real)
            .collect()
    }
}

fn to64(v: V) -> Vec3 {
    [f64::from(v.0), f64::from(v.1), f64::from(v.2)]
}
fn to_real(v: Vec3) -> V {
    V(
        primitive::nearest(v[0]),
        primitive::nearest(v[1]),
        primitive::nearest(v[2]),
    )
}

/// Capsule in a hull-local frame, with f64 endpoints so distant world origins subtract
/// before query geometry is evaluated. This is only a coordinate adapter for the upstream
/// support-map algorithms, not a second GJK/distance implementation.
struct CapsuleSupport {
    p: Vec3,
    q: Vec3,
    radius: f64,
}
impl SupportMap3 for CapsuleSupport {
    fn support_point(&self, direction: Vec3) -> Vec3 {
        let end = if dot(self.p, direction) >= dot(self.q, direction) {
            self.p
        } else {
            self.q
        };
        let len = length(direction);
        let n = if len > 0.0 {
            scale(direction, len.recip())
        } else {
            [1.0, 0.0, 0.0]
        };
        std::array::from_fn(|i| end[i] + n[i] * self.radius)
    }
}

impl PreparedHull {
    fn capsule_contact(
        &self,
        pose: &Body,
        target: &Body,
        work: &mut HullWork,
    ) -> Result<Contact, SweepFailure> {
        let Some((p, q, r)) = terrain::capsule_of(pose) else {
            return Err(SweepFailure::InvalidGeometryInput);
        };
        let origin = to64(target.position);
        let query = CapsuleSupport {
            p: sub(p, origin),
            q: sub(q, origin),
            radius: r,
        };
        let mut segment_work = SegmentHullWork::default();
        let segment_closest = self
            .hull
            .closest_with_work(query.p, query.q, &mut segment_work);
        work.segment_queries += 1;
        work.segment_plane_tests += segment_work.separating_plane_tests;
        work.segment_triangles_considered += segment_work.triangles_considered;
        work.segment_triangles_pruned += segment_work.triangles_pruned;
        work.segment_edges_considered += segment_work.edges_considered;
        work.segment_edges_pruned += segment_work.edges_pruned;
        if let Some(closest) = segment_closest
            && closest.distance > 1e-9
        {
            let normal = scale(
                sub(closest.point_hull, closest.point_segment),
                closest.distance.recip(),
            );
            return Ok(Contact {
                separation: primitive::round_down(closest.distance - query.radius),
                normal: to_real(normal),
                point_query: to_real(std::array::from_fn(|i| {
                    closest.point_segment[i] + query.radius * normal[i] + origin[i]
                })),
                point_hull: to_real(std::array::from_fn(|i| closest.point_hull[i] + origin[i])),
            });
        }
        let hull = ConvexHull3::new(&self.hull.hull().vertices);
        // The public world/query coordinates are f32. The generic kernel defaults to
        // 1e-12 f64 tolerances and can spend dozens of no-progress iterations on an
        // f32-quantized capsule position. The oracle below keeps this query tolerance
        // tied to the actual scene precision, not unattainable f64 input precision.
        let result = gjk_distance_with_config(
            &query,
            &hull,
            GjkDistanceConfig {
                max_iterations: 32,
                relative_tolerance: 1e-7,
                epsilon: 1e-12,
            },
        );
        work.distance_queries += 1;
        work.gjk_iterations += result.iterations as u64;
        work.gjk_intersection_iterations += result.intersection.iterations as u64;
        work.gjk_limit_queries += u64::from(result.status == GjkDistanceStatus::IterationLimit);
        let (separation, normal, query_point, target_point) = match result.status {
            GjkDistanceStatus::Converged
            | GjkDistanceStatus::NoProgress
            | GjkDistanceStatus::IterationLimit
                if result.closest.is_some() =>
            {
                let closest = result.closest.expect("checked above");
                (
                    closest.distance,
                    closest.normal,
                    closest.point_left,
                    closest.point_right,
                )
            }
            GjkDistanceStatus::Intersecting => {
                let epa = epa_penetration_3d(&query, &hull, &result.intersection);
                work.epa_iterations += epa.iterations as u64;
                if epa.status != Epa3Status::Converged {
                    return Err(SweepFailure::IterationLimit);
                }
                let p = epa.penetration.ok_or(SweepFailure::IterationLimit)?;
                (-p.depth, p.normal, p.point_left, p.point_right)
            }
            _ => return Err(SweepFailure::IterationLimit),
        };
        if !separation.is_finite() || !normal.iter().all(|x| x.is_finite()) {
            return Err(SweepFailure::NonFiniteComputation);
        }
        let local = to64(target.position);
        Ok(Contact {
            separation: primitive::round_down(separation),
            normal: to_real(normal),
            point_query: to_real(std::array::from_fn(|i| query_point[i] + local[i])),
            point_hull: to_real(std::array::from_fn(|i| target_point[i] + local[i])),
        })
    }

    /// Conservative advancement: the distance kernel gives a supporting separation plane;
    /// advancing by no more than distance / relative closing speed cannot skip a convex hit.
    pub(super) fn capsule_query(
        &self,
        query: &Body,
        target: &Body,
        displacement: V,
        max_iterations: Option<u32>,
        work: &mut HullWork,
    ) -> Result<Option<(Real, Contact, bool, u32)>, SweepFailure> {
        let mut t = 0.0_f64;
        if max_iterations.is_some() {
            match self.capsule_face_cast(query, target, displacement, work) {
                Some(CapsuleClip::Miss) => return Ok(None),
                Some(CapsuleClip::Face(fraction, contact)) => {
                    return Ok(Some((fraction, contact, false, 0)));
                }
                Some(CapsuleClip::LowerBound(lower)) if lower > 0.0 => {
                    // The supporting-plane envelope cannot be hit before this
                    // point. Stop just short for f32 world-position rounding.
                    t = (lower - 1.0e-6).max(0.0);
                }
                _ => {}
            }
        }
        let mut moving = query.clone();
        if t > 0.0 {
            let d = to64(displacement);
            moving.position = to_real(std::array::from_fn(|i| {
                f64::from(query.position.at(i)) + d[i] * t
            }));
        }
        let initial = self.capsule_contact(&moving, target, work)?;
        if max_iterations.is_none() || initial.separation <= 0.0 {
            return Ok((initial.separation <= 0.0).then_some((
                primitive::toward_zero(t),
                initial,
                t == 0.0 && initial.separation < 0.0,
                0,
            )));
        }
        let d = to64(displacement);
        let mut iterations = 0u32;
        let mut contact = initial;
        let tolerance = (1e-6f64).max(f64::from(query.shape.radius()) * 1e-6);
        while iterations < max_iterations.expect("present above") {
            iterations += 1;
            if f64::from(contact.separation) <= tolerance {
                return Ok(Some((
                    primitive::toward_zero(t),
                    contact,
                    false,
                    iterations,
                )));
            }
            // GJK returns a normal from the query toward the hull: travel in that
            // direction closes the gap. The public query hit reverses the normal.
            let closing = dot(to64(contact.normal), d);
            if closing <= 0.0 {
                return Ok(None);
            }
            let advance = f64::from(contact.separation) / closing;
            if !advance.is_finite() || advance <= 0.0 {
                return Err(SweepFailure::NonFiniteComputation);
            }
            t += advance * 0.999_9;
            if t > 1.0 {
                return Ok(None);
            }
            moving.position = to_real(std::array::from_fn(|i| {
                f64::from(query.position.at(i)) + d[i] * t
            }));
            contact = self.capsule_contact(&moving, target, work)?;
        }
        Err(SweepFailure::IterationLimit)
    }

    /// Segment clipping against the outward face planes (a convex polytope's exact halfspaces).
    pub(super) fn ray(&self, target: &Body, origin: V, displacement: V) -> Option<RayContact> {
        let o = sub(to64(origin), to64(target.position));
        let d = to64(displacement);
        let mut enter = 0.0f64;
        let mut exit = 1.0f64;
        let mut face = None;
        let mut inside = true;
        for plane in &self.planes {
            let signed = dot(plane.normal, o) - plane.offset;
            inside &= signed < -1e-12;
            let velocity = dot(plane.normal, d);
            if velocity == 0.0 {
                if signed > 0.0 {
                    return None;
                }
                continue;
            }
            let fraction = -signed / velocity;
            if velocity < 0.0 && fraction > enter {
                enter = fraction;
                face = Some(plane.face);
            } else if velocity > 0.0 {
                exit = exit.min(fraction);
            }
            if enter > exit {
                return None;
            }
        }
        if !(0.0..=1.0).contains(&enter) || exit < 0.0 {
            return None;
        }
        let normal = face.map_or_else(
            || {
                let len = length(d);
                scale(d, -len.recip())
            },
            |id| self.planes[id as usize].normal,
        );
        Some(RayContact {
            fraction: enter,
            normal,
            face,
            starts_inside: inside,
        })
    }
}
