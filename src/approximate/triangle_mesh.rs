//! Immutable authored static surfaces. Geometry and acceleration remain in rust-kernels.

use super::{
    Body, BodyId, Error, Material, Real, Shape, SweepFailure, Vector as V, World, primitive,
};
use geometry_kernels::triangle_mesh::{
    PreparedTriangleMesh3, TriangleMeshData3, TriangleMeshError3, TriangleMeshWork3,
};
use std::sync::Arc;

/// Stable geometry slot, retained after its body is removed.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct TriangleMeshId(pub(super) u32);

/// Authored local vertices and input-ordered triangles. Surfaces are two-sided; holes stay
/// open. Invalid indices, non-finite vertices and degenerate triangles are rejected.
/// Geometry is immutable after admission; move its fixed body or remove/re-admit a chunk.
#[derive(Clone, Debug, PartialEq)]
pub struct TriangleMeshDesc {
    pub position: V,
    pub vertices: Vec<V>,
    pub triangles: Vec<[u32; 3]>,
    pub material: Material,
}

/// Immutable retained geometry, excluding the world's body/query tree and allocator overhead.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TriangleMeshStats {
    pub triangles: u64,
    pub nodes: u64,
    pub retained_bytes: usize,
}

#[derive(Clone, Debug)]
pub(super) struct TriangleMesh {
    pub prepared: Arc<PreparedTriangleMesh3>,
}

/// A mesh prepared once ([`Self::new`], its local BVH build) and admitted into any number of
/// worlds with [`World::add_prepared_triangle_mesh`]. Clones and admissions share the
/// immutable prepared bytes through an `Arc`.
#[derive(Clone, Debug)]
pub struct PreparedTriangleMesh {
    prepared: Arc<PreparedTriangleMesh3>,
    centre: V,
    half_extents: V,
}

impl PreparedTriangleMesh {
    /// Prepares authored local vertices and triangles (see [`TriangleMeshDesc`]; same validation
    /// as [`World::add_triangle_mesh`]).
    pub fn new(vertices: &[V], triangles: Vec<[u32; 3]>) -> Result<Self, Error> {
        if vertices.is_empty()
            || vertices
                .iter()
                .any(|v| !v.finite() || v.abs().max_component() >= 1e12)
        {
            return Err(Error::InvalidInput);
        }
        let (min, max) = vertices.iter().fold(
            (
                V(Real::INFINITY, Real::INFINITY, Real::INFINITY),
                V(Real::NEG_INFINITY, Real::NEG_INFINITY, Real::NEG_INFINITY),
            ),
            |(min, max), &v| (min.min(v), max.max(v)),
        );
        let centre = min * 0.5 + max * 0.5;
        let vertices = vertices
            .iter()
            .map(|v| {
                [
                    f64::from(v.0) - f64::from(centre.0),
                    f64::from(v.1) - f64::from(centre.1),
                    f64::from(v.2) - f64::from(centre.2),
                ]
            })
            .collect();
        let prepared = PreparedTriangleMesh3::try_new(TriangleMeshData3 {
            vertices,
            triangles,
        })
        .map_err(|_| Error::InvalidInput)?;
        let (min, max) = prepared.bounds();
        let half_extents = V(
            primitive::round_up(min[0].abs().max(max[0].abs())),
            primitive::round_up(min[1].abs().max(max[1].abs())),
            primitive::round_up(min[2].abs().max(max[2].abs())),
        );
        crate::work::add(
            crate::work::Counter::MeshPreparations,
            prepared.data().triangles.len() as u64,
        );
        Ok(Self {
            prepared: Arc::new(prepared),
            centre,
            half_extents,
        })
    }
}

pub(super) fn failure(error: TriangleMeshError3) -> SweepFailure {
    match error {
        TriangleMeshError3::NonFiniteComputation => SweepFailure::NonFiniteComputation,
        TriangleMeshError3::IterationLimit => SweepFailure::IterationLimit,
        _ => SweepFailure::InvalidGeometryInput,
    }
}

pub(super) fn counted(work: TriangleMeshWork3, stats: &mut super::QueryStats) {
    stats.mesh_nodes_visited += work.nodes_visited;
    stats.mesh_triangles_tested += work.triangles_tested;
    stats.sweep_iterations += work.sweep_iterations;
    crate::work::add(crate::work::Counter::MeshNodes, work.nodes_visited);
    crate::work::add(crate::work::Counter::MeshTriangles, work.triangles_tested);
}

pub(super) fn local(p: [f64; 3], position: V) -> [f64; 3] {
    [
        p[0] - f64::from(position.0),
        p[1] - f64::from(position.1),
        p[2] - f64::from(position.2),
    ]
}

/// Adapt a spherical contact or deepest box patch for positional correction. Box response
/// emits every triangle manifold through the primitive adapter. Capsule
/// movement uses the shape-query surface; rotating rigid capsule/mesh response is unsupported.
pub(super) fn manifold(
    prepared: &PreparedTriangleMesh3,
    a: &Body,
    b: &Body,
    margin: Real,
    sweep: Option<Real>,
    stats: &mut super::GeometryStats,
) -> Result<Option<super::contact::Manifold>, SweepFailure> {
    let mesh_first = matches!(a.shape, Shape::TriangleMesh { .. });
    let (target, query) = if mesh_first { (a, b) } else { (b, a) };
    if matches!(query.shape, Shape::Box(_)) {
        if sweep.is_some() {
            return Err(SweepFailure::InvalidGeometryInput);
        }
        let mut deepest = None;
        let mut depth = Real::INFINITY;
        primitive::box_mesh_manifolds(prepared, a, b, margin, stats, |m| {
            let separation = (&m.points)
                .into_iter()
                .map(|p| p.separation)
                .fold(Real::INFINITY, Real::min);
            if separation < depth {
                depth = separation;
                deepest = Some(m);
            }
        })?;
        return Ok(deepest);
    }
    let Shape::Sphere(radius) = query.shape else {
        return Err(SweepFailure::InvalidGeometryInput);
    };
    let p = local(
        [query.position.0, query.position.1, query.position.2].map(f64::from),
        target.position,
    );
    let mut work = TriangleMeshWork3::default();
    let result = if let Some(dt) = sweep {
        stats.sweep_queries += 1;
        let displacement = query.velocity * dt;
        prepared
            .capsule_sweep_counted(
                p,
                p,
                f64::from(radius),
                [displacement.0, displacement.1, displacement.2].map(f64::from),
                128,
                &mut work,
            )
            .map(|hit| hit.map(|hit| (primitive::toward_zero(hit.fraction), hit.contact)))
    } else {
        stats.current_queries += 1;
        prepared
            .capsule_contact_counted(p, p, f64::from(radius), f64::from(margin), &mut work)
            .map(|hit| hit.map(|contact| (0.0, contact)))
    };
    stats.mesh_nodes_visited += work.nodes_visited;
    stats.mesh_triangles_tested += work.triangles_tested;
    crate::work::add(crate::work::Counter::MeshNodes, work.nodes_visited);
    crate::work::add(crate::work::Counter::MeshTriangles, work.triangles_tested);
    let result = result.map_err(failure)?;
    Ok(result.map(|(time, contact)| {
        let normal = V(
            primitive::nearest(contact.normal[0]),
            primitive::nearest(contact.normal[1]),
            primitive::nearest(contact.normal[2]),
        );
        let surface = V(
            primitive::nearest(contact.point[0] + f64::from(target.position.0)),
            primitive::nearest(contact.point[1] + f64::from(target.position.1)),
            primitive::nearest(contact.point[2] + f64::from(target.position.2)),
        );
        let separation = primitive::round_down(contact.separation);
        super::static_sphere::sphere_manifold(
            a, b, mesh_first, time, normal, surface, separation, sweep,
        )
    }))
}

impl World {
    pub fn triangle_mesh_stats(&self, body: BodyId) -> Result<TriangleMeshStats, Error> {
        let body = &self.bodies[self.index(body)?];
        let Shape::TriangleMesh { mesh, .. } = body.shape else {
            return Err(Error::InvalidInput);
        };
        let stats = self.triangle_mesh(mesh).preparation();
        Ok(TriangleMeshStats {
            triangles: stats.triangles_prepared,
            nodes: stats.nodes_prepared,
            retained_bytes: stats.retained_bytes,
        })
    }
    pub(super) fn mesh_shape_hit(
        &self,
        target: &Body,
        query: &Body,
        cast: Option<super::ShapeCast>,
        stats: &mut super::QueryStats,
    ) -> Result<Option<super::QueryHit>, SweepFailure> {
        let Shape::TriangleMesh { mesh, .. } = target.shape else {
            unreachable!("mesh target")
        };
        let prepared = self.triangle_mesh(mesh);
        let mut work = TriangleMeshWork3::default();
        let result = (|| {
            let capsule = super::terrain::capsule_of(query)
                .map(|(p, q, r)| (local(p, target.position), local(q, target.position), r));
            let initial = match capsule {
                Some((p, q, r)) => prepared.capsule_contact_counted(p, q, r, 0.0, &mut work)?,
                None if cast.is_none() => {
                    let polytope =
                        primitive::polytope_of(query).ok_or(TriangleMeshError3::InvalidQuery)?;
                    let vertices = polytope
                        .vertices
                        .iter()
                        .map(|&p| local(p, target.position))
                        .collect::<Vec<_>>();
                    prepared.polytope_contact_counted(
                        geometry_kernels::heightfield::ConvexPolytope3 {
                            vertices: &vertices,
                            face_normals: &polytope.face_normals,
                            edges: &polytope.edges,
                        },
                        &mut work,
                    )?
                }
                None => return Err(TriangleMeshError3::InvalidQuery),
            };
            let outcome = match (cast, capsule) {
                (Some(cast), Some((p, q, r))) => {
                    let d = [
                        cast.displacement.0,
                        cast.displacement.1,
                        cast.displacement.2,
                    ]
                    .map(f64::from);
                    prepared
                        .capsule_sweep_counted(p, q, r, d, cast.max_iterations, &mut work)?
                        .map(|hit| (primitive::toward_zero(hit.fraction), hit.contact))
                        .or(initial.map(|c| (0.0, c)))
                }
                _ => initial.map(|c| (0.0, c)),
            };
            Ok(outcome.map(|(fraction, contact)| {
                let normal = V(
                    primitive::nearest(contact.normal[0]),
                    primitive::nearest(contact.normal[1]),
                    primitive::nearest(contact.normal[2]),
                );
                let point = V(
                    primitive::nearest(contact.point[0] + f64::from(target.position.0)),
                    primitive::nearest(contact.point[1] + f64::from(target.position.1)),
                    primitive::nearest(contact.point[2] + f64::from(target.position.2)),
                );
                let separation = primitive::round_down(contact.separation);
                super::QueryHit {
                    body: target.id,
                    child: None,
                    feature: Some(super::RayFeature::TriangleMeshTriangle(contact.triangle)),
                    fraction,
                    distance: cast.map_or(0.0, |c| c.displacement.length() * fraction),
                    normal,
                    query_point: point + normal * separation,
                    target_point: point,
                    separation,
                    starts_overlapping: fraction == 0.0
                        && initial.is_some_and(|c| c.separation < 0.0),
                    support_velocity: V::ZERO,
                    sensor: target.sensor,
                }
            }))
        })();
        counted(work, stats);
        result.map_err(failure)
    }
}

impl World {
    /// Admit one fixed chunk. Triangle IDs are its input indices, including after world
    /// cloning and body-storage reordering. Clones share immutable prepared geometry.
    pub fn add_triangle_mesh(
        &mut self,
        desc: TriangleMeshDesc,
        key: Option<u64>,
    ) -> Result<BodyId, Error> {
        if !desc.position.finite() || desc.position.abs().max_component() >= 1e12 {
            return Err(Error::InvalidInput);
        }
        let prepared = PreparedTriangleMesh::new(&desc.vertices, desc.triangles)?;
        self.add_prepared_triangle_mesh(&prepared, desc.position, desc.material, key)
    }

    /// [`Self::add_triangle_mesh`] for geometry already prepared by
    /// [`PreparedTriangleMesh::new`]: results are identical and no preparation runs.
    pub fn add_prepared_triangle_mesh(
        &mut self,
        prepared: &PreparedTriangleMesh,
        position: V,
        material: Material,
        key: Option<u64>,
    ) -> Result<BodyId, Error> {
        if !position.finite() || position.abs().max_component() >= 1e12 {
            return Err(Error::InvalidInput);
        }
        let mesh = TriangleMeshId(
            u32::try_from(self.triangle_meshes.len()).map_err(|_| Error::InvalidInput)?,
        );
        self.triangle_meshes.push(Some(TriangleMesh {
            prepared: prepared.prepared.clone(),
        }));
        let mut body = Body::new(
            Shape::TriangleMesh {
                mesh,
                half_extents: prepared.half_extents,
            },
            position + prepared.centre,
            0.0,
        );
        body.key = key;
        body.set_material(material);
        self.insert_body(body).inspect_err(|_| {
            self.triangle_meshes.pop();
        })
    }

    pub(super) fn triangle_mesh(&self, id: TriangleMeshId) -> &PreparedTriangleMesh3 {
        &self.triangle_meshes[id.0 as usize]
            .as_ref()
            .expect("a live mesh body has prepared geometry")
            .prepared
    }
}
