//! Rigid sphere contacts with fixed authored surfaces: triangle meshes (#182), heightfield
//! terrain and convex hulls (#519). Each pair gives one contact point, either the current one
//! within a margin or the first along the sphere's translation over a substep, so a CCD sphere
//! keeps its swept time of impact against these surfaces.

use super::{
    Body, Real, Shape, SweepFailure, Vector as V,
    contact::{Manifold, Point, Points},
    convex_hull::{HullWork, PreparedHull},
    primitive, terrain,
    triangle_mesh::TriangleMesh,
};
use geometry_kernels::{
    heightfield::{HeightfieldWork3, PreparedHeightfield3},
    triangle_mesh::PreparedTriangleMesh3,
};
use std::sync::Arc;

/// A fixed authored surface's prepared geometry.
#[derive(Clone, Copy)]
pub(super) enum Surface<'a> {
    Mesh(&'a PreparedTriangleMesh3),
    Terrain(&'a PreparedHeightfield3),
    Hull(&'a PreparedHull),
}

impl<'a> Surface<'a> {
    /// The prepared geometry of a mesh, terrain or hull body's shape.
    pub(super) fn of(
        shape: Shape,
        meshes: &'a [Option<TriangleMesh>],
        terrains: &'a [terrain::Terrain],
        hulls: &'a [Option<Arc<PreparedHull>>],
    ) -> Option<Self> {
        Some(match shape {
            Shape::TriangleMesh { mesh, .. } => Self::Mesh(
                &meshes[mesh.0 as usize]
                    .as_ref()
                    .expect("a live mesh body's geometry")
                    .prepared,
            ),
            Shape::Heightfield { terrain, .. } => Self::Terrain(
                terrains[terrain.0 as usize]
                    .prepared
                    .as_deref()
                    .expect("a live heightfield body's terrain"),
            ),
            Shape::ConvexHull { hull, .. } => Self::Hull(
                hulls[hull.0 as usize]
                    .as_deref()
                    .expect("a live convex hull's geometry"),
            ),
            _ => return None,
        })
    }

    /// The surface of a sphere/surface pair, and whether `a` is the surface.
    pub(super) fn sphere_pair(
        a: Shape,
        b: Shape,
        meshes: &'a [Option<TriangleMesh>],
        terrains: &'a [terrain::Terrain],
        hulls: &'a [Option<Arc<PreparedHull>>],
    ) -> Option<(Self, bool)> {
        match (a, b) {
            (surface, Shape::Sphere(_)) => {
                Self::of(surface, meshes, terrains, hulls).map(|s| (s, true))
            }
            (Shape::Sphere(_), surface) => {
                Self::of(surface, meshes, terrains, hulls).map(|s| (s, false))
            }
            _ => None,
        }
    }
}

/// The contact of the pair `a`, `b` (one of them the surface): the current one when its
/// separation is at most `margin`, or with `sweep` the first along the translation over that
/// many seconds. Meshes also give boxes their deepest current patch; any other shape fails.
pub(super) fn manifold(
    surface: Surface<'_>,
    a: &Body,
    b: &Body,
    margin: Real,
    sweep: Option<Real>,
    stats: &mut super::GeometryStats,
) -> Result<Option<Manifold>, SweepFailure> {
    match surface {
        Surface::Mesh(mesh) => super::triangle_mesh::manifold(mesh, a, b, margin, sweep, stats),
        Surface::Terrain(field) => terrain_manifold(field, a, b, margin, sweep, stats),
        Surface::Hull(hull) => hull_manifold(hull, a, b, margin, sweep, stats),
    }
}

/// Fixed-collider position correction: as [`manifold`] without a margin, but only for the
/// shapes the surface gives rigid contacts (spheres; boxes on meshes). Box/terrain and
/// box/hull rows are not corrected here.
pub(super) fn correction(
    surface: Surface<'_>,
    a: &Body,
    b: &Body,
    stats: &mut super::GeometryStats,
) -> Result<Option<Manifold>, SweepFailure> {
    let sphere = matches!(a.shape, Shape::Sphere(_)) || matches!(b.shape, Shape::Sphere(_));
    if !sphere && !matches!(surface, Surface::Mesh(_)) {
        return Ok(None);
    }
    manifold(surface, a, b, 0.0, None, stats)
}

/// A one-point manifold from a surface contact found at `time` of `sweep`: the unit `normal`
/// from the surface toward the sphere, the surface point and the separation there.
#[allow(clippy::too_many_arguments)]
pub(super) fn sphere_manifold(
    a: &Body,
    b: &Body,
    surface_first: bool,
    time: Real,
    normal: V,
    surface: V,
    separation: Real,
    sweep: Option<Real>,
) -> Manifold {
    let body_point = surface + normal * separation;
    let (normal, point_a, point_b) = if surface_first {
        (normal, surface, body_point)
    } else {
        (-normal, body_point, surface)
    };
    let offset = sweep.unwrap_or(0.0) * time;
    Manifold {
        normal,
        points: Points::one(Point {
            ra: point_a - (a.position + a.velocity * offset),
            rb: point_b - (b.position + b.velocity * offset),
            separation: separation - (b.velocity - a.velocity).dot(normal) * offset,
        }),
        swept: sweep.is_some(),
        time,
    }
}

/// Sphere and heightfield: the deepest triangle under the sphere within `margin`, or the
/// earliest along the sweep. Only the cells under the sphere's (swept) footprint are visited;
/// holes and the parry diagonal are the terrain's own.
fn terrain_manifold(
    field: &PreparedHeightfield3,
    a: &Body,
    b: &Body,
    margin: Real,
    sweep: Option<Real>,
    stats: &mut super::GeometryStats,
) -> Result<Option<Manifold>, SweepFailure> {
    let first = matches!(a.shape, Shape::Heightfield { .. });
    let query = if first { b } else { a };
    let Shape::Sphere(radius) = query.shape else {
        return Err(SweepFailure::InvalidGeometryInput);
    };
    let p = [query.position.0, query.position.1, query.position.2].map(f64::from);
    let r = f64::from(radius);
    let mut work = HeightfieldWork3::default();
    let result = if let Some(dt) = sweep {
        stats.sweep_queries += 1;
        let d = query.velocity * dt;
        field
            .capsule_sweep_counted(p, p, r, [d.0, d.1, d.2].map(f64::from), 128, &mut work)
            .map(|hit| hit.map(|hit| (primitive::toward_zero(hit.fraction), hit.contact)))
            .map_err(|_| SweepFailure::NonFiniteComputation)
    } else {
        stats.current_queries += 1;
        Ok(field
            .capsule_contact_counted(p, p, r, f64::from(margin), &mut work)
            .map(|contact| (0.0, contact)))
    };
    crate::work::add(crate::work::Counter::TerrainCells, work.cells_visited);
    crate::work::add(
        crate::work::Counter::TerrainTriangles,
        work.triangles_tested,
    );
    let Some((time, contact)) = result? else {
        return Ok(None);
    };
    let (separation, normal, point) = terrain::narrow(contact);
    if !separation.is_finite() || !normal.finite() || !point.finite() {
        return Err(SweepFailure::NonFiniteComputation);
    }
    Ok(Some(sphere_manifold(
        a, b, first, time, normal, point, separation, sweep,
    )))
}

/// Sphere and fixed convex hull: face planes first, then the checked distance/GJK/EPA
/// fallback; a sweep uses the hull's conservative advancement.
fn hull_manifold(
    hull: &PreparedHull,
    a: &Body,
    b: &Body,
    margin: Real,
    sweep: Option<Real>,
    stats: &mut super::GeometryStats,
) -> Result<Option<Manifold>, SweepFailure> {
    let first = matches!(a.shape, Shape::ConvexHull { .. });
    let (target, query) = if first { (a, b) } else { (b, a) };
    let mut work = HullWork::default();
    let result = if let Some(dt) = sweep {
        stats.sweep_queries += 1;
        hull.sphere_sweep(query, target, query.velocity * dt, &mut work)?
    } else {
        stats.current_queries += 1;
        let contact = hull.sphere_contact(query, target, &mut work)?;
        (contact.separation <= margin).then_some((0.0, contact))
    };
    crate::work::add(crate::work::Counter::HullPlanes, work.face_planes_tested);
    let Some((time, contact)) = result else {
        return Ok(None);
    };
    if !contact.separation.is_finite() || !contact.normal.finite() {
        return Err(SweepFailure::NonFiniteComputation);
    }
    // The hull contact normal points from the sphere toward the hull.
    Ok(Some(sphere_manifold(
        a,
        b,
        first,
        time,
        -contact.normal,
        contact.point_hull,
        contact.separation,
        sweep,
    )))
}
