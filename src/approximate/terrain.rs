//! Static heightfield terrain (C17): a regular grid of heights, two triangles per cell split
//! along the same diagonal as parry/Rapier heightfields. The prepared geometry lives in the
//! world; its body carries `Shape::Heightfield` with the terrain's bounds.

use super::{Body, BodyId, Error, Real, Shape, Vector as V, World, primitive};
use geometry_kernels::heightfield::{
    HeightfieldContact3, HeightfieldData3, HeightfieldDiagonal3, PreparedHeightfield3,
};

/// Index of a terrain in its world.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct TerrainId(pub(super) u32);

/// A heightfield: `columns` × `rows` heights, row-major (Z rows, X columns), with the first
/// vertex at `origin` and `cell_size` spacing along X and Z. Heights are added to `origin.1`.
#[derive(Clone, Debug, PartialEq)]
pub struct HeightfieldDesc {
    pub origin: V,
    pub cell_size: (Real, Real),
    pub columns: usize,
    pub rows: usize,
    pub heights: Vec<Real>,
}

/// Immutable prepared terrain, shared by world clones.
#[derive(Clone, Debug)]
pub(super) struct Terrain {
    pub prepared: std::sync::Arc<PreparedHeightfield3>,
}

impl World {
    /// Adds static heightfield terrain and returns its body. `key` is the body's game key.
    pub fn add_heightfield(
        &mut self,
        desc: HeightfieldDesc,
        key: Option<u64>,
    ) -> Result<BodyId, Error> {
        let (dx, dz) = desc.cell_size;
        if desc.columns < 2
            || desc.rows < 2
            || desc.heights.len() != desc.columns * desc.rows
            || !(dx.is_finite() && dx > 0.0 && dz.is_finite() && dz > 0.0)
            || !desc.origin.finite()
            || desc.heights.iter().any(|h| !h.is_finite())
        {
            return Err(Error::InvalidInput);
        }
        let o = desc.origin;
        let columns = (0..desc.columns)
            .map(|i| f64::from(o.0) + f64::from(dx) * i as f64)
            .collect();
        let rows = (0..desc.rows)
            .map(|i| f64::from(o.2) + f64::from(dz) * i as f64)
            .collect();
        let heights = desc
            .heights
            .iter()
            .map(|&h| f64::from(o.1) + f64::from(h))
            .collect();
        let prepared = PreparedHeightfield3::try_new(HeightfieldData3 {
            columns,
            rows,
            heights,
            active_cells: None,
            diagonal: HeightfieldDiagonal3::Fixed,
        })
        .map_err(|_| Error::InvalidInput)?;
        let (low, high) = desc
            .heights
            .iter()
            .fold((Real::INFINITY, Real::NEG_INFINITY), |(l, h), &v| {
                (l.min(v), h.max(v))
            });
        let min = V(o.0, o.1 + low, o.2);
        let max = V(
            o.0 + dx * (desc.columns - 1) as Real,
            o.1 + high,
            o.2 + dz * (desc.rows - 1) as Real,
        );
        let centre = (min + max) * 0.5;
        let half = (max - min) * 0.5;
        let terrain =
            TerrainId(u32::try_from(self.terrains.len()).map_err(|_| Error::InvalidInput)?);
        self.terrains.push(Terrain {
            prepared: std::sync::Arc::new(prepared),
        });
        let mut body = Body::new(
            Shape::Heightfield {
                terrain,
                half_extents: half,
            },
            centre,
            0.0,
        );
        body.key = key;
        self.insert_body(body).inspect_err(|_| {
            self.terrains.pop();
        })
    }

    pub(super) fn terrain(&self, id: TerrainId) -> &PreparedHeightfield3 {
        &self.terrains[id.0 as usize].prepared
    }
}

/// The query shape as a kernel capsule (segment and radius); spheres collapse the segment.
pub(super) fn capsule_of(body: &Body) -> Option<([f64; 3], [f64; 3], f64)> {
    let c = [body.position.0, body.position.1, body.position.2].map(f64::from);
    match body.shape {
        Shape::Sphere(r) => Some((c, c, f64::from(r))),
        Shape::Capsule {
            half_segment,
            radius,
        } => {
            let axis = body.orientation.rotate(V::Y) * half_segment;
            let a = [axis.0, axis.1, axis.2].map(f64::from);
            Some((
                std::array::from_fn(|k| c[k] - a[k]),
                std::array::from_fn(|k| c[k] + a[k]),
                f64::from(radius),
            ))
        }
        _ => None,
    }
}

/// Contact narrowed to the solver scalar: separation toward smaller, normal and point nearest.
pub(super) fn narrow(contact: HeightfieldContact3) -> (Real, V, V) {
    (
        primitive::round_down(contact.separation),
        V(
            primitive::nearest(contact.normal[0]),
            primitive::nearest(contact.normal[1]),
            primitive::nearest(contact.normal[2]),
        ),
        V(
            primitive::nearest(contact.point[0]),
            primitive::nearest(contact.point[1]),
            primitive::nearest(contact.point[2]),
        ),
    )
}

/// A halfspace body's boundary point and outward unit normal (its local +Y).
pub(super) fn plane_of(body: &Body) -> ([f64; 3], [f64; 3]) {
    let n = body.orientation.rotate(V::Y);
    (
        [body.position.0, body.position.1, body.position.2].map(f64::from),
        [n.0, n.1, n.2].map(f64::from),
    )
}
