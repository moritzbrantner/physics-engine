//! Explicit, validated physical products. Nothing here runs during ordinary stepping.
use std::collections::BTreeMap;

use sha2::{Digest, Sha256};

use super::{
    Body, BodyId, BodyStorage, CachedPoint, CharacterSettings, CombineRule, Config, Convergence,
    ConvergenceScope, ConvexHullId, Embedded, HeightfieldDesc, Order, PositionCorrection,
    Quaternion, REAL_BITS, Real, Scalar, Shape, TerrainId, TriangleMeshId, Vector, World,
    character, contact, convex_hull, narrow_time, substep_length, terrain, widen_time,
};
use crate::CollisionLayers3d;

// Each scalar width has its own magic; the layout is otherwise shared. Physical
// values use `Real` at native width; elapsed/prior-substep time is always f64.
const MAGIC_F64: &[u8; 8] = b"PEFLOAT\0";
const MAGIC_F32: &[u8; 8] = b"PEFLT32\0";
const MAGIC: &[u8; 8] = if REAL_BITS == 32 {
    MAGIC_F32
} else {
    MAGIC_F64
};
const FORMAT: u32 = 9;
// Bump when continuation semantics change, even if the byte layout does not.
const ALGORITHM: u32 = 15;
const DIGEST_BYTES: usize = 32;
const REAL_BYTES: usize = size_of::<Real>();
// Fixed fields: id 8, key tag 1, key or serial 8, shape tag 1, flags 2, layers 8, support tag
// 1; 31 physical scalars (a halfspace has no dimensions).
const MIN_BODY_BYTES: usize = 29 + 31 * REAL_BYTES;
// Id 8, offset and slope, four option/count/policy tags, exclusion tag and clear flag.
const MIN_CHARACTER_BYTES: usize = 15 + 2 * REAL_BYTES;
const POINT_BYTES: usize = 13 * REAL_BYTES;
// Conservative envelopes for stored normalization/rotated-normal results. The
// existing normalization can lose unit length for extreme finite input scales;
// a checkpoint preserves that accepted state rather than silently repairing it.
const ORIENTATION_COMPONENT_LIMIT: Real = 2.0;
const NORMAL_COMPONENT_LIMIT: Real = 64.0;

/// Consumer-owned compatibility identities. The build tag must identify the exact
/// engine/dependency/compiler/flags/ABI policy; content identifies static content
/// and the consumer's BodyId mapping. Equal tags do not establish portability.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CheckpointContext {
    pub build: [u8; 32],
    pub content: [u8; 32],
}

/// Explicit resource budgets for decoding untrusted or damaged input. These are
/// checkpoint admission limits, not limits on the number of simulated bodies.
#[derive(Clone, Copy, Debug)]
pub struct CheckpointLimits {
    pub bytes: usize,
    pub bodies: usize,
    pub pairs: usize,
    pub contact_points: usize,
}
impl Default for CheckpointLimits {
    fn default() -> Self {
        Self {
            bytes: 64 * 1024 * 1024,
            bodies: 100_000,
            pairs: 200_000,
            contact_points: 800_000,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CheckpointError {
    InvalidData,
    IntegrityMismatch,
    UnsupportedFormat(u32),
    UnsupportedAlgorithm(u32),
    IncompatibleContext,
    IncompatibleTarget,
    UnsupportedPolicy,
    ResourceLimit,
    /// The world has joints, which checkpoints do not hold (#453); nothing is captured.
    UnsupportedJoints,
    /// Restore was given heightfields that differ from the checkpoint's live terrain (#452):
    /// `index` is the first entry of [`Checkpoint::heightfield_bodies`] whose description is
    /// missing, extra or not the one the world was built with. No world is created.
    TerrainMismatch {
        index: usize,
    },
    /// The bytes come from a build whose floating-solver scalar width differs
    /// from this build's `Real`. Rejected before the version fields.
    ScalarWidthMismatch {
        expected_bits: u32,
        found_bits: u32,
    },
}
impl std::fmt::Display for CheckpointError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "physical checkpoint rejected: {self:?}")
    }
}
impl std::error::Error for CheckpointError {}

/// Physical vector payload only, excluding tree nodes, allocator overhead,
/// inline configuration and disposable acceleration. Encoding allocates separately.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CheckpointStats {
    pub bodies: usize,
    pub pairs: usize,
    pub contact_points: usize,
    pub vector_capacity_bytes: usize,
}

/// Immutable, continuation-complete physical product at an explicit call boundary.
/// Reports/events, scratch and consumer gameplay state are deliberately excluded.
#[derive(Clone, Debug)]
pub struct Checkpoint {
    context: CheckpointContext,
    config: Config,
    elapsed: Scalar,
    last_h: Scalar,
    bodies: Vec<Body>,
    cache: BTreeMap<(Order, Order), Vec<CachedPoint>>,
    /// Generation of every id slot, so restored ids stay valid and stale ids stay rejected.
    slot_generations: Vec<u32>,
    /// Creation serial of the next keyless body.
    next_serial: u64,
    /// Every terrain the world ever added; restore keeps their [`TerrainId`]s.
    terrain_slots: u32,
    meshes: Vec<Option<geometry_kernels::triangle_mesh::TriangleMeshData3>>,
    /// Live heightfields in terrain order: terrain index and geometry fingerprint. Heights are
    /// consumer content: restore takes their descriptions and checks them against these.
    heightfields: Vec<(u32, [u8; 32])>,
    /// Canonical hull vertices in original allocation-slot order. Unlike a heightfield, a
    /// small authored rock proxy is inline in the opt-in checkpoint; zero vertices = removed.
    hulls: Vec<Option<Vec<Vector>>>,
    /// Halfspace bodies in the order the step tests them (their creation order).
    halfspaces: Vec<BodyId>,
    /// Character registrations in slot order.
    characters: Vec<SavedCharacter>,
}

/// A character registration and the state its next move reads (#452).
#[derive(Clone, Copy, Debug)]
struct SavedCharacter {
    id: BodyId,
    settings: CharacterSettings,
    /// The body its latest move excluded; pushes it receives before its next move exclude it.
    exclude: Option<BodyId>,
    /// Known free of overlap: its next move skips separation and depenetration.
    clear: bool,
}

impl World {
    /// Capture exact floating state/history, including queued input and character
    /// registrations. This explicit product copies all bodies and contact history; ordinary
    /// step never calls it. Heightfields are recorded by a fingerprint of their description,
    /// which [`Checkpoint::restore`] takes again; halfspaces are inline bodies. Worlds with
    /// joints are not checkpointed (`UnsupportedJoints`).
    pub fn checkpoint(&self, context: CheckpointContext) -> Result<Checkpoint, CheckpointError> {
        if !self.joints.is_empty() {
            return Err(CheckpointError::UnsupportedJoints);
        }
        // Live bodies in body order, whatever the storage holds between steps.
        let bodies = self.bodies().cloned().collect::<Vec<_>>();
        validate(self.config, self.elapsed, self.last_h, &bodies, &self.cache)?;
        let mut heightfields = bodies
            .iter()
            .filter_map(|b| match b.shape {
                Shape::Heightfield { terrain, .. } => {
                    Some((terrain.0, self.terrains[terrain.0 as usize].fingerprint))
                }
                _ => None,
            })
            .collect::<Vec<_>>();
        heightfields.sort_unstable_by_key(|h| h.0);
        let hulls = self
            .hulls
            .iter()
            .map(|slot| slot.as_ref().map(|prepared| prepared.checkpoint_vertices()))
            .collect::<Vec<_>>();
        let characters = self
            .characters
            .entries(self.bookkeeping.generation)
            .map(|(id, settings, exclude, clear)| SavedCharacter {
                id,
                settings,
                exclude,
                clear,
            })
            .collect::<Vec<_>>();
        let terrain_slots = u32::try_from(self.terrains.len()).expect("terrain ids are u32");
        validate_parts(
            &bodies,
            terrain_slots,
            &heightfields,
            &hulls,
            &self.halfspaces,
            &characters,
        )?;
        Ok(Checkpoint {
            context,
            config: self.config(),
            elapsed: self.elapsed,
            last_h: self.last_h,
            bodies,
            cache: self.cache.clone(),
            slot_generations: self.slots.iter().map(|s| s.generation).collect(),
            next_serial: self.next_serial,
            terrain_slots,
            heightfields,
            meshes: self
                .triangle_meshes
                .iter()
                .map(|m| m.as_ref().map(|m| m.prepared.data().clone()))
                .collect(),
            hulls,
            halfspaces: self.halfspaces.clone(),
            characters,
        })
    }
}

impl Checkpoint {
    pub fn stats(&self) -> CheckpointStats {
        CheckpointStats {
            bodies: self.bodies.len(),
            pairs: self.cache.len(),
            contact_points: self.cache.values().map(Vec::len).sum(),
            vector_capacity_bytes: self.bodies.capacity() * size_of::<Body>()
                + self
                    .cache
                    .values()
                    .map(|p| p.capacity() * size_of::<CachedPoint>())
                    .sum::<usize>()
                + self.meshes.capacity()
                    * size_of::<Option<geometry_kernels::triangle_mesh::TriangleMeshData3>>()
                + self
                    .meshes
                    .iter()
                    .flatten()
                    .map(|data| {
                        data.vertices.capacity() * size_of::<[f64; 3]>()
                            + data.triangles.capacity() * size_of::<[u32; 3]>()
                    })
                    .sum::<usize>()
                + self.heightfields.capacity() * size_of::<(u32, [u8; 32])>()
                + self
                    .hulls
                    .iter()
                    .map(|h| h.as_ref().map_or(0, |v| v.capacity() * size_of::<Vector>()))
                    .sum::<usize>()
                + self.halfspaces.capacity() * size_of::<BodyId>()
                + self.characters.capacity() * size_of::<SavedCharacter>(),
        }
    }

    /// The heightfield bodies whose descriptions [`Checkpoint::restore`] takes, in that order
    /// (the order the world added them).
    pub fn heightfield_bodies(&self) -> impl Iterator<Item = BodyId> + '_ {
        self.heightfields.iter().map(|&(index, _)| {
            self.bodies
                .iter()
                .find(
                    |b| matches!(b.shape, Shape::Heightfield { terrain, .. } if terrain.0 == index),
                )
                .expect("validated heightfield body")
                .id
        })
    }

    /// Consume the validated product into one new mutable authority. Orientation
    /// is not normalized again. Derived bounds/views rebuild; the report starts empty
    /// so restoring cannot re-emit already completed events. Cold-cache work can differ.
    ///
    /// `heightfields` are the descriptions of [`Checkpoint::heightfield_bodies`], in that
    /// order: terrain content stays the consumer's, and each one must be the description the
    /// world was built with (`TerrainMismatch` otherwise). Characters come back registered.
    pub fn restore(self, heightfields: &[HeightfieldDesc]) -> Result<World, CheckpointError> {
        if heightfields.len() != self.heightfields.len() {
            return Err(CheckpointError::TerrainMismatch {
                index: heightfields.len().min(self.heightfields.len()),
            });
        }
        let mut terrains = (0..self.terrain_slots)
            .map(|_| terrain::Terrain {
                prepared: None,
                fingerprint: [0; 32],
            })
            .collect::<Vec<_>>();
        for (index, (&(slot, fingerprint), desc)) in
            self.heightfields.iter().zip(heightfields).enumerate()
        {
            let mismatch = CheckpointError::TerrainMismatch { index };
            let (prepared, centre, half) = terrain::prepare(desc).map_err(|_| mismatch)?;
            let body = self
                .bodies
                .iter()
                .find(
                    |b| matches!(b.shape, Shape::Heightfield { terrain, .. } if terrain.0 == slot),
                )
                .expect("validated heightfield body");
            if prepared.fingerprint != fingerprint
                || body.position != centre
                || body.shape.half_extents() != half
            {
                return Err(mismatch);
            }
            terrains[slot as usize] = prepared;
        }
        let mut hulls = Vec::with_capacity(self.hulls.len());
        for (slot, vertices) in self.hulls.iter().enumerate() {
            let prepared = if let Some(vertices) = vertices {
                let (prepared, half) =
                    convex_hull::prepare(vertices).map_err(|_| CheckpointError::InvalidData)?;
                let body = self.bodies.iter().find(|b| {
                    matches!(b.shape, Shape::ConvexHull { hull, .. } if hull.0 as usize == slot)
                }).ok_or(CheckpointError::InvalidData)?;
                if body.shape.half_extents() != half {
                    return Err(CheckpointError::InvalidData);
                }
                Some(std::sync::Arc::new(prepared))
            } else {
                None
            };
            hulls.push(prepared);
        }
        let mut world = World::new(self.config).expect("checkpoint configuration was validated");
        world.terrains = terrains;
        world.triangle_meshes = self
            .meshes
            .into_iter()
            .map(|data| {
                data.map(|data| {
                    geometry_kernels::triangle_mesh::PreparedTriangleMesh3::try_new(data)
                        .map(|prepared| super::triangle_mesh::TriangleMesh {
                            prepared: std::sync::Arc::new(prepared),
                        })
                        .map_err(|_| CheckpointError::InvalidData)
                })
                .transpose()
            })
            .collect::<Result<Vec<_>, _>>()?;
        world.hulls = hulls;
        world.halfspaces = self.halfspaces;
        world.elapsed = self.elapsed;
        world.last_h = self.last_h;
        world.bodies = self.bodies;
        world.cache = self.cache;
        for body in &mut world.bodies {
            body.cached_bounds = contact::bounds(body);
        }
        world.bookkeeping.activity.count = world
            .bodies
            .iter()
            .filter(|b| b.mass > 0.0 && !b.sleeping)
            .count();
        world.bookkeeping.layout_changed();
        world.slots = self
            .slot_generations
            .iter()
            .map(|&generation| super::Slot {
                generation,
                index: super::FREE_SLOT,
            })
            .collect();
        world.next_serial = self.next_serial;
        world.reindex_slots();
        world
            .query_tree
            .sync(&world.bodies, world.bookkeeping.generation);
        let generation = world.bookkeeping.generation;
        for c in self.characters {
            world
                .characters
                .restore(c.id, c.settings, c.exclude, c.clear.then_some(generation));
        }
        Ok(world)
    }

    /// Canonical little-endian encoding of exact `Real` bits (f64 time). SHA-256 detects damaged
    /// bytes; authentication/persistence and consumer snapshot atomicity stay outside physics.
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(MAGIC);
        out.extend_from_slice(&FORMAT.to_le_bytes());
        out.extend_from_slice(&ALGORITHM.to_le_bytes());
        out.extend_from_slice(&self.context.build);
        out.extend_from_slice(&self.context.content);
        for target in [std::env::consts::ARCH, std::env::consts::OS] {
            // Rust's fixed target names fit in the format's one-byte length.
            out.push(u8::try_from(target.len()).expect("Rust target name fits in u8"));
            out.extend_from_slice(target.as_bytes());
        }
        encode_config(&mut out, self.config);
        time(&mut out, self.elapsed);
        time(&mut out, self.last_h);
        count(&mut out, self.bodies.len());
        for body in &self.bodies {
            encode_body(&mut out, body);
        }
        count(&mut out, self.cache.len());
        for (&(a, b), points) in &self.cache {
            out.extend_from_slice(&a.0.to_le_bytes());
            out.extend_from_slice(&b.0.to_le_bytes());
            count(&mut out, points.len());
            for point in points {
                vector(&mut out, point.a);
                vector(&mut out, point.b);
                vector(&mut out, point.normal);
                scalar(&mut out, point.impulse);
                vector(&mut out, point.tangent);
            }
        }
        out.extend_from_slice(&self.next_serial.to_le_bytes());
        count(&mut out, self.slot_generations.len());
        for generation in &self.slot_generations {
            out.extend_from_slice(&generation.to_le_bytes());
        }
        out.extend_from_slice(&self.terrain_slots.to_le_bytes());
        count(&mut out, self.heightfields.len());
        for (index, fingerprint) in &self.heightfields {
            out.extend_from_slice(&index.to_le_bytes());
            out.extend_from_slice(fingerprint);
        }
        count(&mut out, self.meshes.len());
        for data in &self.meshes {
            out.push(u8::from(data.is_some()));
            if let Some(data) = data {
                count(&mut out, data.vertices.len());
                for vertex in &data.vertices {
                    for &axis in vertex {
                        time(&mut out, axis);
                    }
                }
                count(&mut out, data.triangles.len());
                for triangle in &data.triangles {
                    for &index in triangle {
                        out.extend_from_slice(&index.to_le_bytes());
                    }
                }
            }
        }
        count(&mut out, self.hulls.len());
        for hull in &self.hulls {
            let vertices = hull.as_deref().unwrap_or_default();
            count(&mut out, vertices.len());
            for &v in vertices {
                vector(&mut out, v);
            }
        }
        count(&mut out, self.halfspaces.len());
        for id in &self.halfspaces {
            out.extend_from_slice(&id.to_bits().to_le_bytes());
        }
        count(&mut out, self.characters.len());
        for c in &self.characters {
            encode_character(&mut out, c);
        }
        let digest = Sha256::digest(&out);
        out.extend_from_slice(&digest);
        out
    }

    /// Fully validate before creating a restorable product. Unknown layouts,
    /// target/build/content mismatches, damaged/truncated bytes and exhausted
    /// caller budgets fail without touching any live world. Allocation aborts are excluded.
    pub fn from_bytes(
        bytes: &[u8],
        expected: CheckpointContext,
        limits: CheckpointLimits,
    ) -> Result<Self, CheckpointError> {
        if bytes.len() > limits.bytes {
            return Err(CheckpointError::ResourceLimit);
        }
        let payload_len = bytes
            .len()
            .checked_sub(DIGEST_BYTES)
            .ok_or(CheckpointError::InvalidData)?;
        let (payload, digest) = bytes.split_at(payload_len);
        let mut reader = Reader { bytes: payload };
        match &reader.array::<8>()? {
            magic if magic == MAGIC => {}
            magic if magic == MAGIC_F64 || magic == MAGIC_F32 => {
                return Err(CheckpointError::ScalarWidthMismatch {
                    expected_bits: REAL_BITS,
                    found_bits: if magic == MAGIC_F32 { 32 } else { 64 },
                });
            }
            _ => return Err(CheckpointError::InvalidData),
        }
        let format = reader.u32()?;
        if format != FORMAT {
            return Err(CheckpointError::UnsupportedFormat(format));
        }
        let algorithm = reader.u32()?;
        if algorithm != ALGORITHM {
            return Err(CheckpointError::UnsupportedAlgorithm(algorithm));
        }
        if Sha256::digest(payload).as_slice() != digest {
            return Err(CheckpointError::IntegrityMismatch);
        }
        let context = CheckpointContext {
            build: reader.array()?,
            content: reader.array()?,
        };
        if context != expected {
            return Err(CheckpointError::IncompatibleContext);
        }
        for target in [std::env::consts::ARCH, std::env::consts::OS] {
            let length = usize::from(reader.u8()?);
            if reader.take(length)? != target.as_bytes() {
                return Err(CheckpointError::IncompatibleTarget);
            }
        }
        let config = reader.config()?;
        let elapsed = reader.time()?;
        let last_h = reader.time()?;
        let body_count = reader.count(limits.bodies, MIN_BODY_BYTES)?;
        let mut bodies = Vec::with_capacity(body_count);
        for _ in 0..body_count {
            let body = reader.body()?;
            if bodies.last().is_some_and(|b: &Body| b.order >= body.order) {
                return Err(CheckpointError::InvalidData);
            }
            bodies.push(body);
        }
        let pair_count = reader.count(limits.pairs, 40 + POINT_BYTES)?;
        let mut cache = BTreeMap::new();
        let mut previous = None;
        let mut total_points = 0usize;
        for _ in 0..pair_count {
            let key = (Order(reader.u128()?), Order(reader.u128()?));
            if key.0 >= key.1 || previous.is_some_and(|p| p >= key) {
                return Err(CheckpointError::InvalidData);
            }
            previous = Some(key);
            let points_count = reader.count(limits.contact_points - total_points, POINT_BYTES)?;
            if points_count == 0 {
                return Err(CheckpointError::InvalidData);
            }
            total_points += points_count;
            let mut points = Vec::with_capacity(points_count);
            for _ in 0..points_count {
                points.push(CachedPoint {
                    a: reader.vector()?,
                    b: reader.vector()?,
                    normal: reader.vector()?,
                    impulse: reader.scalar()?,
                    tangent: reader.vector()?,
                });
            }
            cache.insert(key, points);
        }
        let next_serial = reader.u64()?;
        let slot_count = reader.count(limits.bodies.max(bodies.len()) * 2 + 1, 4)?;
        let mut slot_generations = Vec::with_capacity(slot_count);
        for _ in 0..slot_count {
            slot_generations.push(reader.u32()?);
        }
        let terrain_slots = reader.u32()?;
        // Every terrain was added with a body, so there are at most as many as bodies created.
        if u64::from(terrain_slots) > next_serial {
            return Err(CheckpointError::InvalidData);
        }
        if terrain_slots as usize > limits.bodies {
            return Err(CheckpointError::ResourceLimit);
        }
        let heightfield_count = reader.count(limits.bodies, 36)?;
        let mut heightfields = Vec::with_capacity(heightfield_count);
        for _ in 0..heightfield_count {
            heightfields.push((reader.u32()?, reader.array()?));
        }
        let mesh_slots = reader.count(limits.bodies, 1)?;
        if mesh_slots as u64 > next_serial {
            return Err(CheckpointError::InvalidData);
        }
        let mut meshes = Vec::with_capacity(mesh_slots);
        for _ in 0..mesh_slots {
            meshes.push(match reader.u8()? {
                0 => None,
                1 => {
                    let count = reader.count(limits.bytes / 24, 24)?;
                    let mut vertices = Vec::with_capacity(count);
                    for _ in 0..count {
                        vertices.push([reader.time()?, reader.time()?, reader.time()?]);
                    }
                    let count = reader.count(limits.bytes / 12, 12)?;
                    let mut triangles = Vec::with_capacity(count);
                    for _ in 0..count {
                        triangles.push([reader.u32()?, reader.u32()?, reader.u32()?]);
                    }
                    let data = geometry_kernels::triangle_mesh::TriangleMeshData3 {
                        vertices,
                        triangles,
                    };
                    geometry_kernels::triangle_mesh::PreparedTriangleMesh3::try_new(data.clone())
                        .map_err(|_| CheckpointError::InvalidData)?;
                    Some(data)
                }
                _ => return Err(CheckpointError::InvalidData),
            });
        }
        let mut mesh_bodies = std::collections::BTreeSet::new();
        for body in &bodies {
            if let Shape::TriangleMesh { mesh, half_extents } = body.shape {
                if !mesh_bodies.insert(mesh.0) {
                    return Err(CheckpointError::InvalidData);
                }
                let data = meshes
                    .get(mesh.0 as usize)
                    .and_then(Option::as_ref)
                    .ok_or(CheckpointError::InvalidData)?;
                let bounds = data.vertices.iter().fold([0.0f64; 3], |mut bounds, v| {
                    for k in 0..3 {
                        bounds[k] = bounds[k].max(v[k].abs());
                    }
                    bounds
                });
                if half_extents
                    != Vector(
                        super::primitive::round_up(bounds[0]),
                        super::primitive::round_up(bounds[1]),
                        super::primitive::round_up(bounds[2]),
                    )
                {
                    return Err(CheckpointError::InvalidData);
                }
            }
        }
        if meshes
            .iter()
            .enumerate()
            .any(|(slot, data)| data.is_some() != mesh_bodies.contains(&(slot as u32)))
        {
            return Err(CheckpointError::InvalidData);
        }
        let hull_count = reader.count(limits.bodies, 4)?;
        let mut hulls = Vec::with_capacity(hull_count);
        for _ in 0..hull_count {
            let count = reader.count(65_536, 3 * REAL_BYTES)?;
            let vertices = (0..count)
                .map(|_| reader.vector())
                .collect::<Result<Vec<_>, _>>()?;
            hulls.push((count != 0).then_some(vertices));
        }
        let halfspace_count = reader.count(limits.bodies, 8)?;
        let mut halfspaces = Vec::with_capacity(halfspace_count);
        for _ in 0..halfspace_count {
            halfspaces.push(BodyId::from_bits(reader.u64()?));
        }
        let character_count = reader.count(limits.bodies, MIN_CHARACTER_BYTES)?;
        let mut characters = Vec::with_capacity(character_count);
        for _ in 0..character_count {
            characters.push(reader.character()?);
        }
        if !reader.bytes.is_empty() {
            return Err(CheckpointError::InvalidData);
        }
        validate(config, elapsed, last_h, &bodies, &cache)?;
        validate_parts(
            &bodies,
            terrain_slots,
            &heightfields,
            &hulls,
            &halfspaces,
            &characters,
        )?;
        let mut keys = std::collections::BTreeSet::new();
        if bodies.iter().any(|b| {
            slot_generations.get(b.id.slot() as usize) != Some(&b.id.generation())
                || b.key.is_some_and(|k| !keys.insert(k))
                || (b.key.is_none() && (b.order.0 as u64) >= next_serial)
        }) {
            return Err(CheckpointError::InvalidData);
        }
        Ok(Self {
            context,
            config,
            elapsed,
            last_h,
            bodies,
            cache,
            slot_generations,
            next_serial,
            terrain_slots,
            heightfields,
            meshes,
            hulls,
            halfspaces,
            characters,
        })
    }
}

/// Identity of a heightfield description's geometry: the exact bits of its origin, cell
/// size, grid dimensions, heights and removed cells. The material is the body's, saved inline.
pub(super) fn heightfield_fingerprint(desc: &HeightfieldDesc) -> [u8; 32] {
    let mut hash = Sha256::new();
    hash.update(b"PEHFLD1\0");
    let o = desc.origin;
    for x in [o.0, o.1, o.2, desc.cell_size.0, desc.cell_size.1] {
        hash.update(x.to_bits().to_le_bytes());
    }
    hash.update((desc.columns as u64).to_le_bytes());
    hash.update((desc.rows as u64).to_le_bytes());
    for h in &desc.heights {
        hash.update(h.to_bits().to_le_bytes());
    }
    match &desc.removed_cells {
        None => hash.update([0]),
        Some(cells) => {
            hash.update([1]);
            for &removed in cells {
                hash.update([u8::from(removed)]);
            }
        }
    }
    let mut fingerprint = [0; 32];
    fingerprint.copy_from_slice(&hash.finalize());
    fingerprint
}

/// Terrain, halfspace and character records agree with the bodies: each live heightfield
/// listed once in terrain order, each halfspace once, each character a live kinematic capsule
/// with valid settings, in slot order.
fn validate_parts(
    bodies: &[Body],
    terrain_slots: u32,
    heightfields: &[(u32, [u8; 32])],
    hulls: &[Option<Vec<Vector>>],
    halfspaces: &[BodyId],
    characters: &[SavedCharacter],
) -> Result<(), CheckpointError> {
    let invalid = Err(CheckpointError::InvalidData);
    let by_id = bodies
        .iter()
        .map(|b| (b.id.to_bits(), b))
        .collect::<BTreeMap<_, _>>();
    if by_id.len() != bodies.len() {
        return invalid;
    }
    let mut terrain = bodies
        .iter()
        .filter_map(|b| match b.shape {
            Shape::Heightfield { terrain, .. } => Some(terrain.0),
            _ => None,
        })
        .collect::<Vec<_>>();
    terrain.sort_unstable();
    if heightfields.windows(2).any(|w| w[0].0 >= w[1].0)
        || heightfields.last().is_some_and(|h| h.0 >= terrain_slots)
        || !terrain.iter().eq(heightfields.iter().map(|h| &h.0))
    {
        return invalid;
    }
    let mut live_hulls = bodies
        .iter()
        .filter_map(|b| match b.shape {
            Shape::ConvexHull { hull, .. } => Some(hull.0 as usize),
            _ => None,
        })
        .collect::<Vec<_>>();
    live_hulls.sort_unstable();
    if live_hulls.windows(2).any(|w| w[0] == w[1])
        || !live_hulls.into_iter().eq(hulls
            .iter()
            .enumerate()
            .filter_map(|(i, v)| v.as_ref().map(|_| i)))
    {
        return invalid;
    }
    let mut seen = std::collections::BTreeSet::new();
    if halfspaces.iter().any(|id| {
        !seen.insert(id.to_bits())
            || by_id
                .get(&id.to_bits())
                .is_none_or(|b| b.shape != Shape::HalfSpace)
    }) || halfspaces.len()
        != bodies
            .iter()
            .filter(|b| b.shape == Shape::HalfSpace)
            .count()
    {
        return invalid;
    }
    if characters
        .windows(2)
        .any(|w| w[0].id.slot() >= w[1].id.slot())
        || characters.iter().any(|c| {
            !c.settings.valid()
                || by_id
                    .get(&c.id.to_bits())
                    .is_none_or(|b| !character::can_be_character(b))
        })
    {
        return invalid;
    }
    Ok(())
}

fn validate(
    config: Config,
    elapsed: Scalar,
    last_h: Scalar,
    bodies: &[Body],
    cache: &BTreeMap<(Order, Order), Vec<CachedPoint>>,
) -> Result<(), CheckpointError> {
    World::new(config).map_err(|_| CheckpointError::InvalidData)?;
    if !elapsed.is_finite()
        || elapsed < 0.0
        || !last_h.is_finite()
        // The largest substep, as stored: narrowed to `Real` and widened back to f64.
        || !(0.0..=widen_time(substep_length(0.1, config.substeps))).contains(&last_h)
        // `last_h` is the narrowed substep; rounding is monotonic, so compare at that width.
        || (last_h > 0.0 && narrow_time(elapsed) < narrow_time(last_h))
        || bodies.windows(2).any(|p| p[0].order >= p[1].order)
    {
        return Err(CheckpointError::InvalidData);
    }
    for body in bodies {
        if !body.valid()
            || !valid_orientation(body.orientation)
            || !body.quiet_time.is_finite()
            || body.quiet_time < 0.0
            || [body.force, body.torque, body.impulse, body.angular_impulse]
                .into_iter()
                .any(|v| !v.finite())
            || body.linear_support.is_some_and(|v| !v.finite())
        {
            return Err(CheckpointError::InvalidData);
        }
    }
    for (&(a, b), points) in cache {
        if a >= b
            || points.is_empty()
            || [a, b]
                .into_iter()
                .any(|id| bodies.binary_search_by_key(&id, |b| b.order).is_err())
            || points.iter().any(|p| {
                !p.a.finite()
                    || !p.b.finite()
                    || !p.normal.finite()
                    || p.normal.abs().max_component() > NORMAL_COMPONENT_LIMIT
                    || !p.impulse.is_finite()
                    || p.impulse < 0.0
                    || !p.tangent.finite()
            })
        {
            return Err(CheckpointError::InvalidData);
        }
    }
    Ok(())
}

fn valid_orientation(q: Quaternion) -> bool {
    [q.0, q.1, q.2, q.3]
        .into_iter()
        .all(|x| x.abs() <= ORIENTATION_COMPONENT_LIMIT)
}

fn count(out: &mut Vec<u8>, value: usize) {
    out.extend_from_slice(&(value as u64).to_le_bytes());
}
fn scalar(out: &mut Vec<u8>, value: Real) {
    out.extend_from_slice(&value.to_bits().to_le_bytes());
}
fn time(out: &mut Vec<u8>, value: Scalar) {
    out.extend_from_slice(&value.to_bits().to_le_bytes());
}
fn vector(out: &mut Vec<u8>, v: Vector) {
    for x in [v.0, v.1, v.2] {
        scalar(out, x);
    }
}
fn optional_vector(out: &mut Vec<u8>, v: Option<Vector>) {
    out.push(u8::from(v.is_some()));
    if let Some(v) = v {
        vector(out, v);
    }
}
fn encode_config(out: &mut Vec<u8>, config: Config) {
    vector(out, config.gravity);
    out.extend_from_slice(&[
        config.substeps,
        config.velocity_iterations,
        config.fixed_position_iterations,
    ]);
    for x in [
        config.contact_slop,
        config.sleep_speed,
        config.sleep_seconds,
    ] {
        scalar(out, x);
    }
    out.push(u8::from(config.warm_start));
    out.push(u8::from(config.convergence.is_some()));
    if let Some(c) = config.convergence {
        for x in [c.absolute_velocity, c.absolute_impulse, c.relative] {
            scalar(out, x);
        }
    }
    out.push(match config.convergence_scope {
        ConvergenceScope::WholeWorld => 0,
        ConvergenceScope::ContactIslands => 1,
    });
    scalar(out, config.length_unit);
    let position =
        u8::from(config.position_correction == PositionCorrection::AdmittedContacts) << 1;
    let storage = u8::from(config.body_storage == BodyStorage::Indexed) << 2;
    out.push(position | storage);
}
fn encode_body(out: &mut Vec<u8>, b: &Body) {
    out.extend_from_slice(&b.id.to_bits().to_le_bytes());
    // The key, or the creation serial of a keyless body: together they give its order.
    match b.key {
        Some(key) => {
            out.push(1);
            out.extend_from_slice(&key.to_le_bytes());
        }
        None => {
            out.push(0);
            out.extend_from_slice(&(b.order.0 as u64).to_le_bytes());
        }
    }
    vector(out, b.position);
    vector(out, b.velocity);
    for x in [
        b.orientation.0,
        b.orientation.1,
        b.orientation.2,
        b.orientation.3,
    ] {
        scalar(out, x);
    }
    vector(out, b.angular_velocity);
    match b.shape {
        Shape::Box(h) => {
            out.push(0);
            vector(out, h);
        }
        Shape::Sphere(r) => {
            out.push(1);
            scalar(out, r);
        }
        Shape::Capsule {
            half_segment,
            radius,
        } => {
            out.push(2);
            scalar(out, half_segment);
            scalar(out, radius);
        }
        Shape::Wedge(h) => {
            out.push(3);
            vector(out, h);
        }
        Shape::Heightfield {
            terrain,
            half_extents,
        } => {
            out.push(4);
            out.extend_from_slice(&terrain.0.to_le_bytes());
            vector(out, half_extents);
        }
        Shape::HalfSpace => out.push(5),
        Shape::TriangleMesh { mesh, half_extents } => {
            out.push(6);
            out.extend_from_slice(&mesh.0.to_le_bytes());
            vector(out, half_extents);
        }
        Shape::ConvexHull { hull, half_extents } => {
            out.push(7);
            out.extend_from_slice(&hull.0.to_le_bytes());
            vector(out, half_extents);
        }
    }
    for x in [
        b.mass,
        b.friction,
        b.restitution,
        b.linear_damping,
        b.angular_damping,
    ] {
        scalar(out, x);
    }
    let flags = [
        b.rotation_locked,
        b.external,
        b.sensor,
        b.sleep_allowed,
        b.ccd,
        b.retire_on_impact,
        b.sleeping,
    ]
    .into_iter()
    .enumerate()
    .fold(0u16, |bits, (i, yes)| bits | (u16::from(yes) << i))
        | b.friction_combine.bits() << 7
        | b.restitution_combine.bits() << 9;
    out.extend_from_slice(&flags.to_le_bytes());
    out.extend_from_slice(&b.layers.memberships().to_le_bytes());
    out.extend_from_slice(&b.layers.mask().to_le_bytes());
    optional_vector(out, b.linear_support);
    scalar(out, b.quiet_time);
    for v in [b.force, b.torque, b.impulse, b.angular_impulse] {
        vector(out, v);
    }
}

fn optional_scalar(out: &mut Vec<u8>, x: Option<Real>) {
    out.push(u8::from(x.is_some()));
    if let Some(x) = x {
        scalar(out, x);
    }
}
fn encode_character(out: &mut Vec<u8>, c: &SavedCharacter) {
    out.extend_from_slice(&c.id.to_bits().to_le_bytes());
    let s = c.settings;
    scalar(out, s.offset);
    scalar(out, s.max_slope);
    optional_scalar(out, s.snap_to_ground);
    out.push(s.max_slides);
    optional_scalar(out, s.plane);
    optional_scalar(out, s.max_push);
    out.push(match s.embedded {
        Embedded::PushOut => 0,
        Embedded::Stay => 1,
    });
    out.push(u8::from(c.exclude.is_some()));
    if let Some(id) = c.exclude {
        out.extend_from_slice(&id.to_bits().to_le_bytes());
    }
    out.push(u8::from(c.clear));
}

struct Reader<'a> {
    bytes: &'a [u8],
}
impl<'a> Reader<'a> {
    fn take(&mut self, length: usize) -> Result<&'a [u8], CheckpointError> {
        let value = self
            .bytes
            .get(..length)
            .ok_or(CheckpointError::InvalidData)?;
        self.bytes = &self.bytes[length..];
        Ok(value)
    }
    fn array<const N: usize>(&mut self) -> Result<[u8; N], CheckpointError> {
        self.take(N)?
            .try_into()
            .map_err(|_| CheckpointError::InvalidData)
    }
    fn u8(&mut self) -> Result<u8, CheckpointError> {
        Ok(self.array::<1>()?[0])
    }
    fn u16(&mut self) -> Result<u16, CheckpointError> {
        Ok(u16::from_le_bytes(self.array()?))
    }
    fn u32(&mut self) -> Result<u32, CheckpointError> {
        Ok(u32::from_le_bytes(self.array()?))
    }
    fn u128(&mut self) -> Result<u128, CheckpointError> {
        Ok(u128::from_le_bytes(self.array()?))
    }
    fn u64(&mut self) -> Result<u64, CheckpointError> {
        Ok(u64::from_le_bytes(self.array()?))
    }
    fn scalar(&mut self) -> Result<Real, CheckpointError> {
        Ok(Real::from_le_bytes(self.array()?))
    }
    fn time(&mut self) -> Result<Scalar, CheckpointError> {
        Ok(Scalar::from_bits(self.u64()?))
    }
    fn vector(&mut self) -> Result<Vector, CheckpointError> {
        Ok(Vector(self.scalar()?, self.scalar()?, self.scalar()?))
    }
    fn boolean(&mut self) -> Result<bool, CheckpointError> {
        match self.u8()? {
            0 => Ok(false),
            1 => Ok(true),
            _ => Err(CheckpointError::InvalidData),
        }
    }
    fn optional_vector(&mut self) -> Result<Option<Vector>, CheckpointError> {
        if self.boolean()? {
            Ok(Some(self.vector()?))
        } else {
            Ok(None)
        }
    }
    fn optional_scalar(&mut self) -> Result<Option<Real>, CheckpointError> {
        if self.boolean()? {
            Ok(Some(self.scalar()?))
        } else {
            Ok(None)
        }
    }
    fn character(&mut self) -> Result<SavedCharacter, CheckpointError> {
        let id = BodyId::from_bits(self.u64()?);
        let settings = CharacterSettings {
            offset: self.scalar()?,
            max_slope: self.scalar()?,
            snap_to_ground: self.optional_scalar()?,
            max_slides: self.u8()?,
            plane: self.optional_scalar()?,
            max_push: self.optional_scalar()?,
            embedded: match self.u8()? {
                0 => Embedded::PushOut,
                1 => Embedded::Stay,
                _ => return Err(CheckpointError::InvalidData),
            },
        };
        let exclude = if self.boolean()? {
            Some(BodyId::from_bits(self.u64()?))
        } else {
            None
        };
        Ok(SavedCharacter {
            id,
            settings,
            exclude,
            clear: self.boolean()?,
        })
    }
    fn count(&mut self, limit: usize, minimum: usize) -> Result<usize, CheckpointError> {
        let count = usize::try_from(self.u64()?).map_err(|_| CheckpointError::ResourceLimit)?;
        if count > limit {
            return Err(CheckpointError::ResourceLimit);
        }
        if count > self.bytes.len() / minimum {
            return Err(CheckpointError::InvalidData);
        }
        Ok(count)
    }
    fn config(&mut self) -> Result<Config, CheckpointError> {
        let mut config = Config {
            gravity: self.vector()?,
            substeps: self.u8()?,
            velocity_iterations: self.u8()?,
            fixed_position_iterations: self.u8()?,
            position_correction: PositionCorrection::FixedColliders,
            contact_slop: self.scalar()?,
            sleep_speed: self.scalar()?,
            sleep_seconds: self.scalar()?,
            warm_start: self.boolean()?,
            convergence: if self.boolean()? {
                Some(Convergence {
                    absolute_velocity: self.scalar()?,
                    absolute_impulse: self.scalar()?,
                    relative: self.scalar()?,
                })
            } else {
                None
            },
            convergence_scope: match self.u8()? {
                0 => ConvergenceScope::WholeWorld,
                1 => ConvergenceScope::ContactIslands,
                _ => return Err(CheckpointError::InvalidData),
            },
            length_unit: 1.0,
            body_storage: BodyStorage::Ordered,
        };
        config.length_unit = self.scalar()?;
        let policies = self.u8()?;
        if policies & !7 != 0 {
            return Err(CheckpointError::InvalidData);
        }
        if policies & 2 != 0 {
            config.position_correction = PositionCorrection::AdmittedContacts;
        }
        if policies & 4 != 0 {
            config.body_storage = BodyStorage::Indexed;
        }
        let soft = policies & 1 != 0;
        if soft {
            return Err(CheckpointError::UnsupportedPolicy);
        }
        Ok(config)
    }
    fn body(&mut self) -> Result<Body, CheckpointError> {
        let id = BodyId::from_bits(self.u64()?);
        let (key, serial) = match self.u8()? {
            0 => (None, self.u64()?),
            1 => (Some(self.u64()?), 0),
            _ => return Err(CheckpointError::InvalidData),
        };
        let position = self.vector()?;
        let velocity = self.vector()?;
        let orientation = Quaternion(
            self.scalar()?,
            self.scalar()?,
            self.scalar()?,
            self.scalar()?,
        );
        let angular_velocity = self.vector()?;
        let shape = match self.u8()? {
            0 => Shape::Box(self.vector()?),
            1 => Shape::Sphere(self.scalar()?),
            2 => Shape::capsule(self.scalar()?, self.scalar()?),
            3 => Shape::Wedge(self.vector()?),
            4 => Shape::Heightfield {
                terrain: TerrainId(self.u32()?),
                half_extents: self.vector()?,
            },
            5 => Shape::HalfSpace,
            6 => Shape::TriangleMesh {
                mesh: TriangleMeshId(self.u32()?),
                half_extents: self.vector()?,
            },
            7 => Shape::ConvexHull {
                hull: ConvexHullId(self.u32()?),
                half_extents: self.vector()?,
            },
            _ => return Err(CheckpointError::InvalidData),
        };
        let mut body = Body::new(shape, position, self.scalar()?);
        body.id = id;
        body.key = key;
        body.order = Order::new(key, serial);
        body.velocity = velocity;
        body.orientation = orientation;
        body.angular_velocity = angular_velocity;
        body.friction = self.scalar()?;
        body.restitution = self.scalar()?;
        body.linear_damping = self.scalar()?;
        body.angular_damping = self.scalar()?;
        let flags = self.u16()?;
        if flags & !0x7ff != 0 {
            return Err(CheckpointError::InvalidData);
        }
        body.friction_combine = CombineRule::from_bits(flags >> 7);
        body.restitution_combine = CombineRule::from_bits(flags >> 9);
        body.rotation_locked = flags & 1 != 0;
        body.external = flags & 2 != 0;
        body.sensor = flags & 4 != 0;
        body.sleep_allowed = flags & 8 != 0;
        body.ccd = flags & 16 != 0;
        body.retire_on_impact = flags & 32 != 0;
        body.sleeping = flags & 64 != 0;
        body.layers = CollisionLayers3d::new(self.u32()?, self.u32()?);
        body.linear_support = self.optional_vector()?;
        body.quiet_time = self.scalar()?;
        body.force = self.vector()?;
        body.torque = self.vector()?;
        body.impulse = self.vector()?;
        body.angular_impulse = self.vector()?;
        Ok(body)
    }
}
