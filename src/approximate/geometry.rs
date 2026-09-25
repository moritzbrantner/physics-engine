//! Dependency-keyed contact geometry, separate from warm-start impulses and contact topology.
//!
//! Reuse complete current-contact results only for bit-identical poses/shapes/margins. Translations
//! still run SAT and clipping, but unchanged orientations reuse frames and support projections.
//! Velocity, timestep and response state are not geometric keys: CCD always runs fresh after a miss.
use super::{Body, BodyId, Quaternion, Scalar, Shape, Vector, contact};
use std::collections::BTreeMap;

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct GeometryStats {
    pub current_queries: u64,
    pub manifold_hits: u64,
    /// Includes cached negative current-contact results; swept queries are never cached.
    pub negative_hits: u64,
    pub manifold_refreshes: u64,
    pub frame_preparations: u64,
    pub frame_reuses: u64,
    pub projection_preparations: u64,
    pub projection_reuses: u64,
    pub sat_queries: u64,
    pub sat_axes_tested: u64,
    pub clip_passes: u64,
    pub sweep_queries: u64,
    /// Canonical unordered pair dispatches by `PrimitivePair::index()`.
    pub specialized_pair_dispatches: [u64; 15],
    /// Production calls that could not use a specialized primitive kernel.
    ///
    /// All currently supported fixed-topology primitive pairs are specialized, so this is
    /// ratcheted to zero until a deliberately generic shape is introduced.
    pub generic_fallback_calls: u64,
    /// Support-map evaluations performed by specialized primitive kernels.
    pub support_evaluations: u64,
    /// Contact manifolds admitted by fresh narrow-phase generation.
    pub manifold_candidates: u64,
    /// Current-contact queries routed through capsule/wedge specialized geometry.
    pub primitive_queries: u64,
    /// Fixed SAT axes tested by wedge/box polyhedral queries.
    pub primitive_axes_tested: u64,
    /// Wedge vertex dot products; capsules and boxes use analytic support.
    pub primitive_vertex_tests: u64,
    /// Conservative-advance or swept-SAT iterations for new primitive CCD.
    pub primitive_sweep_iterations: u64,
    pub pair_invalidations: u64,
    pub cached_pairs_peak: u64,
    /// Retained payload capacity, excluding BTreeMap node/allocator overhead.
    pub retained_bytes: u64,
}

fn vector_bits(v: Vector) -> [u64; 3] {
    [v.0.to_bits(), v.1.to_bits(), v.2.to_bits()]
}
fn quaternion_bits(q: Quaternion) -> [u64; 4] {
    [q.0.to_bits(), q.1.to_bits(), q.2.to_bits(), q.3.to_bits()]
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct ShapeKey {
    id: BodyId,
    shape: [u64; 4],
    orientation: [u64; 4],
}
impl ShapeKey {
    fn new(b: &Body) -> Self {
        let shape = match b.shape {
            Shape::Box(h) => [0, h.0.to_bits(), h.1.to_bits(), h.2.to_bits()],
            Shape::Sphere(r) => [1, r.to_bits(), 0, 0],
            Shape::Capsule {
                half_segment,
                radius,
            } => [2, half_segment.to_bits(), radius.to_bits(), 0],
            Shape::Wedge(h) => [3, h.0.to_bits(), h.1.to_bits(), h.2.to_bits()],
            Shape::Cylinder {
                half_height,
                radius,
            } => [4, half_height.to_bits(), radius.to_bits(), 0],
        };
        Self {
            id: b.id,
            shape,
            orientation: quaternion_bits(b.orientation),
        }
    }
}
#[derive(Clone, Copy, Debug)]
struct Frame {
    key: ShapeKey,
    axes: [Vector; 3],
    epoch: u64,
}
#[derive(Clone, Debug)]
struct Pair {
    keys: [ShapeKey; 2],
    positions: [[u64; 3]; 2],
    margin: u64,
    // Negative entries retain no manifold allocation; positive storage is reused on refresh.
    current: Option<Box<contact::Manifold>>,
    projections: contact::BoxProjections,
    epoch: u64,
}
#[derive(Clone, Debug, Default)]
pub(super) struct GeometryCache {
    frames: Vec<Option<Frame>>,
    pairs: BTreeMap<(BodyId, BodyId), Pair>,
    clipping: contact::ClipScratch,
    epoch: u64,
    retained_bytes: usize,
}
impl GeometryCache {
    pub fn begin(&mut self, bodies: usize) {
        if self.frames.len() != bodies {
            self.frames.clear();
            self.frames.resize(bodies, None);
        }
        self.epoch = self.epoch.wrapping_add(1);
        if self.epoch == 0 {
            self.pairs.clear();
            self.frames.fill(None);
        }
    }
    fn frame(&mut self, index: usize, b: &Body, work: &mut GeometryStats) -> [Vector; 3] {
        if let Some(f) = self.frames[index]
            && f.epoch == self.epoch
            && f.key.id == b.id
        {
            work.frame_reuses += 1;
            return f.axes;
        }
        let key = ShapeKey::new(b);
        if let Some(f) = &mut self.frames[index]
            && f.key == key
        {
            work.frame_reuses += 1;
            f.epoch = self.epoch;
            return f.axes;
        }
        work.frame_preparations += 1;
        let axes = b.orientation.axes();
        self.frames[index] = Some(Frame {
            key,
            axes,
            epoch: self.epoch,
        });
        axes
    }
    pub fn query(
        &mut self,
        indices: [usize; 2],
        bodies: [&Body; 2],
        margin: Scalar,
        work: &mut GeometryStats,
    ) -> Option<contact::Manifold> {
        let [a, b] = bodies;
        let stable = |b: &Body| b.mass == 0.0 || b.sleeping || b.rotation_locked;
        if !stable(a) || !stable(b) {
            // No pair keys, projections, retained manifold, or map lookup on the rotating path.
            // Frames are nevertheless shared by every pair using the same body in this pass.
            if matches!((a.shape, b.shape), (Shape::Box(_), Shape::Box(_))) {
                let frames = [
                    self.frame(indices[0], a, work),
                    self.frame(indices[1], b, work),
                ];
                work.current_queries += 1;
                work.manifold_refreshes += 1;
                return contact::box_current_with_frames(
                    a,
                    b,
                    margin,
                    frames,
                    work,
                    &mut self.clipping,
                );
            }
            return contact::current_counted(a, b, margin, work);
        }
        self.current(indices, bodies, margin, work)
    }
    pub fn swept(
        &mut self,
        indices: [usize; 2],
        bodies: [&Body; 2],
        dt: Scalar,
        margin: Scalar,
        work: &mut GeometryStats,
    ) -> Option<contact::Manifold> {
        let [a, b] = bodies;
        if matches!((a.shape, b.shape), (Shape::Box(_), Shape::Box(_))) {
            let frames = [
                self.frame(indices[0], a, work),
                self.frame(indices[1], b, work),
            ];
            contact::box_swept_with_frames(a, b, dt, margin, frames, work, &mut self.clipping)
        } else {
            contact::swept(a, b, dt, margin, work)
        }
    }
    pub fn current(
        &mut self,
        indices: [usize; 2],
        bodies: [&Body; 2],
        margin: Scalar,
        work: &mut GeometryStats,
    ) -> Option<contact::Manifold> {
        let [a, b] = bodies;
        let id = (a.id, b.id);
        let keys = [ShapeKey::new(a), ShapeKey::new(b)];
        let positions = [vector_bits(a.position), vector_bits(b.position)];
        work.current_queries += 1;
        if let Some(pair) = self.pairs.get_mut(&id)
            && pair.keys == keys
            && pair.positions == positions
            && pair.margin == margin.to_bits()
        {
            pair.epoch = self.epoch;
            work.manifold_hits += 1;
            work.negative_hits += u64::from(pair.current.is_none());
            return pair.current.as_deref().cloned();
        }
        let frames = if matches!((a.shape, b.shape), (Shape::Box(_), Shape::Box(_))) {
            Some([
                self.frame(indices[0], a, work),
                self.frame(indices[1], b, work),
            ])
        } else {
            None
        };
        let pair = self.pairs.entry(id).or_insert_with(|| Pair {
            keys,
            positions,
            margin: margin.to_bits(),
            current: None,
            projections: contact::BoxProjections::default(),
            epoch: self.epoch,
        });
        let key_changed = pair.keys != keys;
        let result = if let Some(frames) = frames {
            // An empty projection list is represented by zero retained capacity on first use.
            if key_changed || pair.projections.retained_bytes() == 0 {
                pair.projections.refresh(a, b, frames[0], frames[1]);
                work.projection_preparations += 1;
            } else {
                work.projection_reuses += 1;
            }
            work.manifold_refreshes += 1;
            contact::box_prepared(
                a,
                b,
                margin,
                &pair.projections,
                frames,
                work,
                &mut self.clipping,
            )
        } else {
            // Already counted the query; the uncached sphere path records its own work.
            work.current_queries -= 1;
            contact::current_counted(a, b, margin, work)
        };
        pair.keys = keys;
        pair.positions = positions;
        pair.margin = margin.to_bits();
        match (&mut pair.current, &result) {
            (Some(old), Some(new)) => {
                old.normal = new.normal;
                old.points.clone_from(&new.points);
                old.swept = new.swept;
                old.time = new.time;
            }
            _ => pair.current = result.clone().map(Box::new),
        }
        pair.epoch = self.epoch;
        result
    }
    pub fn finish(&mut self, work: &mut GeometryStats) {
        let before = self.pairs.len();
        self.pairs.retain(|_, p| p.epoch == self.epoch);
        work.pair_invalidations += (before - self.pairs.len()) as u64;
        work.cached_pairs_peak = work.cached_pairs_peak.max(self.pairs.len() as u64);
        self.retained_bytes = self.measure_retained_bytes();
        work.retained_bytes = self.retained_bytes as u64;
    }
    pub fn remove(&mut self, id: BodyId) {
        self.pairs.retain(|&(a, b), _| a != id && b != id);
        // Body indices may all move, including remove/reinsert with the same length and id.
        self.frames.clear();
        self.retained_bytes = self.measure_retained_bytes();
    }
    pub fn retained_bytes(&self) -> usize {
        self.retained_bytes
    }
    fn measure_retained_bytes(&self) -> usize {
        self.clipping.retained_bytes()
            + self.frames.capacity() * std::mem::size_of::<Option<Frame>>()
            + self
                .pairs
                .values()
                .map(|p| {
                    std::mem::size_of::<((BodyId, BodyId), Pair)>()
                        + p.projections.retained_bytes()
                        + p.current
                            .as_ref()
                            .map_or(0, |_| std::mem::size_of::<contact::Manifold>())
                })
                .sum::<usize>()
    }
}

#[cfg(test)]
mod tests;
