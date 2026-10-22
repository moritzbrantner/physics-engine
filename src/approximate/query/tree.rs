//! Query acceleration over `rust-kernels`' incremental AABB tree. A body's leaf ID is its id
//! slot, so storage layout changes (inserts, tombstones, compaction) never touch the tree; the
//! world's slot table maps a leaf back to the body. The tree is kept current by the public
//! mutators and at the end of every step; queries fall back to a full scan whenever it is stale,
//! so the tree never changes a query result, only how many bodies are tested.

use super::super::{Body, BodyId, Real, Vector as V};
use bvh_kernels::DynamicAabbTree;
use spatial_kernels::{Aabb, Body as Leaf};

/// Fat margin of the tree leaves, in scene units: small moves update a leaf in place.
const FAT_MARGIN: f32 = 0.1;
/// Steps of motion a reinserted moving leaf is fattened for.
const PREDICTED_STEPS: Real = 4.0;

#[derive(Clone, Debug)]
pub(in crate::approximate) struct QueryTree {
    tree: DynamicAabbTree,
    /// Leaves in the tree.
    leaves: usize,
    /// Layout generation the tree matches; `None` before the first sync.
    generation: Option<u64>,
    /// Upper bound of every leaf coordinate's magnitude, for conservative padding.
    pub magnitude: Real,
    /// Inserts since the last full rebuild; incremental inserts degrade the hierarchy.
    inserts: usize,
}

impl Default for QueryTree {
    fn default() -> Self {
        Self {
            tree: DynamicAabbTree::new(FAT_MARGIN),
            leaves: 0,
            generation: Some(0),
            magnitude: 0.0,
            inserts: 0,
        }
    }
}

fn is_static(b: &Body) -> bool {
    b.mass == 0.0 && !b.external
}

fn aabb((min, max): (V, V)) -> Aabb {
    Aabb {
        min: [min.0, min.1, min.2],
        max: [max.0, max.1, max.2],
    }
}

impl QueryTree {
    pub fn current(&self, generation: u64) -> bool {
        self.generation == Some(generation)
    }

    fn leaf(&mut self, body: &Body) -> Leaf {
        let (min, max) = body.cached_bounds;
        self.magnitude = self
            .magnitude
            .max(min.abs().max_component())
            .max(max.abs().max_component());
        Leaf {
            id: body.id.slot(),
            aabb: aabb(body.cached_bounds),
        }
    }

    /// `body` was just added.
    pub fn inserted(&mut self, body: &Body, generation: u64) {
        let leaf = self.leaf(body);
        self.tree.insert(leaf);
        self.leaves += 1;
        crate::work::add(crate::work::Counter::TreeInserts, 1);
        // Rebuild once inserts reach half the tree: amortized O(log n) per insert.
        self.inserts += 1;
        if self.inserts >= 16.max(self.leaves / 2) {
            crate::work::add(crate::work::Counter::TreeInserts, self.leaves as u64);
            self.tree.rebuild();
            self.inserts = 0;
        }
        self.generation = Some(generation);
    }

    /// The body in id slot `slot` was just removed.
    pub fn removed(&mut self, slot: u32, generation: u64) {
        self.tree.remove(slot);
        self.leaves -= 1;
        self.generation = Some(generation);
    }

    /// The layout generation changed without changing which bodies exist (storage compaction).
    pub fn retag(&mut self, from: u64, to: u64) {
        if self.current(from) {
            self.generation = Some(to);
        }
    }

    /// Brings the tree up to date: a rebuild after an untracked layout change, otherwise a
    /// bounds update of every movable body. Tombstones (`BodyId::NONE`) are skipped.
    pub fn sync(&mut self, bodies: &[Body], generation: u64) {
        let live = bodies.iter().filter(|b| b.id != BodyId::NONE);
        if self.current(generation) {
            for body in live {
                if !is_static(body) {
                    let leaf = self.leaf(body);
                    self.tree.update(leaf);
                }
            }
            return;
        }
        *self = Self::default();
        for body in live {
            let leaf = self.leaf(body);
            self.tree.insert(leaf);
            self.leaves += 1;
        }
        self.tree.rebuild();
        self.generation = Some(generation);
    }

    /// The end of a `dt` step: like [`Self::sync`], but a moving leaf that must be reinserted
    /// is also fattened along its velocity for the next [`PREDICTED_STEPS`] steps, so steadily
    /// moving bodies stop being reinserted every step (#375). Only query candidates change.
    pub fn sync_step(&mut self, bodies: &[Body], generation: u64, dt: Real) {
        if !self.current(generation) {
            return self.sync(bodies, generation);
        }
        let ahead = dt * PREDICTED_STEPS;
        for body in bodies.iter().filter(|b| b.id != BodyId::NONE) {
            if !is_static(body) {
                let leaf = self.leaf(body);
                let d = body.velocity * ahead;
                let displacement = [d.0, d.1, d.2];
                if displacement.iter().all(|d| d.is_finite()) {
                    self.tree.update_predicted(leaf, displacement);
                } else {
                    self.tree.update(leaf);
                }
            }
        }
    }

    /// `body` moved: refit its leaf.
    pub fn moved(&mut self, body: &Body) {
        let leaf = self.leaf(body);
        self.tree.update(leaf);
    }

    pub fn tree(&self) -> &DynamicAabbTree {
        &self.tree
    }
}
