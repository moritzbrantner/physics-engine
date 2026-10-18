//! Query acceleration over `rust-kernels`' incremental AABB tree. Each body owns a compact slot
//! (the tree's collider ID). The tree is kept current by the public mutators and at the end of
//! every step; queries fall back to a full scan whenever it is stale, so the tree never changes
//! a query result, only how many bodies are tested.

use super::super::{Body, Real, Vector as V};
use bvh_kernels::DynamicAabbTree;
use spatial_kernels::{Aabb, Body as Leaf};

/// Fat margin of the tree leaves, in scene units: small moves update a leaf in place.
const FAT_MARGIN: f32 = 0.1;

#[derive(Clone, Debug)]
pub(in crate::approximate) struct QueryTree {
    tree: DynamicAabbTree,
    /// Slot of each body, parallel to `World::bodies`.
    slots: Vec<u32>,
    /// Body index of each slot (`u32::MAX` when free).
    index_of: Vec<u32>,
    free: Vec<u32>,
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
            slots: Vec::new(),
            index_of: Vec::new(),
            free: Vec::new(),
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

    fn leaf(&mut self, slot: u32, body: &Body) -> Leaf {
        let (min, max) = body.cached_bounds;
        self.magnitude = self
            .magnitude
            .max(min.abs().max_component())
            .max(max.abs().max_component());
        Leaf {
            id: slot,
            aabb: aabb(body.cached_bounds),
        }
    }

    /// Body `index` was just inserted; later bodies shifted up by one.
    pub fn inserted(&mut self, bodies: &[Body], index: usize, generation: u64) {
        let slot = self.free.pop().unwrap_or(self.index_of.len() as u32);
        if slot as usize == self.index_of.len() {
            self.index_of.push(u32::MAX);
        }
        for &s in &self.slots[index..] {
            self.index_of[s as usize] += 1;
        }
        self.slots.insert(index, slot);
        self.index_of[slot as usize] = index as u32;
        let leaf = self.leaf(slot, &bodies[index]);
        self.tree.insert(leaf);
        crate::work::add(crate::work::Counter::TreeInserts, 1);
        // Rebuild once inserts reach half the tree: amortized O(log n) per insert.
        self.inserts += 1;
        if self.inserts >= 16.max(self.slots.len() / 2) {
            crate::work::add(crate::work::Counter::TreeInserts, self.slots.len() as u64);
            self.tree.rebuild();
            self.inserts = 0;
        }
        self.generation = Some(generation);
    }

    /// Body `index` was just removed; later bodies shifted down by one.
    pub fn removed(&mut self, index: usize, generation: u64) {
        let slot = self.slots.remove(index);
        self.tree.remove(slot);
        self.index_of[slot as usize] = u32::MAX;
        self.free.push(slot);
        for &s in &self.slots[index..] {
            self.index_of[s as usize] -= 1;
        }
        self.generation = Some(generation);
    }

    /// Brings the tree up to date: a rebuild after an untracked layout change, otherwise a
    /// bounds update of every movable body.
    pub fn sync(&mut self, bodies: &[Body], generation: u64) {
        if self.current(generation) {
            for (i, body) in bodies.iter().enumerate() {
                if !is_static(body) {
                    let leaf = self.leaf(self.slots[i], body);
                    self.tree.update(leaf);
                }
            }
            return;
        }
        *self = Self::default();
        for (i, body) in bodies.iter().enumerate() {
            let slot = i as u32;
            self.slots.push(slot);
            self.index_of.push(slot);
            let leaf = self.leaf(slot, body);
            self.tree.insert(leaf);
        }
        self.tree.rebuild();
        self.generation = Some(generation);
    }

    /// Body `index` moved: refit its leaf.
    pub fn moved(&mut self, bodies: &[Body], index: usize) {
        let leaf = self.leaf(self.slots[index], &bodies[index]);
        self.tree.update(leaf);
    }

    pub fn index_of(&self, slot: u32) -> usize {
        self.index_of[slot as usize] as usize
    }

    pub fn tree(&self) -> &DynamicAabbTree {
        &self.tree
    }
}
