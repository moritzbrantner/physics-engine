//! Query acceleration over `rust-kernels`' AABB trees. A body's leaf ID is its id slot, so
//! storage layout changes (inserts, tombstones, compaction) never touch the trees; the world's
//! slot table maps a leaf back to the body. The trees are kept current by the public mutators
//! and at the end of every step; queries fall back to a full scan whenever they are stale, so
//! the trees never change a query result, only how many bodies are tested.
//!
//! Fixed bodies (level geometry, most of a game's bodies) live apart from moving ones (#415):
//! in an incremental tree with exact bounds and, from the first ray or shape query on, also in
//! a four-wide [`WideBvh`] that queries descend instead. That build is O(n log n), so it waits
//! for a query: a world that only steps (whose broad phase touches a few fixed bodies per awake
//! one) never pays for it, and the step uses the wide tree only once a query has built it. A
//! fixed body added while the wide tree exists also waits in the moving tree, and a removed one
//! is dropped from the wide tree in place, until enough such changes accumulate to drop the
//! wide tree; the next query rebuilds it. A fixed body that moves becomes a moving one.

use super::super::{Body, BodyId, Real, Vector as V};
use bvh_kernels::{DynamicAabbTree, WideBvh};
use spatial_kernels::{Aabb, Body as Leaf};
use std::sync::OnceLock;

/// Fat margin of the moving tree's leaves, in scene units: small moves update a leaf in place.
const FAT_MARGIN: f32 = 0.1;
/// Steps of motion a reinserted moving leaf is fattened for.
const PREDICTED_STEPS: Real = 4.0;

/// Where a body's leaf lives.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Place {
    Absent,
    /// A moving body in the moving tree.
    Moving,
    /// A fixed body at this index of `fixed_leaves`, in the fixed tree (and in the wide tree,
    /// when there is one).
    Fixed(u32),
    /// A fixed body (index into `fixed_leaves`, then into `pending`) added while the wide tree
    /// existed: in the fixed and moving trees until the wide tree is dropped.
    Pending(u32, u32),
}

/// An incremental tree that rebuilds itself once inserts reach half its leaves.
#[derive(Clone, Debug)]
struct Tree {
    tree: DynamicAabbTree,
    /// Inserts since the last full rebuild; incremental inserts degrade the hierarchy.
    inserts: usize,
}

impl Tree {
    fn new(margin: f32) -> Self {
        Self {
            tree: DynamicAabbTree::new(margin),
            inserts: 0,
        }
    }

    /// Inserts `leaf`; rebuilds once inserts reach half the tree: amortized O(log n).
    fn insert(&mut self, leaf: Leaf) {
        self.tree.insert(leaf);
        crate::work::add(crate::work::Counter::TreeInserts, 1);
        self.inserts += 1;
        if self.inserts >= 16.max(self.tree.len() / 2) {
            crate::work::add(crate::work::Counter::TreeInserts, self.tree.len() as u64);
            self.tree.rebuild();
            self.inserts = 0;
        }
    }
}

#[derive(Clone, Debug)]
pub(in crate::approximate) struct QueryTree {
    /// Moving bodies and pending fixed ones.
    moving: Tree,
    /// Every fixed body, exact bounds.
    fixed: Tree,
    /// Exact leaves of every fixed body: what a wide tree is built from.
    fixed_leaves: Vec<Leaf>,
    /// Four-wide tree over `fixed_leaves`, built by the first query that needs it.
    wide: OnceLock<WideBvh>,
    /// Slots of the pending fixed bodies.
    pending: Vec<u32>,
    /// Pending bodies plus leaves dropped from the wide tree.
    stale: usize,
    /// Leaf place per id slot.
    places: Vec<Place>,
    /// Layout generation the trees match; `None` before the first sync.
    generation: Option<u64>,
    /// Upper bound of every leaf coordinate's magnitude, for conservative padding.
    pub magnitude: Real,
}

impl Default for QueryTree {
    fn default() -> Self {
        Self {
            moving: Tree::new(FAT_MARGIN),
            fixed: Tree::new(0.0),
            fixed_leaves: Vec::new(),
            wide: OnceLock::new(),
            pending: Vec::new(),
            stale: 0,
            places: Vec::new(),
            generation: Some(0),
            magnitude: 0.0,
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

    fn place(&self, slot: u32) -> Place {
        self.places
            .get(slot as usize)
            .copied()
            .unwrap_or(Place::Absent)
    }

    fn set_place(&mut self, slot: u32, place: Place) {
        let slot = slot as usize;
        if self.places.len() <= slot {
            self.places.resize(slot + 1, Place::Absent);
        }
        self.places[slot] = place;
    }

    /// The wide tree, built here on first use.
    fn wide(&self) -> &WideBvh {
        self.wide.get_or_init(|| {
            crate::work::add(
                crate::work::Counter::TreeInserts,
                self.fixed_leaves.len() as u64,
            );
            WideBvh::build(&self.fixed_leaves)
        })
    }

    fn insert_fixed(&mut self, leaf: Leaf) -> u32 {
        let index = self.fixed_leaves.len() as u32;
        self.fixed_leaves.push(leaf);
        self.fixed.insert(leaf);
        index
    }

    /// Removes `slot`'s leaf from wherever it lives.
    fn take(&mut self, slot: u32) {
        let leaf = match self.place(slot) {
            Place::Absent => return,
            Place::Moving => {
                self.moving.tree.remove(slot);
                None
            }
            Place::Pending(leaf, index) => {
                self.moving.tree.remove(slot);
                self.pending.swap_remove(index as usize);
                if let Some(&other) = self.pending.get(index as usize)
                    && let Place::Pending(l, _) = self.place(other)
                {
                    self.set_place(other, Place::Pending(l, index));
                }
                self.stale -= 1;
                Some(leaf)
            }
            Place::Fixed(leaf) => {
                if let Some(wide) = self.wide.get_mut() {
                    wide.remove(slot);
                    self.stale += 1;
                }
                Some(leaf)
            }
        };
        if let Some(leaf) = leaf {
            self.fixed.tree.remove(slot);
            self.fixed_leaves.swap_remove(leaf as usize);
            if let Some(other) = self.fixed_leaves.get(leaf as usize).map(|l| l.id) {
                let place = match self.place(other) {
                    Place::Pending(_, index) => Place::Pending(leaf, index),
                    _ => Place::Fixed(leaf),
                };
                self.set_place(other, place);
            }
        }
        self.set_place(slot, Place::Absent);
        self.drop_stale_wide();
    }

    /// Once pending bodies and dropped leaves reach 1/16 of the fixed set, takes the pending
    /// bodies out of the moving tree and drops the wide tree; the next query rebuilds it, so
    /// the rebuild cost is amortized over the changes.
    fn drop_stale_wide(&mut self) {
        if self.stale < 8.max(self.fixed_leaves.len() / 16) {
            return;
        }
        for slot in std::mem::take(&mut self.pending) {
            self.moving.tree.remove(slot);
            if let Place::Pending(leaf, _) = self.place(slot) {
                self.set_place(slot, Place::Fixed(leaf));
            }
        }
        self.stale = 0;
        self.wide = OnceLock::new();
    }

    /// `body` was just added.
    pub fn inserted(&mut self, body: &Body, generation: u64) {
        let leaf = self.leaf(body);
        if !is_static(body) {
            self.set_place(leaf.id, Place::Moving);
            self.moving.insert(leaf);
        } else {
            let index = self.insert_fixed(leaf);
            if self.wide.get().is_some() {
                self.set_place(leaf.id, Place::Pending(index, self.pending.len() as u32));
                self.pending.push(leaf.id);
                // Pending leaves leave when the wide tree is dropped, so they never degrade
                // the moving tree for long and do not count towards its rebuild.
                self.moving.tree.insert(leaf);
                crate::work::add(crate::work::Counter::TreeInserts, 1);
                self.stale += 1;
                self.drop_stale_wide();
            } else {
                self.set_place(leaf.id, Place::Fixed(index));
            }
        }
        self.generation = Some(generation);
    }

    /// The body in id slot `slot` was just removed.
    pub fn removed(&mut self, slot: u32, generation: u64) {
        self.take(slot);
        self.generation = Some(generation);
    }

    /// The layout generation changed without changing which bodies exist (storage compaction).
    pub fn retag(&mut self, from: u64, to: u64) {
        if self.current(from) {
            self.generation = Some(to);
        }
    }

    /// Refits `leaf` (fattened along `displacement` when given) in the moving tree, moving it
    /// there first if it was fixed: a fixed body that moves is no longer fixed geometry.
    fn refit(&mut self, leaf: Leaf, displacement: Option<[f32; 3]>) {
        match self.place(leaf.id) {
            Place::Moving => {}
            Place::Absent => return,
            Place::Pending(..) | Place::Fixed(_) => {
                self.take(leaf.id);
                self.set_place(leaf.id, Place::Moving);
                self.moving.insert(leaf);
            }
        }
        match displacement {
            Some(d) if d.iter().all(|d| d.is_finite()) => {
                self.moving.tree.update_predicted(leaf, d);
            }
            _ => {
                self.moving.tree.update(leaf);
            }
        }
    }

    /// Brings the trees up to date: a rebuild after an untracked layout change, otherwise a
    /// bounds update of every movable body. Tombstones (`BodyId::NONE`) are skipped.
    pub fn sync(&mut self, bodies: &[Body], generation: u64) {
        let live = bodies.iter().filter(|b| b.id != BodyId::NONE);
        if self.current(generation) {
            for body in live {
                if !is_static(body) {
                    let leaf = self.leaf(body);
                    self.refit(leaf, None);
                }
            }
            return;
        }
        *self = Self::default();
        for body in live {
            let leaf = self.leaf(body);
            if is_static(body) {
                let index = self.fixed_leaves.len() as u32;
                self.fixed_leaves.push(leaf);
                self.fixed.tree.insert(leaf);
                self.set_place(leaf.id, Place::Fixed(index));
            } else {
                self.moving.tree.insert(leaf);
                self.set_place(leaf.id, Place::Moving);
            }
        }
        self.fixed.tree.rebuild();
        self.moving.tree.rebuild();
        self.generation = Some(generation);
    }

    /// The end of a `dt` step that moved only the bodies at `moved` (storage indices; `None`
    /// is every body): their leaves return to their bounds, and a leaf that
    /// must be reinserted is also fattened along its velocity for the next [`PREDICTED_STEPS`]
    /// steps, so steadily moving bodies stop being reinserted every step (#375). Without a
    /// current tree this is [`Self::sync`]. Only query candidates change.
    pub fn sync_step(
        &mut self,
        bodies: &[Body],
        moved: Option<&[usize]>,
        generation: u64,
        dt: Real,
    ) {
        if !self.current(generation) {
            crate::work::add(crate::work::Counter::BodyVisits, bodies.len() as u64);
            return self.sync(bodies, generation);
        }
        let count = moved.map_or(bodies.len(), <[usize]>::len);
        crate::work::add(crate::work::Counter::BodyVisits, count as u64);
        let ahead = dt * PREDICTED_STEPS;
        for q in 0..count {
            let body = &bodies[moved.map_or(q, |m| m[q])];
            if body.id != BodyId::NONE && !is_static(body) {
                let leaf = self.leaf(body);
                let d = body.velocity * ahead;
                self.refit(leaf, Some([d.0, d.1, d.2]));
            }
        }
    }

    /// An awake body's leaf holds its broad-phase box `(lo, hi)` (#412, `bookkeeping::BroadBox`)
    /// until [`Self::sync_step`] or a move returns it to the body's bounds. A leaf that must be
    /// reinserted is fattened along the velocity over `dt` as there.
    pub fn swept(&mut self, body: &Body, (lo, hi): (V, V), dt: Real) {
        self.magnitude = self
            .magnitude
            .max(lo.abs().max_component())
            .max(hi.abs().max_component());
        let leaf = Leaf {
            id: body.id.slot(),
            aabb: aabb((lo, hi)),
        };
        let d = body.velocity * (dt * PREDICTED_STEPS);
        self.refit(leaf, Some([d.0, d.1, d.2]));
    }

    /// `body` moved: refit its leaf.
    pub fn moved(&mut self, body: &Body) {
        let leaf = self.leaf(body);
        self.refit(leaf, None);
    }

    /// Visits every leaf whose bounds overlap `query`, fixed bodies first, without allocating:
    /// a query's lookup, which builds the wide tree on first use. Returns the nodes visited.
    pub fn visit_aabb(&self, query: Aabb, mut visit: impl FnMut(Leaf)) -> usize {
        self.wide().visit_aabb(query, &mut visit) + self.moving.tree.visit_aabb(query, visit)
    }

    /// [`Self::visit_aabb`] for the step's broad phase: through the wide tree once a query has
    /// built it, else through the fixed tree, so a world that only steps never builds it.
    pub fn visit_broad_aabb(&self, query: Aabb, mut visit: impl FnMut(Leaf)) -> usize {
        let fixed = match self.wide.get() {
            Some(wide) => wide.visit_aabb(query, &mut visit),
            None => self.fixed.tree.visit_aabb(query, &mut visit),
        };
        fixed + self.moving.tree.visit_aabb(query, visit)
    }

    /// [`Self::visit_aabb`] over the moving bodies only (with any pending fixed ones), for
    /// lookups of sleepers, awake broad-phase boxes and characters: fixed bodies are none of
    /// those.
    pub fn visit_moving_aabb(&self, query: Aabb, visit: impl FnMut(Leaf)) -> usize {
        self.moving.tree.visit_aabb(query, visit)
    }

    /// Visits every leaf whose bounds, padded by `pad`, the segment `origin + t * delta` enters
    /// for `t` in `0..=max_t`, fixed bodies first and nearer entries first within each tree.
    /// `visit(leaf, t)` returns the new `max_t` (negative stops). Builds the wide tree on first
    /// use. Returns the nodes visited.
    pub fn visit_segment(
        &self,
        origin: [f32; 3],
        delta: [f32; 3],
        max_t: f32,
        pad: f32,
        mut visit: impl FnMut(Leaf, f32) -> f32,
    ) -> usize {
        let mut limit = max_t;
        let mut nodes = self
            .wide()
            .visit_segment(origin, delta, max_t, pad, |leaf, t| {
                limit = visit(leaf, t);
                limit
            });
        if limit >= 0.0 && !self.moving.tree.is_empty() {
            nodes += self
                .moving
                .tree
                .visit_segment(origin, delta, limit, pad, visit);
        }
        nodes
    }
}
