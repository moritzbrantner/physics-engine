//! Query acceleration over `rust-kernels`' AABB trees. A body's leaf ID is its id slot, so
//! storage layout changes (inserts, tombstones, compaction) never touch the trees; the world's
//! slot table maps a leaf back to the body. The trees are kept current by the public mutators
//! and at the end of every step, so queries always find them current; a stale tree would fall
//! back to a full scan (debug-asserted and counted as `full-scan-bodies`, budget 0, #414), so
//! the trees never change a query result, only how many bodies are tested.
//!
//! Fixed bodies (level geometry, most of a game's bodies) live apart from moving ones (#415):
//! as a list of exact leaves and a four-wide [`WideBvh`] over them that queries and the broad
//! phase descend (#415). That O(n log n) build waits for its first use, the first ray or shape
//! query or the first step with an awake body, and setup only pushes leaves (#528). Tree shape
//! never changes a result: broad-phase pairs are sorted and queries order ties by body. A
//! fixed body added after that build waits in the moving tree, and a removed one is dropped
//! from the wide tree in place, until enough such changes accumulate to drop the wide tree;
//! the next use rebuilds it. Only a waiting body that has stayed for [`SETTLED_STEPS`] step
//! ends counts as such a change and joins the rebuilt tree, so a lane that adds fixed pieces
//! for one step and removes them after it never rebuilds the wide tree. A fixed body that
//! moves becomes a moving one.

use super::super::{Body, BodyId, Real, Vector as V};
use bvh_kernels::{DynamicAabbTree, WideBvh};
use spatial_kernels::{Aabb, Body as Leaf};
use std::sync::OnceLock;

/// Fat margin of the moving tree's leaves, per `length_unit` (a tenth of a metre in the default
/// metre world): small moves update a leaf in place at every world scale.
const FAT_MARGIN: f32 = 0.1;
/// Steps of motion a reinserted moving leaf is fattened for.
const PREDICTED_STEPS: Real = 4.0;
/// Step ends a fixed body added after the wide tree's build must stay before it counts towards
/// a rebuild and joins the rebuilt tree. A transient lane's resting piece, added before one
/// step and removed after it (#523), sees one step end, so it never does.
const SETTLED_STEPS: u64 = 2;

/// Where a body's leaf lives.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Place {
    Absent,
    /// A moving body in the moving tree.
    Moving,
    /// A fixed body at this index of `fixed_leaves`, in the wide tree when there is one.
    Fixed(u32),
    /// A fixed body (index into `fixed_leaves`, then into `pending`) added after the wide
    /// tree's build: in the moving tree until it has settled and the wide tree is dropped.
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

    /// Inserts `leaf`; rebuilds once inserts reach half the tree: amortized O(log n). A small
    /// tree is not worth rebuilding: a batch of transient bodies grows from empty every tick
    /// (#523).
    fn insert(&mut self, leaf: Leaf) {
        self.tree.insert(leaf);
        crate::work::add(crate::work::Counter::TreeInserts, 1);
        self.inserts += 1;
        if self.inserts >= 64.max(self.tree.len() / 2) {
            crate::work::add(crate::work::Counter::TreeInserts, self.tree.len() as u64);
            crate::work::add(crate::work::Counter::TreeBuilds, 1);
            self.tree.rebuild();
            self.inserts = 0;
        }
    }
}

#[derive(Debug)]
pub(in crate::approximate) struct QueryTree {
    /// Moving bodies and pending fixed ones.
    moving: Tree,
    /// Exact leaves of every fixed body: what the wide tree is built from.
    fixed_leaves: Vec<Leaf>,
    /// Four-wide tree over the `Fixed` leaves of `fixed_leaves`, built by the first query or
    /// broad phase that needs it.
    wide: OnceLock<WideBvh>,
    /// Whether a wide tree was built and then dropped: fixed bodies added before its rebuild
    /// are pending too, so the rebuild does not take in pieces added for one step.
    dropped: bool,
    /// Slots of the pending fixed bodies and the `steps` value when each was added.
    pending: Vec<(u32, u64)>,
    /// Step ends seen, the clock of `pending`.
    steps: u64,
    /// Pending bodies that have settled (seen [`SETTLED_STEPS`] step ends), as of the last
    /// step end, minus those removed since, plus leaves dropped from the wide tree in place.
    stale: usize,
    /// Leaf place per id slot.
    places: Vec<Place>,
    /// Layout generation the trees match; `None` before the first sync.
    generation: Option<u64>,
    /// Upper bound of every leaf coordinate's magnitude, for conservative padding.
    pub magnitude: Real,
}

impl Clone for QueryTree {
    /// Builds the wide tree in the source first, so clones of a statics template all start with
    /// it instead of each building their own.
    fn clone(&self) -> Self {
        if !self.fixed_leaves.is_empty() {
            self.wide();
        }
        Self {
            moving: self.moving.clone(),
            fixed_leaves: self.fixed_leaves.clone(),
            wide: self.wide.clone(),
            dropped: self.dropped,
            pending: self.pending.clone(),
            steps: self.steps,
            stale: self.stale,
            places: self.places.clone(),
            generation: self.generation,
            magnitude: self.magnitude,
        }
    }
}

impl QueryTree {
    /// An empty tree for a world of `length_unit` scene units per metre.
    pub fn new(length_unit: Real) -> Self {
        Self::with_margin(FAT_MARGIN * length_unit)
    }

    fn with_margin(fat_margin: f32) -> Self {
        Self {
            moving: Tree::new(fat_margin),
            fixed_leaves: Vec::new(),
            wide: OnceLock::new(),
            dropped: false,
            pending: Vec::new(),
            steps: 0,
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

    /// The wide tree, built here on first use over the fixed leaves that are not pending.
    fn wide(&self) -> &WideBvh {
        self.wide.get_or_init(|| {
            let built = self.fixed_leaves.len() - self.pending.len();
            crate::work::add(crate::work::Counter::TreeInserts, built as u64);
            crate::work::add(crate::work::Counter::WideTreeLeaves, built as u64);
            crate::work::add(crate::work::Counter::TreeBuilds, 1);
            if self.pending.is_empty() {
                WideBvh::build(&self.fixed_leaves)
            } else {
                let leaves: Vec<Leaf> = self
                    .fixed_leaves
                    .iter()
                    .filter(|l| matches!(self.place(l.id), Place::Fixed(_)))
                    .copied()
                    .collect();
                WideBvh::build(&leaves)
            }
        })
    }

    fn settled(&self, added: u64) -> bool {
        self.steps - added >= SETTLED_STEPS
    }

    fn insert_fixed(&mut self, leaf: Leaf) -> u32 {
        let index = self.fixed_leaves.len() as u32;
        self.fixed_leaves.push(leaf);
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
                let (_, added) = self.pending.swap_remove(index as usize);
                if let Some(&(other, _)) = self.pending.get(index as usize)
                    && let Place::Pending(l, _) = self.place(other)
                {
                    self.set_place(other, Place::Pending(l, index));
                }
                // Counted at the last step end only if it had settled by then.
                if self.settled(added) {
                    self.stale -= 1;
                }
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
            self.fixed_leaves.swap_remove(leaf as usize);
            if let Some(other) = self.fixed_leaves.get(leaf as usize).map(|l| l.id) {
                let place = match self.place(other) {
                    Place::Pending(_, index) => Place::Pending(leaf, index),
                    _ => Place::Fixed(leaf),
                };
                self.set_place(other, place);
            }
        }
        // An empty tree has no hierarchy left to degrade, whichever leaf left it last.
        if self.moving.tree.is_empty() {
            self.moving.inserts = 0;
        }
        self.set_place(slot, Place::Absent);
        self.drop_stale_wide();
    }

    /// Once settled pending bodies and dropped leaves reach 1/16 of the fixed set, takes the
    /// settled pending bodies out of the moving tree and drops the wide tree; the next query
    /// rebuilds it with them, so the rebuild cost is amortized over lasting changes. Pending
    /// bodies that have not settled stay in the moving tree, unless they number half the other
    /// fixed bodies: then all join, so the moving tree stays bounded in a world that never
    /// steps.
    fn drop_stale_wide(&mut self) {
        let others = self.fixed_leaves.len() - self.pending.len();
        let all = self.pending.len() >= 8.max(others / 2);
        if self.stale < 8.max(self.fixed_leaves.len() / 16) && !all {
            return;
        }
        let mut kept = 0;
        for n in 0..self.pending.len() {
            let (slot, added) = self.pending[n];
            let Place::Pending(leaf, _) = self.place(slot) else {
                continue;
            };
            if all || self.settled(added) {
                self.moving.tree.remove(slot);
                self.set_place(slot, Place::Fixed(leaf));
            } else {
                self.pending[kept] = (slot, added);
                self.set_place(slot, Place::Pending(leaf, kept as u32));
                kept += 1;
            }
        }
        self.pending.truncate(kept);
        self.stale = 0;
        self.wide = OnceLock::new();
        self.dropped = true;
    }

    /// A step ended: pending bodies age, and those that settle now count towards a rebuild.
    /// O(pending).
    pub fn step_ended(&mut self) {
        self.steps += 1;
        let steps = self.steps;
        self.stale += self
            .pending
            .iter()
            .filter(|&&(_, added)| steps - added == SETTLED_STEPS)
            .count();
        self.drop_stale_wide();
    }

    /// `body` was just added.
    pub fn inserted(&mut self, body: &Body, generation: u64) {
        let leaf = self.leaf(body);
        if !is_static(body) {
            self.set_place(leaf.id, Place::Moving);
            self.moving.insert(leaf);
        } else {
            let index = self.insert_fixed(leaf);
            if self.dropped || self.wide.get().is_some() {
                self.set_place(leaf.id, Place::Pending(index, self.pending.len() as u32));
                self.pending.push((leaf.id, self.steps));
                // Pending leaves leave when the wide tree is dropped, so they never degrade
                // the moving tree for long and do not count towards its rebuild.
                self.moving.tree.insert(leaf);
                crate::work::add(crate::work::Counter::TreeInserts, 1);
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
        *self = Self::with_margin(self.moving.tree.fat_margin());
        for body in live {
            let leaf = self.leaf(body);
            if is_static(body) {
                let index = self.fixed_leaves.len() as u32;
                self.fixed_leaves.push(leaf);
                self.set_place(leaf.id, Place::Fixed(index));
            } else {
                self.moving.tree.insert(leaf);
                self.set_place(leaf.id, Place::Moving);
            }
        }
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
    /// a query's or broad phase's lookup, which builds the wide tree on first use. Returns the
    /// nodes visited.
    pub fn visit_aabb(&self, query: Aabb, mut visit: impl FnMut(Leaf)) -> usize {
        let fixed = if self.fixed_leaves.is_empty() {
            0
        } else {
            self.wide().visit_aabb(query, &mut visit)
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fat_margin_scales_with_length_unit_and_survives_a_rebuild() {
        let mut tree = QueryTree::new(100.0);
        assert_eq!(tree.moving.tree.fat_margin(), FAT_MARGIN * 100.0);
        // Generation 7 is not current, so this is the full rebuild.
        tree.sync(&[], 7);
        assert_eq!(tree.moving.tree.fat_margin(), FAT_MARGIN * 100.0);
    }

    /// A town of 128 fixed boxes whose wide tree a ray query has built.
    fn town() -> super::super::super::World {
        use super::super::super::{Body, Config, QueryFilter, RayCast, Shape, World};
        let mut world = World::new(Config::default()).unwrap();
        for k in 0..128 {
            let p = V(k as Real * 3.0, 0.0, 0.0);
            world
                .add_body(Body::keyed(k, Shape::Box(V(1.0, 1.0, 1.0)), p, 0.0))
                .unwrap();
        }
        let cast = RayCast::new(V(-10.0, 0.0, 0.0), V(1000.0, 0.0, 0.0));
        world.cast_ray_first(cast, QueryFilter::default()).unwrap();
        assert!(world.query_tree.wide.get().is_some());
        world
    }

    fn pieces(world: &mut super::super::super::World, count: u64) -> Vec<BodyId> {
        use super::super::super::{Body, Shape};
        (0..count)
            .map(|k| {
                let p = V(k as Real * 3.0, 5.0, 0.0);
                world
                    .add_body(Body::new(Shape::Box(V(0.2, 0.2, 0.2)), p, 0.0))
                    .unwrap()
            })
            .collect()
    }

    #[test]
    fn fixed_bodies_added_for_one_step_never_rebuild_the_wide_tree() {
        let mut world = town();
        for _ in 0..8 {
            // Far more than the 1/16 of the town that drops the wide tree once settled.
            let ids = pieces(&mut world, 32);
            world.step(1.0 / 60.0).unwrap();
            world.remove_bodies(&ids).unwrap();
            let tree = &world.query_tree;
            assert!(tree.wide.get().is_some() && !tree.dropped);
            assert!(tree.pending.is_empty() && tree.stale == 0);
        }
    }

    #[test]
    fn fixed_bodies_that_stay_join_the_rebuilt_wide_tree() {
        let mut world = town();
        let ids = pieces(&mut world, 8);
        world.step(1.0 / 60.0).unwrap();
        assert_eq!(world.query_tree.pending.len(), 8);
        // The second step end settles them: 8 is the smallest drop threshold.
        world.step(1.0 / 60.0).unwrap();
        let tree = &world.query_tree;
        assert!(tree.wide.get().is_none() && tree.pending.is_empty());
        assert!(
            ids.iter()
                .all(|id| matches!(tree.place(id.slot()), Place::Fixed(_)))
        );
        // The next query rebuilds the wide tree with them.
        assert!(tree.wide().contains(ids[0].slot()));
    }

    #[test]
    fn half_the_fixed_set_pending_joins_without_a_step() {
        let mut world = town();
        pieces(&mut world, 63);
        assert_eq!(world.query_tree.pending.len(), 63);
        pieces(&mut world, 1);
        let tree = &world.query_tree;
        assert!(tree.pending.is_empty() && tree.wide.get().is_none());
    }
}
