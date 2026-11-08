//! Body storage layout: id slots, body order, tombstones and settling.

use super::{Body, BodyId, BodyStorage, Error, FREE_SLOT, Order, World};

impl World {
    /// Every live body, in body order (keyed bodies by key, then keyless ones by creation).
    pub fn bodies(&self) -> impl Iterator<Item = &Body> {
        let indexed = self.scene.body_storage == BodyStorage::Indexed;
        let ordered = (!indexed).then(|| self.bodies.iter().filter(|b| b.id != BodyId::NONE));
        let index = indexed.then(|| {
            self.order_index
                .values()
                .map(|&id| &self.bodies[self.slot_index(id.slot())])
        });
        ordered
            .into_iter()
            .flatten()
            .chain(index.into_iter().flatten())
    }
    /// Number of live bodies.
    pub fn body_count(&self) -> usize {
        self.bodies.len() - self.tombstones
    }
    /// Storage position of the live body in id slot `slot` (query-tree leaves are slots).
    pub(super) fn slot_index(&self, slot: u32) -> usize {
        self.slots[slot as usize].index as usize
    }
    /// Storage positions follow body order (always for `Ordered`; for `Indexed` until a body
    /// is added out of order). Tombstones may remain.
    pub(super) fn storage_sorted(&self) -> bool {
        self.sorted_len == self.bodies.len()
    }
    /// Positions of every live body, in body order.
    pub(super) fn ordered_positions(&self) -> Vec<usize> {
        if self.storage_sorted() {
            (0..self.bodies.len())
                .filter(|&i| self.bodies[i].id != BodyId::NONE)
                .collect()
        } else {
            self.order_index
                .values()
                .map(|&id| self.slot_index(id.slot()))
                .collect()
        }
    }
    /// [`Self::ordered_positions`] for a lookup that wanted the query tree but found it stale:
    /// unreachable through the public API, whose mutators keep the tree current and whose steps
    /// end with a sync (#414). Kept so a future mutator that forgets the tree stays correct; the
    /// `full-scan-bodies` work counter and the debug assertion catch it.
    pub(super) fn stale_tree_scan(&self) -> Vec<usize> {
        debug_assert!(false, "query tree stale outside a step");
        let all = self.ordered_positions();
        crate::work::add(crate::work::Counter::FullScanBodies, all.len() as u64);
        all
    }
    /// Removes tombstones and sorts storage into body order, so positions are body ranks;
    /// O(1) when already settled. The step and island traversals need settled storage.
    pub(super) fn settle_storage(&mut self) {
        if self.tombstones == 0 && self.storage_sorted() {
            return;
        }
        let records = self.bodies.len();
        let sorted_live = self.bodies[..self.sorted_len]
            .iter()
            .filter(|b| b.id != BodyId::NONE)
            .count();
        self.bodies.retain(|b| b.id != BodyId::NONE);
        if sorted_live < self.bodies.len() {
            // Indexed: merge the sorted prefix with the sorted out-of-order tail.
            self.bodies[sorted_live..].sort_unstable_by_key(|b| b.order);
            if sorted_live > 0
                && self.bodies[sorted_live - 1].order > self.bodies[sorted_live].order
            {
                let scratch = &mut self.settle_scratch;
                scratch.clear();
                scratch.reserve(self.bodies.len());
                let (head, tail) = self.bodies.split_at(sorted_live);
                let (mut a, mut b) = (0, 0);
                while a < head.len() && b < tail.len() {
                    if head[a].order < tail[b].order {
                        scratch.push(head[a].clone());
                        a += 1;
                    } else {
                        scratch.push(tail[b].clone());
                        b += 1;
                    }
                }
                scratch.extend_from_slice(&head[a..]);
                scratch.extend_from_slice(&tail[b..]);
                std::mem::swap(&mut self.bodies, scratch);
            }
        }
        crate::work::add(crate::work::Counter::StorageShifts, records as u64);
        self.tombstones = 0;
        self.sorted_len = self.bodies.len();
        for (i, b) in self.bodies.iter().enumerate() {
            self.slots[b.id.slot() as usize].index = i as u32;
        }
        // Positions moved; the bodies and the tree's slot-keyed leaves did not.
        let from = self.bookkeeping.generation;
        self.bookkeeping.layout_changed();
        self.geometry.positions_moved();
        self.query_tree.retag(from, self.bookkeeping.generation);
    }
    /// Island traversals binary-search storage by body order: sort `Indexed` storage first.
    pub(super) fn sort_storage(&mut self) {
        if !self.storage_sorted() {
            self.settle_storage();
        }
    }
    pub fn body(&self, id: BodyId) -> Option<&Body> {
        self.index(id).ok().map(|i| &self.bodies[i])
    }
    /// O(1) slot lookup; a stale generation never matches.
    pub(super) fn index(&self, id: BodyId) -> Result<usize, Error> {
        self.slots
            .get(id.slot() as usize)
            .filter(|s| s.generation == id.generation() && s.index != FREE_SLOT)
            .map(|s| s.index as usize)
            .filter(|&i| self.bodies.get(i).is_some_and(|b| b.id == id))
            // Mid-step layout changes (retirement) refresh slots only at the step's end.
            .or_else(|| self.bodies.iter().position(|b| b.id == id))
            .ok_or(Error::MissingBody(id))
    }
    /// The live body with game key `key`.
    pub fn body_id(&self, key: u64) -> Option<BodyId> {
        self.keys.get(&key).copied()
    }
    /// Recomputes slot indices, free slots and keys from `bodies` after internal layout
    /// changes (retirement, rollback, restore). Generations are kept.
    pub(super) fn reindex_slots(&mut self) {
        for slot in &mut self.slots {
            slot.index = FREE_SLOT;
        }
        self.keys.clear();
        for (i, b) in self.bodies.iter().enumerate() {
            if b.id == BodyId::NONE {
                continue;
            }
            self.slots[b.id.slot() as usize].index = i as u32;
            if let Some(key) = b.key {
                self.keys.insert(key, b.id);
            }
        }
        self.free = (0..self.slots.len() as u32)
            .rev()
            .filter(|&s| self.slots[s as usize].index == FREE_SLOT)
            .collect();
        // Only ever called on settled storage (step-end retirement, rollback, restore).
        debug_assert_eq!(self.tombstones, 0);
        self.sorted_len = self.bodies.len();
        if self.scene.body_storage == BodyStorage::Indexed {
            self.order_index = self.bodies.iter().map(|b| (b.order, b.id)).collect();
        }
    }
    pub(super) fn shift_slots(&mut self, from: usize) {
        for (i, b) in self.bodies.iter().enumerate().skip(from) {
            if b.id != BodyId::NONE {
                self.slots[b.id.slot() as usize].index = i as u32;
            }
        }
    }
    /// Index of the body with internal order `order` (bodies are sorted by it).
    pub(super) fn index_of_order(&self, order: Order) -> Option<usize> {
        self.bodies.binary_search_by_key(&order, |b| b.order).ok()
    }
}
