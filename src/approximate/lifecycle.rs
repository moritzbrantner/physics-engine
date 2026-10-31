//! Adding and removing bodies.

use super::{
    Body, BodyId, BodyStorage, Error, FREE_SLOT, Order, Real, Shape, Slot, World, contact,
};

impl World {
    /// Adds a body and returns its new id. `body.key`, when set, must be unique among live
    /// bodies; `body.id` is ignored.
    pub fn add_body(&mut self, body: Body) -> Result<BodyId, Error> {
        if matches!(body.shape, Shape::Heightfield { .. }) {
            return Err(Error::InvalidInput);
        }
        self.insert_body(body)
    }
    pub(super) fn insert_body(&mut self, mut body: Body) -> Result<BodyId, Error> {
        body.orientation = body.orientation.normalized();
        if !body.valid() || self.slots.len() >= u32::MAX as usize - 1 {
            return Err(Error::InvalidInput);
        }
        if let Some(key) = body.key
            && self.keys.contains_key(&key)
        {
            return Err(Error::DuplicateKey(key));
        }
        // Reuse the lowest free slot with a new generation, else a fresh slot.
        body.id = match self.free.pop() {
            Some(slot) => {
                let s = &mut self.slots[slot as usize];
                s.generation = s.generation.wrapping_add(1);
                BodyId::new(slot, s.generation)
            }
            None => {
                self.slots.push(Slot {
                    generation: 0,
                    index: FREE_SLOT,
                });
                BodyId::new(self.slots.len() as u32 - 1, 0)
            }
        };
        body.order = Order::new(body.key, self.next_serial);
        self.next_serial += 1;
        body.cached_bounds = contact::bounds(&body);
        // Local topology change: only a genuine new fixed overlap can disturb a sleeper.
        if body.mass == 0.0 && !body.sensor && !body.shape.query_only() {
            let wake = self.sleepers_touching(&body);
            if !wake.is_empty() {
                self.sort_storage();
                for order in wake {
                    self.wake_island(order);
                }
            }
        }
        self.bookkeeping.activity.count += usize::from(body.mass > 0.0 && !body.sleeping);
        let synced = self.query_tree.current(self.bookkeeping.generation);
        let (id, key, order) = (body.id, body.key, body.order);
        if body.shape == Shape::HalfSpace {
            self.halfspaces.push(id);
        }
        let len = self.bodies.len();
        let at = match self.scene.body_storage {
            BodyStorage::Ordered => {
                let i = self.bodies.partition_point(|b| b.order < order);
                // A tombstone next to the insertion point keeps the order sorted when reused.
                let reuse = [i, i.wrapping_sub(1)]
                    .into_iter()
                    .find(|&j| j < len && self.bodies[j].id == BodyId::NONE);
                if let Some(j) = reuse {
                    self.bodies[j] = body;
                    self.tombstones -= 1;
                    j
                } else {
                    self.bodies.insert(i, body);
                    crate::work::add(crate::work::Counter::StorageShifts, (len - i) as u64);
                    self.shift_slots(i + 1);
                    self.sorted_len += 1;
                    i
                }
            }
            BodyStorage::Indexed => {
                // While storage is sorted, the records between the live neighbours in body order
                // are tombstones: reusing one keeps it sorted (re-adding a removed door, say).
                let gap = self.storage_sorted().then(|| {
                    let position = |(_, id): (&Order, &BodyId)| self.slot_index(id.slot());
                    let lo = self
                        .order_index
                        .range(..order)
                        .next_back()
                        .map_or(0, |p| position(p) + 1);
                    let hi = self.order_index.range(order..).next().map_or(len, position);
                    (lo, hi)
                });
                self.order_index.insert(order, id);
                match gap {
                    Some((lo, hi)) if lo < hi => {
                        self.bodies[lo] = body;
                        self.tombstones -= 1;
                        lo
                    }
                    _ => {
                        // Appended: still sorted only after every live body and tombstone.
                        let in_order = gap.is_some()
                            && self.bodies.last().is_none_or(|last| last.order < order);
                        self.bodies.push(body);
                        self.sorted_len += usize::from(in_order);
                        len
                    }
                }
            }
        };
        self.slots[id.slot() as usize].index = at as u32;
        if let Some(key) = key {
            self.keys.insert(key, id);
        }
        self.bookkeeping.layout_changed();
        if synced {
            self.query_tree
                .inserted(&self.bodies[at], self.bookkeeping.generation);
        } else {
            self.query_tree
                .sync(&self.bodies, self.bookkeeping.generation);
            self.bookkeeping.forget_boxes();
        }
        // The slot's previous body's box is gone; boxes near a new resting body lack its leaf
        // (an awake one makes its own box). The broad phase sizes `broad` for new slots.
        self.bookkeeping.drop_box(id.slot());
        let b = &self.bodies[at];
        if !(b.mass > 0.0 && !b.sleeping) {
            let bounds = b.cached_bounds;
            self.resting_changed(at, bounds);
        }
        Ok(id)
    }
    /// Orders of sleeping bodies that `body` (not yet stored) touches, in body order.
    pub(super) fn sleepers_touching(&self, body: &Body) -> Vec<Order> {
        let touches = |b: &Body| {
            b.id != BodyId::NONE
                && b.sleeping
                && b.layers.collides_with(body.layers)
                && contact::current(b, body, 0.0).is_some()
        };
        let mut wake = Vec::new();
        if self.query_tree.current(self.bookkeeping.generation) {
            let tree = &self.query_tree;
            let (lo, hi) = body.cached_bounds;
            let reach = tree
                .magnitude
                .max(lo.abs().max_component())
                .max(hi.abs().max_component());
            let pad = 64.0 * Real::EPSILON * (1.0 + reach);
            tree.visit_moving_aabb(
                spatial_kernels::Aabb {
                    min: [lo.0 - pad, lo.1 - pad, lo.2 - pad],
                    max: [hi.0 + pad, hi.1 + pad, hi.2 + pad],
                },
                |leaf| {
                    let b = &self.bodies[self.slot_index(leaf.id)];
                    if touches(b) {
                        wake.push(b.order);
                    }
                },
            );
            wake.sort_unstable();
        } else {
            for i in self.stale_tree_scan() {
                if touches(&self.bodies[i]) {
                    wake.push(self.bodies[i].order);
                }
            }
        }
        wake
    }
    pub fn remove_body(&mut self, id: BodyId) -> Option<Body> {
        let order = self.bodies[self.index(id).ok()?].order;
        self.characters.remove(id);
        self.halfspaces.retain(|&h| h != id);
        // Bodies in contact with the removed one wake (its contact pairs, in pair order).
        let neighbors = self
            .cache
            .keys()
            .filter_map(|&(a, b)| {
                (a == order)
                    .then_some(b)
                    .or_else(|| (b == order).then_some(a))
            })
            .collect::<Vec<_>>();
        if !neighbors.is_empty() {
            self.sort_storage();
            for n in neighbors {
                self.wake_island(n);
            }
        }
        // O(contact pairs), independent of the body count.
        self.geometry.remove(order);
        self.cache.retain(|&(a, b), _| a != order && b != order);
        let i = self.index(id).ok()?;
        let synced = self.query_tree.current(self.bookkeeping.generation);
        let removed = self.bodies[i].clone();
        self.bodies[i].id = BodyId::NONE;
        self.tombstones += 1;
        if self.scene.body_storage == BodyStorage::Indexed {
            self.order_index.remove(&order);
        }
        let slot = id.slot();
        self.slots[slot as usize].index = FREE_SLOT;
        self.bookkeeping.drop_box(slot);
        // Descending, so `pop` reuses the lowest free slot.
        let at = self.free.partition_point(|&f| f > slot);
        self.free.insert(at, slot);
        if let Some(key) = removed.key {
            self.keys.remove(&key);
        }
        self.bookkeeping.activity.count -= usize::from(removed.mass > 0.0 && !removed.sleeping);
        self.bookkeeping.layout_changed();
        if synced {
            self.query_tree.removed(slot, self.bookkeeping.generation);
        } else {
            self.query_tree
                .sync(&self.bodies, self.bookkeeping.generation);
            self.bookkeeping.forget_boxes();
        }
        // Tombstones are compacted by the next step, or here once they are half the records:
        // amortized O(1) per removal.
        if 2 * self.tombstones > self.bodies.len() {
            self.settle_storage();
        }
        Some(removed)
    }
}
