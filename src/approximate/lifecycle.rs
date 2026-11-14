//! Adding and removing bodies.

use super::{
    Body, BodyId, BodyStorage, Error, FREE_SLOT, Order, Real, Shape, Slot, World, contact,
};

impl World {
    /// Adds a body and returns its new id. `body.key`, when set, must be unique among live
    /// bodies; `body.id` is ignored.
    pub fn add_body(&mut self, body: Body) -> Result<BodyId, Error> {
        if matches!(
            body.shape,
            Shape::Heightfield { .. }
                | Shape::ConvexHull { .. }
                | Shape::TriangleMesh { .. }
                | Shape::Compound(_)
        ) {
            return Err(Error::InvalidInput);
        }
        self.insert_body(body)
    }
    pub(super) fn insert_body(&mut self, mut body: Body) -> Result<BodyId, Error> {
        // A pose read back from a world keeps its exact bits: only a non-unit input normalizes.
        if !body.orientation.unit_to_rounding() {
            body.orientation = body.orientation.normalized();
        }
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
        self.bookkeeping.activity.dynamic += usize::from(body.mass > 0.0);
        self.elevated_bodies += usize::from(body.additional_solver_iterations > 0);
        let synced = self.query_tree.current(self.bookkeeping.generation);
        let (id, key, order) = (body.id, body.key, body.order);
        self.static_changed(&body);
        if body.shape == Shape::HalfSpace {
            self.halfspaces.push(id);
        }
        let len = self.bodies.len();
        let mut shifted = false;
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
                    shifted = i != len;
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
        if shifted {
            self.bookkeeping.layout_changed();
        } else {
            self.body_joined(at);
        }
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
        self.sleepers_within(body, 0.0)
    }
    /// Sleeping bodies within `margin` of `body` (any pose; its `cached_bounds` must match it).
    pub(super) fn sleepers_within(&self, body: &Body, margin: Real) -> Vec<Order> {
        let touches = |b: &Body| {
            b.id != BodyId::NONE
                && b.id != body.id
                && b.sleeping
                && b.layers.collides_with(body.layers)
                && if let Shape::TriangleMesh { mesh, .. } = body.shape {
                    super::triangle_mesh::manifold(
                        self.triangle_mesh(mesh),
                        body,
                        b,
                        margin,
                        None,
                        &mut super::GeometryStats::default(),
                    )
                    .map_or(true, |m| m.is_some())
                } else if body.shape.query_only() {
                    // No closed-form contact here: the bounds overlap is the reach.
                    true
                } else {
                    super::compound::touching(&self.compounds, b, body, margin)
                }
        };
        let mut wake = Vec::new();
        if body.shape == Shape::HalfSpace {
            // Unbounded and outside the tree: every sleeper is in reach of a plane.
            wake.extend(self.bodies.iter().filter(|b| touches(b)).map(|b| b.order));
            wake.sort_unstable();
        } else if self.query_tree.current(self.bookkeeping.generation) {
            let tree = &self.query_tree;
            let (lo, hi) = body.cached_bounds;
            let reach = tree
                .magnitude
                .max(lo.abs().max_component())
                .max(hi.abs().max_component());
            let pad = margin + 64.0 * Real::EPSILON * (1.0 + reach);
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
    /// Position-indexed scratch after the body at `at` was appended or reused a tombstone: no
    /// other body moved, so nothing is rebuilt per body (#523). A new body has no edges.
    fn body_joined(&mut self, at: usize) {
        let s = &mut self.bookkeeping;
        s.generation += 1;
        let b = &self.bodies[at];
        if b.mass > 0.0 && !b.sleeping {
            s.activity.woke(at, &mut s.work);
        }
        s.position.body_added(at, b);
        // A reused tombstone already holds its resting values (`body_left`).
        if s.per_body == Some(s.layout) && at == self.responses.len() {
            self.responses.push(super::PreparedResponse::default());
            s.supported.push(true);
            s.earliest.push(Real::INFINITY);
        }
    }
    /// Position-indexed scratch after the live body at `i` became a tombstone.
    fn body_left(&mut self, i: usize, awake: bool) {
        let s = &mut self.bookkeeping;
        s.generation += 1;
        if awake {
            s.activity.left(i);
        }
        s.position.body_removed(i);
        if s.per_body == Some(s.layout) {
            if let Some(r) = self.responses.get_mut(i) {
                *r = super::PreparedResponse::default();
            }
            if let Some(x) = s.supported.get_mut(i) {
                *x = true;
            }
            if let Some(x) = s.earliest.get_mut(i) {
                *x = Real::INFINITY;
            }
        }
    }
    /// Pops trailing tombstones, O(1) each: no live body moves, so the next step finds settled
    /// storage and keeps its position-indexed scratch (#523).
    fn truncate_tombstones(&mut self) {
        let before = self.bodies.len();
        while self.bodies.last().is_some_and(|b| b.id == BodyId::NONE) {
            self.bodies.pop();
            self.tombstones -= 1;
        }
        let len = self.bodies.len();
        if len != before {
            self.sorted_len = self.sorted_len.min(len);
            self.responses.truncate(len);
            self.bookkeeping.supported.truncate(len);
            self.bookkeeping.earliest.truncate(len);
        }
    }
    /// Compacts once tombstones are half the records: amortized O(1) per removal.
    fn compact_if_sparse(&mut self) {
        if 2 * self.tombstones > self.bodies.len() {
            self.settle_storage();
        }
    }
    /// Takes the body `id` at `i` out of storage. Its contacts and joints are already gone.
    fn release_body(&mut self, id: BodyId, i: usize) -> Body {
        self.characters.remove(id);
        // Sorted by id bits, one target per body.
        if let Ok(at) = self
            .kinematic_poses
            .binary_search_by_key(&id.to_bits(), |t| t.0)
        {
            self.kinematic_poses.remove(at);
        }
        let synced = self.query_tree.current(self.bookkeeping.generation);
        let removed = self.bodies[i].clone();
        self.static_changed(&removed);
        self.elevated_bodies -= usize::from(removed.additional_solver_iterations > 0);
        match removed.shape {
            Shape::HalfSpace => self.halfspaces.retain(|&h| h != id),
            Shape::TriangleMesh { mesh, .. } => self.triangle_meshes[mesh.0 as usize] = None,
            Shape::ConvexHull { hull, .. } => self.hulls[hull.0 as usize] = None,
            _ => {}
        }
        self.bodies[i].id = BodyId::NONE;
        self.tombstones += 1;
        if self.scene.body_storage == BodyStorage::Indexed {
            self.order_index.remove(&removed.order);
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
        let awake = removed.mass > 0.0 && !removed.sleeping;
        self.bookkeeping.activity.count -= usize::from(awake);
        self.bookkeeping.activity.dynamic -= usize::from(removed.mass > 0.0);
        self.body_left(i, awake);
        if synced {
            self.query_tree.removed(slot, self.bookkeeping.generation);
        } else {
            self.query_tree
                .sync(&self.bodies, self.bookkeeping.generation);
            self.bookkeeping.forget_boxes();
        }
        removed
    }
    /// Removes a body. Bodies in contact with it, or joined to it, wake with their islands, and
    /// its contacts report `Stopped` at the next successful step. O(contact pairs + joints);
    /// removing the last records of storage leaves the next step nothing to compact.
    pub fn remove_body(&mut self, id: BodyId) -> Option<Body> {
        let order = self.bodies[self.index(id).ok()?].order;
        // Its joints go with it; the bodies they held wake (#453).
        self.remove_body_joints(order);
        // Bodies in contact with the removed one wake (its contact pairs, in pair order).
        let neighbors = self
            .cache
            .keys()
            .chain(self.parked.keys())
            .filter_map(|&(a, b)| {
                crate::work::add(crate::work::Counter::CachedPairVisits, 1);
                (a == order)
                    .then_some(b)
                    .or_else(|| (b == order).then_some(a))
            })
            .collect::<Vec<_>>();
        // Who wakes without contact history (`warm_start` off): what a rebuilt world sees too.
        let reach = if self.scene.warm_start || self.bodies[self.index(id).ok()?].sensor {
            Vec::new()
        } else {
            self.sleepers_within(
                &self.bodies[self.index(id).ok()?],
                self.scene.contact_slop * 2.0,
            )
        };
        if !neighbors.is_empty() || !reach.is_empty() {
            self.bookkeeping.graph.dirty = true;
            self.sort_storage();
            for n in reach {
                self.wake_island(n);
            }
            for n in neighbors {
                // Its contacts stop; the next successful step reports them.
                let other = self.bodies[self.index_of_order(n).expect("live contact")].id;
                let (key, ids) = if order < n {
                    ((order, n), (id, other))
                } else {
                    ((n, order), (other, id))
                };
                self.contact_changes.push(super::ContactChange {
                    key,
                    ids,
                    started: false,
                    held: false,
                });
                self.removed_contacts = self.contact_changes.len();
                if self.scene.warm_start {
                    self.wake_island(n);
                }
            }
        }
        // O(contact pairs), independent of the body count.
        self.geometry.remove(order);
        let pool = &mut self.bookkeeping;
        let mut discard = |&(a, b): &(Order, Order), points: &mut Vec<_>| {
            crate::work::add(crate::work::Counter::CachedPairVisits, 1);
            let keep = a != order && b != order;
            if !keep {
                pool.recycle_points(std::mem::take(points));
            }
            keep
        };
        self.cache.retain(|key, points| discard(key, points));
        self.parked.retain(|key, points| discard(key, points));
        let i = self.index(id).ok()?;
        let removed = self.release_body(id, i);
        self.truncate_tombstones();
        self.compact_if_sparse();
        Some(removed)
    }
    /// Removes every body of `ids` at once: all of them, or none when one is missing
    /// ([`Error::MissingBody`]) or repeated ([`Error::InvalidInput`]), in which case the world is
    /// unchanged. The bodies come back in the order of `ids`.
    ///
    /// The surviving world and the contact events are those of removing the bodies one by one
    /// in body order, for any order of `ids`: the survivors in contact with or joined to a
    /// removed body wake with their islands, and each cached pair stops once. The bodies of
    /// the batch are not woken, so the returned records keep their `sleeping` state.
    ///
    /// One pass over the contact pairs and joints and one island traversal: O(k log k + P + J),
    /// independent of bodies not in contact.
    pub fn remove_bodies(&mut self, ids: &[BodyId]) -> Result<Vec<Body>, Error> {
        let mut batch = Vec::with_capacity(ids.len());
        for (at, &id) in ids.iter().enumerate() {
            batch.push((self.bodies[self.index(id)?].order, at, id));
        }
        batch.sort_unstable_by_key(|&(order, ..)| order);
        if batch.windows(2).any(|w| w[0].0 == w[1].0) {
            return Err(Error::InvalidInput);
        }
        let orders = batch.iter().map(|&(order, ..)| order).collect::<Vec<_>>();
        // The surviving neighbours of the removed bodies are the roots of everything that wakes:
        // an island reached through a removed body is also reached from a survivor beside it.
        let mut roots = self.remove_batch_joints(&orders);
        // Without contact history (`warm_start` off) the geometry names the sleepers in reach.
        if !self.scene.warm_start {
            for &(_, _, id) in &batch {
                let body = &self.bodies[self.index(id)?];
                if !body.sensor {
                    let near = self.sleepers_within(body, self.scene.contact_slop * 2.0);
                    roots.extend(
                        near.into_iter()
                            .filter(|o| orders.binary_search(o).is_err()),
                    );
                }
            }
        }
        let warm_start = self.scene.warm_start;
        if !self.cache.is_empty() || !self.parked.is_empty() {
            self.sort_storage();
            let (cache, parked, bodies, changes) = (
                &mut self.cache,
                &mut self.parked,
                &self.bodies,
                &mut self.contact_changes,
            );
            let id_of = |order: Order| match orders.binary_search(&order) {
                Ok(at) => batch[at].2,
                Err(_) => {
                    bodies[bodies
                        .binary_search_by_key(&order, |b| b.order)
                        .expect("live contact")]
                    .id
                }
            };
            let pool = &mut self.bookkeeping;
            let mut discard = |&(a, b): &(Order, Order), points: &mut Vec<_>| {
                crate::work::add(crate::work::Counter::CachedPairVisits, 1);
                let (gone_a, gone_b) = (
                    orders.binary_search(&a).is_ok(),
                    orders.binary_search(&b).is_ok(),
                );
                if !(gone_a || gone_b) {
                    return true;
                }
                if warm_start && !gone_a {
                    roots.push(a);
                }
                if warm_start && !gone_b {
                    roots.push(b);
                }
                // A pair's changes stay ordered by pair: this order needs no sort by removal.
                changes.push(super::ContactChange {
                    key: (a, b),
                    ids: (id_of(a), id_of(b)),
                    started: false,
                    held: false,
                });
                pool.recycle_points(std::mem::take(points));
                false
            };
            cache.retain(|key, points| discard(key, points));
            parked.retain(|key, points| discard(key, points));
            if !roots.is_empty() {
                self.bookkeeping.graph.dirty = true;
            }
            self.removed_contacts = self.contact_changes.len();
        }
        self.geometry.remove_many(&orders);
        let mut removed = Vec::with_capacity(batch.len());
        for &(_, at, id) in &batch {
            let i = self.index(id).expect("validated above");
            removed.push((at, self.release_body(id, i)));
        }
        self.truncate_tombstones();
        roots.sort_unstable();
        roots.dedup();
        if !roots.is_empty() {
            self.sort_storage();
            self.wake_islands(&roots);
        }
        self.compact_if_sparse();
        removed.sort_unstable_by_key(|&(at, _)| at);
        Ok(removed.into_iter().map(|(_, b)| b).collect())
    }
}
