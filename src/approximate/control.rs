//! Body placement, velocity, impulses and forces, and the island wake they trigger.

use super::{BodyId, Error, Order, Real, Shape, Vector, World, bookkeeping, contact, query};
use crate::CollisionLayers3d;

impl World {
    /// Places an external (kinematic) body at `position` without resolving contact, as Rapier's
    /// `set_next_kinematic_translation`. Sleeping bodies it was in contact with (within twice
    /// the contact slop at its old pose) or now touches wake, so a crate left behind falls and
    /// its contact stops (#456): two descents of the query tree, no contact-cache scan.
    pub fn set_kinematic_position(&mut self, id: BodyId, position: Vector) -> Result<(), Error> {
        // Writing back the current position changes nothing a character relies on.
        let unchanged = self.body(id).is_some_and(|b| b.position == position);
        let solid = self
            .body(id)
            .is_some_and(|b| !b.sensor && !b.shape.query_only());
        let mut wake = if solid && !unchanged {
            let i = self.index(id)?;
            self.sleepers_within(&self.bodies[i], self.scene.contact_slop * 2.0)
        } else {
            Vec::new()
        };
        self.place_kinematic(id, position)?;
        if !unchanged {
            if solid {
                let i = self.index(id)?;
                wake.extend(self.sleepers_touching(&self.bodies[i]));
            }
            self.wake_orders(wake);
            self.body_placed(id);
        }
        Ok(())
    }
    /// Wakes the islands of `wake` (body orders), once each.
    fn wake_orders(&mut self, mut wake: Vec<Order>) {
        if wake.is_empty() {
            return;
        }
        wake.sort_unstable();
        wake.dedup();
        self.sort_storage();
        for order in wake {
            self.wake_island(order);
        }
    }
    /// Moves a static body (mass 0, not external) to `position` in place, as Rapier moves a
    /// fixed collider: the layout and every other body's caches stay valid. Sleeping bodies
    /// it was in contact with or now touches wake; characters it now reaches check for
    /// overlap again. Halfspaces and heightfields stay where they were added.
    pub fn set_static_position(&mut self, id: BodyId, position: Vector) -> Result<(), Error> {
        let i = self.index(id)?;
        let body = &self.bodies[i];
        if body.mass != 0.0
            || body.external
            || matches!(body.shape, Shape::HalfSpace | Shape::Heightfield { .. })
            || !position.finite()
            || position.abs().max_component() >= 1e12
        {
            return Err(Error::InvalidInput);
        }
        if body.position == position {
            return Ok(());
        }
        let order = body.order;
        let mut wake = self
            .cache
            .keys()
            .filter_map(|&(a, b)| {
                crate::work::add(crate::work::Counter::CachedPairVisits, 1);
                (a == order)
                    .then_some(b)
                    .or_else(|| (b == order).then_some(a))
            })
            .collect::<Vec<_>>();
        let body = &mut self.bodies[i];
        let before = body.cached_bounds;
        body.position = position;
        body.cached_bounds = contact::bounds(body);
        self.bookkeeping.poses_changed();
        if self.query_tree.current(self.bookkeeping.generation) {
            self.query_tree.moved(&self.bodies[i]);
        }
        self.resting_changed(i, before);
        if !self.bodies[i].sensor && !self.bodies[i].shape.query_only() {
            wake.extend(self.sleepers_touching(&self.bodies[i]));
        }
        self.wake_orders(wake);
        self.body_placed(id);
        Ok(())
    }
    /// Replaces a body's collision layers, as Rapier's `Collider::set_collision_groups`; the
    /// solver, CCD, queries and character moves read the new layers from the next call. Its
    /// cached contact pairs the new layers exclude stop in the next step (`ContactEvent::Stopped`):
    /// the sleeping side of each wakes, as does the body itself when it sleeps. Sleeping bodies it
    /// now touches under the new layers wake; characters near it check for overlap again. One
    /// contact-cache scan and one query-tree descent; no other cache depends on layers.
    pub fn set_layers(&mut self, id: BodyId, layers: CollisionLayers3d) -> Result<(), Error> {
        if self.bodies[self.index(id)?].layers == layers {
            return Ok(());
        }
        // Cached pairs name bodies by order: look them up in sorted storage.
        self.sort_storage();
        let i = self.index(id)?;
        let body = &self.bodies[i];
        let (order, solid) = (body.order, !body.sensor && !body.shape.query_only());
        let sleeper = |b: &super::Body| b.movable() && b.sleeping;
        let mut wake = Vec::new();
        let mut excluded = false;
        for &(a, b) in self.cache.keys() {
            crate::work::add(crate::work::Counter::CachedPairVisits, 1);
            let other = match (a == order, b == order) {
                (true, _) => b,
                (_, true) => a,
                _ => continue,
            };
            let j = self.index_of_order(other).expect("live contact");
            if !layers.collides_with(self.bodies[j].layers) {
                excluded = true;
                if sleeper(&self.bodies[j]) {
                    wake.push(other);
                }
            }
        }
        if excluded && sleeper(&self.bodies[i]) {
            wake.push(order);
        }
        self.bodies[i].layers = layers;
        if solid {
            wake.extend(self.sleepers_touching(&self.bodies[i]));
        }
        self.wake_orders(wake);
        self.body_placed(id);
        Ok(())
    }
    pub(super) fn place_kinematic(&mut self, id: BodyId, position: Vector) -> Result<(), Error> {
        let i = self.index(id)?;
        if !self.bodies[i].external || !position.finite() || position.abs().max_component() >= 1e12
        {
            return Err(Error::InvalidInput);
        }
        let body = &mut self.bodies[i];
        let before = body.cached_bounds;
        body.position = position;
        body.cached_bounds = contact::bounds(body);
        self.bookkeeping.poses_changed();
        if self.query_tree.current(self.bookkeeping.generation) {
            self.query_tree.moved(&self.bodies[i]);
        }
        // An awake body's leaf no longer holds its box; a resting one changed under others.
        let b = &self.bodies[i];
        if b.mass > 0.0 && !b.sleeping {
            self.bookkeeping.drop_box(b.id.slot());
        } else {
            self.resting_changed(i, before);
        }
        Ok(())
    }
    pub fn set_velocity(&mut self, id: BodyId, v: Vector) -> Result<(), Error> {
        if !v.finite() || v.abs().max_component() >= 1e12 {
            return Err(Error::InvalidInput);
        }
        self.sort_storage();
        let i = self.index(id)?;
        if self.bodies[i].mass == 0.0 {
            return if v == Vector::ZERO {
                Ok(())
            } else {
                Err(Error::InvalidInput)
            };
        }
        if self.bodies[i].velocity != v {
            self.wake_island(self.bodies[i].order);
            self.bodies[i].velocity = v;
        }
        Ok(())
    }
    pub fn apply_impulse(
        &mut self,
        id: BodyId,
        j: Vector,
        world_point: Vector,
    ) -> Result<(), Error> {
        if !j.finite() || !world_point.finite() {
            return Err(Error::InvalidInput);
        }
        self.sort_storage();
        let i = self.index(id)?;
        if !self.bodies[i].movable() {
            return Err(Error::InvalidInput);
        }
        let angular = (world_point - self.bodies[i].position).cross(j);
        let pending = self.bodies[i].impulse + j;
        let spin = self.bodies[i].angular_impulse + angular;
        if !pending.finite() || !spin.finite() {
            return Err(Error::InvalidInput);
        }
        if j != Vector::ZERO {
            self.wake_island(self.bodies[i].order);
        }
        self.bodies[i].impulse = pending;
        self.bodies[i].angular_impulse = spin;
        Ok(())
    }
    pub fn add_force(&mut self, id: BodyId, force: Vector) -> Result<(), Error> {
        if !force.finite() {
            return Err(Error::InvalidInput);
        }
        self.sort_storage();
        let i = self.index(id)?;
        if !self.bodies[i].movable() {
            return Err(Error::InvalidInput);
        }
        let total = self.bodies[i].force + force;
        if !total.finite() {
            return Err(Error::InvalidInput);
        }
        if force != Vector::ZERO {
            self.wake_island(self.bodies[i].order);
        }
        self.bodies[i].force = total;
        Ok(())
    }
    pub(super) fn wake_island(&mut self, root: Order) -> u64 {
        debug_assert!(
            self.storage_sorted(),
            "island traversal over unsorted storage"
        );
        let Some(root) = self.index_of_order(root) else {
            return 0;
        };
        let s = &mut self.bookkeeping;
        s.ensure_graph(&self.bodies, &self.cache);
        s.traversal.begin(self.bodies.len(), &mut s.work);
        s.traversal
            .collect(root, &self.bodies, &s.graph, &mut s.work);
        // Retain only newly awakened indices for same-substep force preparation.
        // The next traversal resets this scratch; no persistent wake-state copy is added.
        s.traversal.island.retain(|&i| {
            let b = &mut self.bodies[i];
            if !b.sleeping {
                return false;
            }
            self.transaction.body(i, b);
            b.sleeping = false;
            b.quiet_time = 0.0;
            true
        });
        let count = s.traversal.island.len();
        s.activity.count += count;
        for &i in &s.traversal.island {
            s.activity.woke(i, &mut s.work);
        }
        count as u64
    }
    pub fn has_support(&self, id: BodyId) -> bool {
        let Ok(i) = self.index(id) else {
            return false;
        };
        let id = self.bodies[i].order;
        let up = (-self.scene.gravity).unit();
        self.cache.iter().any(|(&(a, b), p)| {
            crate::work::add(crate::work::Counter::CachedPairVisits, 1);
            p.iter().any(|p| {
                (b == id && p.normal.dot(up) > 0.5) || (a == id && p.normal.dot(up) < -0.5)
            })
        })
    }
    /// A resting body's bounds change between `before` and its current bounds (it moved, was
    /// added or is about to be removed): broad-phase boxes near either may lack its leaf.
    pub(super) fn resting_changed(&mut self, i: usize, before: (Vector, Vector)) {
        let s = &mut self.bookkeeping;
        if !s.boxes_live {
            return;
        }
        if !self.query_tree.current(s.generation) {
            s.forget_boxes();
            return;
        }
        let slop = self.scene.contact_slop;
        drop_boxes_near(&self.query_tree, &mut s.broad, before, slop);
        let now = self.bodies[i].cached_bounds;
        if now != before {
            drop_boxes_near(&self.query_tree, &mut s.broad, now, slop);
        }
    }
}
/// Drops the broad-phase boxes whose leaves overlap `(lo, hi)` padded by `slop` (#412).
pub(super) fn drop_boxes_near(
    tree: &query::QueryTree,
    broad: &mut [bookkeeping::BroadBox],
    (lo, hi): (Vector, Vector),
    slop: Real,
) {
    if broad.is_empty() || !(lo.finite() && hi.finite()) {
        return;
    }
    let pad = slop
        + 64.0 * Real::EPSILON * (1.0 + lo.abs().max_component().max(hi.abs().max_component()));
    tree.visit_moving_aabb(
        spatial_kernels::Aabb {
            min: [lo.0 - pad, lo.1 - pad, lo.2 - pad],
            max: [hi.0 + pad, hi.1 + pad, hi.2 + pad],
        },
        |leaf| {
            if let Some(e) = broad.get_mut(leaf.id as usize) {
                e.epoch = 0;
            }
        },
    );
}
