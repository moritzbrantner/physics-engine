//! Body placement, velocity, impulses and forces, and the island wake they trigger.

use super::{BodyId, Error, Order, Real, Shape, Vector, World, bookkeeping, contact, query};

impl World {
    /// Places an external (kinematic) body at `position` without resolving contact.
    pub fn set_kinematic_position(&mut self, id: BodyId, position: Vector) -> Result<(), Error> {
        // Writing back the current position changes nothing a character relies on.
        let unchanged = self.body(id).is_some_and(|b| b.position == position);
        self.place_kinematic(id, position)?;
        if !unchanged {
            self.body_placed(id);
        }
        Ok(())
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
        if !wake.is_empty() {
            wake.sort_unstable();
            wake.dedup();
            self.sort_storage();
            for order in wake {
                self.wake_island(order);
            }
        }
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
