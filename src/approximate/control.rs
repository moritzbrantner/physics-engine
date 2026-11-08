//! Body placement, velocity, impulses and forces, and the island wake they trigger.

use super::{
    BodyId, Error, Order, Quaternion, Real, Shape, Vector, World, bookkeeping, contact, query,
};
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
    /// Moves an external (kinematic) body with positive mass to `position` and `orientation`
    /// over the next [`World::step`], as Rapier's `set_next_kinematic_position(Pose)` on a
    /// `kinematic_position_based` body (#520). The step gives the body the linear and angular
    /// velocity of that pose change over its `dt` (the shorter arc), so contacts see the
    /// body's real point velocities and push what it strikes; the step ends with the body
    /// exactly at the pose and at rest, until the next call. A later call before the step
    /// replaces the target; `step(0.0)` and a failed step keep it. Nothing is placed or woken by
    /// the call: the moving body wakes the sleeping bodies it touches, leaves or meets on the
    /// way in the step's contact phase, as any moving kinematic body does (the moving body's
    /// swept test against a sleeper). Characters are moved by
    /// [`World::move_characters`] and are rejected, as are non-finite or out-of-range positions
    /// and an orientation farther than 1e-4 from unit length (it is normalized).
    pub fn set_next_kinematic_pose(
        &mut self,
        id: BodyId,
        position: Vector,
        orientation: Quaternion,
    ) -> Result<(), Error> {
        let i = self.index(id)?;
        let b = &self.bodies[i];
        let norm = (orientation.0 * orientation.0
            + orientation.1 * orientation.1
            + orientation.2 * orientation.2
            + orientation.3 * orientation.3)
            .sqrt();
        if !b.external
            || b.mass <= 0.0
            || self.characters.contains(id)
            || !position.finite()
            || position.abs().max_component() >= 1e12
            || !orientation.finite()
            || !norm.is_finite()
            || (norm - 1.0).abs() > 1e-4
        {
            return Err(Error::InvalidInput);
        }
        let orientation = orientation.normalized();
        if b.rotation_locked && orientation != b.orientation {
            return Err(Error::InvalidInput);
        }
        let bits = id.to_bits();
        let target = (bits, position, orientation);
        match self.kinematic_poses.binary_search_by_key(&bits, |t| t.0) {
            Ok(at) => self.kinematic_poses[at] = target,
            Err(at) => self.kinematic_poses.insert(at, target),
        }
        Ok(())
    }
    /// The velocity and angular velocity taking body `i` to `(position, orientation)` over
    /// `1 / rate` seconds, or `None` when out of range.
    fn pose_motion(
        &self,
        i: usize,
        position: Vector,
        orientation: Quaternion,
        rate: Real,
    ) -> Option<(Vector, Vector)> {
        let b = &self.bodies[i];
        let v = (position - b.position) * rate;
        let w = b.orientation.rotation_to(orientation) * rate;
        (v.finite()
            && v.abs().max_component() < 1e12
            && w.finite()
            && w.abs().max_component() < 1e9)
            .then_some((v, w))
    }
    /// Step start (after the transaction began): each pending pose target sets its body's
    /// velocity and angular velocity to reach it over `dt`. Rejects, before changing anything,
    /// a target whose velocity is out of range.
    pub(super) fn begin_kinematic_poses(
        &mut self,
        dt: crate::numeric::Scalar,
    ) -> Result<(), Error> {
        if self.kinematic_poses.is_empty() {
            return Ok(());
        }
        let rate = (1.0 / dt) as Real;
        for &(bits, position, orientation) in &self.kinematic_poses {
            if let Ok(i) = self.index(BodyId::from_bits(bits))
                && self.pose_motion(i, position, orientation, rate).is_none()
            {
                return Err(Error::InvalidInput);
            }
        }
        for n in 0..self.kinematic_poses.len() {
            let (bits, position, orientation) = self.kinematic_poses[n];
            let Ok(i) = self.index(BodyId::from_bits(bits)) else {
                continue;
            };
            let (v, w) = self
                .pose_motion(i, position, orientation, rate)
                .expect("checked above");
            let b = &mut self.bodies[i];
            self.transaction.body(i, b);
            b.velocity = v;
            b.angular_velocity = w;
        }
        Ok(())
    }
    /// Step end (inside the transaction): each pose-driven body lands exactly on its target and
    /// stops. A successful step then consumes the targets; a rollback keeps them (see `step`).
    pub(super) fn finish_kinematic_poses(&mut self) {
        for n in 0..self.kinematic_poses.len() {
            let (bits, position, orientation) = self.kinematic_poses[n];
            let Ok(i) = self.index(BodyId::from_bits(bits)) else {
                continue;
            };
            let b = &mut self.bodies[i];
            self.transaction.body(i, b);
            b.position = position;
            b.orientation = orientation;
            b.velocity = Vector::ZERO;
            b.angular_velocity = Vector::ZERO;
            b.cached_bounds = contact::bounds(b);
        }
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
        self.static_revision += 1;
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
        self.static_revision += u64::from(self.bodies[i].is_static());
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
        // Fixed and external roots are traversal boundaries. Removing a transient
        // body commonly reaches its static mesh; there is no island to index or wake.
        // No sleeper anywhere: no island has anything to wake (#523).
        if !self.bodies[root].movable() || !self.bookkeeping.activity.any_sleeper() {
            self.bookkeeping.traversal.island.clear();
            return 0;
        }
        let s = &mut self.bookkeeping;
        s.ensure_graph(&self.bodies, &self.cache, self.joints.pairs());
        s.traversal.begin(self.bodies.len(), &mut s.work);
        s.traversal
            .collect(root, &self.bodies, &s.graph, &mut s.work);
        self.wake_collected()
    }
    /// Wakes the sleepers of the islands reached from `roots` (body orders), which must be
    /// live; one traversal, so each island is visited once however many roots reach it.
    pub(super) fn wake_islands(&mut self, roots: &[Order]) {
        debug_assert!(
            self.storage_sorted(),
            "island traversal over unsorted storage"
        );
        if roots.is_empty() || !self.bookkeeping.activity.any_sleeper() {
            return;
        }
        let s = &mut self.bookkeeping;
        s.ensure_graph(&self.bodies, &self.cache, self.joints.pairs());
        s.traversal.begin(self.bodies.len(), &mut s.work);
        for &root in roots {
            let root = self.index_of_order(root).expect("wake root is a live body");
            let s = &mut self.bookkeeping;
            s.traversal
                .collect(root, &self.bodies, &s.graph, &mut s.work);
            self.wake_collected();
        }
    }
    /// Wakes the sleepers of the traversal's current island.
    fn wake_collected(&mut self) -> u64 {
        let s = &mut self.bookkeeping;
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
