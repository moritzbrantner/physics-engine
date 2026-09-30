//! Before-images of touched motion and contact history, never a world snapshot.
use std::collections::{BTreeMap, btree_map::Entry};

use super::{
    Body, BodyId, BookkeepingStats, CachedPoint, GeometryStats, Quaternion, Report, Scalar, Vector,
    World,
};

/// Rollback work for one attempted positive-duration step. Vector capacity
/// excludes allocator/BTreeMap node overhead and the authoritative world.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct TransactionStats {
    pub journaled_bodies: u64,
    pub journaled_pairs: u64,
    pub copied_contact_points: u64,
    pub retired_bodies: u64,
    pub vector_capacity_bytes: u64,
    pub vector_growths: u64,
    pub rolled_back: bool,
}

/// Discarded solver work from a returned numerical error. No success event or
/// retired-body identity is published here; every attempted change was rolled back.
#[derive(Clone, Debug, Default)]
pub struct FailedStepWork {
    pub transaction: TransactionStats,
    pub attempted_substeps: u32,
    pub pair_tests: u64,
    pub narrow_tests: u64,
    pub contact_points: u64,
    pub integration_attempts: u64,
    pub woken_bodies: u64,
    pub retirement_attempts: usize,
    pub constraint_visits: u64,
    pub position_contact_tests: u64,
    pub bookkeeping: BookkeepingStats,
    pub geometry: GeometryStats,
}
impl FailedStepWork {
    pub(super) fn capture(report: &Report, bookkeeping: BookkeepingStats) -> Self {
        Self {
            attempted_substeps: report.substeps,
            pair_tests: report.pair_tests,
            narrow_tests: report.narrow_tests,
            contact_points: report.contact_points,
            integration_attempts: report.integrated_bodies,
            woken_bodies: report.woken_bodies,
            retirement_attempts: report.retired.len(),
            constraint_visits: report.convergence.constraint_visits,
            position_contact_tests: report.position.contact_tests,
            bookkeeping,
            geometry: report.geometry.clone(),
            ..Self::default()
        }
    }
}

#[cfg(test)]
mod tests;

#[derive(Clone, Copy, Debug)]
struct Motion {
    position: Vector,
    velocity: Vector,
    orientation: Quaternion,
    angular_velocity: Vector,
    sleeping: bool,
    quiet_time: Scalar,
    force: Vector,
    torque: Vector,
    impulse: Vector,
    angular_impulse: Vector,
}
impl Motion {
    fn capture(b: &Body) -> Self {
        Self {
            position: b.position,
            velocity: b.velocity,
            orientation: b.orientation,
            angular_velocity: b.angular_velocity,
            sleeping: b.sleeping,
            quiet_time: b.quiet_time,
            force: b.force,
            torque: b.torque,
            impulse: b.impulse,
            angular_impulse: b.angular_impulse,
        }
    }
    fn restore(self, b: &mut Body) {
        b.position = self.position;
        b.velocity = self.velocity;
        b.orientation = self.orientation;
        b.angular_velocity = self.angular_velocity;
        b.sleeping = self.sleeping;
        b.quiet_time = self.quiet_time;
        b.force = self.force;
        b.torque = self.torque;
        b.impulse = self.impulse;
        b.angular_impulse = self.angular_impulse;
        b.cached_bounds = super::contact::bounds(b);
    }
}

type Pair = (BodyId, BodyId);
#[derive(Clone, Debug, Default)]
pub(super) struct Journal {
    active: bool,
    epoch: u64,
    stamps: Vec<u64>,
    motion: Vec<(BodyId, Motion)>,
    removed: Vec<Body>,
    pairs: BTreeMap<Pair, Option<Vec<CachedPoint>>>,
    stats: TransactionStats,
}
impl Journal {
    pub fn begin(&mut self, bodies: usize) {
        debug_assert!(!self.active);
        debug_assert!(self.motion.is_empty() && self.removed.is_empty() && self.pairs.is_empty());
        self.active = true;
        self.stats = TransactionStats::default();
        let capacity = self.stamps.capacity();
        self.stamps.resize(bodies, 0);
        self.stats.vector_growths += u64::from(capacity != self.stamps.capacity());
        self.epoch = self.epoch.wrapping_add(1);
        if self.epoch == 0 {
            self.stamps.fill(0);
            self.epoch = 1;
        }
    }
    pub fn body(&mut self, index: usize, body: &Body) {
        if !self.active || self.stamps[index] == self.epoch {
            return;
        }
        self.stamps[index] = self.epoch;
        let capacity = self.motion.capacity();
        self.motion.push((body.id, Motion::capture(body)));
        self.stats.vector_growths += u64::from(capacity != self.motion.capacity());
        self.stats.journaled_bodies += 1;
    }
    pub fn pair(&mut self, key: Pair, points: Option<&Vec<CachedPoint>>) {
        if !self.active {
            return;
        }
        if let Entry::Vacant(entry) = self.pairs.entry(key) {
            self.stats.journaled_pairs += 1;
            self.stats.copied_contact_points += points.map_or(0, |p| p.len() as u64);
            self.stats.vector_growths += u64::from(points.is_some_and(|p| !p.is_empty()));
            entry.insert(points.cloned());
        }
    }
    pub fn remove(&mut self, index: usize, body: Body) {
        debug_assert!(self.active);
        self.stamps.remove(index);
        let capacity = self.removed.capacity();
        self.removed.push(body);
        self.stats.vector_growths += u64::from(capacity != self.removed.capacity());
        self.stats.retired_bodies += 1;
    }
    pub fn stats(&self, rolled_back: bool) -> TransactionStats {
        TransactionStats {
            vector_capacity_bytes: self.capacity_bytes(),
            rolled_back,
            ..self.stats
        }
    }
    pub fn capacity_bytes(&self) -> u64 {
        (self.stamps.capacity() * size_of::<u64>()
            + self.motion.capacity() * size_of::<(BodyId, Motion)>()
            + self.removed.capacity() * size_of::<Body>()
            + self
                .pairs
                .values()
                .flatten()
                .map(|v| v.capacity() * size_of::<CachedPoint>())
                .sum::<usize>()) as u64
    }
    pub fn commit(&mut self) {
        self.active = false;
        self.motion.clear();
        self.removed.clear();
        self.pairs.clear();
    }
    pub fn rollback(&mut self, world: &mut World) {
        // Membership first, then before-images by stable identity: retirement
        // can shift every later storage index during an earlier substep.
        for body in self.removed.drain(..) {
            let index = world
                .bodies
                .binary_search_by_key(&body.id, |b| b.id)
                .expect_err("retired body is absent during this step");
            world.bodies.insert(index, body);
        }
        for (id, motion) in self.motion.drain(..) {
            let index = world
                .index(id)
                .expect("journaled body restored before its motion");
            motion.restore(&mut world.bodies[index]);
        }
        for (key, points) in std::mem::take(&mut self.pairs) {
            if let Some(points) = points {
                world.cache.insert(key, points);
            } else {
                world.cache.remove(&key);
            }
        }
        self.stamps.clear();
        self.active = false;
        // These are disposable accelerators. Invalidating on failure avoids
        // accepting evidence about the discarded intermediate poses/layout.
        world.bookkeeping.layout_changed();
        world.bookkeeping.work = super::BookkeepingStats::default();
        world.geometry.invalidate();
        world.responses.clear();
        world.constraints.clear();
        world.manifold_scratch.clear();
    }
}
