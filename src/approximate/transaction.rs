//! Before-images of touched motion and contact history, never a world snapshot.
use super::{
    Body, BodyId, BookkeepingStats, CachedPoint, GeometryStats, Order, Quaternion, Real, Report,
    Vector, World,
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
    pub position_geometry: GeometryStats,
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
            position_geometry: report.position.geometry.clone(),
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
    quiet_time: Real,
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

type Pair = (Order, Order);
#[derive(Clone, Debug, Default)]
pub(super) struct Journal {
    active: bool,
    epoch: u64,
    stamps: Vec<u64>,
    motion: Vec<(BodyId, Motion)>,
    removed: Vec<Body>,
    /// Before-images of contact-cache entries, sorted by pair: the range of their points in
    /// `points`, or `None` for a pair that was absent. Flat storage keeps journaling
    /// allocation-free once warm (#375).
    pairs: Vec<(Pair, Option<(u32, u32)>)>,
    points: Vec<CachedPoint>,
    /// Position just past the last journaled or found pair. Calls mostly come in pair order,
    /// so the next pair is usually found here without a search.
    cursor: usize,
    stats: TransactionStats,
}
impl Journal {
    pub fn begin(&mut self, bodies: usize) {
        debug_assert!(!self.active);
        debug_assert!(
            self.motion.is_empty()
                && self.removed.is_empty()
                && self.pairs.is_empty()
                && self.points.is_empty()
        );
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
        let found = self.locate(key);
        self.cursor = found.unwrap_or_else(|at| at) + 1;
        let Err(at) = found else {
            return;
        };
        self.stats.journaled_pairs += 1;
        self.stats.copied_contact_points += points.map_or(0, |p| p.len() as u64);
        let capacities = (self.pairs.capacity(), self.points.capacity());
        let range = points.map(|points| {
            let start = self.points.len() as u32;
            self.points.extend_from_slice(points);
            (start, self.points.len() as u32)
        });
        self.pairs.insert(at, (key, range));
        self.stats.vector_growths += u64::from(capacities.0 != self.pairs.capacity())
            + u64::from(capacities.1 != self.points.capacity());
    }
    /// `binary_search` over the journaled pairs, trying the cursor first.
    fn locate(&self, key: Pair) -> Result<usize, usize> {
        let search = |pairs: &[(Pair, Option<(u32, u32)>)]| {
            pairs.binary_search_by_key(&key, |(key, _)| *key)
        };
        let at = self.cursor.min(self.pairs.len());
        if at > 0 && key <= self.pairs[at - 1].0 {
            return search(&self.pairs[..at]);
        }
        match self.pairs.get(at) {
            None => Err(at),
            Some(&(next, _)) if key < next => Err(at),
            Some(&(next, _)) if key == next => Ok(at),
            Some(_) => search(&self.pairs[at..])
                .map(|i| i + at)
                .map_err(|i| i + at),
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
            + self.pairs.capacity() * size_of::<(Pair, Option<(u32, u32)>)>()
            + self.points.capacity() * size_of::<CachedPoint>()) as u64
    }
    pub fn commit(&mut self) {
        self.active = false;
        self.motion.clear();
        self.removed.clear();
        self.pairs.clear();
        self.cursor = 0;
        self.points.clear();
    }
    pub fn rollback(&mut self, world: &mut World) {
        // Membership first, then before-images by stable identity: retirement
        // can shift every later storage index during an earlier substep.
        for body in self.removed.drain(..) {
            let index = world
                .bodies
                .binary_search_by_key(&body.order, |b| b.order)
                .expect_err("retired body is absent during this step");
            world.bodies.insert(index, body);
        }
        for (id, motion) in self.motion.drain(..) {
            let index = world
                .index(id)
                .expect("journaled body restored before its motion");
            motion.restore(&mut world.bodies[index]);
        }
        for &(key, range) in &self.pairs {
            if let Some((start, end)) = range {
                world
                    .cache
                    .insert(key, self.points[start as usize..end as usize].to_vec());
            } else {
                world.cache.remove(&key);
            }
        }
        self.pairs.clear();
        self.cursor = 0;
        self.points.clear();
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
