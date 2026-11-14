//! Remaining-time response for translational CCD bodies.
//!
//! A CCD body whose substep produced an admitted swept impact is not limited by a speculative
//! constraint. Instead it advances to the independent time of impact, receives the pair's
//! normal/friction/restitution response there, and re-sweeps the rest of the substep along the
//! response velocity against every candidate. The loop is bounded by the velocity-pass budget;
//! when that is exhausted the body stays at its last impact for the rest of the substep
//! (`GeometryStats::ccd_budget_fallbacks`). Rotation and non-CCD bodies are unchanged.
use super::{
    Body, Error, Real, Report, Shape, Vector, World,
    bookkeeping::push,
    contact,
    step::{contact_velocity, effective_mass},
};

/// Whether a body advances through its own swept impacts in this substep.
///
/// A body that retires on impact has no remaining-time trajectory; it keeps the speculative
/// constraint, whose simultaneous solve reaches every equal-time target (e.g. a projectile
/// bridging two disconnected bodies).
pub(super) fn advances(b: &Body) -> bool {
    b.ccd && b.movable() && !b.sleeping && !b.retire_on_impact && b.linear_support.is_none()
}

/// Swept pairs handled by remaining-time advancement instead of a speculative constraint.
pub(super) fn deferred(a: &Body, b: &Body) -> bool {
    (advances(a) || advances(b)) && a.linear_support.is_none() && b.linear_support.is_none()
}

fn overlaps((lo, hi): (Vector, Vector), (bl, bh): (Vector, Vector)) -> bool {
    !(bl.0 > hi.0 || bh.0 < lo.0 || bl.1 > hi.1 || bh.1 < lo.1 || bl.2 > hi.2 || bh.2 < lo.2)
}

fn union((lo, hi): (Vector, Vector), (bl, bh): (Vector, Vector)) -> (Vector, Vector) {
    (
        Vector(lo.0.min(bl.0), lo.1.min(bl.1), lo.2.min(bl.2)),
        Vector(hi.0.max(bh.0), hi.1.max(bh.1), hi.2.max(bh.2)),
    )
}

/// How far a turning kinematic body's points reach through rotation in `time` (#520).
fn turning_reach(b: &Body, time: Real) -> Real {
    if b.external {
        contact::spin_reach(b) * time
    } else {
        0.0
    }
}

fn ccd_contact_velocity(body: &Body, arm: Vector, frozen: bool) -> Vector {
    if frozen {
        Vector::ZERO
    } else {
        contact_velocity(body, arm, true)
    }
}

#[derive(Clone, Copy)]
struct TickTime {
    elapsed: Real,
    h: Real,
}

impl TickTime {
    fn remaining(self) -> Real {
        (1.0 - self.elapsed) * self.h
    }
}

/// An unchanged body's earliest contact stays valid on the shared clock when an unrelated
/// pair collides. Keep the original manifold as well as its absolute substep time: re-sweeping
/// an equal-time contact after advancing the clock can round past a thin obstacle.
struct ScheduledHit {
    absolute: Real,
    other: usize,
    manifold: contact::Manifold,
    first: bool,
}

/// Search constraints are passed together so single-body and event-queue paths keep the same
/// candidate admission policy without growing a long positional-argument API.
struct SearchOptions<'a> {
    frozen: &'a [(usize, Vector)],
    always: &'a [usize],
    targets: Option<&'a [usize]>,
    /// Swept bounds of the bounded moving targets that are not in `always` (#469).
    moving: Option<&'a MovingIndex>,
}

/// Transient, conservative index of the bodies that `ccd_unpruned_target` keeps out of the
/// fixed-body tree pruning: each leaf is the padded swept bound of the body's committed
/// trajectory from the clock time it was last set to the end of the substep (#469).
///
/// A linear trajectory's later remainder lies inside its whole remainder, so a leaf stays
/// conservative while the clock advances. Every trajectory change goes through an impact
/// response, which refreshes both participants' leaves from the impact time (never from the
/// pre-impact pose); a wake can change bodies beyond the pair and rebuilds every leaf.
/// Unbounded targets (half-spaces, non-finite bounds) stay in the unpruned list.
struct MovingIndex {
    tree: bvh_kernels::DynamicAabbTree,
    /// Current leaf per body index; `None` when the body is not indexed.
    leaves: Vec<Option<spatial_kernels::Aabb>>,
}

impl MovingIndex {
    fn bounds(&self, index: usize) -> Option<spatial_kernels::Aabb> {
        self.leaves.get(index).copied().flatten()
    }

    fn set(&mut self, index: usize, aabb: spatial_kernels::Aabb) {
        let leaf = spatial_kernels::Body {
            id: index as u32,
            aabb,
        };
        if self.leaves[index].replace(aabb).is_some() {
            if self.tree.update(leaf) {
                crate::work::add(crate::work::Counter::TreeInserts, 1);
            }
        } else {
            self.tree.insert(leaf);
            crate::work::add(crate::work::Counter::TreeInserts, 1);
        }
    }

    fn remove(&mut self, index: usize) {
        if self.leaves[index].take().is_some() {
            self.tree.remove(index as u32);
        }
    }

    fn visit(&self, query: spatial_kernels::Aabb, visit: impl FnMut(spatial_kernels::Body)) {
        let nodes = self.tree.visit_aabb(query, visit);
        crate::work::add(crate::work::Counter::TreeNodes, nodes as u64);
    }
}

/// Excluded pairs of the shared clock. An exclusion lasts until either endpoint changes
/// trajectory; per-body epochs retire such entries in O(1) instead of rescanning every
/// exclusion after each impact (#469).
struct Exclusions {
    pairs: std::collections::BTreeMap<(usize, usize), (u32, u32)>,
    epochs: Vec<u32>,
}

impl Exclusions {
    fn contains(&self, a: usize, b: usize) -> bool {
        let pair = (a.min(b), a.max(b));
        self.pairs
            .get(&pair)
            .is_some_and(|&epochs| epochs == (self.epochs[pair.0], self.epochs[pair.1]))
    }

    fn insert(&mut self, (a, b): (usize, usize)) {
        self.pairs.insert((a, b), (self.epochs[a], self.epochs[b]));
    }

    /// Drops every exclusion with `index` as an endpoint.
    fn retire(&mut self, index: usize) {
        self.epochs[index] += 1;
    }
}

/// Heap entry of one source's cached event: the earliest absolute time first, then the stable
/// body-order key of the pair, then the source slot, exactly the linear scan's selection order.
/// `version` marks entries superseded by a later reschedule of the same slot.
struct Pending {
    absolute: Real,
    key: (super::Order, super::Order),
    slot: usize,
    version: u32,
}

impl PartialEq for Pending {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other).is_eq()
    }
}

impl Eq for Pending {}

impl PartialOrd for Pending {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Pending {
    /// Reversed: `BinaryHeap` is a max-heap and the scheduler pops the earliest event.
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        other
            .absolute
            .total_cmp(&self.absolute)
            .then_with(|| other.key.cmp(&self.key))
            .then_with(|| other.slot.cmp(&self.slot))
    }
}

impl World {
    /// Advance queued CCD bodies, sharing a time-of-impact clock when they can interact.
    pub(super) fn advance_ccd_bodies(&mut self, h: Real, report: &mut Report) -> Result<(), Error> {
        let mut queued = std::mem::take(&mut self.bookkeeping.ccd);
        queued.sort_unstable();
        queued.dedup();
        if queued.is_empty() {
            self.bookkeeping.ccd = queued;
            return Ok(());
        }
        // Fixed geometry stays in the query tree. Moving or unbounded targets can acquire a
        // different swept bound after an impulse, so they remain in the unpruned candidate set.
        let always: Vec<_> = self
            .bodies
            .iter()
            .enumerate()
            .filter_map(|(index, b)| Self::ccd_unpruned_target(b).then_some(index))
            .collect();
        let result = if queued.len() > 1 {
            self.advance_ccd_group(&queued, h, &always, report)
        } else if let Some(&i) = queued.first() {
            self.advance_ccd_body(i, h, &always, report)
        } else {
            Ok(())
        };
        // Keep the admitted set until integration; its impacts have already been resolved.
        self.bookkeeping.ccd = queued;
        result
    }

    fn ensure_ccd_offsets(&mut self) {
        let count = self.bodies.len();
        if self.bookkeeping.ccd_offsets.len() < count {
            let s = &mut self.bookkeeping;
            super::reserve(&mut s.ccd_offsets, count, &mut s.work);
            s.ccd_offsets.resize(count, Vector::ZERO);
        }
    }

    /// Position of body `j` at `fraction` of this substep along its committed trajectory.
    fn position_at(&self, j: usize, fraction: Real, h: Real) -> Vector {
        let b = &self.bodies[j];
        let offset = self
            .bookkeeping
            .ccd_offsets
            .get(j)
            .copied()
            .unwrap_or(Vector::ZERO);
        if b.movable() && !b.sleeping || b.external {
            b.position + b.velocity * (fraction * h) + offset
        } else {
            b.position
        }
    }

    /// A turning kinematic target's orientation at the clock (#520): the remaining-time search
    /// sweeps it on from there (`contact::turning_sweep`).
    fn turn_to(&self, target: &mut Body, clock: TickTime) {
        if target.external && contact::spin_reach(target) > 0.0 && clock.elapsed > 0.0 {
            target.orientation = target
                .orientation
                .integrate(target.angular_velocity, clock.elapsed * clock.h);
        }
    }

    fn ccd_unpruned_target(b: &Body) -> bool {
        b.mass > 0.0
            || b.external
            || b.velocity != Vector::ZERO
            || matches!(b.shape, Shape::HalfSpace)
    }

    /// Query only conservatively swept fixed-body bounds from the existing world tree.
    /// Keep every movable/kinematic/unbounded body as a candidate: its velocity can change
    /// during CCD response before the tree is refitted. A stale tree falls back to a scan.
    fn ccd_candidates(
        &self,
        swept_bounds: (Vector, Vector),
        probe: &Body,
        travel: Vector,
        always: &[usize],
        moving: Option<&MovingIndex>,
    ) -> Vec<usize> {
        if !self.query_tree.current(self.bookkeeping.generation)
            || !swept_bounds.0.finite()
            || !swept_bounds.1.finite()
        {
            return (0..self.bodies.len()).collect();
        }
        let magnitude = self
            .query_tree
            .magnitude
            .max(probe.position.abs().max_component())
            .max(travel.abs().max_component())
            + probe.shape.radius();
        // Bounding-tree coordinates and swept endpoint additions can round independently.
        // Err wide: pruning is not allowed to turn a narrow-phase failure into a missed hit.
        let pad = 128.0 * Real::EPSILON * (1.0 + magnitude) + self.scene.contact_slop;
        let query = spatial_kernels::Aabb {
            min: [
                swept_bounds.0.0 - pad,
                swept_bounds.0.1 - pad,
                swept_bounds.0.2 - pad,
            ],
            max: [
                swept_bounds.1.0 + pad,
                swept_bounds.1.1 + pad,
                swept_bounds.1.2 + pad,
            ],
        };
        let mut candidates = always.to_vec();
        let nodes = self.query_tree.visit_aabb(query, |leaf| {
            let index = self.slot_index(leaf.id);
            if !Self::ccd_unpruned_target(&self.bodies[index]) {
                candidates.push(index);
            }
        });
        crate::work::add(crate::work::Counter::TreeNodes, nodes as u64);
        if let Some(moving) = moving {
            moving.visit(query, |leaf| candidates.push(leaf.id as usize));
        }
        candidates.sort_unstable_by_key(|&index| self.bodies[index].order);
        candidates.dedup();
        candidates
    }

    fn advance_ccd_body(
        &mut self,
        i: usize,
        h: Real,
        always: &[usize],
        report: &mut Report,
    ) -> Result<(), Error> {
        if !advances(&self.bodies[i]) {
            return Ok(());
        }
        self.ensure_ccd_offsets();
        let count = self.bodies.len();
        if self.bookkeeping.ccd_start.len() < count {
            let s = &mut self.bookkeeping;
            super::reserve(&mut s.ccd_start, count, &mut s.work);
            s.ccd_start.resize(count, 0.0);
        }
        self.transaction.body(i, &self.bodies[i]);
        // A body already struck by an earlier-advanced CCD body continues from that impact
        // along its committed trajectory instead of replaying the substep from its start pose.
        let mut elapsed: Real = self.bookkeeping.ccd_start[i];
        let mut position = self.position_at(i, elapsed, h);
        let mut impacts: u8 = 0;
        let limit = self.scene.velocity_iterations.max(1);
        let mut excluded = std::mem::take(&mut self.bookkeeping.ccd_excluded);
        excluded.clear();
        let outcome = loop {
            let remaining = 1.0 - elapsed;
            if remaining <= 0.0 {
                break Ok(());
            }
            let hit = self.next_ccd_hit(
                i,
                position,
                TickTime { elapsed, h },
                |j| excluded.contains(&j),
                SearchOptions {
                    frozen: &[],
                    always,
                    targets: None,
                    moving: None,
                },
                report,
            );
            let Some((time, j, m, first)) = (match hit {
                Ok(hit) => hit,
                Err(error) => {
                    self.bookkeeping.ccd_excluded = excluded;
                    return Err(error);
                }
            }) else {
                position += self.bodies[i].velocity * (remaining * h);
                break Ok(());
            };
            let step = time * remaining;
            position += self.bodies[i].velocity * (step * h);
            elapsed += step;
            if !self.respond_ccd_impact(
                (i, j),
                &m,
                first,
                [false, false],
                TickTime { elapsed, h },
                report,
            ) {
                // Touching while separating is not an impact and does not use the budget;
                // do not re-test this partner.
                push(&mut excluded, j, &mut self.bookkeeping.work);
                continue;
            }
            impacts += 1;
            excluded.clear();
            push(&mut excluded, j, &mut self.bookkeeping.work);
            if impacts >= limit && elapsed < 1.0 {
                report.geometry.ccd_budget_fallbacks += 1;
                break Ok(());
            }
        };
        self.bookkeeping.ccd_excluded = excluded;
        // Integration adds velocity * h; the offset makes it land on the advanced position.
        let b = &self.bodies[i];
        self.bookkeeping.ccd_offsets[i] = position - (b.position + b.velocity * h);
        outcome
    }

    /// Multiple CCD bodies share a chronological clock. Advancing one body's entire trajectory
    /// before another's earlier collision would let a later, no-longer-possible CCD pair fire.
    /// The single-body path above retains its no-allocation fast path and exact budget fallback.
    ///
    /// Each source caches its earliest event; the events wait in a versioned min-heap in the
    /// same chronological order as a linear scan, and moving targets' swept bounds live in a
    /// transient [`MovingIndex`]. After an impact only the sources whose swept bound overlaps
    /// a changed participant's old or new trajectory are re-searched (#469).
    fn advance_ccd_group(
        &mut self,
        queued: &[usize],
        h: Real,
        always: &[usize],
        report: &mut Report,
    ) -> Result<(), Error> {
        self.ensure_ccd_offsets();
        let n = queued.len();
        let mut counts = Vec::new();
        super::reserve(&mut counts, n, &mut self.bookkeeping.work);
        counts.resize(n, 0u8);
        // A budget-exhausted body remains at the impact pose for the rest of this substep.
        let mut frozen: Vec<(usize, Vector)> = Vec::new();
        let mut is_frozen = vec![false; n];
        let mut excluded = Exclusions {
            pairs: std::collections::BTreeMap::new(),
            epochs: vec![0; self.bodies.len()],
        };
        let limit = self.scene.velocity_iterations.max(1);
        let mut elapsed = 0.0;
        let (mut moving, mut unbounded) =
            self.ccd_moving_index(always, &frozen, TickTime { elapsed, h });
        // A source outside the index is never found by an overlap query: rescan every source.
        let mut sources_indexed = queued.iter().all(|&i| moving.bounds(i).is_some());

        // The old all-sources re-sweep after every hit both repeated O(m*n) work and lost
        // independent equal-time contacts to f32 TOI rounding. Cache each source's first hit;
        // after an impulse only the changed partners can introduce new earlier contacts.
        let mut upcoming = Vec::new();
        super::reserve(&mut upcoming, n, &mut self.bookkeeping.work);
        for &i in queued {
            upcoming.push(self.next_scheduled_hit(
                i,
                TickTime { elapsed, h },
                &excluded,
                SearchOptions {
                    frozen: &frozen,
                    always: &unbounded,
                    targets: None,
                    moving: Some(&moving),
                },
                report,
            )?);
        }
        let mut versions = vec![0u32; n];
        let mut heap = std::collections::BinaryHeap::with_capacity(n);
        for (slot, event) in upcoming.iter().enumerate() {
            if let Some(event) = event {
                heap.push(self.pending(queued[slot], event, slot, 0));
                crate::work::add(crate::work::Counter::CcdHeapPushes, 1);
            }
        }
        // Per-slot re-search mark: 0 none, 1 may newly hit a changed body, 2 full re-search.
        let mut marks = vec![0u8; n];
        let mut affected: Vec<usize> = Vec::new();
        // The first event re-checks every source's admission, as the linear scheduler did.
        let mut check_all = true;
        while elapsed < 1.0 {
            let slot = loop {
                match heap.pop() {
                    None => break None,
                    Some(entry) if entry.version == versions[entry.slot] => break Some(entry.slot),
                    Some(_) => {
                        crate::work::add(crate::work::Counter::CcdHeapStalePops, 1);
                    }
                }
            };
            let Some(slot) = slot else {
                break;
            };
            let event = upcoming[slot].take().expect("selected candidate exists");
            versions[slot] += 1;
            let (i, j) = (queued[slot], event.other);
            elapsed = event.absolute;
            affected.clear();
            push(&mut affected, slot, &mut self.bookkeeping.work);
            let pair = (i.min(j), i.max(j));
            let previous = [
                (self.bodies[i].velocity, self.bodies[i].angular_velocity),
                (self.bodies[j].velocity, self.bodies[j].angular_velocity),
            ];
            let wakes = report.woken_bodies;
            let frozen_at_impact = if frozen.is_empty() {
                [false, false]
            } else {
                [
                    queued.binary_search(&i).is_ok_and(|k| is_frozen[k]),
                    queued.binary_search(&j).is_ok_and(|k| is_frozen[k]),
                ]
            };
            let closing = self.respond_ccd_impact(
                (i, j),
                &event.manifold,
                event.first,
                frozen_at_impact,
                TickTime { elapsed, h },
                report,
            );
            if !closing {
                excluded.insert(pair);
                // Both sources of the excluded pair might have cached the same event.
                for index in [i, j] {
                    if let Ok(k) = queued.binary_search(&index) {
                        push(&mut affected, k, &mut self.bookkeeping.work);
                    }
                }
            } else {
                for index in [i, j] {
                    if let Ok(k) = queued.binary_search(&index) {
                        if counts[k] >= limit {
                            continue;
                        }
                        counts[k] += 1;
                        if counts[k] == limit && elapsed < 1.0 {
                            let position = self.position_at(index, elapsed, h);
                            push(&mut frozen, (index, position), &mut self.bookkeeping.work);
                            is_frozen[k] = true;
                            report.geometry.ccd_budget_fallbacks += 1;
                        }
                    }
                }
                if !frozen.is_empty() {
                    self.stop_into_frozen_partner(
                        (i, j),
                        &event.manifold,
                        event.first,
                        [
                            queued.binary_search(&i).is_ok_and(|k| is_frozen[k]),
                            queued.binary_search(&j).is_ok_and(|k| is_frozen[k]),
                        ],
                        TickTime { elapsed, h },
                        report,
                    );
                }
            }
            // Detect both changed velocities and newly frozen bodies. Static walls do not
            // change, so one ball's wall impact cannot invalidate the other balls' contacts.
            let mut changed = [usize::MAX; 2];
            let mut changed_len = 0;
            for (index, before) in [(i, previous[0]), (j, previous[1])] {
                let b = &self.bodies[index];
                if (b.velocity, b.angular_velocity) != before
                    || queued.binary_search(&index).is_ok_and(|k| is_frozen[k])
                {
                    changed[changed_len] = index;
                    changed_len += 1;
                }
            }
            let changed = &changed[..changed_len];
            let woke = report.woken_bodies != wakes;
            if woke {
                // Waking an island can change admissibility beyond the two responding bodies.
                excluded.pairs.clear();
            } else if closing {
                // Exclusions are valid until an endpoint changes trajectory. An unrelated
                // ball hitting the same static wall must not re-admit every other ball's
                // just-hit wall pair and resweep their entire remaining trajectories.
                for &index in changed {
                    excluded.retire(index);
                }
            }
            if closing {
                excluded.insert(pair);
            }
            for &k in &affected {
                marks[k] = 2;
            }
            let mut search_all = !sources_indexed;
            if woke {
                // Woken bodies changed trajectory without an impact: re-index from now.
                (moving, unbounded) =
                    self.ccd_moving_index(always, &frozen, TickTime { elapsed, h });
                sources_indexed = queued.iter().all(|&i| moving.bounds(i).is_some());
                marks.fill(2);
                check_all = true;
            } else {
                // Refresh changed leaves from the impact time; a source that cached an event
                // on the old trajectory or can reach the new one overlaps their union.
                for &index in changed {
                    let Some(old) = moving.bounds(index) else {
                        search_all = true;
                        continue;
                    };
                    match self.ccd_swept_leaf(index, &frozen, TickTime { elapsed, h }) {
                        Some(new) => {
                            moving.visit(old.union(new), |leaf| {
                                if let Ok(k) = queued.binary_search(&(leaf.id as usize)) {
                                    marks[k] = marks[k].max(1);
                                    affected.push(k);
                                }
                            });
                            moving.set(index, new);
                        }
                        None => {
                            // An unsafe bound leaves the index for the conservative list.
                            moving.remove(index);
                            push(&mut unbounded, index, &mut self.bookkeeping.work);
                            unbounded.sort_unstable();
                            search_all = true;
                        }
                    }
                }
            }
            if search_all {
                for mark in &mut marks {
                    *mark = (*mark).max(1);
                }
            }
            let full = check_all || search_all;
            check_all = false;
            if full {
                affected.clear();
                affected.extend(0..n);
            } else {
                affected.sort_unstable();
                affected.dedup();
            }
            for &slot in &affected {
                let source = queued[slot];
                let mark = std::mem::take(&mut marks[slot]);
                if is_frozen[slot] || !advances(&self.bodies[source]) {
                    if upcoming[slot].take().is_some() {
                        versions[slot] += 1;
                    }
                    continue;
                }
                if mark == 0 {
                    continue;
                }
                // A refreshed manifold can change without changing the event's heap key.
                // Keep its existing live heap entry if time and partner are bit-identical;
                // the entry resolves its manifold through `upcoming[slot]` when popped.
                let prior_key = upcoming[slot]
                    .as_ref()
                    .map(|event| (event.absolute.to_bits(), event.other));
                let options = SearchOptions {
                    frozen: &frozen,
                    always: &unbounded,
                    targets: None,
                    moving: Some(&moving),
                };
                // Rebase an imminent cached pair to the current shared clock:
                // near-simultaneous f32 TOIs can change by a few ULPs. Check
                // only its previous partner and the changed bodies first,
                // preserving the sparse source/target work contract.
                let near_time = elapsed + 8.0 * Real::EPSILON * (1.0 + elapsed);
                let imminent = mark == 1
                    && upcoming[slot]
                        .as_ref()
                        .is_some_and(|candidate| candidate.absolute <= near_time);
                let rescheduled = if mark == 2
                    || upcoming[slot]
                        .as_ref()
                        .is_some_and(|candidate| changed.contains(&candidate.other))
                {
                    upcoming[slot] = self.next_scheduled_hit(
                        source,
                        TickTime { elapsed, h },
                        &excluded,
                        options,
                        report,
                    )?;
                    true
                } else if let Some(old_other) = upcoming[slot]
                    .as_ref()
                    .filter(|_| imminent)
                    .map(|event| event.other)
                {
                    let mut targets = [old_other; 3];
                    let mut len = 1;
                    for &j in changed {
                        if !targets[..len].contains(&j) {
                            targets[len] = j;
                            len += 1;
                        }
                    }
                    let candidate = self.next_scheduled_hit(
                        source,
                        TickTime { elapsed, h },
                        &excluded,
                        SearchOptions {
                            targets: Some(&targets[..len]),
                            ..options
                        },
                        report,
                    )?;
                    // Unchanged trajectories keep their previously certified
                    // contact. A later f32 clock may round a thin hit away, so
                    // never discard that event merely because the resweep
                    // didn't find it again. Rebase only an observed partner.
                    let previous = upcoming[slot].take();
                    upcoming[slot] = match (previous, candidate) {
                        (Some(old), Some(new))
                            if new.other == old.other
                                || new.absolute < old.absolute
                                || (new.absolute == old.absolute
                                    && self.bodies[new.other].order
                                        < self.bodies[old.other].order) =>
                        {
                            Some(new)
                        }
                        (Some(old), _) => Some(old),
                        (None, candidate) => candidate,
                    };
                    true
                } else if !changed.is_empty() {
                    // A changed partner may now be the earliest hit even if the source's
                    // previously predicted partner was unchanged.
                    let candidate = self.next_scheduled_hit(
                        source,
                        TickTime { elapsed, h },
                        &excluded,
                        SearchOptions {
                            targets: Some(changed),
                            ..options
                        },
                        report,
                    )?;
                    let replace = candidate.as_ref().is_some_and(|candidate| {
                        upcoming[slot].as_ref().is_none_or(|old| {
                            candidate.absolute < old.absolute
                                || (candidate.absolute == old.absolute
                                    && self.bodies[candidate.other].order
                                        < self.bodies[old.other].order)
                        })
                    });
                    if replace {
                        upcoming[slot] = candidate;
                    }
                    replace
                } else {
                    false
                };
                let next_key = upcoming[slot]
                    .as_ref()
                    .map(|event| (event.absolute.to_bits(), event.other));
                if rescheduled && prior_key != next_key {
                    versions[slot] += 1;
                    if let Some(event) = &upcoming[slot] {
                        heap.push(self.pending(source, event, slot, versions[slot]));
                        crate::work::add(crate::work::Counter::CcdHeapPushes, 1);
                    }
                }
            }
        }
        // The velocity is retained for the next tick even when the current tick exhausted its
        // impact budget; only this tick's integration is pinned to the last safe impact pose.
        for (i, position) in frozen {
            let body = &self.bodies[i];
            self.bookkeeping.ccd_offsets[i] = position - (body.position + body.velocity * h);
        }
        Ok(())
    }

    fn pending(&self, source: usize, event: &ScheduledHit, slot: usize, version: u32) -> Pending {
        let a = self.bodies[source].order;
        let b = self.bodies[event.other].order;
        Pending {
            absolute: event.absolute,
            key: (a.min(b), a.max(b)),
            slot,
            version,
        }
    }

    /// The conservative leaf of `j`'s remaining trajectory from `clock`: the same swept target
    /// bound `next_ccd_hit` tests at any later clock time, padded for the rounding between
    /// clock times. `None` for unbounded or non-finite bounds.
    fn ccd_swept_leaf(
        &self,
        j: usize,
        frozen: &[(usize, Vector)],
        clock: TickTime,
    ) -> Option<spatial_kernels::Aabb> {
        let b = &self.bodies[j];
        if matches!(b.shape, Shape::HalfSpace) {
            return None;
        }
        let mut target = b.clone();
        let (motion, turn) =
            if let Some((_, position)) = frozen.iter().find(|(index, _)| *index == j) {
                target.position = *position;
                (Vector::ZERO, 0.0)
            } else {
                target.position = self.position_at(j, clock.elapsed, clock.h);
                self.turn_to(&mut target, clock);
                (
                    target.velocity * clock.remaining(),
                    turning_reach(&target, clock.remaining()),
                )
            };
        let now = contact::bounds(&target);
        let (lo, hi) = union(now, (now.0 + motion, now.1 + motion));
        let (lo, hi) = (lo - Vector(turn, turn, turn), hi + Vector(turn, turn, turn));
        if !lo.finite() || !hi.finite() {
            return None;
        }
        let magnitude = lo
            .abs()
            .max_component()
            .max(hi.abs().max_component())
            .max(motion.abs().max_component());
        let pad = 128.0 * Real::EPSILON * (1.0 + magnitude) + self.scene.contact_slop;
        let aabb = spatial_kernels::Aabb {
            min: [lo.0 - pad, lo.1 - pad, lo.2 - pad],
            max: [hi.0 + pad, hi.1 + pad, hi.2 + pad],
        };
        (aabb.min.iter().chain(&aabb.max).all(|v| v.is_finite())).then_some(aabb)
    }

    /// Index every bounded unpruned target's remaining swept trajectory; the rest stay in the
    /// returned conservative list (ascending body index, like `always`).
    fn ccd_moving_index(
        &self,
        always: &[usize],
        frozen: &[(usize, Vector)],
        clock: TickTime,
    ) -> (MovingIndex, Vec<usize>) {
        let mut index = MovingIndex {
            tree: bvh_kernels::DynamicAabbTree::new(0.0),
            leaves: vec![None; self.bodies.len()],
        };
        let mut unbounded = Vec::new();
        for &j in always {
            match self.ccd_swept_leaf(j, frozen, clock) {
                Some(aabb) => index.set(j, aabb),
                None => unbounded.push(j),
            }
        }
        (index, unbounded)
    }

    fn next_scheduled_hit(
        &mut self,
        i: usize,
        clock: TickTime,
        excluded: &Exclusions,
        search: SearchOptions<'_>,
        report: &mut Report,
    ) -> Result<Option<ScheduledHit>, Error> {
        let at = self.position_at(i, clock.elapsed, clock.h);
        Ok(self
            .next_ccd_hit(i, at, clock, |j| excluded.contains(i, j), search, report)?
            .map(|(time, other, manifold, first)| ScheduledHit {
                absolute: clock.elapsed + time * (1.0 - clock.elapsed),
                other,
                manifold,
                first,
            }))
    }

    fn next_ccd_hit(
        &mut self,
        i: usize,
        position: Vector,
        clock: TickTime,
        excluded: impl Fn(usize) -> bool,
        search: SearchOptions<'_>,
        report: &mut Report,
    ) -> Result<Option<(Real, usize, contact::Manifold, bool)>, Error> {
        let mut probe = self.bodies[i].clone();
        probe.position = position;
        let travel = probe.velocity * clock.remaining();
        let probe_now = contact::bounds(&probe);
        let swept_bounds = union(probe_now, (probe_now.0 + travel, probe_now.1 + travel));
        let mut best: Option<(Real, usize, contact::Manifold, bool)> = None;
        let candidates = search.targets.map_or_else(
            || self.ccd_candidates(swept_bounds, &probe, travel, search.always, search.moving),
            <[usize]>::to_vec,
        );
        for j in candidates {
            if j == i || excluded(j) {
                continue;
            }
            let other = &self.bodies[j];
            if other.sensor || !probe.layers.collides_with(other.layers) {
                continue;
            }
            // Bodies joined by a revolute joint without contacts never touch (#568).
            let (lo, hi) = if probe.order < other.order {
                (probe.order, other.order)
            } else {
                (other.order, probe.order)
            };
            if self.joints.any_contacts_disabled() && self.joints.contacts_disabled(lo, hi) {
                continue;
            }
            report.pair_tests += 1;
            let mut target = other.clone();
            if let Some((_, position)) = search.frozen.iter().find(|(index, _)| *index == j) {
                target.position = *position;
                target.velocity = Vector::ZERO;
            } else {
                target.position = self.position_at(j, clock.elapsed, clock.h);
                self.turn_to(&mut target, clock);
            }
            let target_now = contact::bounds(&target);
            let motion = target.velocity * clock.remaining();
            let turn = turning_reach(&target, clock.remaining());
            let target_bounds = union(target_now, (target_now.0 + motion, target_now.1 + motion));
            let target_bounds = (
                target_bounds.0 - Vector(turn, turn, turn),
                target_bounds.1 + Vector(turn, turn, turn),
            );
            if !overlaps(swept_bounds, target_bounds) {
                continue;
            }
            if let Some((time, ..)) = &best
                && !self.may_hit_by(&probe, &target, (probe_now, target_now), *time, clock)
            {
                continue;
            }
            report.narrow_tests += 1;
            let first = probe.order < target.order;
            let (a, b) = if first {
                (&probe, &target)
            } else {
                (&target, &probe)
            };
            let children = super::compound::children(&self.compounds, &target);
            let swept = if !children.is_empty() {
                // A compound target: the earliest of its children's sweeps (ties keep the
                // lower child), its arm moved to the compound's centre.
                let mut earliest: Option<contact::Manifold> = None;
                for c in children {
                    let child = super::compound::child_body(&target, c);
                    let (a, b) = if first {
                        (&probe, &child)
                    } else {
                        (&child, &probe)
                    };
                    let m = contact::swept(
                        a,
                        b,
                        clock.remaining(),
                        self.scene.contact_slop,
                        &mut report.geometry,
                    )
                    .map_err(|reason| Error::CollisionSearchFailed {
                        bodies: [a.id, b.id],
                        reason,
                    })?;
                    if let Some(mut m) = m
                        && earliest.as_ref().is_none_or(|e| m.time < e.time)
                    {
                        for p in &mut m.points {
                            if first {
                                p.rb = super::compound::parent_arm(&child, &target, p.rb);
                            } else {
                                p.ra = super::compound::parent_arm(&child, &target, p.ra);
                            }
                        }
                        earliest = Some(m);
                    }
                }
                Ok(earliest)
            } else if let Some(surface) = super::static_sphere::Surface::of(
                target.shape,
                &self.triangle_meshes,
                &self.terrains,
                &self.hulls,
            ) {
                // Spheres sweep meshes, terrain and hulls; any other probe fails explicitly.
                super::static_sphere::manifold(
                    surface,
                    a,
                    b,
                    self.scene.contact_slop,
                    Some(clock.remaining()),
                    &mut report.geometry,
                )
            } else {
                contact::swept(
                    a,
                    b,
                    clock.remaining(),
                    self.scene.contact_slop,
                    &mut report.geometry,
                )
            }
            .map_err(|reason| Error::CollisionSearchFailed {
                bodies: [a.id, b.id],
                reason,
            })?;
            if let Some(m) = swept
                && best.as_ref().is_none_or(|(time, index, ..)| {
                    m.time < *time || (m.time == *time && other.order < self.bodies[*index].order)
                })
            {
                best = Some((m.time, j, m, first));
            }
        }
        Ok(best)
    }

    /// Whether `probe` and `target` can touch by `time`, a fraction of the remaining substep.
    /// Once a hit at `time` is known, a candidate that cannot reach the probe by then can be
    /// neither the earliest nor the order-tied hit, so its exact sweep can be skipped (#485)
    /// without changing which event is selected. Only the closed-form sphere/box sweeps
    /// qualify: they report the first surface contact (within 1e-6 of the radius), never an
    /// earlier admission-margin time, and cannot fail on finite relative motion, so a skipped
    /// sweep would have reported no hit or a later one. The window bounds are padded for that
    /// contact tolerance, the slop and rounding, as the swept candidate bounds are.
    fn may_hit_by(
        &self,
        probe: &Body,
        target: &Body,
        (probe_now, target_now): ((Vector, Vector), (Vector, Vector)),
        time: Real,
        clock: TickTime,
    ) -> bool {
        let analytic = |shape| matches!(shape, Shape::Sphere(_) | Shape::Box(_));
        if !analytic(probe.shape) || !analytic(target.shape) || contact::driven_spin(probe, target)
        {
            return true;
        }
        let relative = (target.velocity - probe.velocity) * clock.remaining();
        let span = time * clock.remaining();
        let (a, b) = (probe.velocity * span, target.velocity * span);
        let window_a = union(probe_now, (probe_now.0 + a, probe_now.1 + a));
        let window_b = union(target_now, (target_now.0 + b, target_now.1 + b));
        let magnitude = [window_a.0, window_a.1, window_b.0, window_b.1, relative]
            .iter()
            .fold(0.0, |m: Real, v| m.max(v.abs().max_component()));
        let pad = 128.0 * Real::EPSILON * (1.0 + magnitude)
            + 1e-5 * (1.0 + probe.shape.radius() + target.shape.radius())
            + self.scene.contact_slop;
        if !pad.is_finite() {
            return true;
        }
        let padding = Vector(pad, pad, pad);
        overlaps((window_a.0 - padding, window_a.1 + padding), window_b)
    }

    /// A body reaching its CCD pass limit is fixed at this TOI for the remainder of the tick.
    /// The two-body impulse that led to exhaustion can still leave its partner approaching the
    /// now-fixed contact. Remove only that closing normal velocity. Without this correction the
    /// just-handled pair is excluded from the scheduler and can interpenetrate by tick end.
    /// This is a budget fallback, not another restitution event (#483).
    fn stop_into_frozen_partner(
        &mut self,
        (i, j): (usize, usize),
        m: &contact::Manifold,
        first: bool,
        frozen: [bool; 2],
        clock: TickTime,
        report: &mut Report,
    ) {
        let (mobile, outward, first_arm) = match frozen {
            [false, true] => (i, if first { -m.normal } else { m.normal }, true),
            [true, false] => (j, if first { m.normal } else { -m.normal }, false),
            _ => return,
        };
        let mut arm = Vector::ZERO;
        for point in &m.points {
            arm += match (first, first_arm) {
                (true, true) | (false, false) => point.ra,
                _ => point.rb,
            };
        }
        arm = arm * (1.0 / m.points.len().max(1) as Real);
        let approach = contact_velocity(&self.bodies[mobile], arm, true).dot(outward);
        if approach >= 0.0 {
            return;
        }
        let response = self.responses[mobile];
        let angular = response.inertia(arm.cross(outward), report);
        let k = response.inverse_mass + angular.cross(arm).dot(outward);
        if k <= 1e-14 {
            return;
        }
        let impulse = outward * (-approach / k);
        self.transaction.body(mobile, &self.bodies[mobile]);
        let body = &mut self.bodies[mobile];
        let old_velocity = body.velocity;
        body.velocity += impulse * response.inverse_mass;
        body.angular_velocity += response.inertia(arm.cross(impulse), report);
        self.bookkeeping.ccd_offsets[mobile] -=
            (body.velocity - old_velocity) * (clock.elapsed * clock.h);
    }

    /// Apply exactly one admitted impact at its TOI, retaining piecewise linear trajectories
    /// for both participants. A budget-frozen target is stationary, including in the impulse
    /// calculation; retaining its pre-freeze velocity would misclassify genuine incoming hits
    /// as separating touches. Returns false for non-closing touches (which do not use a pass).
    fn respond_ccd_impact(
        &mut self,
        (i, j): (usize, usize),
        m: &contact::Manifold,
        first: bool,
        frozen: [bool; 2],
        clock: TickTime,
        report: &mut Report,
    ) -> bool {
        let count = m.points.len().max(1) as Real;
        let (mut ra, mut rb) = (Vector::ZERO, Vector::ZERO);
        for p in &m.points {
            ra += p.ra;
            rb += p.rb;
        }
        let (ra, rb) = (ra * (1.0 / count), rb * (1.0 / count));
        let (n, r_self, r_other) = if first {
            (m.normal, ra, rb)
        } else {
            (-m.normal, rb, ra)
        };
        let closing = (ccd_contact_velocity(&self.bodies[j], r_other, frozen[1])
            - ccd_contact_velocity(&self.bodies[i], r_self, frozen[0]))
        .dot(n);
        if closing >= 0.0 {
            return false;
        }
        // An admitted closing impact is wake evidence; real mass/inertia (and the parked
        // body's skipped forces) are active before its impulse.
        if self.bodies[j].sleeping && self.bodies[j].movable() {
            let id = self.bodies[j].order;
            let woke = self.wake_contact_island(id, clock.h, report);
            report.woken_bodies += woke;
        }
        let body = &self.bodies[i];
        let partner = &self.bodies[j];
        let relative = ccd_contact_velocity(partner, r_other, frozen[1])
            - ccd_contact_velocity(body, r_self, frozen[0]);
        let approach = relative.dot(n).min(0.0);
        self.transaction.body(i, body);
        self.transaction.body(j, partner);
        let pair = [(body, &self.responses[i]), (partner, &self.responses[j])];
        let arms = [r_self, r_other];
        let response = [!frozen[0], !frozen[1]];
        let k = effective_mass(pair, arms, n, response, true, report);
        if k > 1e-14 {
            let restitution = if approach < -self.restitution_speed {
                super::pair_restitution(body, partner)
            } else {
                0.0
            };
            let normal = -(1.0 + restitution) * approach / k;
            let tangential = relative - n * approach;
            let mut impulse = n * normal;
            let mu = super::pair_friction(body, partner);
            if mu > 0.0 && tangential.length() > 0.0 {
                let t = tangential.unit();
                let kt = effective_mass(pair, arms, t, response, true, report);
                if kt > 1e-14 {
                    impulse -= t * (tangential.length() / kt).min(mu * normal);
                }
            }
            let before = [self.bodies[i].velocity, self.bodies[j].velocity];
            for (index, arm, impulse, stopped) in [
                (i, r_self, -impulse, frozen[0]),
                (j, r_other, impulse, frozen[1]),
            ] {
                if stopped {
                    continue;
                }
                let b = &mut self.bodies[index];
                let prepared = self.responses[index];
                b.velocity += impulse * prepared.inverse_mass;
                b.angular_velocity += prepared.inertia(arm.cross(impulse), report);
            }
            for (index, velocity) in [(i, before[0]), (j, before[1])] {
                let change = self.bodies[index].velocity - velocity;
                self.bookkeeping.ccd_offsets[index] -= change * (clock.elapsed * clock.h);
            }
            if let Some(start) = self.bookkeeping.ccd_start.get_mut(j) {
                *start = start.max(clock.elapsed);
            }
            // `contact_points` counts solver rows; a remaining-time impact is not one.
            report.swept_contacts += 1;
            let (a, b) = if self.bodies[i].order < self.bodies[j].order {
                (&self.bodies[i], &self.bodies[j])
            } else {
                (&self.bodies[j], &self.bodies[i])
            };
            if !self.cache.contains_key(&(a.order, b.order)) {
                self.contact_changes
                    .extend(super::ContactChange::impact(a, b));
            }
            let s = &mut self.bookkeeping;
            for index in [i, j] {
                if self.bodies[index].retire_on_impact {
                    push(&mut s.ccd_retired, self.bodies[index].order, &mut s.work);
                }
            }
        }
        true
    }
}
