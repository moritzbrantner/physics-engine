//! The step core. `World::step` runs `advance_substeps`, whose substeps go through named
//! phases: prepare (responses, impulses, forces), contacts (manifolds, contact wakes, CCD),
//! constraint rows, solve, contact cache, integrate, position correction, sleep and
//! retirement. The step then commits or rolls back its transaction and syncs the query tree.
use super::bookkeeping::push;
use super::{
    Body, BodyId, BookkeepingStats, CachedPoint, Constraint, Convergence, ConvergenceScope, Error,
    FailedStepWork, Order, PreparedResponse, Real, Report, Shape, TransactionStats, Vector, World,
    bookkeeping, ccd, contact, convergence, drop_boxes_near, islands, narrow_time, pair_friction,
    pair_restitution, reserve, substep_length, widen_time,
};
use crate::numeric::Scalar;

/// A substep's contact pairs: storage indices in pair order and their manifold.
type Pairs = Vec<(usize, usize, contact::Manifold)>;
/// Steps of motion a new broad-phase box (`bookkeeping::BroadBox`) leaves room for (#412).
const BOX_STEPS: Real = 4.0;

impl World {
    pub(super) fn wake_contact_island(&mut self, root: Order, h: Real, report: &mut Report) -> u64 {
        let woke = self.wake_island(root);
        for n in 0..self.bookkeeping.traversal.island.len() {
            let i = self.bookkeeping.traversal.island[n];
            let s = &mut self.bookkeeping;
            push(&mut s.stepped, i, &mut s.work);
            self.responses[i] = PreparedResponse::new(&self.bodies[i], report);
            // These bodies skipped the ordinary force phase while parked.
            self.apply_substep_forces(i, h, report);
        }
        woke
    }
    /// Advance up to 0.1 seconds in a bounded number of substeps. No dropped remainder.
    /// A returned error preserves physical state/history, pending inputs, time
    /// and the previous successful report. Panics are outside this guarantee.
    pub fn step(&mut self, dt: Scalar) -> Result<Report, Error> {
        self.failed_work = None;
        self.last_transaction = TransactionStats {
            vector_capacity_bytes: self.transaction.capacity_bytes(),
            ..TransactionStats::default()
        };
        if !dt.is_finite() || !(0.0..=0.1).contains(&dt) {
            return Err(Error::InvalidInput);
        }
        if dt == 0.0 {
            self.last_report = Report::default();
            self.last_report.islands.scratch_retained_bytes = self.island_scratch.retained_bytes();
            self.last_report.geometry.retained_bytes = self.geometry.retained_bytes() as u64;
            self.last_report.position.scratch_retained_bytes =
                self.bookkeeping.position.retained_bytes();
            return Ok(self.last_report.clone());
        }
        if substep_length(dt, self.scene.substeps) == 0.0 {
            return Err(Error::InvalidInput);
        }
        let mut report = Report::default();
        if self.is_quiescent() {
            self.elapsed += dt;
            self.finish_bookkeeping(&mut report);
            self.last_report = report.clone();
            return Ok(report);
        }
        // The solver iterates positions as body ranks.
        self.settle_storage();
        let layout = self.bookkeeping.generation;
        self.transaction.begin(self.bodies.len());
        let prior_h = self.last_h;
        let prior_activity = self.bookkeeping.activity.count;
        let result = self.advance_substeps(dt);
        self.last_transaction = self.transaction.stats(result.is_err());
        if let Some(work) = &mut self.failed_work {
            work.transaction = self.last_transaction;
        }
        let result = match result {
            Ok(report) => {
                self.transaction.commit();
                self.last_report = report.clone();
                Ok(report)
            }
            Err(error) => {
                let mut journal = std::mem::take(&mut self.transaction);
                journal.rollback(self);
                self.transaction = journal;
                self.last_h = prior_h;
                self.bookkeeping.activity.count = prior_activity;
                Err(error)
            }
        };
        self.sync_query_tree(layout, dt);
        result
    }
    /// Commit phase, after the transaction settled: query-tree leaves follow the step's bodies.
    fn sync_query_tree(&mut self, layout: u64, dt: Scalar) {
        let generation = self.bookkeeping.generation;
        let moved = if self.bookkeeping.generation != layout
            || !self.query_tree.current(self.bookkeeping.generation)
        {
            // Every leaf returns to its body's bounds.
            if self.bookkeeping.generation != layout {
                self.reindex_slots();
            }
            self.bookkeeping.forget_boxes();
            None
        } else {
            // Bodies awake at some point of the step, sorted for a canonical refit order. An
            // awake body whose box still holds its bounds keeps the box in its leaf for the
            // next step; any other leaf returns to its body's bounds.
            let s = &mut self.bookkeeping;
            let epoch = s.box_epoch();
            s.stepped.sort_unstable();
            s.stepped.dedup();
            let (bodies, broad) = (&self.bodies, &mut s.broad);
            s.stepped.retain(|&i| {
                let b = &bodies[i];
                let Some(e) = broad.get_mut(b.id.slot() as usize) else {
                    return true;
                };
                let (lo, hi) = b.cached_bounds;
                let keep = b.mass > 0.0
                    && !b.sleeping
                    && e.epoch == epoch
                    && e.lo.0 <= lo.0
                    && e.lo.1 <= lo.1
                    && e.lo.2 <= lo.2
                    && hi.0 <= e.hi.0
                    && hi.1 <= e.hi.1
                    && hi.2 <= e.hi.2;
                if !keep {
                    e.epoch = 0;
                }
                !keep
            });
            Some(s.stepped.as_slice())
        };
        self.query_tree
            .sync_step(&self.bodies, moved, generation, dt as Real);
    }
    fn advance_substeps(&mut self, dt: Scalar) -> Result<Report, Error> {
        let mut report = Report::default();
        let h = substep_length(dt, self.scene.substeps);
        let layout = self.bookkeeping.generation;
        self.begin_substeps();
        // Contacts are generated once per step and their constraints reused by later substeps,
        // as Rapier does (#378). A substep regenerates only after something the reused rows
        // cannot represent: a sweep or CCD advance, a wake, a sleep or a retirement.
        let mut reuse = false;
        for substep in 0..self.scene.substeps {
            let lookahead = h * Real::from(self.scene.substeps - 1 - substep);
            report.substeps += 1;
            self.prepare_substep(substep == 0, reuse, h, &mut report);
            let mut pairs = std::mem::take(&mut self.manifold_scratch);
            let mut reuse_next = reuse;
            if !reuse {
                match self.contacts(h, lookahead, &mut pairs, &mut report) {
                    Ok(next) => reuse_next = next,
                    Err(error) => {
                        self.manifold_scratch = pairs;
                        return Err(self.fail(&report, error));
                    }
                }
            }
            let mut constraints = std::mem::take(&mut self.constraints);
            if reuse {
                self.advance_constraints(&mut constraints, h, &mut report);
            } else {
                self.build_constraints(&mut pairs, &mut constraints, h, &mut report);
            }
            self.manifold_scratch = pairs;
            self.solve_constraints(&mut constraints, &mut report);
            self.cache_contacts(&constraints, &mut report);
            self.integrate(h, &mut report)?;
            self.constraints = constraints;
            if let Err(error) = self.correct_positions(h, &mut report.position) {
                return Err(self.fail(&report, error));
            }
            let active = self.bookkeeping.activity.count;
            self.sleep_quiet_islands();
            reuse_next &= self.bookkeeping.activity.count == active;
            self.retire_bodies(&mut report);
            // Reused rows hold storage indices: a retirement shifts them.
            reuse = reuse_next && self.bookkeeping.retired.is_empty();
            self.last_h = widen_time(h);
        }
        self.clear_forces(layout);
        self.elapsed += dt;
        self.finish_bookkeeping(&mut report);
        Ok(report)
    }
    /// Records the failed step's work and returns its error.
    fn fail(&mut self, report: &Report, error: Error) -> Error {
        self.failed_work = Some(FailedStepWork::capture(
            report,
            BookkeepingStats {
                scratch_retained_bytes: self.bookkeeping.retained_bytes() as u64,
                ..self.bookkeeping.work
            },
        ));
        error
    }
    /// Step setup: every per-body loop of the substeps runs over the awake bodies only (#412).
    fn begin_substeps(&mut self) {
        let s = &mut self.bookkeeping;
        s.activity.refresh(&self.bodies, &mut s.work);
        s.stepped.clear();
        reserve(&mut s.stepped, s.activity.indices.len(), &mut s.work);
        s.stepped.extend_from_slice(&s.activity.indices);
        // Replaced boxes leave their leaves behind in `broad_slots`: keep only the awake
        // bodies' boxes once that garbage outgrows them.
        let epoch = s.box_epoch();
        let live: usize = s
            .activity
            .indices
            .iter()
            .map(|&i| {
                let e = s.broad.get(self.bodies[i].id.slot() as usize);
                e.filter(|e| e.epoch == epoch).map_or(0, |e| e.len as usize)
            })
            .sum();
        if s.broad_slots.len() > 1024.max(2 * live) {
            let mut spare = std::mem::take(&mut s.broad_spare);
            spare.clear();
            reserve(&mut spare, live, &mut s.work);
            for &i in &s.activity.indices {
                let Some(e) = s.broad.get_mut(self.bodies[i].id.slot() as usize) else {
                    continue;
                };
                if e.epoch == epoch {
                    let start = spare.len();
                    spare.extend_from_slice(
                        &s.broad_slots[e.start as usize..(e.start + e.len) as usize],
                    );
                    e.start = start as u32;
                }
            }
            // Resting bodies hold no box (they dropped it on falling asleep).
            s.broad_spare = std::mem::replace(&mut s.broad_slots, spare);
        }
    }
    /// Prepare phase: responses (unless rows are reused), the step's external impulses on the
    /// first substep, then this substep's forces.
    fn prepare_substep(&mut self, first: bool, reuse: bool, h: Real, report: &mut Report) {
        self.bookkeeping.ccd.clear();
        self.bookkeeping.ccd_offsets.clear();
        self.bookkeeping.ccd_start.clear();
        self.bookkeeping.ccd_retired.clear();
        self.rest_per_body();
        let awake = self.bookkeeping.activity.indices.len();
        // Responses are prepared with the contacts: reused rows keep the step's response,
        // as their masses do (#378). Resting bodies keep the default response.
        if !reuse {
            crate::work::add(crate::work::Counter::BodyVisits, awake as u64);
            for &i in &self.bookkeeping.activity.indices {
                self.responses[i] = PreparedResponse::new(&self.bodies[i], report);
            }
        }
        // External impulses still apply exactly once, before the first force integration.
        // Only awake bodies hold them: `apply_impulse` wakes its body.
        if first {
            crate::work::add(crate::work::Counter::BodyVisits, awake as u64);
            for &i in &self.bookkeeping.activity.indices {
                let b = &mut self.bodies[i];
                if b.movable() || b.impulse != Vector::ZERO || b.angular_impulse != Vector::ZERO {
                    self.transaction.body(i, b);
                }
                if b.movable() {
                    let prepared = self.responses[i];
                    b.velocity += b.impulse * prepared.inverse_mass;
                    b.angular_velocity += prepared.inertia(b, b.angular_impulse, report);
                }
                b.impulse = Vector::ZERO;
                b.angular_impulse = Vector::ZERO;
            }
        }
        crate::work::add(crate::work::Counter::BodyVisits, awake as u64);
        for q in 0..awake {
            let i = self.bookkeeping.activity.indices[q];
            if self.bodies[i].movable() {
                self.apply_substep_forces(i, h, report);
            }
        }
    }
    /// Contacts phase: this substep's manifolds into `pairs`, wakes the islands they disturb
    /// (regenerating the manifolds after a wake), then advances CCD bodies through their
    /// remaining time. Returns whether the next substep may reuse this substep's rows.
    fn contacts(
        &mut self,
        h: Real,
        lookahead: Real,
        pairs: &mut Pairs,
        report: &mut Report,
    ) -> Result<bool, Error> {
        let regenerate = self.manifolds(h, lookahead, report, pairs)?;
        let mut roots = std::mem::take(&mut self.bookkeeping.roots);
        roots.clear();
        reserve(&mut roots, pairs.len() * 2, &mut self.bookkeeping.work);
        let up = (-self.scene.gravity).unit();
        roots.extend(
            pairs
                .iter()
                // Only a sleeping side can be a root.
                .filter(|(i, j, _)| self.bodies[*i].sleeping || self.bodies[*j].sleeping)
                .flat_map(|(i, j, m)| {
                    let a = &self.bodies[*i];
                    let b = &self.bodies[*j];
                    let approach = -(b.velocity - a.velocity).dot(m.normal);
                    // A separating command alone is not wake evidence. An admitted
                    // previously load-bearing support moving away is a dependency loss.
                    let departing_support = approach < 0.0
                        && ((a.external && b.sleeping && m.normal.dot(up) > 0.5)
                            || (b.external && a.sleeping && m.normal.dot(up) < -0.5))
                        && self.cache.get(&(a.order, b.order)).is_some_and(|points| {
                            points
                                .iter()
                                .any(|p| p.impulse > 0.0 && p.normal.dot(m.normal) > 0.99)
                        });
                    let driven_contact_motion = (a.sleeping || b.sleeping)
                        && (a.external || b.external)
                        && (&m.points).into_iter().any(|point| {
                            let spin = a.linear_support.is_none() && b.linear_support.is_none();
                            let relative = contact_velocity(b, point.rb, spin)
                                - contact_velocity(a, point.ra, spin);
                            relative != Vector::ZERO && relative.dot(m.normal) <= 0.0
                        });
                    let disruptive = m.swept
                        || approach > self.scene.sleep_speed
                        || ((a.external || b.external) && approach > 0.0)
                        || departing_support
                        || driven_contact_motion;
                    [
                        if disruptive && a.sleeping && b.mass > 0.0 {
                            Some(a.order)
                        } else {
                            None
                        },
                        if disruptive && b.sleeping && a.mass > 0.0 {
                            Some(b.order)
                        } else {
                            None
                        },
                    ]
                })
                .flatten(),
        );
        let mut woke = 0;
        for root in roots.drain(..) {
            woke += self.wake_contact_island(root, h, report);
        }
        self.bookkeeping.roots = roots;
        report.woken_bodies += woke;
        if woke > 0 {
            // Real response and forces are ready before the regenerated contact impulse.
            self.manifolds(h, lookahead, report, pairs)?;
        }
        // CCD remaining-time impacts respond before the contact solve, so a struck body's
        // current contacts (e.g. its floor) still resist the impulse in this substep.
        {
            let s = &mut self.bookkeeping;
            for (i, j, m) in pairs.iter() {
                if m.swept && ccd::deferred(&self.bodies[*i], &self.bodies[*j]) {
                    for k in [*i, *j] {
                        if ccd::advances(&self.bodies[k]) {
                            push(&mut s.ccd, k, &mut s.work);
                        }
                    }
                }
            }
        }
        if !self.bookkeeping.ccd.is_empty() {
            self.advance_ccd_bodies(h, report)?;
        }
        Ok(lookahead > 0.0
            && !regenerate
            && woke == 0
            && self.bookkeeping.ccd.is_empty()
            && !pairs.iter().any(|&(i, j, ref m)| {
                // A tumbling corner can reach a feature this manifold lacks.
                let tumbles = |b: &Body| {
                    !matches!(b.shape, Shape::Sphere(_))
                        && b.angular_velocity.length() * b.shape.radius() * lookahead
                            > self.scene.contact_slop
                };
                m.swept || tumbles(&self.bodies[i]) || tumbles(&self.bodies[j])
            }))
    }
    /// Constraint phase: fresh rows for this substep's pairs, warm-started from the contact
    /// cache. Drains `pairs`.
    fn build_constraints(
        &mut self,
        pairs: &mut Pairs,
        constraints: &mut Vec<Constraint>,
        h: Real,
        report: &mut Report,
    ) {
        constraints.clear();
        reserve(
            constraints,
            pairs.iter().map(|(_, _, m)| m.points.len()).sum(),
            &mut self.bookkeeping.work,
        );
        // Pairs and cache entries are both in pair order: a cursor finds each pair's history.
        let mut history = self.cache.iter().peekable();
        let mut last_history = None;
        for (i, j, m) in pairs.drain(..) {
            // Remaining-time advancement already resolved every contact of an advanced CCD
            // body along its actual path; start-pose constraints would act on its response
            // velocity a second time.
            if (m.swept && ccd::deferred(&self.bodies[i], &self.bodies[j]))
                || self.bookkeeping.ccd.binary_search(&i).is_ok()
                || self.bookkeeping.ccd.binary_search(&j).is_ok()
            {
                continue;
            }
            let a = &self.bodies[i];
            let b = &self.bodies[j];
            let n = m.normal;
            let linear = a.linear_support.is_some() || b.linear_support.is_some();
            let mut response = [true, true];
            if let Some(s) = a.linear_support
                && n.dot(s.unit()) > 0.5
            {
                response[1] = false;
            }
            if let Some(s) = b.linear_support
                && (-n).dot(s.unit()) > 0.5
            {
                response[0] = false;
            }
            // A side without prepared inverse mass takes no impulse; skipping it changes no value.
            let prepared = [self.responses[i], self.responses[j]];
            response[0] &= prepared[0].inverse_mass > 0.0;
            response[1] &= prepared[1].inverse_mass > 0.0;
            // Without rotational response every direction has the same effective mass.
            let spins = !linear
                && ((response[0] && prepared[0].spins()) || (response[1] && prepared[1].spins()));
            // Rows read and write angular velocity only when a side can spin in response or
            // already spins (#375): upright crates on a fixed floor skip the cross products,
            // whose zero terms would change at most the sign of a zero.
            let row_spin = !linear
                && (prepared[0].spins()
                    || prepared[1].spins()
                    || a.angular_velocity != Vector::ZERO
                    || b.angular_velocity != Vector::ZERO);
            let t1 = if n.0.abs() < 0.7 {
                n.cross(Vector::X).unit()
            } else {
                n.cross(Vector::Y).unit()
            };
            let t2 = n.cross(t1);
            let key = (a.order, b.order);
            // A pair with several manifolds (a box on heightfield triangles) shares
            // one cache entry: its later manifolds read the same history.
            let cached = match last_history {
                Some((last, points)) if last == key => points,
                _ => {
                    while history.next_if(|(k, _)| **k < key).is_some() {}
                    history
                        .next_if(|(k, _)| **k == key)
                        .map(|(_, points)| points)
                }
            };
            last_history = Some((key, cached));
            // A cached point warm-starts only within this reach. A half-space has zero
            // radius, so its contacts never match and skip the search.
            let warm_reach = 0.1 * a.shape.radius().min(b.shape.radius());
            let used = &mut self.bookkeeping.used;
            used.clear();
            reserve(used, m.points.len(), &mut self.bookkeeping.work);
            for point in &m.points {
                let mut point = *point;
                if !m.swept
                    && !linear
                    && a.movable()
                    && b.movable()
                    && !a.sleeping
                    && !b.sleeping
                    && response == [true, true]
                {
                    // Opposite impulses at separate witnesses create an internal
                    // friction couple. Reciprocal dynamic current contacts share
                    // the midpoint along the contact normal. Projecting the witness
                    // gap avoids feeding tangential reconstruction roundoff into
                    // resting friction rows; separation still owns the target.
                    let delta = (b.position - a.position) + point.rb - point.ra;
                    let gap = if matches!((a.shape, b.shape), (Shape::Box(_), Shape::Box(_))) {
                        // Box clipping projects witnesses along this normal;
                        // generic primitive support vertices need the full gap.
                        n * delta.dot(n)
                    } else {
                        delta
                    };
                    let shift = gap * 0.5;
                    point.ra += shift;
                    point.rb -= shift;
                }
                let response_bodies = [(a, &self.responses[i]), (b, &self.responses[j])];
                let arms = [point.ra, point.rb];
                let k = effective_mass(response_bodies, arms, n, response, !linear, report);
                if k <= 1e-14 {
                    continue;
                }
                let rel =
                    contact_velocity(b, point.rb, !linear) - contact_velocity(a, point.ra, !linear);
                let restitution = if cached.is_none() {
                    pair_restitution(a, b)
                } else {
                    0.0
                };
                let bias = contact_bias(
                    point.separation,
                    restitution,
                    rel.dot(n),
                    h,
                    self.scene.contact_slop,
                    self.restitution_speed,
                );
                let mut c = Constraint {
                    a: i,
                    b: j,
                    n,
                    ra: point.ra,
                    rb: point.rb,
                    t1,
                    t2,
                    normal_mass: 1.0 / k,
                    tangent_mass: if spins {
                        [
                            effective_mass(response_bodies, arms, t1, response, !linear, report)
                                .recip(),
                            effective_mass(response_bodies, arms, t2, response, !linear, report)
                                .recip(),
                        ]
                    } else {
                        [k.recip(); 2]
                    },
                    bias,
                    // Additional corners can limit approach while still separated at the
                    // admitted contact time. They have no frictional load yet.
                    friction: if linear
                        || (contact::fixed_box_interval(a, b)
                            && point.separation + (b.velocity - a.velocity).dot(n) * h * m.time
                                > self.scene.contact_slop)
                    {
                        0.0
                    } else {
                        pair_friction(a, b)
                    },
                    normal_impulse: 0.0,
                    tangent_impulse: [0.0; 2],
                    sep: point.separation,
                    swept: m.swept,
                    response,
                    spin: row_spin,
                    offset: b.position - a.position,
                    restitution,
                };
                if self.scene.warm_start
                    && warm_reach > 0.0
                    && point.separation <= self.scene.contact_slop
                    && self.last_h > 0.0
                    && let Some(old) = cached
                {
                    let la = a.orientation.inverse_rotate(c.ra);
                    let lb = b.orientation.inverse_rotate(c.rb);
                    // The nearest unused cached point with this normal; ties keep the first.
                    let mut nearest: Option<(usize, &CachedPoint, Real)> = None;
                    for (idx, p) in old.iter().enumerate() {
                        if used.contains(&idx) || p.normal.dot(n) <= 0.99 {
                            continue;
                        }
                        let d = (p.a - la).dot(p.a - la) + (p.b - lb).dot(p.b - lb);
                        if nearest.is_none_or(|(_, _, best)| d.total_cmp(&best).is_lt()) {
                            nearest = Some((idx, p, d));
                        }
                    }
                    if let Some((idx, p, _)) = nearest
                        && (p.a - la).length() + (p.b - lb).length() < warm_reach
                    {
                        used.push(idx);
                        let scale = (h / narrow_time(self.last_h)).clamp(0.0, 2.0);
                        c.normal_impulse = p.impulse * scale;
                        c.tangent_impulse = [p.tangent.dot(t1) * scale, p.tangent.dot(t2) * scale];
                    }
                }
                report.max_penetration = report.max_penetration.max((-point.separation).max(0.0));
                constraints.push(c);
            }
        }
    }
    /// Solve phase: warm-start impulses, then the velocity iterations.
    fn solve_constraints(&mut self, constraints: &mut [Constraint], report: &mut Report) {
        report.contact_points += constraints.len() as u64;
        for c in constraints.iter() {
            self.transaction.body(c.a, &self.bodies[c.a]);
            self.transaction.body(c.b, &self.bodies[c.b]);
            apply(
                &mut self.bodies,
                &self.responses,
                c,
                c.n * c.normal_impulse + c.t1 * c.tangent_impulse[0] + c.t2 * c.tangent_impulse[1],
                report,
            );
        }
        match self.scene.convergence {
            Some(tolerances)
                if self.scene.convergence_scope == ConvergenceScope::ContactIslands =>
            {
                islands::solve(
                    &mut self.bodies,
                    &self.responses,
                    constraints,
                    self.scene.velocity_iterations,
                    tolerances,
                    &mut self.island_scratch,
                    report,
                )
            }
            Some(tolerances) => convergence::solve::<true>(
                &mut self.bodies,
                &self.responses,
                constraints,
                self.scene.velocity_iterations,
                tolerances,
                report,
            ),
            None => convergence::solve::<false>(
                &mut self.bodies,
                &self.responses,
                constraints,
                self.scene.velocity_iterations,
                Convergence::default(),
                report,
            ),
        }
    }
    /// Contact cache phase: the solved rows' impulses become the cache's warm-start history,
    /// and support propagates from kinematic and resting bodies.
    fn cache_contacts(&mut self, constraints: &[Constraint], report: &mut Report) {
        let scratch = &mut self.bookkeeping;
        scratch.retired.clear();
        scratch.support_edges.clear();
        // Immovable and sleeping bodies stay supported (`Scratch::per_body`); awake ones
        // start from whether they are kinematic at rest.
        scratch.activity.refresh(&self.bodies, &mut scratch.work);
        crate::work::add(
            crate::work::Counter::BodyVisits,
            scratch.activity.indices.len() as u64,
        );
        for &i in &scratch.activity.indices {
            let b = &self.bodies[i];
            scratch.supported[i] =
                b.external && b.velocity == Vector::ZERO && b.angular_velocity == Vector::ZERO;
        }
        // Reuse point allocations when the pair survives. Warm starting has already read the
        // old impulses; only adjacency-key changes invalidate the graph, not updated impulses.
        // Cache entries and a pair's contiguous constraint run are both in pair order: walk
        // them together, so neither side searches the other (#375). New pairs are inserted
        // after the walk.
        let up = (-self.scene.gravity).unit();
        debug_assert!(
            constraints.is_sorted_by_key(|c| { (self.bodies[c.a].order, self.bodies[c.b].order) })
        );
        let mut runs = constraints
            .chunk_by(|x, y| x.a == y.a && x.b == y.b)
            .peekable();
        let mut entries = self.cache.iter_mut().peekable();
        let mut fresh = Vec::new();
        loop {
            let run_key = runs
                .peek()
                .map(|run| (self.bodies[run[0].a].order, self.bodies[run[0].b].order));
            let entry_key = entries.peek().map(|(key, _)| **key);
            let (run, entry) = match (run_key, entry_key) {
                (None, None) => break,
                (Some(r), Some(e)) if r == e => (runs.next(), entries.next()),
                (Some(r), e) if e.is_none_or(|e| r < e) => (runs.next(), None),
                _ => (None, entries.next()),
            };
            let entry = entry.map(|(&key, points)| {
                let (a, b) = match run {
                    Some(run) => (run[0].a, run[0].b),
                    None => (
                        self.bodies
                            .binary_search_by_key(&key.0, |b| b.order)
                            .expect("live contact"),
                        self.bodies
                            .binary_search_by_key(&key.1, |b| b.order)
                            .expect("live contact"),
                    ),
                };
                if !scratch.supported[a] || !scratch.supported[b] {
                    if !points.is_empty() {
                        self.transaction.pair(key, Some(points));
                    }
                    points.clear();
                }
                points
            });
            let Some(run) = run else {
                continue;
            };
            let a = &self.bodies[run[0].a];
            let b = &self.bodies[run[0].b];
            let key = (a.order, b.order);
            let cached = run.iter().any(|c| c.sep <= self.scene.contact_slop * 2.0);
            let mut new = None;
            let mut points = match entry {
                _ if !cached => None,
                Some(points) => {
                    self.transaction.pair(key, Some(points));
                    Some(points)
                }
                None => {
                    self.transaction.pair(key, None);
                    scratch.graph.dirty = true;
                    Some(new.insert(Vec::new()))
                }
            };
            for c in run {
                if c.normal_impulse > 0.0 {
                    if a.retire_on_impact {
                        push(&mut scratch.retired, a.order, &mut scratch.work);
                    }
                    if b.retire_on_impact {
                        push(&mut scratch.retired, b.order, &mut scratch.work);
                    }
                    if c.swept {
                        report.swept_contacts += 1;
                    }
                }
                if c.sep <= self.scene.contact_slop * 2.0
                    && let Some(points) = points.as_deref_mut()
                {
                    points.push(CachedPoint {
                        a: a.orientation.inverse_rotate(c.ra),
                        b: b.orientation.inverse_rotate(c.rb),
                        normal: c.n,
                        impulse: c.normal_impulse,
                        tangent: c.t1 * c.tangent_impulse[0] + c.t2 * c.tangent_impulse[1],
                    });
                    if c.n.dot(up) > 0.5 {
                        push(&mut scratch.support_edges, (c.a, c.b), &mut scratch.work);
                    }
                    if c.n.dot(up) < -0.5 {
                        push(&mut scratch.support_edges, (c.b, c.a), &mut scratch.work);
                    }
                }
            }
            if let Some(points) = new {
                fresh.push((key, points));
            }
        }
        self.cache.extend(fresh);
        self.cache.retain(|&key, points| {
            if points.is_empty() {
                self.transaction.pair(key, Some(points));
                scratch.graph.dirty = true;
                false
            } else {
                true
            }
        });
        let scratch = &mut self.bookkeeping;
        scratch.support.propagate(
            &mut scratch.supported,
            &scratch.support_edges,
            &mut scratch.work,
        );
    }
    /// Integrate phase: awake bodies move by their solved velocities (and CCD offsets), and
    /// their quiet time and bounds update.
    fn integrate(&mut self, h: Real, report: &mut Report) -> Result<(), Error> {
        let scratch = &mut self.bookkeeping;
        scratch.activity.refresh(&self.bodies, &mut scratch.work);
        crate::work::add(
            crate::work::Counter::BodyVisits,
            scratch.activity.indices.len() as u64,
        );
        for &i in &scratch.activity.indices {
            let b = &mut self.bodies[i];
            self.transaction.body(i, b);
            report.integrated_bodies += 1;
            let (movement, spin, correction_quiet) = (b.velocity, b.angular_velocity, true);
            b.position += movement * h;
            if let Some(offset) = scratch.ccd_offsets.get(i) {
                b.position += *offset;
            }
            if !b.rotation_locked {
                b.orientation = b.orientation.integrate(spin, h);
            }
            debug_assert_eq!(b.motion_valid(), b.valid());
            if !b.motion_valid() {
                let error = Error::NonFiniteState(b.id);
                self.failed_work = Some(FailedStepWork::capture(
                    report,
                    BookkeepingStats {
                        scratch_retained_bytes: scratch.retained_bytes() as u64,
                        ..scratch.work
                    },
                ));
                return Err(error);
            }
            if b.movable()
                && b.sleep_allowed
                && scratch.supported[i]
                && correction_quiet
                && b.velocity.length() + b.angular_velocity.length() * b.shape.radius()
                    < self.scene.sleep_speed
            {
                b.quiet_time += h;
            } else {
                b.quiet_time = 0.0;
            }
            b.cached_bounds = contact::bounds(b);
        }
        scratch.ccd_offsets.clear();
        Ok(())
    }
    /// Retirement phase: bodies retired on impact this substep leave the world.
    fn retire_bodies(&mut self, report: &mut Report) {
        // Retirement is local; remove all affected cached edges before indexed graph reuse.
        let s = &mut self.bookkeeping;
        for n in 0..s.ccd_retired.len() {
            push(&mut s.retired, s.ccd_retired[n], &mut s.work);
        }
        s.ccd_retired.clear();
        self.bookkeeping.retired.sort_unstable();
        self.bookkeeping.retired.dedup();
        for n in 0..self.bookkeeping.retired.len() {
            let id = self.bookkeeping.retired[n];
            if let Some(i) = self.index_of_order(id) {
                let b = self.bodies.remove(i);
                let handle = b.id;
                self.bookkeeping.activity.count -= usize::from(b.mass > 0.0 && !b.sleeping);
                self.geometry.remove(id);
                self.cache.retain(|&(a, b), points| {
                    if a == id || b == id {
                        self.transaction.pair((a, b), Some(points));
                        false
                    } else {
                        true
                    }
                });
                self.transaction.remove(i, b);
                // Later substeps look bodies up by query-tree slot.
                self.shift_slots(i);
                let synced = self.query_tree.current(self.bookkeeping.generation);
                self.bookkeeping.layout_changed();
                if synced {
                    self.query_tree
                        .removed(handle.slot(), self.bookkeeping.generation);
                } else {
                    self.bookkeeping.forget_boxes();
                }
                self.bookkeeping.drop_box(handle.slot());
                report.retired.push(handle);
            }
        }
    }
    /// Forces and torques clear after the last substep. Only bodies awake at the step's start
    /// hold forces: `add_force` wakes its body.
    fn clear_forces(&mut self, layout: u64) {
        let s = &mut self.bookkeeping;
        let all = s.generation != layout;
        let count = if all {
            self.bodies.len()
        } else {
            s.stepped.len()
        };
        crate::work::add(crate::work::Counter::BodyVisits, count as u64);
        for q in 0..count {
            let i = if all { q } else { s.stepped[q] };
            let b = &mut self.bodies[i];
            if b.force != Vector::ZERO || b.torque != Vector::ZERO {
                self.transaction.body(i, b);
            }
            b.force = Vector::ZERO;
            b.torque = Vector::ZERO;
        }
    }
    /// Carries the step's constraint rows into the next substep (#378). Normals, arms, masses and
    /// friction stay as generated; each row's separation advances by the bodies' relative
    /// motion along the normal and the position target follows it. Translation is measured
    /// (it includes position correction); rotation is linearised at the fixed arms, so a
    /// rolling sphere's support stays exact. The accumulated impulses stay on the rows and
    /// warm-start this substep.
    fn advance_constraints(&self, constraints: &mut [Constraint], h: Real, report: &mut Report) {
        let slop = self.scene.contact_slop;
        for run in constraints.chunk_by_mut(|x, y| x.a == y.a && x.b == y.b) {
            // A pair stops bouncing once it has a cached contact, as a regenerated row would.
            let cached = run.iter().any(|c| c.sep <= slop * 2.0);
            for c in run {
                let (a, b) = (&self.bodies[c.a], &self.bodies[c.b]);
                let offset = b.position - a.position;
                c.sep += (offset - c.offset).dot(c.n);
                if c.spin {
                    c.sep += (b.angular_velocity.cross(c.rb) - a.angular_velocity.cross(c.ra))
                        .dot(c.n)
                        * h;
                }
                c.offset = offset;
                if cached {
                    c.restitution = 0.0;
                }
                let approach = if c.restitution > 0.0 {
                    (contact_velocity(b, c.rb, c.spin) - contact_velocity(a, c.ra, c.spin)).dot(c.n)
                } else {
                    0.0
                };
                c.bias = contact_bias(
                    c.sep,
                    c.restitution,
                    approach,
                    h,
                    slop,
                    self.restitution_speed,
                );
                report.max_penetration = report.max_penetration.max((-c.sep).max(0.0));
            }
        }
    }
    fn apply_substep_forces(&mut self, index: usize, h: Real, report: &mut Report) {
        let b = &mut self.bodies[index];
        self.transaction.body(index, b);
        let prepared = self.responses[index];
        b.velocity += (self.scene.gravity + b.force / b.mass) * h;
        b.angular_velocity += prepared.inertia(b, b.torque, report) * h;
        if b.linear_damping > 0.0 {
            b.velocity = b.velocity * (1.0 / (1.0 + h * b.linear_damping));
        }
        if b.angular_damping > 0.0 {
            b.angular_velocity = b.angular_velocity * (1.0 / (1.0 + h * b.angular_damping));
        }
    }
    fn finish_bookkeeping(&mut self, report: &mut Report) {
        report.islands.scratch_retained_bytes = self.island_scratch.retained_bytes();
        report.position.scratch_retained_bytes = self.bookkeeping.position.retained_bytes();
        report.geometry.retained_bytes = self.geometry.retained_bytes() as u64;
        self.bookkeeping.work.scratch_retained_bytes = (self.bookkeeping.retained_bytes()
            + bookkeeping::bytes(&self.constraints)
            + bookkeeping::bytes(&self.manifold_scratch))
            as u64;
        report.bookkeeping = std::mem::take(&mut self.bookkeeping.work);
    }
    fn sleep_quiet_islands(&mut self) {
        let s = &mut self.bookkeeping;
        s.activity.refresh(&self.bodies, &mut s.work);
        s.ensure_graph(&self.bodies, &self.cache);
        s.traversal.begin(self.bodies.len(), &mut s.work);
        let mut slept = 0;
        // Fully sleeping components need no rediscovery. An awake root still reaches every
        // dynamic neighbor, including sleeping members, so support and wake semantics are unchanged.
        for &root in &s.activity.indices {
            s.traversal
                .collect(root, &self.bodies, &s.graph, &mut s.work);
            if s.traversal.island.iter().all(|&i| {
                let b = &self.bodies[i];
                b.sleeping || (b.sleep_allowed && b.quiet_time >= self.scene.sleep_seconds)
            }) {
                for &i in &s.traversal.island {
                    let b = &mut self.bodies[i];
                    if b.sleeping {
                        continue;
                    }
                    self.transaction.body(i, b);
                    slept += 1;
                    b.sleeping = true;
                    b.velocity = Vector::ZERO;
                    b.angular_velocity = Vector::ZERO;
                    // Resting values (`Scratch::per_body`), and a leaf at the body's bounds:
                    // later substeps find it as a resting body. Boxes that overlapped its box
                    // may not hold its leaf (it may have held theirs): they query again.
                    self.responses[i] = PreparedResponse::default();
                    s.supported[i] = true;
                    let slot = b.id.slot();
                    if self.query_tree.current(s.generation) {
                        let e = s.broad.get(slot as usize).copied().unwrap_or_default();
                        let near = if e.epoch == s.broad_epoch + 1 {
                            (e.lo, e.hi)
                        } else {
                            b.cached_bounds
                        };
                        drop_boxes_near(
                            &self.query_tree,
                            &mut s.broad,
                            near,
                            self.scene.contact_slop,
                        );
                        self.query_tree.moved(b);
                    } else {
                        s.broad_epoch += 1;
                        s.boxes_live = false;
                    }
                    if let Some(e) = s.broad.get_mut(slot as usize) {
                        e.epoch = 0;
                    }
                }
            }
        }
        s.activity.count -= slept;
        if slept > 0 {
            s.activity.slept(&self.bodies);
        }
    }
    /// Gives every resting body its resting per-body values (`Scratch::per_body`) after a
    /// layout change or rollback; O(1) otherwise.
    fn rest_per_body(&mut self) {
        let s = &mut self.bookkeeping;
        s.activity.refresh(&self.bodies, &mut s.work);
        let n = self.bodies.len();
        if s.per_body == Some(s.generation)
            && self.responses.len() == n
            && s.supported.len() == n
            && s.earliest.len() == n
        {
            return;
        }
        crate::work::add(crate::work::Counter::BodyVisits, n as u64);
        self.responses.clear();
        reserve(&mut self.responses, n, &mut s.work);
        self.responses.resize(n, PreparedResponse::default());
        s.supported.clear();
        reserve(&mut s.supported, n, &mut s.work);
        s.supported.resize(n, true);
        s.earliest.clear();
        reserve(&mut s.earliest, n, &mut s.work);
        s.earliest.resize(n, Real::INFINITY);
        s.earliest_set.clear();
        s.per_body = Some(s.generation);
    }

    /// Contacts for this substep plus `lookahead`, the rest of the step: pairs within reach
    /// before the step ends are admitted as speculative contacts, so later substeps can reuse
    /// the constraints (#378). Returns whether the next substep must regenerate instead: a pair
    /// needed a swept test, which covers only this substep, or two dynamic bodies can close.
    pub(super) fn manifolds(
        &mut self,
        h: Real,
        lookahead: Real,
        report: &mut Report,
        out: &mut Vec<(usize, usize, contact::Manifold)>,
    ) -> Result<bool, Error> {
        let mut regenerate = false;
        let interval = h + lookahead;
        // Displacement per unit of constant acceleration over the later substeps: velocity
        // gains a·h per substep after this one, so the step ends a·L·(L + h)/2 further on.
        let gain = lookahead * interval * 0.5;
        let step = h * Real::from(self.scene.substeps);
        self.rest_per_body();
        let in_order = self.storage_sorted();
        let s = &mut self.bookkeeping;
        self.geometry.begin(self.bodies.len());
        out.clear();
        // Broad phase (#412), as Rapier's: the query tree holds every body, so pairs come only
        // from awake bodies and a static, sleeping or resting kinematic body is visited only when
        // an awake one reaches it. Pairs of two resting bodies produce no contact. An awake
        // body's leaf holds its box (`BroadBox`, a fat AABB with room for a few steps of motion);
        // the body queries the tree only when its swept bounds leave the box, and keeps the
        // leaves it found until then. Of two awake bodies whose bounds overlap, the one whose box
        // queried last found the other. A resting body that changes drops the boxes near it.
        if !self.query_tree.current(s.generation) {
            crate::work::add(crate::work::Counter::BodyVisits, self.bodies.len() as u64);
            self.query_tree.sync(&self.bodies, s.generation);
            s.forget_boxes();
        }
        s.activity.refresh(&self.bodies, &mut s.work);
        s.bounds.clear();
        reserve(&mut s.bounds, s.activity.indices.len(), &mut s.work);
        crate::work::add(
            crate::work::Counter::BodyVisits,
            s.activity.indices.len() as u64,
        );
        // A body's candidate bounds over the rest of the step; a resting body's are its bounds
        // padded by the contact slop.
        let swept = |b: &Body| {
            let (lo, hi) = b.cached_bounds;
            let delta = if b.sleeping {
                Vector::ZERO
            } else {
                b.velocity * interval
            };
            let angular = b.angular_velocity.length() * b.shape.radius() * interval;
            // Gravity and forces keep accelerating the body through the later substeps.
            let drift = step_drift(b, self.scene.gravity, gain).abs();
            let pad = Vector(angular, angular, angular)
                + drift
                + Vector(1.0, 1.0, 1.0)
                    * (self.scene.contact_slop
                        // Endpoint addition can round at the displacement's scale even
                        // when the original pose and collider are small.
                        + 8.0 * Real::EPSILON * delta.abs().max_component());
            (lo.min(lo + delta) - pad, hi.max(hi + delta) + pad)
        };
        let finite = |(lo, hi): (Vector, Vector)| lo.finite() && hi.finite();
        let epoch = s.box_epoch();
        if s.broad.len() < self.slots.len() {
            reserve(&mut s.broad, self.slots.len(), &mut s.work);
            s.broad
                .resize(self.slots.len(), bookkeeping::BroadBox::default());
        }
        // Room a new box leaves around the swept bounds, besides the motion it predicts.
        let room = Vector(1.0, 1.0, 1.0) * (20.0 * self.scene.contact_slop);
        let ahead = step * BOX_STEPS;
        let mut made = 0;
        for &i in &s.activity.indices {
            let b = &self.bodies[i];
            let (lo, hi) = swept(b);
            let e = &mut s.broad[b.id.slot() as usize];
            e.row = s.bounds.len() as u32;
            push(&mut s.bounds, (i, lo, hi), &mut s.work);
            s.work.bounds_updates += 1;
            // Non-finite bounds (an overflowing velocity) cannot enter the tree: that body
            // tests every body below, and its narrow phase reports the failure.
            if !finite((lo, hi)) {
                e.epoch = 0;
                continue;
            }
            if e.epoch == epoch
                && e.lo.0 <= lo.0
                && e.lo.1 <= lo.1
                && e.lo.2 <= lo.2
                && hi.0 <= e.hi.0
                && hi.1 <= e.hi.1
                && hi.2 <= e.hi.2
            {
                continue;
            }
            let d = b.velocity * ahead;
            let (bl, bh) = (lo.min(lo + d) - room, hi.max(hi + d) + room);
            if !finite((bl, bh)) {
                e.epoch = 0;
                continue;
            }
            (e.epoch, e.lo, e.hi) = (epoch, bl, bh);
            s.boxes_live = true;
            // Marks the box for a query below.
            e.len = u32::MAX;
            self.query_tree.swept(b, (bl, bh), step);
            made += 1;
        }
        // New boxes query the tree once every awake leaf holds its box. The leaves a box
        // overlaps within the contact slop are kept with it.
        let slop = Vector(1.0, 1.0, 1.0) * self.scene.contact_slop;
        let mut nodes = 0;
        if made > 0 {
            let needed = s.broad_slots.len() + made * 4;
            reserve(&mut s.broad_slots, needed, &mut s.work);
        }
        for &i in &s.activity.indices {
            let own = self.bodies[i].id.slot();
            let e = s.broad[own as usize];
            if e.epoch != epoch || e.len != u32::MAX {
                continue;
            }
            let start = s.broad_slots.len();
            nodes += self.query_tree.tree().visit_aabb(
                spatial_kernels::Aabb {
                    min: [e.lo.0 - slop.0, e.lo.1 - slop.1, e.lo.2 - slop.2],
                    max: [e.hi.0 + slop.0, e.hi.1 + slop.1, e.hi.2 + slop.2],
                },
                |leaf| {
                    if leaf.id != own {
                        push(&mut s.broad_slots, leaf.id, &mut s.work);
                    }
                },
            );
            s.queries += 1;
            let e = &mut s.broad[own as usize];
            e.query = s.queries;
            e.start = start as u32;
            e.len = (s.broad_slots.len() - start) as u32;
        }
        crate::work::add(crate::work::Counter::TreeNodes, nodes as u64);
        let mut candidates = std::mem::take(&mut s.candidates);
        candidates.clear();
        let (rows, broad) = (&s.bounds, &s.broad);
        let has_box = |b: &Body| broad[b.id.slot() as usize].epoch == epoch;
        // The candidate pair of awake `i` (bounds `lo`, `hi`) and any body `j`, reported once:
        // a pair of awake bodies by the one whose box queried last (it holds the other's leaf),
        // a pair with a body without a box by that body's scan.
        let mut consider = |i: usize, (lo, hi): (Vector, Vector), j: usize| {
            let Some(b) = self.bodies.get(j) else {
                // The slot of a body removed since the box queried.
                return;
            };
            if j == i || b.id == BodyId::NONE {
                return;
            }
            let (bl, bh) = if b.mass > 0.0 && !b.sleeping {
                let e = broad[b.id.slot() as usize];
                let (_, bl, bh) = rows[e.row as usize];
                match (has_box(&self.bodies[i]), has_box(b)) {
                    (true, true) if broad[self.bodies[i].id.slot() as usize].query < e.query => {
                        return;
                    }
                    (true, false) => return,
                    (false, false) if j < i => return,
                    _ => {}
                }
                (bl, bh)
            } else if b.velocity == Vector::ZERO && b.angular_velocity == Vector::ZERO {
                // `swept` of a body at rest.
                let (bl, bh) = b.cached_bounds;
                (bl - slop, bh + slop)
            } else {
                swept(b)
            };
            report.pair_tests += 1;
            if bl.0 > hi.0
                || bh.0 < lo.0
                || bl.1 > hi.1
                || bh.1 < lo.1
                || bl.2 > hi.2
                || bh.2 < lo.2
            {
                return;
            }
            let pair = if self.bodies[i].order < b.order {
                (i, j)
            } else {
                (j, i)
            };
            push(&mut candidates, pair, &mut s.work);
        };
        for &(i, lo, hi) in rows {
            let e = broad[self.bodies[i].id.slot() as usize];
            if e.epoch != epoch {
                crate::work::add(crate::work::Counter::BodyVisits, self.bodies.len() as u64);
                for j in 0..self.bodies.len() {
                    consider(i, (lo, hi), j);
                }
                continue;
            }
            for &slot in &s.broad_slots[e.start as usize..(e.start + e.len) as usize] {
                consider(i, (lo, hi), self.slots[slot as usize].index as usize);
            }
        }
        // Narrow phase in pair order, so the first failure and every cache visit are canonical.
        // Settled storage is in body order.
        if in_order {
            candidates.sort_unstable();
        } else {
            candidates.sort_unstable_by_key(|&(i, j)| (self.bodies[i].order, self.bodies[j].order));
        }
        debug_assert!(candidates.windows(2).all(|w| w[0] != w[1]));
        for &(i, j) in &candidates {
            {
                let a = &self.bodies[i];
                let b = &self.bodies[j];
                let driven_contact_candidate = (a.movable()
                    && b.external
                    && (b.velocity != Vector::ZERO || b.angular_velocity != Vector::ZERO))
                    || (b.movable()
                        && a.external
                        && (a.velocity != Vector::ZERO || a.angular_velocity != Vector::ZERO));
                let driven_wake_candidate = driven_contact_candidate && (a.sleeping || b.sleeping);
                // Boxes take rigid contacts with heightfield terrain, one manifold per triangle.
                // As with halfspaces, the approach over the step is the speculative margin.
                if let (Shape::Heightfield { terrain, .. }, body)
                | (body, Shape::Heightfield { terrain, .. }) = (a.shape, b.shape)
                    && let Shape::Box(_) = body
                {
                    let moving = if matches!(a.shape, Shape::Box(_)) {
                        a
                    } else {
                        b
                    };
                    if moving.inverse_mass() == 0.0
                        || moving.sensor
                        || moving.sleeping
                        || !a.layers.collides_with(b.layers)
                    {
                        continue;
                    }
                    report.narrow_tests += 1;
                    let approach = moving.velocity.length() * interval
                        + step_drift(moving, self.scene.gravity, gain).length()
                        + moving.angular_velocity.length() * moving.shape.radius() * interval;
                    contact::heightfield_box(
                        a,
                        b,
                        &self.terrains[terrain.0 as usize].prepared,
                        self.scene.contact_slop + approach,
                        interval,
                        |m| push(out, (i, j, m), &mut s.work),
                    );
                    continue;
                }
                if a.sensor
                    || b.sensor
                    // Terrain answers queries and character moves; halfspace contacts with
                    // dynamic bodies are generated after this sweep.
                    || a.shape.query_only()
                    || b.shape.query_only()
                    || !a.layers.collides_with(b.layers)
                    || (a.inverse_mass() == 0.0
                        && b.inverse_mass() == 0.0
                        && !driven_wake_candidate)
                {
                    continue;
                }
                let speed = (b.velocity - a.velocity).length();
                let travel = speed * h;
                // Retain full-interval discovery after response preparation wakes the
                // body; the second geometry pass must not discard an admitted sweep.
                // A fixed boundary may be crossed below the shape-sized fast threshold.
                // The current margin only rules out travel no larger than that margin.
                let fast = driven_contact_candidate
                    || a.ccd
                    || b.ccd
                    || ((a.mass == 0.0 || b.mass == 0.0) && travel > self.scene.contact_slop)
                    || travel
                        > 0.5
                            * a.shape
                                .half_extents()
                                .min_component()
                                .min(b.shape.half_extents().min_component());
                // A tolerance-only manifold is not wake evidence. Probe the actual surface
                // and full interval before replacing the parked body's zero response.
                let margin = if driven_wake_candidate {
                    0.0
                } else {
                    self.scene.contact_slop
                };
                // A separated pair slow enough to skip the sweep is admitted speculatively
                // when it can close before the step ends; a fast pair keeps this substep's
                // sweep instead.
                let reach = if fast || lookahead == 0.0 {
                    0.0
                } else {
                    (speed
                        + a.angular_velocity.length() * a.shape.radius()
                        + b.angular_velocity.length() * b.shape.radius())
                        * lookahead
                        + (step_drift(b, self.scene.gravity, gain)
                            - step_drift(a, self.scene.gravity, gain))
                        .length()
                };
                report.narrow_tests += 1;
                // GeometryCache shares read-only frames even for rotating pairs; it retains
                // complete pair results only when both orientation dependencies are stable.
                let current =
                    self.geometry
                        .query_interval([i, j], [a, b], h, margin, &mut report.geometry);
                let m = if current.is_some() {
                    current
                } else if fast {
                    regenerate = true;
                    self.geometry
                        .swept([i, j], [a, b], h, margin, &mut report.geometry)
                        .map_err(|reason| Error::CollisionSearchFailed {
                            bodies: [a.id, b.id],
                            reason,
                        })?
                } else if reach > 0.0 {
                    let margin = margin + reach;
                    let speculative = self.geometry.query_interval(
                        [i, j],
                        [a, b],
                        h,
                        margin,
                        &mut report.geometry,
                    );
                    if speculative.is_some() && a.mass > 0.0 && b.mass > 0.0 {
                        // Speculative rows between two dynamic bodies make stacks rock: such
                        // a pair is discovered by the next substep's regeneration instead.
                        regenerate = true;
                        None
                    } else {
                        speculative
                    }
                } else {
                    None
                };
                if let Some(m) = m {
                    push(out, (i, j, m), &mut s.work);
                }
            }
        }
        s.candidates = candidates;
        self.geometry.finish(&mut report.geometry);
        // Pairs are solved in pair order. The broad phase's pairs were tested in pair order, a
        // pair's several manifolds (a box on heightfield triangles) in generation order; each
        // halfspace's pairs below come in pair order too.
        let sorted = self.storage_sorted();
        debug_assert!(out.is_sorted_by(|x, y| pair_order(&self.bodies, sorted)(x, y).is_le()));
        let swept_pairs = out.len();
        // Halfspaces are unbounded and stay out of the sweep: each one is tested against every
        // awake dynamic body. The approach over the substep is admitted as a speculative margin,
        // so a fast body closes exactly to the plane.
        for p in 0..self.halfspaces.len() {
            let Ok(plane) = self.index(self.halfspaces[p]) else {
                continue;
            };
            let n = self.bodies[plane].orientation.rotate(Vector::Y);
            let awake = self.bookkeeping.activity.indices.len();
            crate::work::add(crate::work::Counter::BodyVisits, awake as u64);
            // Only awake dynamic bodies take halfspace contacts.
            for q in 0..awake {
                let k = self.bookkeeping.activity.indices[q];
                let (a, b) = (&self.bodies[plane], &self.bodies[k]);
                if b.id == BodyId::NONE
                    || b.inverse_mass() == 0.0
                    || b.sensor
                    || b.shape.query_only()
                    || !a.layers.collides_with(b.layers)
                {
                    continue;
                }
                report.narrow_tests += 1;
                let approach = (-b.velocity.dot(n)).max(0.0) * interval
                    + (-step_drift(b, self.scene.gravity, gain).dot(n)).max(0.0)
                    + b.angular_velocity.length() * b.shape.radius() * interval;
                let (i, j) = if a.order < b.order {
                    (plane, k)
                } else {
                    (k, plane)
                };
                if let Some(m) = contact::halfspace(
                    &self.bodies[i],
                    &self.bodies[j],
                    self.scene.contact_slop + approach,
                    interval,
                ) {
                    push(out, (i, j, m), &mut self.bookkeeping.work);
                }
            }
        }
        // A floor added before every dynamic body precedes all of their pairs: then the
        // halfspace pairs only rotate to the front.
        let order = pair_order(&self.bodies, sorted);
        let (sweep, planes) = out.split_at(swept_pairs);
        if planes.is_sorted_by(|x, y| order(x, y).is_le())
            && (sweep.is_empty()
                || planes.is_empty()
                || order(&planes[planes.len() - 1], &sweep[0]).is_lt())
        {
            out.rotate_left(swept_pairs);
        } else {
            out.sort_unstable_by(order);
        }
        let s = &mut self.bookkeeping;
        // A fast projectile's original trajectory must not activate bodies behind its first hit.
        // Keep all equal-time contacts, but defer later speculative contacts to the next substep,
        // where the already-updated velocity becomes authoritative. No event-restart loop is added.
        // `earliest` is infinity outside this pass (see `Scratch::per_body`).
        for (i, j, m) in out.iter() {
            for index in [*i, *j] {
                if self.bodies[index].ccd {
                    if s.earliest[index] == Real::INFINITY {
                        push(&mut s.earliest_set, index, &mut s.work);
                    }
                    s.earliest[index] = s.earliest[index].min(m.time);
                }
            }
        }
        let bodies = &self.bodies;
        out.retain(|(i, j, m)| {
            let keep = [i, j]
                .into_iter()
                .all(|index| m.time <= s.earliest[*index] + 1e-7);
            // A deferred later sweep is not wake evidence, but its CCD body still advances
            // through the whole substep and re-sweeps every candidate.
            if !keep && m.swept && ccd::deferred(&bodies[*i], &bodies[*j]) {
                for k in [*i, *j] {
                    if ccd::advances(&bodies[k]) {
                        push(&mut s.ccd, k, &mut s.work);
                    }
                }
            }
            keep
        });
        for &index in &s.earliest_set {
            s.earliest[index] = Real::INFINITY;
        }
        s.earliest_set.clear();
        Ok(regenerate)
    }
}
/// Pair order. Settled storage (`sorted`) is in body order, so storage positions compare
/// pairs as their body orders do.
fn pair_order(
    bodies: &[Body],
    sorted: bool,
) -> impl Fn(
    &(usize, usize, contact::Manifold),
    &(usize, usize, contact::Manifold),
) -> std::cmp::Ordering
+ '_ {
    move |&(a, b, _), &(c, d, _)| {
        if sorted {
            (a, b).cmp(&(c, d))
        } else {
            (bodies[a].order, bodies[b].order).cmp(&(bodies[c].order, bodies[d].order))
        }
    }
}
/// The normal row's velocity target. A separated (speculative) row permits closing exactly up
/// to its surface. Restitution (`restitution` is the pair's coefficient, `approach` the
/// relative normal velocity) and softness beyond the slop keep the hard response; within the
/// slop the positional target is zero, so a support keeps its load-bearing impulse.
fn contact_bias(
    sep: Real,
    restitution: Real,
    approach: Real,
    h: Real,
    slop: Real,
    restitution_speed: Real,
) -> Real {
    if sep > 0.0 {
        return -sep / h;
    }
    let bounce = if approach < -restitution_speed && sep <= slop {
        -restitution * approach
    } else {
        0.0
    };
    (0.2 * (-sep - slop).max(0.0) / h).min(60.0).max(bounce)
}
/// Extra displacement from gravity and accumulated force, which keep acting on an awake
/// dynamic body through the rest of the step; `gain` is zero on the step's last substep.
fn step_drift(b: &Body, gravity: Vector, gain: Real) -> Vector {
    if gain > 0.0 && b.movable() && !b.sleeping {
        (gravity + b.force / b.mass) * gain
    } else {
        Vector::ZERO
    }
}
#[inline]
pub(super) fn contact_velocity(b: &Body, r: Vector, spin: bool) -> Vector {
    b.velocity
        + if spin {
            b.angular_velocity.cross(r)
        } else {
            Vector::ZERO
        }
}
#[inline]
pub(super) fn effective_mass(
    bodies: [(&Body, &PreparedResponse); 2],
    arms: [Vector; 2],
    n: Vector,
    response: [bool; 2],
    spin: bool,
    report: &mut Report,
) -> Real {
    let mut k = 0.0;
    for (((body, cached), r), yes) in bodies.into_iter().zip(arms).zip(response) {
        if yes {
            let prepared = *cached;
            k += prepared.inverse_mass;
            if spin {
                k += prepared.inertia(body, r.cross(n), report).cross(r).dot(n);
            }
        }
    }
    k
}
#[inline]
pub(super) fn apply(
    bodies: &mut [Body],
    responses: &[PreparedResponse],
    c: &Constraint,
    j: Vector,
    report: &mut Report,
) {
    for (i, r, j, yes) in [
        (c.a, c.ra, -j, c.response[0]),
        (c.b, c.rb, j, c.response[1]),
    ] {
        if yes {
            let b = &mut bodies[i];
            let prepared = responses[i];
            b.velocity += j * prepared.inverse_mass;
            if c.spin {
                b.angular_velocity += prepared.inertia(b, r.cross(j), report);
            }
        }
    }
}
