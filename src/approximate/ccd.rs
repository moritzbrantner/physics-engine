//! Remaining-time response for translational CCD bodies.
//!
//! A CCD body whose substep produced an admitted swept impact is not limited by a speculative
//! constraint. Instead it advances to the independent time of impact, receives the pair's
//! normal/friction/restitution response there, and re-sweeps the rest of the substep along the
//! response velocity against every candidate. The loop is bounded by the velocity-pass budget;
//! when that is exhausted the body stays at its last impact for the rest of the substep
//! (`GeometryStats::ccd_budget_fallbacks`). Rotation and non-CCD bodies are unchanged.
use super::{
    Body, Error, Real, Report, Vector, World,
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

impl World {
    /// Advance queued CCD bodies, sharing a time-of-impact clock when they can interact.
    pub(super) fn advance_ccd_bodies(&mut self, h: Real, report: &mut Report) -> Result<(), Error> {
        let mut queued = std::mem::take(&mut self.bookkeeping.ccd);
        queued.sort_unstable();
        queued.dedup();
        let result = if queued.len() > 1 {
            self.advance_ccd_group(&queued, h, report)
        } else if let Some(&i) = queued.first() {
            self.advance_ccd_body(i, h, report)
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

    fn advance_ccd_body(&mut self, i: usize, h: Real, report: &mut Report) -> Result<(), Error> {
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
                &[],
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
            if !self.respond_ccd_impact(i, j, &m, first, TickTime { elapsed, h }, report) {
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
    fn advance_ccd_group(
        &mut self,
        queued: &[usize],
        h: Real,
        report: &mut Report,
    ) -> Result<(), Error> {
        self.ensure_ccd_offsets();
        let mut counts = Vec::new();
        super::reserve(&mut counts, queued.len(), &mut self.bookkeeping.work);
        counts.resize(queued.len(), 0u8);
        // A budget-exhausted body remains at the impact pose for the rest of this substep.
        let mut frozen: Vec<(usize, Vector)> = Vec::new();
        let mut excluded: Vec<(usize, usize)> = Vec::new();
        let limit = self.scene.velocity_iterations.max(1);
        let mut elapsed = 0.0;
        while elapsed < 1.0 {
            let mut best: Option<(Real, usize, usize, contact::Manifold, bool)> = None;
            for &i in queued {
                if frozen.iter().any(|(index, _)| *index == i) || !advances(&self.bodies[i]) {
                    continue;
                }
                let at = self.position_at(i, elapsed, h);
                let candidate = self.next_ccd_hit(
                    i,
                    at,
                    TickTime { elapsed, h },
                    |j| excluded.contains(&(i.min(j), i.max(j))),
                    &frozen,
                    report,
                )?;
                if let Some((time, j, manifold, first)) = candidate {
                    let key = |i: usize, j: usize| {
                        let a = self.bodies[i].order;
                        let b = self.bodies[j].order;
                        (a.min(b), a.max(b))
                    };
                    if best.as_ref().is_none_or(|(t, a, b, ..)| {
                        time < *t || (time == *t && key(i, j) < key(*a, *b))
                    }) {
                        best = Some((time, i, j, manifold, first));
                    }
                }
            }
            let Some((time, i, j, manifold, first)) = best else {
                break;
            };
            elapsed += time * (1.0 - elapsed);
            let pair = (i.min(j), i.max(j));
            if !self.respond_ccd_impact(i, j, &manifold, first, TickTime { elapsed, h }, report) {
                push(&mut excluded, pair, &mut self.bookkeeping.work);
                continue;
            }
            excluded.clear();
            push(&mut excluded, pair, &mut self.bookkeeping.work);
            for index in [i, j] {
                if let Some(k) = queued
                    .iter()
                    .position(|&queued_index| queued_index == index)
                {
                    if counts[k] >= limit {
                        continue;
                    }
                    counts[k] += 1;
                    if counts[k] == limit && elapsed < 1.0 {
                        let position = self.position_at(index, elapsed, h);
                        push(&mut frozen, (index, position), &mut self.bookkeeping.work);
                        report.geometry.ccd_budget_fallbacks += 1;
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

    fn next_ccd_hit(
        &mut self,
        i: usize,
        position: Vector,
        clock: TickTime,
        excluded: impl Fn(usize) -> bool,
        frozen: &[(usize, Vector)],
        report: &mut Report,
    ) -> Result<Option<(Real, usize, contact::Manifold, bool)>, Error> {
        let mut probe = self.bodies[i].clone();
        probe.position = position;
        let travel = probe.velocity * clock.remaining();
        let swept_bounds = {
            let now = contact::bounds(&probe);
            union(now, (now.0 + travel, now.1 + travel))
        };
        let mut best: Option<(Real, usize, contact::Manifold, bool)> = None;
        for j in 0..self.bodies.len() {
            if j == i || excluded(j) {
                continue;
            }
            let other = &self.bodies[j];
            if other.sensor || !probe.layers.collides_with(other.layers) {
                continue;
            }
            report.pair_tests += 1;
            let mut target = other.clone();
            if let Some((_, position)) = frozen.iter().find(|(index, _)| *index == j) {
                target.position = *position;
                target.velocity = Vector::ZERO;
            } else {
                target.position = self.position_at(j, clock.elapsed, clock.h);
            }
            let target_bounds = {
                let now = contact::bounds(&target);
                let motion = target.velocity * clock.remaining();
                union(now, (now.0 + motion, now.1 + motion))
            };
            if !overlaps(swept_bounds, target_bounds) {
                continue;
            }
            report.narrow_tests += 1;
            let first = probe.order < target.order;
            let (a, b) = if first {
                (&probe, &target)
            } else {
                (&target, &probe)
            };
            let swept = contact::swept(
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

    /// Apply exactly one admitted impact at its TOI, retaining piecewise linear trajectories
    /// for both participants. Returns false for non-closing touches (which do not use a pass).
    fn respond_ccd_impact(
        &mut self,
        i: usize,
        j: usize,
        m: &contact::Manifold,
        first: bool,
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
        let closing = (contact_velocity(&self.bodies[j], r_other, true)
            - contact_velocity(&self.bodies[i], r_self, true))
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
        let relative =
            contact_velocity(partner, r_other, true) - contact_velocity(body, r_self, true);
        let approach = relative.dot(n).min(0.0);
        self.transaction.body(i, body);
        self.transaction.body(j, partner);
        let pair = [(body, &self.responses[i]), (partner, &self.responses[j])];
        let arms = [r_self, r_other];
        let k = effective_mass(pair, arms, n, [true, true], true, report);
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
                let kt = effective_mass(pair, arms, t, [true, true], true, report);
                if kt > 1e-14 {
                    impulse -= t * (tangential.length() / kt).min(mu * normal);
                }
            }
            let before = [self.bodies[i].velocity, self.bodies[j].velocity];
            for (index, arm, impulse) in [(i, r_self, -impulse), (j, r_other, impulse)] {
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
