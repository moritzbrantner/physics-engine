//! Remaining-time response for translational CCD bodies.
//!
//! A CCD body whose substep produced an admitted swept impact is not limited by a speculative
//! constraint. Instead it advances to the independent time of impact, receives the pair's
//! normal/friction/restitution response there, and re-sweeps the rest of the substep along the
//! response velocity against every candidate. The loop is bounded by the velocity-pass budget;
//! when that is exhausted the body stays at its last impact for the rest of the substep
//! (`GeometryStats::ccd_budget_fallbacks`). Rotation and non-CCD bodies are unchanged.
use super::{
    Body, Error, Real, Report, Vector, World, bookkeeping::push, contact, contact_velocity,
    effective_mass,
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

impl World {
    /// Advance every queued CCD body in canonical BodyId (index) order.
    pub(super) fn advance_ccd_bodies<const PREPARED: bool>(
        &mut self,
        h: Real,
        report: &mut Report,
    ) -> Result<(), Error> {
        let mut queued = std::mem::take(&mut self.bookkeeping.ccd);
        queued.sort_unstable();
        queued.dedup();
        let mut result = Ok(());
        for &i in &queued {
            result = self.advance_ccd_body::<PREPARED>(i, h, report);
            if result.is_err() {
                break;
            }
        }
        // Kept sorted for this substep: the advanced bodies' contacts are already resolved.
        self.bookkeeping.ccd = queued;
        result
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

    fn advance_ccd_body<const PREPARED: bool>(
        &mut self,
        i: usize,
        h: Real,
        report: &mut Report,
    ) -> Result<(), Error> {
        if !advances(&self.bodies[i]) {
            return Ok(());
        }
        let count = self.bodies.len();
        if self.bookkeeping.ccd_offsets.len() < count {
            let s = &mut self.bookkeeping;
            super::reserve(&mut s.ccd_offsets, count, &mut s.work);
            s.ccd_offsets.resize(count, Vector::ZERO);
        }
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
        let limit = self.config.velocity_iterations.max(1);
        let mut excluded = std::mem::take(&mut self.bookkeeping.ccd_excluded);
        excluded.clear();
        let outcome = loop {
            let remaining = 1.0 - elapsed;
            if remaining <= 0.0 {
                break Ok(());
            }
            let mut probe = self.bodies[i].clone();
            probe.position = position;
            let travel = probe.velocity * (remaining * h);
            let swept_bounds = {
                let now = contact::bounds(&probe);
                union(now, (now.0 + travel, now.1 + travel))
            };
            let mut best: Option<(Real, usize, contact::Manifold, bool)> = None;
            for j in 0..self.bodies.len() {
                if j == i || excluded.contains(&j) {
                    continue;
                }
                let other = &self.bodies[j];
                if other.sensor || !probe.layers.collides_with(other.layers) {
                    continue;
                }
                report.pair_tests += 1;
                let mut target = other.clone();
                target.position = self.position_at(j, elapsed, h);
                let target_bounds = {
                    let now = contact::bounds(&target);
                    let motion = target.velocity * (remaining * h);
                    union(now, (now.0 + motion, now.1 + motion))
                };
                if !overlaps(swept_bounds, target_bounds) {
                    continue;
                }
                report.narrow_tests += 1;
                let first = probe.id < target.id;
                let (a, b) = if first {
                    (&probe, &target)
                } else {
                    (&target, &probe)
                };
                let swept = contact::swept(
                    a,
                    b,
                    remaining * h,
                    self.config.contact_slop,
                    &mut report.geometry,
                )
                .map_err(|reason| Error::CollisionSearchFailed {
                    bodies: [a.id, b.id],
                    reason,
                });
                let m = match swept {
                    Ok(m) => m,
                    Err(error) => {
                        self.bookkeeping.ccd_excluded = excluded;
                        return Err(error);
                    }
                };
                if let Some(m) = m
                    && best.as_ref().is_none_or(|(t, ..)| m.time < *t)
                {
                    best = Some((m.time, j, m, first));
                }
            }
            let Some((time, j, m, first)) = best else {
                position += travel;
                break Ok(());
            };
            let step = time * remaining;
            position += probe.velocity * (step * h);
            elapsed += step;
            // Normal from the CCD body toward its partner, arms at the impact pose.
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
                // Touching while separating is not an impact and does not use the budget;
                // do not re-test this partner.
                push(&mut excluded, j, &mut self.bookkeeping.work);
                continue;
            }
            impacts += 1;
            // An admitted closing impact is wake evidence; real mass/inertia (and the parked
            // body's skipped forces) are active before its impulse.
            if self.bodies[j].sleeping && self.bodies[j].movable() {
                let id = self.bodies[j].id;
                let woke = self.wake_contact_island::<PREPARED>(id, h, report);
                report.woken_bodies += woke;
            }
            let body = &self.bodies[i];
            let partner = &self.bodies[j];
            let relative =
                contact_velocity(partner, r_other, true) - contact_velocity(body, r_self, true);
            let approach = relative.dot(n).min(0.0);
            self.transaction.body(j, partner);
            let pair = [(body, &self.responses[i]), (partner, &self.responses[j])];
            let arms = [r_self, r_other];
            let k = effective_mass::<PREPARED>(pair, arms, n, [true, true], true, report);
            if k > 1e-14 {
                let restitution = if approach < -1.0 {
                    body.restitution.min(partner.restitution)
                } else {
                    0.0
                };
                let normal = -(1.0 + restitution) * approach / k;
                let tangential = relative - n * approach;
                let mut impulse = n * normal;
                let mu = body.friction.max(partner.friction);
                if mu > 0.0 && tangential.length() > 0.0 {
                    let t = tangential.unit();
                    let kt = effective_mass::<PREPARED>(pair, arms, t, [true, true], true, report);
                    if kt > 1e-14 {
                        impulse -= t * (tangential.length() / kt).min(mu * normal);
                    }
                }
                let before = self.bodies[j].velocity;
                for (index, arm, impulse) in [(i, r_self, -impulse), (j, r_other, impulse)] {
                    let b = &mut self.bodies[index];
                    let prepared = self.responses[index].select::<PREPARED>(b, report);
                    b.velocity += impulse * prepared.inverse_mass;
                    b.angular_velocity += prepared.inertia(b, arm.cross(impulse), report);
                }
                // The partner moved with its earlier velocity until the impact.
                let change = self.bodies[j].velocity - before;
                self.bookkeeping.ccd_offsets[j] -= change * (elapsed * h);
                let start = &mut self.bookkeeping.ccd_start[j];
                *start = start.max(elapsed);
                // `contact_points` counts solver rows; a remaining-time impact is not one.
                report.swept_contacts += 1;
                let s = &mut self.bookkeeping;
                for index in [i, j] {
                    if self.bodies[index].retire_on_impact {
                        push(&mut s.ccd_retired, self.bodies[index].id, &mut s.work);
                    }
                }
            }
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
}
