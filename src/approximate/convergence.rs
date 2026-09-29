//! Bounded velocity solving with a projected fixed-point residual, never a wall-clock budget.
use super::{Body, Constraint, PreparedResponse, Report, Scalar, apply, contact_velocity};

/// Absolute scene-unit tolerances plus a relative per-contact scale. Zero selects exact checks.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Convergence {
    /// Velocity residual in scene units per second.
    pub absolute_velocity: Scalar,
    /// Incremental impulse in mass-units * scene-units per second.
    pub absolute_impulse: Scalar,
    pub relative: Scalar,
}
impl Default for Convergence {
    fn default() -> Self {
        Self {
            absolute_velocity: 1e-5,
            absolute_impulse: 1e-7,
            relative: 1e-6,
        }
    }
}
impl Convergence {
    pub(super) fn valid(self) -> bool {
        [self.absolute_velocity, self.absolute_impulse, self.relative]
            .into_iter()
            .all(|x| x.is_finite() && x >= 0.0)
            && self.relative <= 0.01
    }
    fn small(
        self,
        delta: Scalar,
        mass: Scalar,
        impulse_scale: Scalar,
        velocity_scale: Scalar,
    ) -> bool {
        let impulse_limit = self.absolute_impulse + self.relative * impulse_scale;
        let velocity_limit = self.absolute_velocity + self.relative * velocity_scale;
        delta.is_finite()
            && mass.is_finite()
            && mass > 0.0
            && impulse_limit.is_finite()
            && velocity_limit.is_finite()
            && delta.abs() <= impulse_limit
            && delta.abs() / mass <= velocity_limit
    }
}

/// Work and accepted-exit diagnostics. A zero residual with no residual_checks means unmeasured.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ConvergenceStats {
    pub constraint_visits: u64,
    pub residual_checks: u64,
    pub residual_constraint_visits: u64,
    pub converged_substeps: u64,
    pub capped_substeps: u64,
    pub empty_substeps: u64,
    pub skipped_iterations: u64,
    /// Maximum final-pass impulse change across accepted early exits only.
    pub max_exit_impulse_delta: Scalar,
    /// Maximum projected velocity residual across accepted early exits only.
    pub max_exit_velocity_residual: Scalar,
    pub fixed_substeps: u64,
    pub probe_passes: u64,
    pub delta_constraint_checks: u64,
}

// A sliding contact may retain tangential velocity at the friction limit. Measure the projected
// impulse correction, not raw slip velocity; an inactive normal must likewise permit separation.
#[cfg(test)]
fn projected_residual(
    bodies: &[Body],
    constraints: &[Constraint],
    tolerance: Convergence,
    stats: &mut ConvergenceStats,
) -> Option<Scalar> {
    projected_residual_rows(bodies, constraints.iter(), tolerance, stats)
}

fn projected_residual_rows<'a>(
    bodies: &[Body],
    constraints: impl Iterator<Item = &'a Constraint>,
    tolerance: Convergence,
    stats: &mut ConvergenceStats,
) -> Option<Scalar> {
    stats.residual_checks += 1;
    let mut max_residual: Scalar = 0.0;
    for c in constraints {
        stats.residual_constraint_visits += 1;
        let rel = contact_velocity(&bodies[c.b], c.rb, c.spin)
            - contact_velocity(&bodies[c.a], c.ra, c.spin);
        let vn = rel.dot(c.n);
        let normal_delta = next_normal(c, vn) - c.normal_impulse;
        let mut tangent = [
            c.tangent_impulse[0] - rel.dot(c.t1) * c.tangent_mass[0],
            c.tangent_impulse[1] - rel.dot(c.t2) * c.tangent_mass[1],
        ];
        let length = tangent[0].hypot(tangent[1]);
        let limit = c.friction * c.normal_impulse;
        if length > limit && length > 0.0 {
            tangent[0] *= limit / length;
            tangent[1] *= limit / length;
        }
        let normal_error = normal_error(c, vn);
        let normal_residual = if c.normal_impulse > 0.0 {
            normal_error.abs()
        } else {
            normal_error.max(0.0)
        };
        let normal_limit =
            tolerance.absolute_velocity + tolerance.relative * vn.abs().max(c.bias.abs());
        if !normal_residual.is_finite() || normal_residual > normal_limit {
            return None;
        }
        max_residual = max_residual.max(normal_residual);
        // A large accumulated impulse can absorb a nonzero correction when rounded to f64.
        // Check the friction disk's first-order stationarity independently of lambda + delta.
        let old_length = c.tangent_impulse[0].hypot(c.tangent_impulse[1]);
        if limit > 0.0 {
            let vt = [rel.dot(c.t1), rel.dot(c.t2)];
            let mut gradient = [-vt[0] * c.tangent_mass[0], -vt[1] * c.tangent_mass[1]];
            if old_length > 0.0 && old_length >= limit - 8.0 * Scalar::EPSILON * limit {
                let unit = [
                    c.tangent_impulse[0] / old_length,
                    c.tangent_impulse[1] / old_length,
                ];
                let outward = (gradient[0] * unit[0] + gradient[1] * unit[1]).max(0.0);
                gradient[0] -= unit[0] * outward;
                gradient[1] -= unit[1] * outward;
            }
            for axis in 0..2 {
                let residual = gradient[axis].abs() / c.tangent_mass[axis];
                if !residual.is_finite()
                    || residual > tolerance.absolute_velocity + tolerance.relative * vt[axis].abs()
                {
                    return None;
                }
                max_residual = max_residual.max(residual);
            }
        }
        let impulse_scale = c
            .normal_impulse
            .abs()
            .max(c.tangent_impulse[0].abs())
            .max(c.tangent_impulse[1].abs());
        let velocity_scales = [
            vn.abs().max(c.bias.abs()),
            rel.dot(c.t1).abs(),
            rel.dot(c.t2).abs(),
        ];
        if !rel.finite() || !vn.is_finite() || !length.is_finite() {
            return None;
        }
        for ((delta, mass), velocity_scale) in [
            (normal_delta, c.normal_mass),
            (tangent[0] - c.tangent_impulse[0], c.tangent_mass[0]),
            (tangent[1] - c.tangent_impulse[1], c.tangent_mass[1]),
        ]
        .into_iter()
        .zip(velocity_scales)
        {
            if !tolerance.small(delta, mass, impulse_scale, velocity_scale) {
                return None;
            }
            max_residual = max_residual.max(delta.abs() / mass);
        }
    }
    Some(max_residual)
}

// Same row kernel and arithmetic for contiguous/global and indexed/island solving.
// Indirection is specialized away for the existing single-island/fixed-pass path.
trait Rows: Copy {
    fn len(self) -> usize;
    fn index(self, row: usize) -> usize;
}
#[derive(Clone, Copy)]
struct All(usize);
impl Rows for All {
    fn len(self) -> usize {
        self.0
    }
    fn index(self, row: usize) -> usize {
        row
    }
}
impl Rows for &[usize] {
    fn len(self) -> usize {
        <[usize]>::len(self)
    }
    fn index(self, row: usize) -> usize {
        self[row]
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Exit {
    Pending,
    Empty,
    Converged,
    Capped,
    Fixed,
}
#[derive(Clone, Copy, Debug)]
pub(super) struct Outcome {
    pub passes: u8,
    pub exit: Exit,
}

// Historical substep counters remain substep counters, NOT a sum over islands. In a partitioned
// solve, passes is the maximum over its islands (the number of equivalent round-robin rounds).
// Actual work is constraint_visits and the separately reported sum of island_iterations.
pub(super) fn record_substep(outcome: Outcome, limit: u8, report: &mut Report) {
    report.impulse_iterations += u64::from(outcome.passes);
    report.convergence.skipped_iterations += u64::from(limit - outcome.passes);
    match outcome.exit {
        Exit::Empty => report.convergence.empty_substeps += 1,
        Exit::Converged => report.convergence.converged_substeps += 1,
        Exit::Capped => report.convergence.capped_substeps += 1,
        Exit::Fixed => report.convergence.fixed_substeps += 1,
        Exit::Pending => unreachable!("a shared prefix is not a complete substep"),
    }
}

pub(super) fn solve<const PREPARED: bool, const EARLY: bool>(
    bodies: &mut [Body],
    responses: &[PreparedResponse],
    constraints: &mut [Constraint],
    iterations: u8,
    tolerance: Convergence,
    report: &mut Report,
) {
    let outcome = solve_rows::<PREPARED, EARLY, 0, false, _>(
        bodies,
        responses,
        constraints,
        All(constraints.len()),
        iterations,
        tolerance,
        report,
    );
    record_substep(outcome, iterations, report);
}

pub(super) fn solve_selected<const PREPARED: bool>(
    bodies: &mut [Body],
    responses: &[PreparedResponse],
    constraints: &mut [Constraint],
    selected: &[usize],
    iterations: u8,
    tolerance: Convergence,
    report: &mut Report,
) -> Outcome {
    solve_rows::<PREPARED, true, 2, false, _>(
        bodies,
        responses,
        constraints,
        selected,
        iterations,
        tolerance,
        report,
    )
}

pub(super) fn solve_all_after_prefix<const PREPARED: bool>(
    bodies: &mut [Body],
    responses: &[PreparedResponse],
    constraints: &mut [Constraint],
    iterations: u8,
    tolerance: Convergence,
    report: &mut Report,
) -> Outcome {
    solve_rows::<PREPARED, true, 2, false, _>(
        bodies,
        responses,
        constraints,
        All(constraints.len()),
        iterations,
        tolerance,
        report,
    )
}

pub(super) fn solve_prefix<const PREPARED: bool>(
    bodies: &mut [Body],
    responses: &[PreparedResponse],
    constraints: &mut [Constraint],
    iterations: u8,
    tolerance: Convergence,
    report: &mut Report,
) -> Outcome {
    solve_rows::<PREPARED, true, 0, true, _>(
        bodies,
        responses,
        constraints,
        All(constraints.len()),
        iterations,
        tolerance,
        report,
    )
}

fn solve_rows<
    const PREPARED: bool,
    const EARLY: bool,
    const START: u8,
    const PREFIX: bool,
    R: Rows,
>(
    bodies: &mut [Body],
    responses: &[PreparedResponse],
    constraints: &mut [Constraint],
    rows: R,
    iterations: u8,
    tolerance: Convergence,
    report: &mut Report,
) -> Outcome {
    if EARLY && rows.len() == 0 {
        return Outcome {
            passes: 0,
            exit: Exit::Empty,
        };
    }
    for pass in START..iterations {
        report.convergence.constraint_visits += rows.len() as u64;
        let completed = u32::from(pass) + 1;
        // A first pass is never certified. Probe after 2/4/8/... complete passes, below the ceiling.
        let mut candidate = EARLY
            && completed >= 2
            && completed.is_power_of_two()
            && completed < u32::from(iterations);
        let mut max_delta: Scalar = 0.0;
        let mut unchecked = 0;
        if candidate {
            report.convergence.probe_passes += 1;
            for n in 0..rows.len() {
                let c = &mut constraints[rows.index(n)];
                unchecked = n + 1;
                report.convergence.delta_constraint_checks += 1;
                let change = solve_row::<PREPARED, true>(bodies, responses, c, report);
                let impulse_scale = c
                    .normal_impulse
                    .abs()
                    .max(c.tangent_impulse[0].abs())
                    .max(c.tangent_impulse[1].abs());
                candidate = tolerance.small(
                    change.impulse[0],
                    c.normal_mass,
                    impulse_scale,
                    change.velocity_scale[0],
                ) && tolerance.small(
                    change.impulse[1],
                    c.tangent_mass[0],
                    impulse_scale,
                    change.velocity_scale[1],
                ) && tolerance.small(
                    change.impulse[2],
                    c.tangent_mass[1],
                    impulse_scale,
                    change.velocity_scale[2],
                );
                if !candidate {
                    break;
                }
                max_delta = max_delta
                    .max(change.impulse[0].abs())
                    .max(change.impulse[1].abs())
                    .max(change.impulse[2].abs());
            }
        }
        // Never omit the rest of a pass after a large correction: use the identical cheaper kernel.
        for n in unchecked..rows.len() {
            solve_row::<PREPARED, false>(
                bodies,
                responses,
                &mut constraints[rows.index(n)],
                report,
            );
        }
        // Check final velocities of every row in this island after completing the ENTIRE pass.
        // Partition construction includes mutable read dependencies, including one-way responses.
        if candidate
            && let Some(residual) = projected_residual_rows(
                bodies,
                (0..rows.len()).map(|n| &constraints[rows.index(n)]),
                tolerance,
                &mut report.convergence,
            )
        {
            report.convergence.max_exit_impulse_delta =
                report.convergence.max_exit_impulse_delta.max(max_delta);
            report.convergence.max_exit_velocity_residual =
                report.convergence.max_exit_velocity_residual.max(residual);
            return Outcome {
                passes: pass + 1,
                exit: Exit::Converged,
            };
        }
        if PREFIX && pass == 1 {
            return Outcome {
                passes: 2,
                exit: Exit::Pending,
            };
        }
    }
    Outcome {
        passes: iterations,
        exit: if EARLY { Exit::Capped } else { Exit::Fixed },
    }
}

// Compile-time opt-in: ordinary builds retain the original hard row formula and layout.
#[inline(always)]
fn next_normal(c: &Constraint, vn: Scalar) -> Scalar {
    #[cfg(feature = "experimental-soft-contact")]
    if !c.hard_normal && !c.relaxing_normal {
        let coefficients = c.normal_coefficients;
        let delta = coefficients.mass_scale * c.normal_mass * (c.bias - vn)
            - coefficients.impulse_scale * c.normal_impulse;
        return (c.normal_impulse + delta).max(0.0);
    }
    (c.normal_impulse + (c.bias - vn) * c.normal_mass).max(0.0)
}
#[inline(always)]
fn normal_error(c: &Constraint, vn: Scalar) -> Scalar {
    #[cfg(feature = "experimental-soft-contact")]
    if !c.hard_normal && !c.relaxing_normal {
        let coefficients = c.normal_coefficients;
        return c.bias
            - vn
            - coefficients.impulse_scale / (coefficients.mass_scale * c.normal_mass)
                * c.normal_impulse;
    }
    c.bias - vn
}

#[derive(Default)]
struct Change {
    impulse: [Scalar; 3],
    velocity_scale: [Scalar; 3],
}

// The arithmetic order is identical in both specializations. READ_CHANGE=false removes only
// observation calculations; it is also the fixed-iteration reference row, not different physics.
#[inline(always)]
fn solve_row<const PREPARED: bool, const READ_CHANGE: bool>(
    bodies: &mut [Body],
    responses: &[PreparedResponse],
    c: &mut Constraint,
    report: &mut Report,
) -> Change {
    let vn = (contact_velocity(&bodies[c.b], c.rb, c.spin)
        - contact_velocity(&bodies[c.a], c.ra, c.spin))
    .dot(c.n);
    let next = next_normal(c, vn);
    let dj = next - c.normal_impulse;
    c.normal_impulse = next;
    apply::<PREPARED>(bodies, responses, c, c.n * dj, report);
    let rel =
        contact_velocity(&bodies[c.b], c.rb, c.spin) - contact_velocity(&bodies[c.a], c.ra, c.spin);
    let vt = [rel.dot(c.t1), rel.dot(c.t2)];
    let mut tangent = [
        c.tangent_impulse[0] - vt[0] * c.tangent_mass[0],
        c.tangent_impulse[1] - vt[1] * c.tangent_mass[1],
    ];
    let len = tangent[0].hypot(tangent[1]);
    let limit = c.friction * c.normal_impulse;
    if len > limit && len > 0.0 {
        tangent[0] *= limit / len;
        tangent[1] *= limit / len;
    }
    let dt = [
        tangent[0] - c.tangent_impulse[0],
        tangent[1] - c.tangent_impulse[1],
    ];
    let jt = c.t1 * dt[0] + c.t2 * dt[1];
    c.tangent_impulse = tangent;
    apply::<PREPARED>(bodies, responses, c, jt, report);
    if READ_CHANGE {
        Change {
            impulse: [dj, dt[0], dt[1]],
            velocity_scale: [vn.abs().max(c.bias.abs()), vt[0].abs(), vt[1].abs()],
        }
    } else {
        Change::default()
    }
}

#[cfg(test)]
mod tests;
