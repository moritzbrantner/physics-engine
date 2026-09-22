//! Bounded velocity solving with a projected fixed-point residual, never a wall-clock budget.
use super::correction::Coefficients;
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
fn projected_residual<const SOFT: bool>(
    bodies: &[Body],
    constraints: &[Constraint],
    tolerance: Convergence,
    stats: &mut ConvergenceStats,
    coefficients: Coefficients,
) -> Option<Scalar> {
    stats.residual_checks += 1;
    let mut max_residual: Scalar = 0.0;
    for c in constraints {
        stats.residual_constraint_visits += 1;
        let rel = contact_velocity(&bodies[c.b], c.rb, c.spin)
            - contact_velocity(&bodies[c.a], c.ra, c.spin);
        let vn = rel.dot(c.n);
        let normal_delta = next_normal::<SOFT>(c, vn, coefficients) - c.normal_impulse;
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
        let normal_error = if SOFT && !c.hard_normal {
            c.bias
                - vn
                - coefficients.impulse_scale / (coefficients.mass_scale * c.normal_mass)
                    * c.normal_impulse
        } else {
            c.bias - vn
        };
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

pub(super) fn solve<const PREPARED: bool, const EARLY: bool>(
    bodies: &mut [Body],
    responses: &[PreparedResponse],
    constraints: &mut [Constraint],
    iterations: u8,
    tolerance: Convergence,
    report: &mut Report,
) {
    solve_impl::<PREPARED, EARLY, false>(
        bodies,
        responses,
        constraints,
        iterations,
        tolerance,
        Coefficients::RIGID,
        report,
    );
}

pub(super) fn solve_soft<const PREPARED: bool>(
    bodies: &mut [Body],
    responses: &[PreparedResponse],
    constraints: &mut [Constraint],
    iterations: u8,
    tolerance: Option<Convergence>,
    coefficients: Coefficients,
    report: &mut Report,
) {
    match tolerance {
        Some(t) => solve_impl::<PREPARED, true, true>(
            bodies,
            responses,
            constraints,
            iterations,
            t,
            coefficients,
            report,
        ),
        None => solve_impl::<PREPARED, false, true>(
            bodies,
            responses,
            constraints,
            iterations,
            Convergence::default(),
            coefficients,
            report,
        ),
    }
}

#[inline(always)]
fn next_normal<const SOFT: bool>(c: &Constraint, vn: Scalar, coefficients: Coefficients) -> Scalar {
    if SOFT && !c.hard_normal {
        let delta = coefficients.mass_scale * c.normal_mass * (c.bias - vn)
            - coefficients.impulse_scale * c.normal_impulse;
        (c.normal_impulse + delta).max(0.0)
    } else {
        (c.normal_impulse + (c.bias - vn) * c.normal_mass).max(0.0)
    }
}

fn solve_impl<const PREPARED: bool, const EARLY: bool, const SOFT: bool>(
    bodies: &mut [Body],
    responses: &[PreparedResponse],
    constraints: &mut [Constraint],
    iterations: u8,
    tolerance: Convergence,
    coefficients: Coefficients,
    report: &mut Report,
) {
    if EARLY && constraints.is_empty() {
        report.convergence.empty_substeps += 1;
        report.convergence.skipped_iterations += u64::from(iterations);
        return;
    }
    for pass in 0..iterations {
        report.impulse_iterations += 1;
        report.convergence.constraint_visits += constraints.len() as u64;
        // An initial warm-started pass alone is not accepted. No final residual scan is needed
        // when the iteration ceiling has already been reached or an earlier correction is large.
        let completed = u32::from(pass) + 1;
        // Probe after 2/4/8/... completed passes, never after reaching the ceiling.
        // Hard stacks do not pay for a convergence probe on every iteration.
        let mut candidate = EARLY
            && completed >= 2
            && completed.is_power_of_two()
            && completed < u32::from(iterations);
        let mut max_delta: Scalar = 0.0;
        let mut rows = constraints.iter_mut();
        if candidate {
            report.convergence.probe_passes += 1;
            for c in rows.by_ref() {
                report.convergence.delta_constraint_checks += 1;
                let change =
                    solve_row::<PREPARED, true, SOFT>(bodies, responses, c, coefficients, report);
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
        // After the first large correction this pass cannot finish early. The rest uses the
        // identical uninstrumented row kernel, not a per-contact convergence branch.
        for c in rows {
            solve_row::<PREPARED, false, SOFT>(bodies, responses, c, coefficients, report);
        }
        // Later contacts can disturb an earlier row: all candidates must be checked again using
        // the final velocities of this complete pass before any remaining passes are skipped.
        if candidate
            && let Some(residual) = projected_residual::<SOFT>(
                bodies,
                constraints,
                tolerance,
                &mut report.convergence,
                coefficients,
            )
        {
            report.convergence.converged_substeps += 1;
            report.convergence.skipped_iterations += u64::from(iterations - pass - 1);
            report.convergence.max_exit_impulse_delta =
                report.convergence.max_exit_impulse_delta.max(max_delta);
            report.convergence.max_exit_velocity_residual =
                report.convergence.max_exit_velocity_residual.max(residual);
            return;
        }
    }
    if EARLY {
        report.convergence.capped_substeps += 1;
    } else {
        report.convergence.fixed_substeps += 1;
    }
}

#[derive(Default)]
struct Change {
    impulse: [Scalar; 3],
    velocity_scale: [Scalar; 3],
}

// The arithmetic order is identical in both specializations. READ_CHANGE=false removes only
// observation calculations; it is also the fixed-iteration reference row, not different physics.
#[inline(always)]
fn solve_row<const PREPARED: bool, const READ_CHANGE: bool, const SOFT: bool>(
    bodies: &mut [Body],
    responses: &[PreparedResponse],
    c: &mut Constraint,
    coefficients: Coefficients,
    report: &mut Report,
) -> Change {
    let vn = (contact_velocity(&bodies[c.b], c.rb, c.spin)
        - contact_velocity(&bodies[c.a], c.ra, c.spin))
    .dot(c.n);
    let next = next_normal::<SOFT>(c, vn, coefficients);
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
