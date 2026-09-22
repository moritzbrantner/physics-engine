//! Opt-in soft normal constraints and frozen-contact velocity relaxation.
//! Based on the mass-independent soft-constraint formulation described in
//! https://box2d.org/posts/2024/02/solver2d/. The default solver remains Baumgarte.
use super::{Constraint, Convergence, Report, Scalar, Vector, World, convergence};

/// Experimental correction policy. It does not change global damping or sleeping thresholds.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoftContact {
    /// Natural frequency in cycles/second. Capped at one quarter of the substep rate.
    pub frequency_hz: Scalar,
    /// Dimensionless damping of the constraint spring, not whole-body drag.
    pub damping_ratio: Scalar,
    /// Additional bias-free velocity passes per nonempty substep, after saving the motion.
    pub relaxation_iterations: u8,
}
impl Default for SoftContact {
    fn default() -> Self {
        Self {
            frequency_hz: 60.0,
            damping_ratio: 1.0,
            relaxation_iterations: 2,
        }
    }
}
impl SoftContact {
    pub(super) fn valid(self) -> bool {
        self.frequency_hz.is_finite()
            && (0.01..=10_000.0).contains(&self.frequency_hz)
            && self.damping_ratio.is_finite()
            && (0.0..=10.0).contains(&self.damping_ratio)
            && self.relaxation_iterations <= 8
    }
    pub(super) fn prepare(self, h: Scalar) -> Option<Coefficients> {
        let frequency = self.frequency_hz.min(0.25 / h);
        let omega = 2.0 * std::f64::consts::PI * frequency;
        let a1 = 2.0 * self.damping_ratio + omega * h;
        let a2 = h * omega * a1;
        let impulse_scale = 1.0 / (1.0 + a2);
        let out = Coefficients {
            bias_rate: omega / a1,
            mass_scale: a2 * impulse_scale,
            impulse_scale,
        };
        (out.bias_rate.is_finite()
            && out.mass_scale.is_finite()
            && out.mass_scale > 0.0
            && out.impulse_scale.is_finite())
        .then_some(out)
    }
}
#[derive(Clone, Copy, Debug)]
pub(super) struct Coefficients {
    pub bias_rate: Scalar,
    pub mass_scale: Scalar,
    pub impulse_scale: Scalar,
}
impl Coefficients {
    pub const RIGID: Self = Self {
        bias_rate: 0.0,
        mass_scale: 1.0,
        impulse_scale: 0.0,
    };
}
#[derive(Clone, Copy, Debug, Default)]
pub(super) struct Motion {
    pub velocity: Vector,
    pub angular: Vector,
}
#[derive(Clone, Copy, Debug, Default)]
pub struct CorrectionStats {
    pub softened_points: u64,
    pub hard_support_points: u64,
    pub relaxation_iterations: u64,
    pub relaxation_constraint_visits: u64,
    pub relaxation_residual_visits: u64,
    pub relaxation_skipped_iterations: u64,
    pub relaxation_motion_bytes: u64,
    pub unchanged_converged_skips: u64,
    pub unchanged_target_checks: u64,
}

impl World {
    /// Save bias-driven motion before relaxation; integrate that motion once while retaining
    /// the relaxed velocity for the next substep. Contact geometry/inertia stays frozen.
    pub(super) fn relax_contacts<const PREPARED: bool>(
        &mut self,
        constraints: &mut [Constraint],
        primary_converged: bool,
        report: &mut Report,
    ) -> bool {
        let relax = self
            .config
            .soft_contact
            .map_or(0, |c| c.relaxation_iterations);
        let relaxing = relax > 0 && !constraints.is_empty();
        if relaxing && primary_converged {
            // The primary solve already checked final whole-pass normal/friction residuals.
            // That certificate is reusable ONLY when relaxation would solve the same hard
            // equations on the same frozen state. A soft row or changed bias requires work.
            let unchanged = constraints.len() == self.relaxation_bias.len()
                && constraints
                    .iter()
                    .zip(&self.relaxation_bias)
                    .all(|(c, bias)| {
                        report.correction.unchanged_target_checks += 1;
                        c.hard_normal && c.bias.to_bits() == bias.to_bits()
                    });
            if unchanged {
                report.correction.unchanged_converged_skips += 1;
                report.correction.relaxation_skipped_iterations += u64::from(relax);
                return false;
            }
        }
        if relaxing {
            // Freeze the integration velocity, then relax using the SAME pre-integration
            // geometry/inertia. Pose is advanced once from this saved velocity; the relaxed
            // velocity survives for the next substep. This is a frozen-contact approximation,
            // not a second integration and not a re-use of stale inertia after rotation.
            self.relaxation_motion
                .resize(self.bodies.len(), Motion::default());
            for (motion, b) in self.relaxation_motion.iter_mut().zip(&self.bodies) {
                *motion = Motion {
                    velocity: b.velocity,
                    angular: b.angular_velocity,
                };
            }
            for (c, bias) in constraints.iter_mut().zip(&self.relaxation_bias) {
                c.bias = *bias;
            }
            let primary = std::mem::take(&mut report.convergence);
            let iterations_before = report.impulse_iterations;
            match self.config.convergence {
                Some(tolerance) => convergence::solve::<PREPARED, true>(
                    &mut self.bodies,
                    &self.responses,
                    constraints,
                    relax,
                    tolerance,
                    report,
                ),
                None => convergence::solve::<PREPARED, false>(
                    &mut self.bodies,
                    &self.responses,
                    constraints,
                    relax,
                    Convergence::default(),
                    report,
                ),
            }
            report.correction.relaxation_iterations +=
                report.impulse_iterations - iterations_before;
            report.correction.relaxation_constraint_visits += report.convergence.constraint_visits;
            report.correction.relaxation_residual_visits +=
                report.convergence.residual_constraint_visits;
            report.correction.relaxation_skipped_iterations +=
                report.convergence.skipped_iterations;
            report.convergence = primary;
        }
        relaxing
    }
}

#[cfg(test)]
mod tests;
