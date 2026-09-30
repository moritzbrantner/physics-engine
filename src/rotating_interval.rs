//! Atomic, uniformly partitioned intervals for the integer compatibility rotating world.

use std::{error::Error, fmt};

use crate::RotatingWorldError3d;

/// One requested physical interval, partitioned into `substeps` equal solver commands.
///
/// Angular damping applies only to physics-owned bodies, once after all substeps, using the legacy integer milli scale
/// and rounding to nearest (ties away from zero). Damping deltas are included in the last substep report.
/// A zero interval skips integration/sleep updates and still applies explicitly requested damping.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RotatingIntervalConfig3d {
    pub timestep_numerator: i32,
    pub timestep_denominator: i32,
    pub substeps: u8,
    pub angular_damping_milli: u16,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RotatingIntervalError3d {
    InvalidPartition,
    InvalidAngularDamping(u16),
    BallisticBodiesUnsupported,
    World(RotatingWorldError3d),
}

impl fmt::Display for RotatingIntervalError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidPartition => formatter.write_str("invalid rotating interval partition"),
            Self::InvalidAngularDamping(value) => {
                write!(formatter, "angular damping exceeds 1000: {value}")
            }
            Self::BallisticBodiesUnsupported => {
                formatter.write_str("atomic rotating intervals currently support rigid boxes only")
            }
            Self::World(error) => error.fmt(formatter),
        }
    }
}

impl Error for RotatingIntervalError3d {}

/// Work includes discarded attempts. Counts are available independently of diagnostic counters.
/// Contact counters are diagnostic and are zero when `performance-counters` is disabled.
/// Before-image counts describe touched entries, excluding allocator/tree overhead and solver scratch.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RotatingIntervalWork3d {
    pub completed_substeps: u8,
    pub motion_before_images: usize,
    pub sleep_before_images: usize,
    pub parked_before_images: usize,
    pub damping_body_visits: usize,
    pub damping_changes: usize,
    /// Broad-phase queries, tail queries, response passes, continuation evaluations, including probes.
    pub contact_work: [u64; 4],
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RotatingIntervalFailure3d {
    pub error: RotatingIntervalError3d,
    pub work: RotatingIntervalWork3d,
}

impl fmt::Display for RotatingIntervalFailure3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.error.fmt(formatter)
    }
}

impl Error for RotatingIntervalFailure3d {}

impl RotatingIntervalConfig3d {
    pub(crate) fn substep_denominator(self) -> Result<i32, RotatingIntervalError3d> {
        if self.timestep_numerator < 0 || self.timestep_denominator <= 0 || self.substeps == 0 {
            return Err(RotatingIntervalError3d::InvalidPartition);
        }
        if self.angular_damping_milli > 1000 {
            return Err(RotatingIntervalError3d::InvalidAngularDamping(
                self.angular_damping_milli,
            ));
        }
        self.timestep_denominator
            .checked_mul(i32::from(self.substeps))
            .ok_or(RotatingIntervalError3d::InvalidPartition)
    }
}
