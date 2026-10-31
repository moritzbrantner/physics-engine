//! World configuration and body-storage modes.

use super::{Convergence, ConvergenceScope, Real, Vector};

/// How a world stores its bodies; fixed when the world is created. Both modes give identical
/// results for the same calls (every traversal and tie follows body order); they differ only
/// in what adding and removing bodies costs. See docs/world-api-contract.md.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum BodyStorage {
    /// Bodies stay in body order in one array. Adding a body after every existing one and
    /// removing any body are amortized O(1) (removal leaves a tombstone that the next step,
    /// or a later removal once half the records are tombstones, compacts away); adding a body
    /// in the middle of the order moves the later records, O(n).
    #[default]
    Ordered,
    /// Bodies are appended to an unordered arena with a separate order index: adding and
    /// removing any body is O(log n). The next step (or the next call that wakes an island)
    /// sorts the arena back into body order once, O(n + k log k) for k out-of-order adds.
    Indexed,
}

/// Eligible pairs in the bounded post-integration position stage.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum PositionCorrection {
    /// Preserve the original fixed-collider-only comparison policy.
    #[default]
    FixedColliders,
    /// Also correct residual overlap between awake dynamic pairs admitted by CCD/contact
    /// generation in this substep. This does not discover contacts or replace swept admission.
    AdmittedContacts,
}

#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// m/s², default (0, −9.81, 0) as in Rapier; scaled by `length_unit`.
    pub gravity: Vector,
    pub substeps: u8,
    pub velocity_iterations: u8,
    /// Shared post-integration position-pass budget (historical field name).
    /// Zero disables correction; fixed and admitted dynamic pairs share each selected pass.
    pub fixed_position_iterations: u8,
    pub position_correction: PositionCorrection,
    /// Metres, default 0.005 (Rapier's allowed linear error); scaled by `length_unit`.
    pub contact_slop: Real,
    /// m/s, default 0.05 (Rapier's sleep threshold); scaled by `length_unit`.
    pub sleep_speed: Real,
    /// Seconds below `sleep_speed` before a body sleeps, default 0.5 as in Rapier.
    pub sleep_seconds: Real,
    pub warm_start: bool,
    /// None retains the fixed-pass reference. Some permits a checked early exit.
    pub convergence: Option<Convergence>,
    /// Contact-island-local stopping, or the original whole-world convergence reference.
    /// With `convergence: None`, both scopes use the unchanged fixed-pass solver.
    pub convergence_scope: ConvergenceScope,
    /// Scene units per metre (100 for a centimetre game), as Rapier's `length_unit`. The
    /// dynamics fields above (`gravity`, `contact_slop`, `sleep_speed`) are in metres and
    /// scaled by it once in [`World::new`](super::World::new), as are built-in tolerances and
    /// character defaults ([`World::character_settings`](super::World::character_settings)); body
    /// and shape values stay in scene units.
    pub length_unit: Real,
    /// Body storage layout, fixed for the world's lifetime.
    pub body_storage: BodyStorage,
}
impl Default for Config {
    fn default() -> Self {
        Self {
            gravity: Vector(0.0, -9.81, 0.0),
            substeps: 4,
            velocity_iterations: 8,
            fixed_position_iterations: 0,
            position_correction: PositionCorrection::FixedColliders,
            contact_slop: 0.005,
            sleep_speed: 0.05,
            sleep_seconds: 0.5,
            warm_start: true,
            convergence: Some(Convergence::default()),
            convergence_scope: ConvergenceScope::ContactIslands,
            length_unit: 1.0,
            // The test-only `indexed-storage` feature runs every suite on `Indexed` storage.
            body_storage: if cfg!(feature = "indexed-storage") {
                BodyStorage::Indexed
            } else {
                BodyStorage::Ordered
            },
        }
    }
}
#[cfg(test)]
impl Config {
    /// The pre-#318 scene-unit dynamics settings that legacy solver tests were tuned with.
    pub(crate) fn legacy() -> Self {
        Self {
            gravity: Vector(0.0, -3600.0, 0.0),
            contact_slop: 0.02,
            sleep_speed: 1.0,
            ..Self::default()
        }
    }
}
