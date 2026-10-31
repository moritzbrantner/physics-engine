//! Errors returned by world operations.

use crate::BodyId;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    InvalidInput,
    /// Another live body already has this game key.
    DuplicateKey(u64),
    MissingBody(BodyId),
    NonFiniteState(BodyId),
    /// A failed search cannot establish collision absence. The attempted step is rolled back.
    CollisionSearchFailed {
        bodies: [BodyId; 2],
        reason: SweepFailure,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SweepFailure {
    InvalidGeometryInput,
    NonFiniteComputation,
    IterationLimit,
}
impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "fixed-step approximation: {self:?}")
    }
}
impl std::error::Error for Error {}
