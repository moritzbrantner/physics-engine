//! Numerical policy for simulation code.
//!
//! New simulation state and algorithms should use [`Scalar`] rather than grow exact integer
//! fractions. Integer IDs, masks, counters and explicit serialization/compatibility formats remain
//! integers. The `exact-reference` feature is a diagnostic replay backend, not the production default.

/// Default CPU simulation scalar. GPU/storage boundaries may explicitly narrow to `f32`.
pub type Scalar = f64;

/// Identifies the arithmetic backend compiled into this build.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum NumericalBackend {
    Float64,
    ExactReference,
}

/// Numerical backend used by time composition and shared scaled-arithmetic helpers.
pub const NUMERICAL_BACKEND: NumericalBackend = if cfg!(feature = "exact-reference") {
    NumericalBackend::ExactReference
} else {
    NumericalBackend::Float64
};

#[cfg(not(feature = "exact-reference"))]
pub(crate) use crate::float_math::{
    ArithmeticError, Ratio, mul_div_round_i128, mul_div_round_u128,
};
#[cfg(feature = "exact-reference")]
pub(crate) use crate::wide_ratio::{
    ExactRatio as Ratio, WideRatioError as ArithmeticError, mul_div_round_i128, mul_div_round_u128,
};

#[cfg(test)]
mod tests {
    use super::{NUMERICAL_BACKEND, NumericalBackend, Scalar};

    #[test]
    fn default_scalar_is_float64() {
        let half: Scalar = 0.5;
        assert_eq!(size_of::<Scalar>(), 8);
        assert_eq!(half + half, 1.0);
        assert_eq!(
            NUMERICAL_BACKEND,
            if cfg!(feature = "exact-reference") {
                NumericalBackend::ExactReference
            } else {
                NumericalBackend::Float64
            }
        );
    }
}
